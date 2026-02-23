package voxelbinning

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.must.Matchers

class VoxelBinningSpec
    extends AnyFreeSpec
    with Matchers
    with ChiselSim {

  "VoxelBinning should stream a window after enough bins, with correct flattened counts" in {
    simulate(new VoxelBinning(
      win = 400,
      w_bins = 5,
      r_bins = 5,
      grid = 16,
      counter = 8,
      parallelReads = 1,
      TIME_W = 32
      )) { dut =>

      // Reset
      dut.reset.poke(true.B)
      dut.clock.step(2)
      dut.reset.poke(false.B)

      // Always ready to consume readout stream
      dut.io.readout_ready.poke(true.B)

      // Helper: send one event using proper ready/valid handshake
      // Wait until event_ready is high, then assert event_valid for ONE cycle.
      def sendEvent(t: Int, x: Int, y: Int, pol: Boolean = true): Unit = {
        // keep valid low while waiting
        dut.io.event_valid.poke(false.B)

        // wait for DUT to be ready (it is not ready during CAPTURE/OUTPUT/CLEAR)
        var guard = 0
        while (!dut.io.event_ready.peek().litToBoolean && guard < 10000) {
          dut.clock.step()
          guard += 1
        }
        // if it never becomes ready, fail clearly
        dut.io.event_ready.expect(true.B)

        // drive event for one cycle
        dut.io.event_valid.poke(true.B)
        dut.io.event_time.poke(t.U)
        dut.io.event_x.poke(x.S)        // SInt (centered coords)
        dut.io.event_y.poke(y.S)
        dut.io.polarity.poke(pol.B)

        dut.clock.step()                // accept event on this edge (fire = valid && ready)

        // deassert valid
        dut.io.event_valid.poke(false.B)

        // small gap cycle (helps avoid edge cases)
        dut.clock.step()
      }

      // I'll pick (0,0) which maps to mx=8, my=8 -> cellIdx = 8*16 + 8 = 136
      val xC = 0
      val yC = 0
      val mappedX = xC + 16/2
      val mappedY = yC + 16/2
      val cellIdx = mappedY * 16 + mappedX // 136

      // dt = win / w_bins
      val dt = 400 / 5 // win/w_bins

      // Fill bins: put 1 event in the SAME cell in each bin.
      // The first event initializes timing/bin state and is also counted in bin0.
      sendEvent(t = 1,        x = xC, y = yC) // increments bin0
      sendEvent(t = 1 + 1*dt, x = xC, y = yC) // increments bin1
      sendEvent(t = 1 + 2*dt, x = xC, y = yC) // increments bin2
      sendEvent(t = 1 + 3*dt, x = xC, y = yC) // increments bin3
      sendEvent(t = 1 + 4*dt, x = xC, y = yC) // increments bin4 (binsInMem should now be >= 5)

      // Next boundary-crossing event should trigger CAPTURE/OUTPUT since binsInMem >= r_bins
      sendEvent(t = 1 + 5*dt, x = xC, y = yC)

      // Collect streamed window: featsPerWindow = r_bins * grid * grid = 5*16*16 = 1280
      // Flatten order is: oldest bin first -> newest bin, within each bin cell 0..255.
      // So the feature indices that should be 1 are:
      //   bin0 cellIdx: 0*256 + cellIdx
      //   bin1 cellIdx: 1*256 + cellIdx
      //   ...
      //   bin4 cellIdx: 4*256 + cellIdx
      val featsPerWindow = 5 * 16 * 16 // 1280
      val onesAt = Set(
        0 * 256 + cellIdx,
        1 * 256 + cellIdx,
        2 * 256 + cellIdx,
        3 * 256 + cellIdx,
        4 * 256 + cellIdx
      )

      var sawStart = false
      var sawLast  = false

      // Wait until readout_valid goes high (capture may take time)
      var waitCycles = 0
      while (!dut.io.readout_valid.peek().litToBoolean && waitCycles < 20000) {
        dut.clock.step()
        waitCycles += 1
      }
      dut.io.readout_valid.expect(true.B) // must eventually start outputting

      // Consume exactly featsPerWindow beats (parallelReads=1 => 1 feature per cycle)
      for (i <- 0 until featsPerWindow) {
        dut.io.readout_valid.expect(true.B)

        if (i == 0) {
          dut.io.readout_start.expect(true.B)
          sawStart = true
        } else {
          dut.io.readout_start.expect(false.B)
        }

        if (i == featsPerWindow - 1) {
          dut.io.readout_last.expect(true.B)
          sawLast = true
        } else {
          dut.io.readout_last.expect(false.B)
        }

        val v = dut.io.readout_data(0).peek().litValue.toInt
        val expected = if (onesAt.contains(i)) 1 else 0
        v mustBe expected

        dut.clock.step()
      }

      sawStart mustBe true
      sawLast mustBe true
    }
  }

  "VoxelBinning should produce exactly r_bins ones in the window for one event per bin" in {
    val win         = 400
    val wBins       = 5
    val rBins       = 5
    val grid        = 16
    val counterBits = 8
    val parallel    = 1
    val timeW       = 32

    simulate(new VoxelBinning(
      win = win,
      w_bins = wBins,
      r_bins = rBins,
      grid = grid,
      counter = counterBits,
      parallelReads = parallel,
      TIME_W = timeW
    )) { dut =>

      // Reset
      dut.reset.poke(true.B)
      dut.clock.step(2)
      dut.reset.poke(false.B)

      // Always ready to consume output
      dut.io.readout_ready.poke(true.B)

      // Helper: wait for ready then fire one event
      def sendEvent(t: Int, x: Int, y: Int): Unit = {
        dut.io.event_valid.poke(false.B)

        var guard = 0
        while (!dut.io.event_ready.peek().litToBoolean && guard < 10000) {
          dut.clock.step()
          guard += 1
        }
        dut.io.event_ready.expect(true.B)

        dut.io.event_valid.poke(true.B)
        dut.io.event_time.poke(t.U)
        dut.io.event_x.poke(x.S)
        dut.io.event_y.poke(y.S)
        dut.io.polarity.poke(true.B)

        dut.clock.step()

        dut.io.event_valid.poke(false.B)
        dut.clock.step()
      }

      val dt = win / wBins // 80
      val xC = 0
      val yC = 0

      // Put exactly one event into each bin.
      // The first event initializes timing/bin state and is also counted in bin0.
      sendEvent(1,           xC, yC) // bin0
      sendEvent(1 + 1 * dt,  xC, yC) // bin1
      sendEvent(1 + 2 * dt,  xC, yC) // bin2
      sendEvent(1 + 3 * dt,  xC, yC) // bin3
      sendEvent(1 + 4 * dt,  xC, yC) // bin4

      // Trigger a readout (boundary crossing after we have rBins bins)
      sendEvent(1 + 5 * dt,  xC, yC)

      // Wait for readout_valid
      var waitCycles = 0
      while (!dut.io.readout_valid.peek().litToBoolean && waitCycles < 20000) {
        dut.clock.step()
        waitCycles += 1
      }
      dut.io.readout_valid.expect(true.B)

      val featsPerWindow = rBins * grid * grid // 1280

      var ones = 0
      var sawStart = false
      var sawLast  = false

      for (i <- 0 until featsPerWindow) {
        dut.io.readout_valid.expect(true.B)

        if (i == 0) {
          dut.io.readout_start.expect(true.B)
          sawStart = true
        } else {
          dut.io.readout_start.expect(false.B)
        }

        if (i == featsPerWindow - 1) {
          dut.io.readout_last.expect(true.B)
          sawLast = true
        } else {
          dut.io.readout_last.expect(false.B)
        }

        val v = dut.io.readout_data(0).peek().litValue.toInt
        if (v == 1) ones += 1
        // also sanity: we should never see >1 in this scenario
        v must (be (0) or be (1))

        dut.clock.step()
      }

      // Since we placed one event in each of the rBins bins, total ones should be rBins
      ones mustBe rBins
      sawStart mustBe true
      sawLast mustBe true
    }
  }
}
