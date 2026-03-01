package VoxelBinner

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.must.Matchers

class VoxelBinnerSpec
    extends AnyFreeSpec
    with Matchers
    with ChiselSim {

  "VoxelBinner should increment correct pixel" in {
    simulate(new VoxelBinner(numBins = 2)) { dut =>

      // Reset
      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val x = 3
      val y = 5
      val idx = y * 16 + x

      // Send one event
      dut.io.inValid.poke(true.B)
      dut.io.x.poke(x.U)
      dut.io.y.poke(y.U)
      dut.io.polarity.poke(true.B)
      dut.io.timestamp.poke(0.U)

      dut.clock.step()

      dut.io.inValid.poke(false.B)

      // Read back from bin 0
      dut.io.readBin.poke(0.U)
      dut.io.readAddr.poke(idx.U)

      dut.clock.step()

      dut.io.readData.expect(1.U)
    }
  }

  "VoxelBinner should saturate at 9-bit max (511)" in {
    simulate(new VoxelBinner(numBins = 1)) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val x = 0
      val y = 0
      val idx = 0

      // Write 600 events (should saturate at 511)
      for (_ <- 0 until 600) {
        dut.io.inValid.poke(true.B)
        dut.io.x.poke(x.U)
        dut.io.y.poke(y.U)
        dut.io.polarity.poke(true.B)
        dut.io.timestamp.poke(0.U)
        dut.clock.step()
      }

      dut.io.readBin.poke(0.U)
      dut.io.readAddr.poke(idx.U)
      dut.clock.step()

      dut.io.readData.expect(511.U)
    }
  }

  "VoxelBinner should switch bins and clear new active bin" in {
    simulate(new VoxelBinner(numBins = 2)) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val x = 1
      val y = 1
      val idx = y * 16 + x

      // Write one event into bin 0
      dut.io.inValid.poke(true.B)
      dut.io.x.poke(x.U)
      dut.io.y.poke(y.U)
      dut.io.polarity.poke(true.B)
      dut.io.timestamp.poke(0.U)
      dut.clock.step()

      // Advance bin
      dut.io.inValid.poke(false.B)
      dut.io.advanceBin.poke(true.B)
      dut.clock.step()
      dut.io.advanceBin.poke(false.B)

      // New bin should be cleared
      dut.io.readBin.poke(1.U)
      dut.io.readAddr.poke(idx.U)
      dut.clock.step()

      dut.io.readData.expect(0.U)

      // Old bin should still contain value
      dut.io.readBin.poke(0.U)
      dut.io.readAddr.poke(idx.U)
      dut.clock.step()

      dut.io.readData.expect(1.U)
    }
  }

  "VoxelBinner should accumulate independently per bin" in {
    simulate(new VoxelBinner(numBins = 2)) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val x = 2
      val y = 2
      val idx = y * 16 + x

      // Write 3 events in bin 0
      for (_ <- 0 until 3) {
        dut.io.inValid.poke(true.B)
        dut.io.x.poke(x.U)
        dut.io.y.poke(y.U)
        dut.io.polarity.poke(true.B)
        dut.io.timestamp.poke(0.U)
        dut.clock.step()
      }

      // Advance bin
      dut.io.inValid.poke(false.B)
      dut.io.advanceBin.poke(true.B)
      dut.clock.step()
      dut.io.advanceBin.poke(false.B)

      // Write 2 events in bin 1
      for (_ <- 0 until 2) {
        dut.io.inValid.poke(true.B)
        dut.io.x.poke(x.U)
        dut.io.y.poke(y.U)
        dut.io.polarity.poke(true.B)
        dut.io.timestamp.poke(0.U)
        dut.clock.step()
      }

      // Check bin 0
      dut.io.readBin.poke(0.U)
      dut.io.readAddr.poke(idx.U)
      dut.clock.step()
      dut.io.readData.expect(3.U)

      // Check bin 1
      dut.io.readBin.poke(1.U)
      dut.io.readAddr.poke(idx.U)
      dut.clock.step()
      dut.io.readData.expect(2.U)
    }
  }
}
