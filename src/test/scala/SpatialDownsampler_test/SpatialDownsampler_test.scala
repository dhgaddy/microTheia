package SpatialDownsampler

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.must.Matchers
import scala.util.Random

class SpatialDownsamplerSpec
    extends AnyFreeSpec
    with Matchers
    with ChiselSim {

  "SpatialDownsampler should correctly downscale 320x320 to 16x16" in {
    simulate(new SpatialDownsampler) { dut =>

      // Reset
      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // Test cases
      val testCases = Seq(
        (0, 0),
        (19, 19),     // still (0,0)
        (20, 20),     // becomes (1,1)
        (39, 39),     // (1,1)
        (319, 319),   // (15,15)
        (100, 250)
      )

      for ((x, y) <- testCases) {

        val expectedX = x / 20
        val expectedY = y / 20

        val polarity = true
        val timestamp = BigInt(12345)

        // Apply inputs
        dut.io.inValid.poke(true.B)
        dut.io.inX.poke(x.U)
        dut.io.inY.poke(y.U)
        dut.io.polarity.poke(polarity.B)
        dut.io.timestamp.poke(timestamp.U)

        dut.clock.step()   // pipeline stage

        // Now outputs should be valid
        dut.io.outValid.expect(true.B)
        dut.io.outX.expect(expectedX.U)
        dut.io.outY.expect(expectedY.U)
        dut.io.outPolarity.expect(polarity.B)
        dut.io.outTimestamp.expect(timestamp.U)
      }
    }
  }

  "SpatialDownsampler should delay valid by 1 cycle" in {
    simulate(new SpatialDownsampler) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      dut.io.inValid.poke(true.B)
      dut.io.inX.poke(40.U)
      dut.io.inY.poke(60.U)
      dut.io.polarity.poke(false.B)
      dut.io.timestamp.poke(999.U)

      // First cycle: output not yet valid
      dut.io.outValid.expect(false.B)

      dut.clock.step()

      // Second cycle: valid appears
      dut.io.outValid.expect(true.B)
    }
  }

  "SpatialDownsampler should map full range correctly" in {
    simulate(new SpatialDownsampler) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      for (x <- 0 until 320 by 37; y <- 0 until 320 by 53) {

        val expectedX = x / 20 // Will probably change it to not use division later, instead mul and shift
        val expectedY = y / 20 // Will probably change it to not use division later, instead mul and shift

        dut.io.inValid.poke(true.B)
        dut.io.inX.poke(x.U)
        dut.io.inY.poke(y.U)
        dut.io.polarity.poke(false.B)
        dut.io.timestamp.poke(1.U)

        dut.clock.step()

        dut.io.outValid.expect(true.B)
        dut.io.outX.expect(expectedX.U)
        dut.io.outY.expect(expectedY.U)
      }
    }
  }

  "SpatialDownsampler will wrap around with numbers higher than 319" in {
    simulate(new SpatialDownsampler) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      dut.io.inValid.poke(true.B)
      dut.io.inX.poke(400.U)
      dut.io.inY.poke(600.U)
      dut.io.polarity.poke(false.B)
      dut.io.timestamp.poke(999.U)

      dut.clock.step()

      dut.io.outValid.expect(true.B)
      dut.io.outX.expect(4.U) // Expected is 20 but this only takes the lowest 4 bits so in reality it's 4
      dut.io.outY.expect(14.U) // Expected is 30 but this only takes the lowest 4 bits so in reality it's 14
      dut.io.outPolarity.expect(false.B)
      dut.io.outTimestamp.expect(999.U)
    }
  }

  "SpatialDownsampler: Testing the boundaries of dividing by SCALE" in {
    simulate(new SpatialDownsampler) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // Test cases
      val testCases = Seq(
        (19, 19), // Gets (0,0)
        (20, 20), // becomes (1,1)
        (39, 39), // (1, 1)
        (40, 40), // (2, 2)
        (59, 59), // (2, 2)
        (60, 60), // (3, 3)
        (79, 79), // (3, 3)
        (80, 80), // (4, 4)
        (299, 299), // (14, 14)
        (300, 300), // (15, 15)
        (319, 319) // (15,15)
      )

      for ((x, y) <- testCases) {

        val expectedX = x / 20
        val expectedY = y / 20

        val polarity = true
        val timestamp = BigInt(12345)

        // Apply inputs
        dut.io.inValid.poke(true.B)
        dut.io.inX.poke(x.U)
        dut.io.inY.poke(y.U)
        dut.io.polarity.poke(polarity.B)
        dut.io.timestamp.poke(timestamp.U)

        dut.clock.step()   // pipeline stage

        // Now outputs should be valid
        dut.io.outValid.expect(true.B)
        dut.io.outX.expect(expectedX.U)
        dut.io.outY.expect(expectedY.U)
        dut.io.outPolarity.expect(polarity.B)
        dut.io.outTimestamp.expect(timestamp.U)
      }
    }
  }

  "SpatialDownsampler: inValid bubble test" in {
    simulate(new SpatialDownsampler) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // Cycle 0: VALID event A
      dut.io.inValid.poke(true.B)
      dut.io.inX.poke(40.U)        
      dut.io.inY.poke(60.U)        
      dut.io.polarity.poke(false.B)
      dut.io.timestamp.poke(999.U)

      dut.io.outValid.expect(false.B)

      dut.clock.step()

      // Cycle 1 output corresponds to Cycle 0 input (event A)
      dut.io.outValid.expect(true.B)
      dut.io.outX.expect(2.U)
      dut.io.outY.expect(3.U)
      dut.io.outPolarity.expect(false.B)
      dut.io.outTimestamp.expect(999.U)

      // Cycle 1: BUBBLE (inValid=0)
      dut.io.inValid.poke(false.B)
      // (data can be anything during bubble, but set to 0 to keep it clean)
      dut.io.inX.poke(0.U)
      dut.io.inY.poke(0.U)
      dut.io.polarity.poke(false.B)
      dut.io.timestamp.poke(0.U)

      dut.clock.step()

      // Cycle 2 output corresponds to Cycle 1 bubble
      dut.io.outValid.expect(false.B)

      // Cycle 2: VALID event B (different values)
      dut.io.inValid.poke(true.B)
      dut.io.inX.poke(100.U)
      dut.io.inY.poke(250.U)
      dut.io.polarity.poke(true.B)
      dut.io.timestamp.poke(123.U)

      dut.clock.step()

      // Cycle 3 output corresponds to Cycle 2 input (event B)
      dut.io.outValid.expect(true.B)
      dut.io.outX.expect(5.U)
      dut.io.outY.expect(12.U)
      dut.io.outPolarity.expect(true.B)
      dut.io.outTimestamp.expect(123.U)
    }
  }

  "SpatialDownsampler should handle back-to-back events correctly" in {
    simulate(new SpatialDownsampler) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // 10 back-to-back events
      // Each cycle: drive a new (x,y,pol,ts) with inValid=1
      val testCases = Seq(
        (  0,   0, false, BigInt(1000)),
        ( 37,  53, true,  BigInt(1001)),
        ( 74, 106, false, BigInt(1002)),
        (111, 159, true,  BigInt(1003)),
        (148, 212, false, BigInt(1004)),
        (185, 265, true,  BigInt(1005)),
        (222, 318, false, BigInt(1006)),
        (259,  17, true,  BigInt(1007)),
        (296,  70, false, BigInt(1008)),
        (319, 319, true,  BigInt(1009))
      )

      // Drive first input (cycle 0)
      val (x0, y0, pol0, ts0) = testCases.head
      dut.io.inValid.poke(true.B)
      dut.io.inX.poke(x0.U)
      dut.io.inY.poke(y0.U)
      dut.io.polarity.poke(pol0.B)
      dut.io.timestamp.poke(ts0.U)

      // First cycle: output not yet valid (1-cycle pipeline)
      dut.io.outValid.expect(false.B)

      dut.clock.step()

      // For each next cycle:
      // - check outputs match the *previous* cycle's input
      // - then apply the next input
      for (i <- 1 until testCases.length) {

        val (prevX, prevY, prevPol, prevTs) = testCases(i - 1)
        val expectedX = prevX / 20
        val expectedY = prevY / 20

        // Now outputs should be valid and aligned to previous input
        dut.io.outValid.expect(true.B)
        dut.io.outX.expect(expectedX.U)
        dut.io.outY.expect(expectedY.U)
        dut.io.outPolarity.expect(prevPol.B)
        dut.io.outTimestamp.expect(prevTs.U)

        // Apply next input for the next cycle
        val (x, y, pol, ts) = testCases(i)
        dut.io.inValid.poke(true.B)
        dut.io.inX.poke(x.U)
        dut.io.inY.poke(y.U)
        dut.io.polarity.poke(pol.B)
        dut.io.timestamp.poke(ts.U)

        dut.clock.step()
      }

      // After the loop, we still need to check the LAST input's output
      val (lastX, lastY, lastPol, lastTs) = testCases.last
      val lastExpectedX = lastX / 20
      val lastExpectedY = lastY / 20

      dut.io.outValid.expect(true.B)
      dut.io.outX.expect(lastExpectedX.U)
      dut.io.outY.expect(lastExpectedY.U)
      dut.io.outPolarity.expect(lastPol.B)
      dut.io.outTimestamp.expect(lastTs.U)
    }
  }

  "SpatialDownsampler should pass randomized property test" in {
    simulate(new SpatialDownsampler) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val rand = new Random(12345) // repeatable

      // (inValid, x, y, polarity, timestamp)
      val testCases = Seq.fill(200) {
        val inValid   = rand.nextBoolean()
        val x         = rand.nextInt(2048)              // 11-bit range
        val y         = rand.nextInt(2048)
        val polarity  = rand.nextBoolean()
        val timestamp = BigInt(rand.nextInt(1 << 20))
        (inValid, x, y, polarity, timestamp)
      }

      for ((inValid, x, y, polarity, timestamp) <- testCases) {

        // What the hardware is doing:
        // scaledX = (inX / 20)(3,0)  -> truncate to 4 bits
        val expectedX = ((x / 20) & 0xF)
        val expectedY = ((y / 20) & 0xF)

        // Apply inputs
        dut.io.inValid.poke(inValid.B)
        dut.io.inX.poke(x.U)
        dut.io.inY.poke(y.U)
        dut.io.polarity.poke(polarity.B)
        dut.io.timestamp.poke(timestamp.U)

        dut.clock.step() // pipeline stage

        // After the step, outputs should reflect the inputs we just applied
        dut.io.outValid.expect(inValid.B)

        if (inValid) {
          dut.io.outX.expect(expectedX.U)
          dut.io.outY.expect(expectedY.U)
          dut.io.outPolarity.expect(polarity.B)
          dut.io.outTimestamp.expect(timestamp.U)
        }
      }
    }
  }
}
