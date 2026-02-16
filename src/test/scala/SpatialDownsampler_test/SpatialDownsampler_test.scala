package SpatialDownsampler

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.must.Matchers

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
}
