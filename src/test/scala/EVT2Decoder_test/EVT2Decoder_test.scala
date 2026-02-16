// See README.md for license details.

package EVT2Decoder

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.freespec.AnyFreeSpec
import org.scalatest.matchers.must.Matchers

class EVT2DecoderSpec extends AnyFreeSpec with Matchers with ChiselSim {

  "EVT2Decoder should decode EVT_TIME_HIGH correctly" in {
    simulate(new EVT2Decoder) { dut =>

      // Reset
      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // Send EVT_TIME_HIGH (1000)
      val timeHighValue = 0x1234567L
      val word = (8L << 28) | timeHighValue  // type=1000

      dut.io.inWord.poke(word.U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      dut.io.outValid.expect(false.B)
    }
  }

  "EVT2Decoder should decode CD_OFF correctly" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // First set time high
      val timeHighValue = 0x1ABCDEFL
      dut.io.inWord.poke(((8L << 28) | timeHighValue).U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      // Now send CD_OFF
      val tsLow = 0x15
      val x     = 100
      val y     = 200

      val cdOffWord =
        (0L << 28) |          // CD_OFF
        (tsLow << 22) |
        (x << 11) |
        y

      dut.io.inWord.poke(cdOffWord.U)
      dut.clock.step()

      dut.io.outValid.expect(true.B)
      dut.io.isCD.expect(true.B)
      dut.io.polarity.expect(false.B)
      dut.io.x.expect(x.U)
      dut.io.y.expect(y.U)

      val expectedTimestamp =
        (timeHighValue << 6) | tsLow

      dut.io.timestamp.expect(expectedTimestamp.U)
    }
  }

  "EVT2Decoder should decode CD_ON correctly" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // Set time high
      dut.io.inWord.poke(((8L << 28) | 0x10L).U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      val tsLow = 0x3
      val x     = 5
      val y     = 7

      val cdOnWord =
        (1L << 28) |
        (tsLow << 22) |
        (x << 11) |
        y

      dut.io.inWord.poke(cdOnWord.U)
      dut.clock.step()

      dut.io.outValid.expect(true.B)
      dut.io.isCD.expect(true.B)
      dut.io.polarity.expect(true.B)
      dut.io.x.expect(x.U)
      dut.io.y.expect(y.U)
    }
  }

  "EVT2Decoder should decode EXT_TRIGGER correctly" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // Set time high
      dut.io.inWord.poke(((8L << 28) | 0x20L).U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      val tsLow = 0x2
      val triggerId = 1
      val triggerVal = 1

      val triggerWord =
        (0xAL << 28) |
        (tsLow << 22) |
        (triggerId << 8) |
        triggerVal

      dut.io.inWord.poke(triggerWord.U)
      dut.clock.step()

      dut.io.outValid.expect(true.B)
      dut.io.isTrigger.expect(true.B)
      dut.io.triggerId.expect(triggerId.U)
      dut.io.triggerValue.expect(triggerVal.B)
    }
  }

  "EVT2Decoder should store CONTINUED data" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val continuedPayload = 0x00ABCDEFL

      val continuedWord =
        (0xFL << 28) | continuedPayload

      dut.io.inWord.poke(continuedWord.U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      dut.io.outValid.expect(true.B)
      dut.io.isContinued.expect(true.B)
      dut.io.continuedData.expect(continuedPayload.U)
    }
  }
}
