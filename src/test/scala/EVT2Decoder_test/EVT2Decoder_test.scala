package GradientMapArchitecture

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

  "EVT2Decoder should handle CD event before any EVT_TIME_HIGH (timeHigh=0 after reset)" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val tsLow = 0x3F   // max 6-bit
      val x     = 1
      val y     = 2

      val cdOffWord =
        (0L << 28) |
        (tsLow.toLong << 22) |
        (x.toLong << 11) |
        y.toLong

      dut.io.inWord.poke(cdOffWord.U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      dut.io.outValid.expect(true.B)
      dut.io.isCD.expect(true.B)

      // Since no TIME_HIGH happened, timeHigh should still be 0 (from reset)
      val expectedTs = tsLow // (0 << 6) | tsLow
      dut.io.timestamp.expect(expectedTs.U)
    }
  }

  "EVT2Decoder should decode CD_ON with max field values (boundary test)" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val timeHighMax = (1L << 28) - 1 // 0x0FFFFFFF
      dut.io.inWord.poke(((8L << 28) | timeHighMax).U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      val tsLowMax = 0x3F
      val xMax     = 0x7FF
      val yMax     = 0x7FF

      val cdOnWord =
        (1L << 28) |
        (tsLowMax.toLong << 22) |
        (xMax.toLong << 11) |
        yMax.toLong

      dut.io.inWord.poke(cdOnWord.U)
      dut.clock.step()

      dut.io.outValid.expect(true.B)
      dut.io.isCD.expect(true.B)
      dut.io.polarity.expect(true.B)
      dut.io.x.expect(xMax.U)
      dut.io.y.expect(yMax.U)

      val expectedTs = (timeHighMax << 6) | tsLowMax
      dut.io.timestamp.expect(expectedTs.U)
    }
  }

  "EVT2Decoder should ignore unknown event types (outValid must stay low)" in {
    simulate(new EVT2Decoder) { dut =>
    
      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val unknownType = 0x2L // 0010 (not handled)
      val word = (unknownType << 28) | 0x00ABCDEF

      dut.io.inWord.poke(word.U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      dut.io.outValid.expect(false.B)
      dut.io.isCD.expect(false.B)
      dut.io.isTrigger.expect(false.B)
      dut.io.isContinued.expect(false.B)
    }
  }

  "EVT2Decoder should NOT update state when inValid is low" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      // Try to send TIME_HIGH but with inValid=0
      val timeHighValue = 0x1111111L
      dut.io.inWord.poke(((8L << 28) | timeHighValue).U)
      dut.io.inValid.poke(false.B)
      dut.clock.step()

      // Now send a CD_OFF with inValid=1.
      // If TIME_HIGH incorrectly latched while inValid=0, timestamp would be wrong.
      val tsLow = 0x01
      val x = 3
      val y = 4
      val cdOffWord = (0L << 28) | (tsLow.toLong << 22) | (x.toLong << 11) | y.toLong

      dut.io.inWord.poke(cdOffWord.U)
      dut.io.inValid.poke(true.B)
      dut.clock.step()

      dut.io.outValid.expect(true.B)
      val expectedTs = tsLow // timeHigh should still be 0
      dut.io.timestamp.expect(expectedTs.U)
    }
  }

  "EVT2Decoder should overwrite continuedData with latest CONTINUED payload" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      val p1 = 0x0001234L
      val p2 = 0x00ABCDEFL

      dut.io.inValid.poke(true.B)

      dut.io.inWord.poke(((0xFL << 28) | p1).U)
      dut.clock.step()
      dut.io.continuedData.expect(p1.U)

      dut.io.inWord.poke(((0xFL << 28) | p2).U)
      dut.clock.step()
      dut.io.continuedData.expect(p2.U)
    }
  }

  "EVT2Decoder should use newest timeHigh for timestamps after update" in {
    simulate(new EVT2Decoder) { dut =>

      dut.reset.poke(true.B)
      dut.clock.step()
      dut.reset.poke(false.B)

      dut.io.inValid.poke(true.B)

      // timeHigh = 0x10
      dut.io.inWord.poke(((8L << 28) | 0x10L).U)
      dut.clock.step()

      // CD_OFF with tsLow=1 => timestamp = (0x10 << 6) | 1
      val cd1 = (0L << 28) | (1L << 22) | (1L << 11) | 1L
      dut.io.inWord.poke(cd1.U)
      dut.clock.step()
      dut.io.timestamp.expect(((0x10L << 6) | 1L).U)

      // Update timeHigh = 0x20
      dut.io.inWord.poke(((8L << 28) | 0x20L).U)
      dut.clock.step()
      dut.io.outValid.expect(false.B)

      // CD_OFF again tsLow=1 => timestamp should now use 0x20
      dut.io.inWord.poke(cd1.U)
      dut.clock.step()
      dut.io.timestamp.expect(((0x20L << 6) | 1L).U)
    }
  }

  "EVT2Decoder should not assert outValid when inValid is false between events" in {
    simulate(new EVT2Decoder) { dut =>
      dut.reset.poke(true.B); dut.clock.step(); dut.reset.poke(false.B)

      // Cycle 1: no valid
      dut.io.inValid.poke(false.B)
      dut.io.inWord.poke(0.U)
      dut.clock.step()
      dut.io.outValid.expect(false.B)

      // Cycle 2: CD_ON
      dut.io.inValid.poke(true.B)
      val cdOn = (1L << 28) | (0x1L << 22) | (2L << 11) | 3L
      dut.io.inWord.poke(cdOn.U)
      dut.clock.step()
      dut.io.outValid.expect(true.B)

      // Cycle 3: back to no valid
      dut.io.inValid.poke(false.B)
      dut.clock.step()
      dut.io.outValid.expect(false.B)
    }
  }
}
