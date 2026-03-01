package EVT2Decoder

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class EVT2Decoder extends Module {
  val io = IO(new Bundle {
    val inWord   = Input(UInt(32.W))
    val inValid  = Input(Bool())

    val outValid = Output(Bool())

    val x        = Output(UInt(11.W))
    val y        = Output(UInt(11.W))
    val polarity = Output(Bool())

    val timestamp = Output(UInt(34.W))

    val isCD        = Output(Bool())
    val isTrigger   = Output(Bool())
    val isContinued = Output(Bool())

    val triggerId    = Output(UInt(5.W))
    val triggerValue = Output(Bool())

    val continuedData = Output(UInt(28.W))
  })

  // Registers
  val timeHigh      = RegInit(0.U(28.W))
  val lastWord      = RegInit(0.U(32.W))
  val lastType      = RegInit(0.U(4.W))
  val continuedReg  = RegInit(0.U(28.W))

  val validReg = RegInit(false.B)

  // Default outputs
  io.outValid       := validReg
  io.x              := 0.U
  io.y              := 0.U
  io.polarity       := false.B
  io.timestamp      := 0.U
  io.isCD           := false.B
  io.isTrigger      := false.B
  io.isContinued    := false.B
  io.triggerId      := 0.U
  io.triggerValue   := false.B
  io.continuedData  := continuedReg

  validReg := false.B

  when(io.inValid) {

    val eventType = io.inWord(31,28)

    // Store last word + type
    lastWord := io.inWord
    lastType := eventType

    switch(eventType) {

      // CD_OFF (0000)
      is("b0000".U) {
        val tsLow = io.inWord(27,22)

        io.x := io.inWord(21,11)
        io.y := io.inWord(10,0)
        io.polarity := false.B

        io.timestamp := Cat(timeHigh, tsLow)
        io.isCD := true.B

        validReg := true.B
      }

      // CD_ON (0001)
      is("b0001".U) {
        val tsLow = io.inWord(27,22)

        io.x := io.inWord(21,11)
        io.y := io.inWord(10,0)
        io.polarity := true.B

        io.timestamp := Cat(timeHigh, tsLow)
        io.isCD := true.B

        validReg := true.B
      }

      // EVT_TIME_HIGH (1000)
      is("b1000".U) {
        timeHigh := io.inWord(27,0)
      }

      // EXT_TRIGGER (1010)
      is("b1010".U) {
        val tsLow = io.inWord(27,22)

        io.timestamp := Cat(timeHigh, tsLow)
        io.isTrigger := true.B
        io.triggerId := io.inWord(12,8)
        io.triggerValue := io.inWord(0)

        validReg := true.B
      }

      // CONTINUED (1111)
      is("b1111".U) {

        continuedReg := io.inWord(27,0)

        io.isContinued := true.B

        // CONTINUED extends previous event
        // Vendor specific — we simply expose extension bits
        validReg := true.B
      }
    }
  }
}

object EVT2Decoder extends App {
  ChiselStage.emitSystemVerilogFile(
    new EVT2Decoder,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
  )
}