package VoxelBinArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class GestureClassifier(
  val ACC_SUM_BITS: Int = 18,
  val PERSISTENCE_COUNT: Int = 2
) extends Module {

  val io = IO(new Bundle {
    val class_gesture   = Input(UInt(2.W))
    val class_valid     = Input(Bool())
    val class_pass      = Input(Bool())
    val abs_delta_x     = Input(UInt(ACC_SUM_BITS.W))
    val abs_delta_y     = Input(UInt(ACC_SUM_BITS.W))

    val gesture         = Output(UInt(2.W))
    val gesture_valid   = Output(Bool())
    val gesture_confidence = Output(UInt(4.W))
    val debug_state     = Output(UInt(3.W))
  })

  // FSM States
  val states = Enum(3)
  val ST_IDLE = states(0)
  val ST_TRACKING = states(1)
  val ST_CONFIRMED = states(2)
  val state = RegInit(ST_IDLE)
  io.debug_state := state

  // Registers
  val lastGesture  = RegInit(0.U(2.W))
  val matchCount   = RegInit(0.U(3.W))
  val gestureReg   = RegInit(0.U(2.W))
  val validReg     = RegInit(false.B)
  val confidenceReg= RegInit(0.U(4.W))

  val latchedAbsX  = RegInit(0.U(ACC_SUM_BITS.W))
  val latchedAbsY  = RegInit(0.U(ACC_SUM_BITS.W))

  io.gesture := gestureReg
  io.gesture_valid := validReg
  io.gesture_confidence := confidenceReg

  // Default: valid is pulse
  validReg := false.B

  // Main Logic
  when (io.class_valid) {

    when (io.class_pass) {

      when (io.class_gesture === lastGesture) {

        // Increment persistence counter
        when (matchCount < PERSISTENCE_COUNT.U) {
          matchCount := matchCount + 1.U
        }

        // Confirm gesture
        when (matchCount >= (PERSISTENCE_COUNT - 1).U) {

          state       := ST_CONFIRMED
          gestureReg  := io.class_gesture
          validReg    := true.B
          matchCount  := 0.U

          // Confidence = min(15, dominant >> 4)
          val dominant = Mux(io.abs_delta_x > io.abs_delta_y,
                             io.abs_delta_x,
                             io.abs_delta_y)

          confidenceReg := Mux(dominant > 255.U,
                               15.U,
                               dominant(7,4))

        }.otherwise {
          state := ST_TRACKING
        }

      }.otherwise {
        // New gesture detected
        lastGesture := io.class_gesture
        matchCount  := 0.U
        state       := ST_TRACKING
      }

      // Latch magnitudes (matches SV behavior)
      latchedAbsX := io.abs_delta_x
      latchedAbsY := io.abs_delta_y

    }.otherwise {
      // class_pass == 0
      matchCount := 0.U
      state      := ST_IDLE
    }
  }
}

object GestureClassifier extends App {
  ChiselStage.emitSystemVerilogFile(
    new GestureClassifier,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
