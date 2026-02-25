package GradientMapArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

import GradientMapArchitecture.TimeSurfaceEncoder
import GradientMapArchitecture.SpatioTemporalClassifier
import UartDebug.UartDebug

class GradientMapProcessedTop(
  val CLK_FREQ_HZ: Int     = 12000000,
  val BAUD_RATE: Int       = 115200,
  val FRAME_PERIOD_MS: Int = 50,
  val DECAY_SHIFT: Int     = 6,
  val MIN_MASS_THRESH: Int = 2000
) extends Module {

  val io = IO(new Bundle {

    val clk = Input(Clock()) // if using external clock wrapper
    // NOTE: If your project already wraps clock, remove this.

    val event_valid    = Input(Bool())
    val event_x        = Input(UInt(4.W))
    val event_y        = Input(UInt(4.W))
    val event_polarity = Input(Bool())
    val event_ready    = Output(Bool())

    val uart_tx        = Output(Bool())

    val led_heartbeat  = Output(Bool())
    val led_activity   = Output(Bool())
    val led_up         = Output(Bool())
    val led_down       = Output(Bool())
    val led_left       = Output(Bool())
    val led_right      = Output(Bool())
  })

  // Power-On Reset (POR)
  val por_cnt = RegInit(0.U(5.W))
  val rst_n_internal = por_cnt.andR

  when (!rst_n_internal) {
    por_cnt := por_cnt + 1.U
  }

  val rst_sync = RegInit(0.U(4.W))
  rst_sync := Cat(rst_sync(2,0), !rst_n_internal)

  val rst = rst_sync(3)

  // Global Timestamp
  val global_timestamp = RegInit(0.U(16.W))

  when (rst) {
    global_timestamp := 0.U
  } .otherwise {
    global_timestamp := global_timestamp + 1.U
  }

  // Always ready (no FIFO)
  io.event_ready := true.B

  // Time Surface Encoder
  val ts_read_addr   = Wire(UInt(8.W))
  val ts_read_enable = Wire(Bool())
  val ts_read_value  = Wire(UInt(8.W))

  val timeSurface = Module(new TimeSurfaceEncoder(
    GRID_SIZE   = 16,
    ADDR_BITS   = 8,
    TS_BITS     = 16,
    VALUE_BITS  = 8,
    MAX_VALUE   = 255,
    DECAY_SHIFT = DECAY_SHIFT
  ))

  timeSurface.io.t_now       := global_timestamp
  timeSurface.io.event_valid := io.event_valid
  timeSurface.io.event_x     := io.event_x
  timeSurface.io.event_y     := io.event_y
  timeSurface.io.event_ts    := global_timestamp
  timeSurface.io.read_enable := ts_read_enable
  timeSurface.io.read_addr   := ts_read_addr

  ts_read_value := timeSurface.io.read_value

  // Spatio-Temporal Classifier
  val classifier = Module(new SpatioTemporalClassifier(
    CLK_FREQ_HZ     = CLK_FREQ_HZ,
    FRAME_PERIOD_MS = FRAME_PERIOD_MS,
    GRID_SIZE       = 16,
    ADDR_BITS       = 8,
    VALUE_BITS      = 8,
    MOMENT_BITS     = 24,
    WEIGHT_BITS     = 8,
    SCORE_BITS      = 24,
    NUM_CLASSES     = 4,
    MIN_MASS_THRESH = MIN_MASS_THRESH
  ))

  classifier.io.ts_read_addr   <> ts_read_addr
  classifier.io.ts_read_enable <> ts_read_enable
  classifier.io.ts_read_value  := ts_read_value

  val gesture_class      = classifier.io.gesture_class
  val gesture_valid      = classifier.io.gesture_valid
  val gesture_confidence = classifier.io.gesture_confidence

  // UART Debug
  val uart = Module(new UartDebug(
    CLK_FREQ_HZ = CLK_FREQ_HZ,
    BAUD_RATE   = BAUD_RATE
  ))

  uart.io.gesture_class      := gesture_class
  uart.io.gesture_valid      := gesture_valid
  uart.io.gesture_confidence := gesture_confidence

  io.uart_tx := uart.io.uart_tx

  // Heartbeat LED (~1.5Hz at 12MHz)
  val heartbeat_cnt = RegInit(0.U(23.W))

  when (rst) {
    heartbeat_cnt := 0.U
  } .otherwise {
    heartbeat_cnt := heartbeat_cnt + 1.U
  }

  io.led_heartbeat := heartbeat_cnt(22)

  // Activity LED
  val activity_cnt = RegInit(0.U(20.W))

  when (rst) {
    activity_cnt := 0.U
  } .elsewhen (io.event_valid) {
    activity_cnt := Fill(20, 1.U(1.W))
  } .elsewhen (activity_cnt > 0.U) {
    activity_cnt := activity_cnt - 1.U
  }

  io.led_activity := activity_cnt > 0.U

  // Gesture LEDs (~500ms stretch)
  val LED_STRETCH = 6000000.U(24.W)

  val led_up_cnt    = RegInit(0.U(24.W))
  val led_down_cnt  = RegInit(0.U(24.W))
  val led_left_cnt  = RegInit(0.U(24.W))
  val led_right_cnt = RegInit(0.U(24.W))

  when (rst) {
    led_up_cnt    := 0.U
    led_down_cnt  := 0.U
    led_left_cnt  := 0.U
    led_right_cnt := 0.U
  } .otherwise {

    def stretch(counter: UInt, condition: Bool): UInt =
      Mux(condition, LED_STRETCH,
        Mux(counter > 0.U, counter - 1.U, 0.U))

    led_up_cnt    := stretch(led_up_cnt,    gesture_valid && gesture_class === 0.U)
    led_down_cnt  := stretch(led_down_cnt,  gesture_valid && gesture_class === 1.U)
    led_left_cnt  := stretch(led_left_cnt,  gesture_valid && gesture_class === 2.U)
    led_right_cnt := stretch(led_right_cnt, gesture_valid && gesture_class === 3.U)
  }

  io.led_up    := led_up_cnt    > 0.U
  io.led_down  := led_down_cnt  > 0.U
  io.led_left  := led_left_cnt  > 0.U
  io.led_right := led_right_cnt > 0.U
}

object GradientMapProcessedTop extends App {
  ChiselStage.emitSystemVerilogFile(
    new GradientMapProcessedTop,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
