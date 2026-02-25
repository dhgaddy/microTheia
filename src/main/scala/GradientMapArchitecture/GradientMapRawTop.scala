package GradientMapArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

import UartRx.UartRx
import GradientMapArchitecture.InputFIFO
import GradientMapArchitecture.EVT2Decoder
import GradientMapArchitecture.TimeSurfaceEncoder
import GradientMapArchitecture.SpatioTemporalClassifier
import UartDebug.UartDebug

class GradientMapRawTop(
  val CLK_FREQ_HZ: Int     = 12000000,
  val BAUD_RATE: Int       = 115200,
  val FRAME_PERIOD_MS: Int = 50,
  val DECAY_SHIFT: Int     = 6,
  val MIN_MASS_THRESH: Int = 2000
) extends Module {

  val io = IO(new Bundle {
    val uart_rx       = Input(Bool())
    val uart_tx       = Output(Bool())

    val led_heartbeat = Output(Bool())
    val led_activity  = Output(Bool())
    val led_up        = Output(Bool())
    val led_down      = Output(Bool())
    val led_left      = Output(Bool())
    val led_right     = Output(Bool())
  })

  // Power-On Reset
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

  // UART RX
  val uartRx = Module(new UartRx(
    CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE
  ))

  uartRx.io.rx  := io.uart_rx
  uartRx.io.rst := rst

  val uart_rx_data  = uartRx.io.data
  val uart_rx_valid = uartRx.io.valid

  // 5-byte Packet Assembler
  val EVT_CD_OFF = 0.U(4.W)
  val EVT_CD_ON  = 1.U(4.W)

  val uart_byte_idx = RegInit(0.U(3.W))

  val uart_x_hi = Reg(UInt(8.W))
  val uart_x_lo = Reg(UInt(8.W))
  val uart_y_hi = Reg(UInt(8.W))
  val uart_y_lo = Reg(UInt(8.W))

  val uart_evt_word    = Reg(UInt(32.W))
  val uart_evt_pending = RegInit(false.B)

  val uart_x      = Cat(uart_x_hi(0), uart_x_lo)
  val uart_y      = Cat(uart_y_hi(0), uart_y_lo)
  val uart_ts_lsb = global_timestamp(5,0)

  when (rst) {
    uart_byte_idx    := 0.U
    uart_evt_pending := false.B
  } .elsewhen (uart_rx_valid) {

    switch(uart_byte_idx) {
      is (0.U) { uart_x_hi := uart_rx_data }
      is (1.U) { uart_x_lo := uart_rx_data }
      is (2.U) { uart_y_hi := uart_rx_data }
      is (3.U) { uart_y_lo := uart_rx_data }
      is (4.U) {
        when (!uart_evt_pending) {
          uart_evt_word :=
            Cat(
              Mux(uart_rx_data(0), EVT_CD_ON, EVT_CD_OFF),
              uart_ts_lsb,
              uart_x,
              uart_y
            )
          uart_evt_pending := true.B
        }
      }
    }

    uart_byte_idx :=
      Mux(uart_byte_idx === 4.U, 0.U, uart_byte_idx + 1.U)
  }

  // FIFO
  val fifo = Module(new InputFIFO(
    depth     = 128,
    ptrBits   = 7,
    dataWidth = 32
  ))

  fifo.io.clk := clock
  fifo.io.rst := rst

  val fifo_wr_en = uart_evt_pending && !fifo.io.full

  fifo.io.wr_en   := fifo_wr_en
  fifo.io.wr_data := uart_evt_word

  when (fifo_wr_en) {
    uart_evt_pending := false.B
  }

  fifo.io.rd_en := !fifo.io.empty

  val fifo_rd_valid = RegNext(fifo.io.rd_en && !fifo.io.empty, false.B)
  val fifo_rd_data  = fifo.io.rd_data

  // EVT2 Decoder
  val decoder = Module(new EVT2Decoder())

    decoder.io.inWord  := fifo_rd_data
    decoder.io.inValid := fifo_rd_valid

    val decoded_x        = decoder.io.x
    val decoded_y        = decoder.io.y
    val decoded_polarity = decoder.io.polarity
    val decoded_valid    = decoder.io.outValid

  // Time Surface
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
  timeSurface.io.event_valid := decoded_valid
  timeSurface.io.event_x     := decoded_x
  timeSurface.io.event_y     := decoded_y
  timeSurface.io.event_ts    := global_timestamp
  timeSurface.io.read_enable := ts_read_enable
  timeSurface.io.read_addr   := ts_read_addr

  ts_read_value := timeSurface.io.read_value

  // Classifier
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

  // UART Debug TX
  val uartDebug = Module(new UartDebug(
    CLK_FREQ_HZ = CLK_FREQ_HZ,
    BAUD_RATE   = BAUD_RATE
  ))

  uartDebug.io.gesture_class      := gesture_class
  uartDebug.io.gesture_valid      := gesture_valid
  uartDebug.io.gesture_confidence := gesture_confidence

  io.uart_tx := uartDebug.io.uart_tx

  // Heartbeat LED
  val heartbeat_cnt = RegInit(0.U(23.W))
  when (rst) { heartbeat_cnt := 0.U }
    .otherwise { heartbeat_cnt := heartbeat_cnt + 1.U }

  io.led_heartbeat := heartbeat_cnt(22)

  // Activity LED
  val activity_cnt = RegInit(0.U(20.W))

  when (rst) {
    activity_cnt := 0.U
  } .elsewhen (decoded_valid) {
    activity_cnt := Fill(20, 1.U(1.W))
  } .elsewhen (activity_cnt > 0.U) {
    activity_cnt := activity_cnt - 1.U
  }

  io.led_activity := activity_cnt > 0.U

  // Gesture LEDs
  val LED_STRETCH = 6000000.U(24.W)

  def stretch(counter: UInt, cond: Bool): UInt =
    Mux(cond, LED_STRETCH,
      Mux(counter > 0.U, counter - 1.U, 0.U))

  val led_up_cnt    = RegInit(0.U(24.W))
  val led_down_cnt  = RegInit(0.U(24.W))
  val led_left_cnt  = RegInit(0.U(24.W))
  val led_right_cnt = RegInit(0.U(24.W))

  led_up_cnt    := stretch(led_up_cnt,    gesture_valid && gesture_class === 0.U)
  led_down_cnt  := stretch(led_down_cnt,  gesture_valid && gesture_class === 1.U)
  led_left_cnt  := stretch(led_left_cnt,  gesture_valid && gesture_class === 2.U)
  led_right_cnt := stretch(led_right_cnt, gesture_valid && gesture_class === 3.U)

  io.led_up    := led_up_cnt    > 0.U
  io.led_down  := led_down_cnt  > 0.U
  io.led_left  := led_left_cnt  > 0.U
  io.led_right := led_right_cnt > 0.U
}

object GradientMapRawTop extends App {
  ChiselStage.emitSystemVerilogFile(
    new GradientMapRawTop,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
