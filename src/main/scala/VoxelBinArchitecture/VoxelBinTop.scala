package VoxelBinArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class VoxelBinTop(
  val CLK_FREQ: Int = 12000000,
  val BAUD_RATE: Int = 115200,
  val WINDOW_MS: Int = 400,
  val GRID_SIZE: Int = 16,
  val MIN_EVENT_THRESH: Int = 20,
  val MOTION_THRESH: Int = 8,
  val PERSISTENCE_COUNT: Int = 1,
  val CYCLES_PER_BIN: Int = 600,
  val CORE_PARALLEL_READS: Int = 2
) extends Module {

  val CLKS_PER_BIT = CLK_FREQ / BAUD_RATE

  val io = IO(new Bundle {
    val uart_rx = Input(Bool())
    val uart_tx = Output(Bool())

    val led_heartbeat = Output(Bool())
    val led_gesture_valid = Output(Bool())
    val led_activity = Output(Bool())
    val led_up = Output(Bool())
    val led_down = Output(Bool())
    val led_left = Output(Bool())
    val led_right = Output(Bool())
  })

  // Power-on reset (5 cycles)
  val porCnt = RegInit(0.U(5.W))
  when(porCnt =/= "b11111".U) {
    porCnt := porCnt + 1.U
  }

  val rstPor = porCnt =/= "b11111".U
  val softRst = RegInit(false.B)
  val rst = rstPor || softRst

  // UART
  val uartRx = Module(new UartRx(CLKS_PER_BIT))
  val uartTx = Module(new UartTx(CLKS_PER_BIT))

  uartRx.io.rx := io.uart_rx
  uartRx.io.rst := rst

  uartTx.io.rst := rst
  io.uart_tx := uartTx.io.tx

  // Core
  val core = Module(new VoxelBinCore(
    CLK_FREQ_HZ = CLK_FREQ,
    WINDOW_MS = WINDOW_MS,
    GRID_SIZE = GRID_SIZE,
    PERSISTENCE_COUNT = PERSISTENCE_COUNT,
    CYCLES_PER_BIN = CYCLES_PER_BIN,
    PARALLEL_READS = CORE_PARALLEL_READS
  ))

  core.io.rst := rst

  // EVT2 Word Packetizer
  val states = Enum(4)
  val PKT_B0 = states(0)
  val PKT_B1 = states(1)
  val PKT_B2 = states(2)
  val PKT_B3 = states(2)
  val state = RegInit(PKT_B0)

  val wordShift = Reg(UInt(32.W))
  val uartEvtWord = Reg(UInt(32.W))
  val uartEvtPending = RegInit(false.B)

  core.io.evt_word := uartEvtWord
  core.io.evt_word_valid := uartEvtPending

  when(uartEvtPending && core.io.evt_word_ready) {
    uartEvtPending := false.B
  }

  // RX State Machine
  softRst := false.B

  when(uartRx.io.valid) {
    switch(pktState) {

      is(PKT_B0) {
        switch(uartRx.io.data) {
          is("hFF".U) { /* echo handled in TX FSM */ }
          is("hFE".U) { /* status */ }
          is("hFD".U) { /* config */ }
          is("hFC".U) { softRst := true.B }
          otherwise {
            wordShift := Cat(uartRx.io.data, 0.U(24.W))
            pktState := PKT_B1
          }
        }
      }

      is(PKT_B1) {
        wordShift := Cat(wordShift(31,24), uartRx.io.data, 0.U(16.W))
        pktState := PKT_B2
      }

      is(PKT_B2) {
        wordShift := Cat(wordShift(31,16), uartRx.io.data, 0.U(8.W))
        pktState := PKT_B3
      }

      is(PKT_B3) {
        when(!uartEvtPending) {
          uartEvtWord := Cat(wordShift(31,8), uartRx.io.data)
          uartEvtPending := true.B
        }
        pktState := PKT_B0
      }
    }
  }

  // TX State Machine
  val states_2 = Enum(4)
  val TX_IDLE    = states_2(0)
  val TX_G_CMD   = states_2(1)
  val TX_G_WAIT  = states_2(2)
  val TX_G_CONF  = states_2(2)
  val txState = RegInit(TX_IDLE)

  val pendingGesture = Reg(UInt(2.W))
  val pendingConfidence = Reg(UInt(4.W))

  uartTx.io.valid := false.B
  uartTx.io.data := 0.U

  when(core.io.gesture_valid && txState === TX_IDLE) {
    pendingGesture := core.io.gesture
    pendingConfidence := core.io.gesture_confidence
    txState := TX_G_CMD
  }

  switch(txState) {

    is(TX_IDLE)

    is(TX_G_CMD) {
      when(!uartTx.io.busy) {
        uartTx.io.data := Cat("hA".U(4.W), 0.U(2.W), pendingGesture)
        uartTx.io.valid := true.B
        txState := TX_G_WAIT
      }
    }

    is(TX_G_WAIT) {
      when(uartTx.io.busy) {
        txState := TX_G_CONF
      }
    }

    is(TX_G_CONF) {
      when(!uartTx.io.busy) {
        uartTx.io.data := Cat(pendingConfidence, core.io.debug_event_count(7,4))
        uartTx.io.valid := true.B
        txState := TX_IDLE
      }
    }
  }

  // LEDs
  val heartbeatCnt = RegInit(0.U(24.W))
  heartbeatCnt := Mux(rst, 0.U, heartbeatCnt + 1.U)
  io.led_heartbeat := !heartbeatCnt(22)

  val gestureLedCnt = RegInit(0.U(20.W))
  when(rst) {
    gestureLedCnt := 0.U
  }.elsewhen(core.io.gesture_valid) {
    gestureLedCnt := Fill(20, 1.U(1.W))
  }.elsewhen(gestureLedCnt =/= 0.U) {
    gestureLedCnt := gestureLedCnt - 1.U
  }
  io.led_gesture_valid := !(gestureLedCnt =/= 0.U)

  val dirLedCnt = RegInit(0.U(20.W))
  val lastGesture = Reg(UInt(2.W))
  val lastGestureValid = RegInit(false.B)

  when(rst) {
    dirLedCnt := 0.U
    lastGestureValid := false.B
  }.elsewhen(core.io.gesture_valid) {
    dirLedCnt := Fill(20, 1.U(1.W))
    lastGesture := core.io.gesture
    lastGestureValid := true.B
  }.elsewhen(dirLedCnt =/= 0.U) {
    dirLedCnt := dirLedCnt - 1.U
  }.otherwise {
    lastGestureValid := false.B
  }

  io.led_up    := dirLedCnt =/= 0.U && lastGestureValid && lastGesture === 0.U
  io.led_down  := dirLedCnt =/= 0.U && lastGestureValid && lastGesture === 1.U
  io.led_left  := dirLedCnt =/= 0.U && lastGestureValid && lastGesture === 2.U
  io.led_right := dirLedCnt =/= 0.U && lastGestureValid && lastGesture === 3.U

  val activityCnt = RegInit(0.U(18.W))
  when(rst) {
    activityCnt := 0.U
  }.elsewhen(uartEvtPending && core.io.evt_word_ready) {
    activityCnt := Fill(18, 1.U(1.W))
  }.elsewhen(activityCnt =/= 0.U) {
    activityCnt := activityCnt - 1.U
  }

  io.led_activity := !(activityCnt =/= 0.U)
}

object VoxelBinTop extends App {
  ChiselStage.emitSystemVerilogFile(
    new VoxelBinTop,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
