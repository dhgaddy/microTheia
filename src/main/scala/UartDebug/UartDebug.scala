package UartDebug

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

import UartTx.UartTx

class UartDebug(
  val CLK_FREQ_HZ: Int = 12000000,
  val BAUD_RATE: Int   = 115200
) extends Module {

  val io = IO(new Bundle {
    val gesture_class      = Input(UInt(2.W))  // 0=UP,1=DOWN,2=LEFT,3=RIGHT
    val gesture_valid      = Input(Bool())
    val gesture_confidence = Input(UInt(8.W))  // (unused, matches SV)
    val uart_tx            = Output(Bool())
  })

  val CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE

  // Message ROMs
  val msg_up = VecInit(Seq(
    'U'.U(8.W),
    'P'.U(8.W),
    0x0D.U(8.W),
    0x0A.U(8.W)
  ))

  val msg_down = VecInit(Seq(
    'D'.U(8.W),
    'O'.U(8.W),
    'W'.U(8.W),
    'N'.U(8.W),
    0x0D.U(8.W),
    0x0A.U(8.W)
  ))

  val msg_left = VecInit(Seq(
    'L'.U(8.W),
    'E'.U(8.W),
    'F'.U(8.W),
    'T'.U(8.W),
    0x0D.U(8.W),
    0x0A.U(8.W)
  ))

  val msg_right = VecInit(Seq(
    'R'.U(8.W),
    'I'.U(8.W),
    'G'.U(8.W),
    'H'.U(8.W),
    'T'.U(8.W),
    0x0D.U(8.W),
    0x0A.U(8.W)
  ))

  // Message lengths
  val MSG_UP_LEN    = 4.U(3.W)
  val MSG_DOWN_LEN  = 6.U(3.W)
  val MSG_LEFT_LEN  = 6.U(3.W)
  val MSG_RIGHT_LEN = 7.U(3.W)

  // UART TX instance
  val uartTx = Module(new UartTx(CLKS_PER_BIT))

  uartTx.io.data  := 0.U
  uartTx.io.valid := false.B

  io.uart_tx := uartTx.io.tx

  // FSM
  val states = Enum(4)
  val S_IDLE = states(0)
  val S_LOAD = states(1)
  val S_SEND = states(2)
  val S_NEXT = states(3)

  val state = RegInit(S_IDLE)

  val current_gesture = Reg(UInt(2.W))
  val byte_idx        = RegInit(0.U(3.W))
  val msg_length      = Reg(UInt(3.W))
  val current_byte    = Reg(UInt(8.W))

  // Drive defaults via Wires to avoid combinational cycle
  val tx_data  = WireDefault(0.U(8.W))
  val tx_valid = WireDefault(false.B)

  uartTx.io.data  := tx_data
  uartTx.io.valid := tx_valid

  io.uart_tx := uartTx.io.tx

  switch(state) {

    is (S_IDLE) {
      when (io.gesture_valid && !uartTx.io.busy) {
        current_gesture := io.gesture_class
        byte_idx        := 0.U

        msg_length := MuxLookup(io.gesture_class, MSG_UP_LEN)(Seq(
          0.U -> MSG_UP_LEN,
          1.U -> MSG_DOWN_LEN,
          2.U -> MSG_LEFT_LEN,
          3.U -> MSG_RIGHT_LEN
        ))

        state := S_LOAD
      }
    }

    is (S_LOAD) {
      current_byte := MuxLookup(current_gesture, msg_up(byte_idx(1,0)))(Seq(
        0.U -> msg_up(byte_idx(1,0)),
        1.U -> msg_down(byte_idx),
        2.U -> msg_left(byte_idx),
        3.U -> msg_right(byte_idx)
      ))

      state := S_SEND
    }

    is (S_SEND) {
      // UartTx is never busy here (we only arrive from S_LOAD,
      // and we waited for !busy before leaving S_IDLE)
      tx_data  := current_byte
      tx_valid := true.B
      state    := S_NEXT
    }

    is (S_NEXT) {
      when (uartTx.io.busy) {
        when (byte_idx >= (msg_length - 1.U)) {
          state := S_IDLE
        } .otherwise {
          byte_idx := byte_idx + 1.U
          state    := S_LOAD
        }
      }
    }
  }

}

object UartDebug extends App {
  ChiselStage.emitSystemVerilogFile(
    new UartDebug(),
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
