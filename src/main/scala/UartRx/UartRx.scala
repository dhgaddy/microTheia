package UartRx

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class UartRx(
  val CLKS_PER_BIT: Int = 104   // 12MHz / 115200
) extends Module {

  val io = IO(new Bundle {
    val rx    = Input(Bool())
    val rst   = Input(Bool())   // active-high synchronous reset
    val data  = Output(UInt(8.W))
    val valid = Output(Bool())
  })

  // State machine
  val states = Enum(4)
  val IDLE  = states(0)
  val START = states(1)
  val DATA  = states(2)
  val STOP  = states(3)

  val state = RegInit(IDLE)

  // Registers
  val clk_cnt = RegInit(0.U(log2Ceil(CLKS_PER_BIT).W))
  val bit_idx = RegInit(0.U(3.W))
  val rx_data = Reg(UInt(8.W))

  val dataReg  = RegInit(0.U(8.W))
  val validReg = RegInit(false.B)

  io.data  := dataReg
  io.valid := validReg

  // Double-flop synchronizer
  val rx_sync = RegInit(true.B)
  val rx_d    = RegInit(true.B)

  when (io.rst) {
    rx_sync := true.B
    rx_d    := true.B
  } .otherwise {
    rx_sync := io.rx
    rx_d    := rx_sync
  }

  // FSM
  when (io.rst) {
    state    := IDLE
    clk_cnt  := 0.U
    bit_idx  := 0.U
    rx_data  := 0.U
    dataReg  := 0.U
    validReg := false.B
  } .otherwise {

    validReg := false.B   // default: pulse for one cycle only

    switch(state) {

      is (IDLE) {
        clk_cnt := 0.U
        bit_idx := 0.U

        when (rx_d === false.B) {
          state := START
        }
      }

      is (START) {

        // sample in middle of start bit
        when (clk_cnt === ((CLKS_PER_BIT - 1) / 2).U) {
          when (rx_d === false.B) {
            clk_cnt := 0.U
            state   := DATA
          } .otherwise {
            state := IDLE
          }
        } .otherwise {
          clk_cnt := clk_cnt + 1.U
        }
      }

      is (DATA) {

        when (clk_cnt === (CLKS_PER_BIT - 1).U) {
          clk_cnt := 0.U

          rx_data := rx_data.bitSet(bit_idx, rx_d)

          when (bit_idx === 7.U) {
            bit_idx := 0.U
            state   := STOP
          } .otherwise {
            bit_idx := bit_idx + 1.U
          }

        } .otherwise {
          clk_cnt := clk_cnt + 1.U
        }
      }

      is (STOP) {

        when (clk_cnt === (CLKS_PER_BIT - 1).U) {
          clk_cnt := 0.U
          state   := IDLE

          when (rx_d === true.B) {
            dataReg  := rx_data
            validReg := true.B
          }

        } .otherwise {
          clk_cnt := clk_cnt + 1.U
        }
      }
    }
  }
}

object UartRx extends App {
  ChiselStage.emitSystemVerilogFile(
    new UartRx(),
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
