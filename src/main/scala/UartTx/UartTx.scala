package UartTx

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class UartTx(
  val CLKS_PER_BIT: Int = 104  // 12MHz / 115200
) extends Module {

  val io = IO(new Bundle {
    val data  = Input(UInt(8.W))
    val valid = Input(Bool())

    val tx   = Output(Bool())
    val busy = Output(Bool())
  })

  // State Machine
  val states = Enum(4)
  val IDLE  = states(0)
  val START = states(1)
  val DATA  = states(2)
  val STOP  = states(3)

  val state = RegInit(IDLE)

  // Registers
  val clk_cnt = RegInit(0.U(log2Ceil(CLKS_PER_BIT).W))
  val bit_idx = RegInit(0.U(3.W))
  val tx_data = Reg(UInt(8.W))

  // Default outputs
  io.tx   := true.B
  io.busy := false.B

  // FSM
  switch(state) {

    is (IDLE) {
      io.tx   := true.B
      io.busy := false.B
      clk_cnt := 0.U
      bit_idx := 0.U

      when (io.valid) {
        tx_data := io.data
        io.busy := true.B
        state   := START
      }
    }

    is (START) {
      io.tx   := false.B   // Start bit
      io.busy := true.B

      when (clk_cnt === (CLKS_PER_BIT - 1).U) {
        clk_cnt := 0.U
        state   := DATA
      } .otherwise {
        clk_cnt := clk_cnt + 1.U
      }
    }

    is (DATA) {
      io.tx   := tx_data(bit_idx)
      io.busy := true.B

      when (clk_cnt === (CLKS_PER_BIT - 1).U) {
        clk_cnt := 0.U

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
      io.tx   := true.B    // Stop bit
      io.busy := true.B

      when (clk_cnt === (CLKS_PER_BIT - 1).U) {
        clk_cnt := 0.U
        state   := IDLE
        io.busy := false.B
      } .otherwise {
        clk_cnt := clk_cnt + 1.U
      }
    }
  }
}

object UartTx extends App {
  ChiselStage.emitSystemVerilogFile(
    new UartTx(),
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}