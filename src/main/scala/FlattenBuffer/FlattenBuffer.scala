package FlattenBuffer

import chisel3._
import chisel3.util._
import firrtl.options.TargetDirAnnotation
import chisel3.util._
import tool._
import _root_.circt.stage.ChiselStage

class FlattenBuffer(
  val GRID_SIZE: Int  = 16,
  val VALUE_BITS: Int = 8,
  val NUM_CELLS: Int  = 256,
  val PIPE_DEPTH: Int = 2
) extends Module {

  require(NUM_CELLS == GRID_SIZE * GRID_SIZE)

  val CNT_BITS = log2Ceil(NUM_CELLS + PIPE_DEPTH + 2)

  val io = IO(new Bundle {
    val start = Input(Bool())

    val ts_addr = Output(UInt(log2Ceil(NUM_CELLS).W))
    val ts_en   = Output(Bool())
    val ts_val  = Input(UInt(VALUE_BITS.W))

    val flat_valid = Output(Bool())
    val flat_data  = Output(UInt((NUM_CELLS * VALUE_BITS).W))
  })

  // Internal storage for flattened data
  val flat_data_int = Reg(Vec(NUM_CELLS, UInt(VALUE_BITS.W)))

  // FSM
  val states = Enum(4)
  val S_IDLE = states(0)
  val S_FILL = states(1)
  val S_SCAN = states(2)
  val S_DONE = states(3)
  val state = RegInit(S_IDLE)

  val issue_cnt   = RegInit(0.U(CNT_BITS.W))
  val capture_cnt = RegInit(0.U(CNT_BITS.W))

  // Default outputs
  io.ts_addr    := 0.U
  io.ts_en      := false.B
  io.flat_valid := false.B

  // FSM behavior
  switch(state) {

    is (S_IDLE) {
      when (io.start) {
        issue_cnt   := 1.U
        capture_cnt := 0.U
        io.ts_addr  := 0.U
        io.ts_en    := true.B
        state       := S_FILL
      }
    }

    // Fill pipeline (wait for PIPE_DEPTH latency)
    is (S_FILL) {

      when (issue_cnt < NUM_CELLS.U) {
        io.ts_addr := issue_cnt(log2Ceil(NUM_CELLS)-1, 0)
        io.ts_en   := true.B
        issue_cnt  := issue_cnt + 1.U
      } .otherwise {
        io.ts_en := false.B
      }

      capture_cnt := capture_cnt + 1.U

      when (capture_cnt === (PIPE_DEPTH-1).U) {
        state := S_SCAN
        capture_cnt := 0.U
      }
    }

    // Capture incoming values
    is (S_SCAN) {

      flat_data_int(
        capture_cnt(log2Ceil(NUM_CELLS)-1, 0)
      ) := io.ts_val

      capture_cnt := capture_cnt + 1.U

      when (issue_cnt < NUM_CELLS.U) {
        io.ts_addr := issue_cnt(log2Ceil(NUM_CELLS)-1, 0)
        io.ts_en   := true.B
        issue_cnt  := issue_cnt + 1.U
      } .otherwise {
        io.ts_en   := false.B
        io.ts_addr := 0.U
      }

      when (capture_cnt === (NUM_CELLS-1).U) {
        state := S_DONE
      }
    }

    // Done pulse
    is (S_DONE) {
      io.flat_valid := true.B
      issue_cnt     := 0.U
      capture_cnt   := 0.U
      state         := S_IDLE
    }
  }

  // Pack flat_data_int into flat bus
  io.flat_data := flat_data_int.asUInt
}

object FlattenBuffer extends App {
    ChiselStage.emitSystemVerilogFile(
        new FlattenBuffer,
        Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
    )
}
