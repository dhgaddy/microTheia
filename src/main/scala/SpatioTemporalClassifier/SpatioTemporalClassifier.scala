package SpatioTemporalClassifier

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

import WeightRom.WeightRom
import SystolicArray.SystolicArray

class SpatioTemporalClassifier(
  val CLK_FREQ_HZ: Int     = 12000000,
  val FRAME_PERIOD_MS: Int = 10,
  val GRID_SIZE: Int       = 16,
  val ADDR_BITS: Int       = 8,
  val VALUE_BITS: Int      = 8,
  val MOMENT_BITS: Int     = 24,
  val WEIGHT_BITS: Int     = 8,
  val SCORE_BITS: Int      = 24,
  val NUM_CLASSES: Int     = 4,
  val MIN_MASS_THRESH: Int = 100
) extends Module {

  val NUM_CELLS  = GRID_SIZE * GRID_SIZE
  val PIPE_DEPTH = 2
  val CNT_BITS   = log2Ceil(NUM_CELLS + PIPE_DEPTH + 2)

  val FRAME_CYCLES   = (CLK_FREQ_HZ / 1000) * FRAME_PERIOD_MS
  val FRAME_CNT_BITS = log2Ceil(FRAME_CYCLES + 1)

  val io = IO(new Bundle {

    val ts_read_addr   = Output(UInt(ADDR_BITS.W))
    val ts_read_enable = Output(Bool())
    val ts_read_value  = Input(UInt(VALUE_BITS.W))

    val gesture_class      = Output(UInt(log2Ceil(NUM_CLASSES).W))
    val gesture_valid      = Output(Bool())
    val gesture_confidence = Output(UInt(8.W))

    val debug_m00   = Output(UInt(MOMENT_BITS.W))
    val debug_m10   = Output(UInt(MOMENT_BITS.W))
    val debug_m01   = Output(UInt(MOMENT_BITS.W))
    val debug_state = Output(UInt(3.W))
  })

  // Default outputs
  io.ts_read_addr       := 0.U
  io.ts_read_enable     := false.B
  io.gesture_class      := 0.U
  io.gesture_valid      := false.B
  io.gesture_confidence := 0.U

  // Frame timer
  val frame_counter = RegInit(0.U(FRAME_CNT_BITS.W))
  val frame_pulse   = RegInit(false.B)

  when (frame_counter === (FRAME_CYCLES-1).U) {
    frame_counter := 0.U
    frame_pulse   := true.B
  } .otherwise {
    frame_counter := frame_counter + 1.U
    frame_pulse   := false.B
  }

  // Scan logic (direct BRAM streaming)
  val scan_active = RegInit(false.B)
  val scan_addr   = RegInit(0.U(ADDR_BITS.W))
  val scan_cnt    = RegInit(0.U(CNT_BITS.W))
  val energy_acc  = RegInit(0.U(MOMENT_BITS.W))
  val data_valid  = RegInit(false.B)

  when (frame_pulse) {
    scan_active := true.B
    scan_addr   := 0.U
    scan_cnt    := 0.U
    energy_acc  := 0.U
  } .elsewhen (scan_active) {

    data_valid := scan_cnt >= PIPE_DEPTH.U

    when (data_valid) {
      energy_acc := energy_acc + io.ts_read_value
    }

    when (scan_addr < (NUM_CELLS-1).U) {
      scan_addr := scan_addr + 1.U
      scan_cnt  := scan_cnt + 1.U
    } .otherwise {
      when (scan_cnt >= (NUM_CELLS + PIPE_DEPTH - 1).U) {
        scan_active := false.B
        scan_cnt    := 0.U
      } .otherwise {
        scan_cnt := scan_cnt + 1.U
      }
    }
  }

  io.ts_read_addr   := scan_addr
  io.ts_read_enable := scan_active && (scan_addr < NUM_CELLS.U)

  // Weight ROMs (4 parallel)
  val wrom0 = Module(new WeightRom(0))
  val wrom1 = Module(new WeightRom(1))
  val wrom2 = Module(new WeightRom(2))
  val wrom3 = Module(new WeightRom(3))

  wrom0.io.cell_addr := scan_addr
  wrom1.io.cell_addr := scan_addr
  wrom2.io.cell_addr := scan_addr
  wrom3.io.cell_addr := scan_addr

  val w_data_flat = Cat(
    wrom3.io.dout.asUInt,
    wrom2.io.dout.asUInt,
    wrom1.io.dout.asUInt,
    wrom0.io.dout.asUInt
  )

  // Systolic array
  val systolic = Module(new SystolicArray(
    NUM_CLASSES,
    NUM_CELLS,
    VALUE_BITS,
    WEIGHT_BITS,
    SCORE_BITS
  ))

  val sa_start        = RegInit(false.B)
  val sa_feature_in   = Reg(UInt(VALUE_BITS.W))
  val sa_feeding      = RegInit(false.B)
  val sa_feature_cnt  = RegInit(0.U(ADDR_BITS.W))

  systolic.io.w_data_flat := w_data_flat

  sa_start := false.B

  when (!sa_feeding && data_valid && scan_active) {
    sa_start       := true.B
    sa_feature_in  := io.ts_read_value
    sa_feeding     := true.B
    sa_feature_cnt := 1.U
  } .elsewhen (sa_feeding && data_valid) {

    sa_feature_in := io.ts_read_value

    when (sa_feature_cnt < (NUM_CELLS-1).U) {
      sa_feature_cnt := sa_feature_cnt + 1.U
    } .otherwise {
      sa_feeding     := false.B
      sa_feature_cnt := 0.U
    }
  }

  when (frame_pulse) {
    sa_feeding     := false.B
    sa_feature_cnt := 0.U
  }

  systolic.io.start      := sa_start
  systolic.io.feature_in := sa_feature_in

  // Threshold gate
  when (systolic.io.result_valid) {
    when (energy_acc >= MIN_MASS_THRESH.U) {
      io.gesture_valid := true.B
      io.gesture_class := systolic.io.best_class

      io.gesture_confidence :=
        Mux(energy_acc > 65535.U, 255.U, energy_acc(15,8))
    }
  }

  // Debug
  io.debug_m00 := energy_acc
  io.debug_m10 := systolic.io.scores_flat(SCORE_BITS-1, 0)
  io.debug_m01 := systolic.io.scores_flat(2*SCORE_BITS-1, SCORE_BITS)
  io.debug_state := Cat(0.U(1.W), sa_feeding, scan_active)
}

object SpatioTemporalClassifier extends App {
    ChiselStage.emitSystemVerilogFile(
        new SpatioTemporalClassifier,
        Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
    )
}
