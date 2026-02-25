package GradientMapArchitecture

import chisel3._
import chisel3.util._
import firrtl.options.TargetDirAnnotation
import chisel3.util._
import tool._
import _root_.circt.stage.ChiselStage

class SystolicArray(
    val NUM_CLASSES: Int = 4,
    val NUM_CELLS: Int   = 1024,
    val VALUE_BITS: Int  = 8,
    val WEIGHT_BITS: Int = 8,
    val ACC_BITS: Int    = 24
) extends Module {

    val CNT_BITS = log2Ceil(NUM_CELLS + 4)

    val io = IO(new Bundle {

    val start = Input(Bool())
    val feature_in = Input(UInt(VALUE_BITS.W))

    val w_addr = Output(UInt(log2Ceil(NUM_CELLS).W))
    val w_data_flat = Input(UInt((NUM_CLASSES * WEIGHT_BITS).W))

    val result_valid = Output(Bool())
    val best_class   = Output(UInt(log2Ceil(NUM_CLASSES).W))
    val scores_flat  = Output(UInt((NUM_CLASSES * ACC_BITS).W))
})

     // State machine
    val states = Enum(4)
    val S_IDLE    = states(0)
    val S_RUNNING = states(1)
    val S_DRAIN   = states(2)
    val S_ARGMAX  = states(3)
    val state = RegInit(S_IDLE)
  
    // Counters
    val cell_cnt = RegInit(0.U(CNT_BITS.W))

    // Accumulators
    val acc   = RegInit(VecInit(Seq.fill(NUM_CLASSES)(0.S(ACC_BITS.W))))
    val acc_r = RegInit(VecInit(Seq.fill(NUM_CLASSES)(0.S(ACC_BITS.W))))

    // Feature pipeline (align with ROM latency)
    val feat_pipe_r  = Reg(UInt(VALUE_BITS.W))
    val pipe_valid_r = RegInit(false.B)

    // Unpack weights
    val w = Wire(Vec(NUM_CLASSES, SInt(WEIGHT_BITS.W)))

    for (k <- 0 until NUM_CLASSES) {
        w(k) := io.w_data_flat((k+1)*WEIGHT_BITS-1, k*WEIGHT_BITS).asSInt
    }

    // Combinational argmax
    val comb_best_class = Wire(UInt(log2Ceil(NUM_CLASSES).W))

    // Build parallel comparison wires to avoid self-referential cycle
    val is_max = Wire(Vec(NUM_CLASSES, Bool()))

    for (k <- 0 until NUM_CLASSES) {
    val beats_all_others = (0 until NUM_CLASSES).filter(_ != k).map { j =>
        acc(k) >= acc(j)
    }.reduce(_ && _)
    is_max(k) := beats_all_others
    }

    // Priority encode: pick lowest index that is max (handles ties)
    comb_best_class := (NUM_CLASSES - 1).U
    for (k <- (NUM_CLASSES - 2) to 0 by -1) {
    when(is_max(k)) {
        comb_best_class := k.U
    }
    }

    // Main FSM
    io.result_valid := false.B

    io.w_addr       := 0.U
    io.best_class   := 0.U
    io.result_valid := false.B

    switch(state) {

        is (S_IDLE) {

        pipe_valid_r := false.B

        when (io.start) {
            for (k <- 0 until NUM_CLASSES) {
            acc(k) := 0.S
            }

            cell_cnt    := 0.U
            io.w_addr   := 0.U
            feat_pipe_r := io.feature_in
            state       := S_RUNNING
        } .otherwise {
            io.w_addr := 0.U
        }
        }

        is (S_RUNNING) {

        // Accumulate previous feature × weight
        when (pipe_valid_r) {
            for (k <- 0 until NUM_CLASSES) {

            val mult = feat_pipe_r.asSInt * w(k)
                acc(k) := acc(k) + mult
            }
        }

        pipe_valid_r := true.B
        feat_pipe_r  := io.feature_in

        cell_cnt := cell_cnt + 1.U

        io.w_addr := cell_cnt

        when (cell_cnt === (NUM_CELLS-1).U) {
            state := S_DRAIN
        }
        }

        is (S_DRAIN) {

        for (k <- 0 until NUM_CLASSES) {
            val mult = feat_pipe_r.asSInt * w(k)
            acc(k) := acc(k) + mult
        }

        pipe_valid_r := false.B
        io.w_addr    := 0.U
        state        := S_ARGMAX
        }

        is (S_ARGMAX) {

        for (k <- 0 until NUM_CLASSES) {
            acc_r(k) := acc(k)
        }

        io.best_class   := comb_best_class
        io.result_valid := true.B
        io.w_addr       := 0.U

        state := S_IDLE
        }
    }

    // Drive flat score bus
    val scores = Wire(Vec(NUM_CLASSES, UInt(ACC_BITS.W)))

    for (k <- 0 until NUM_CLASSES) {
        scores(k) := acc_r(k).asUInt
    }

    io.scores_flat := scores.asUInt
}

object SystolicArray extends App {
    ChiselStage.emitSystemVerilogFile(
        new SystolicArray,
        Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
    )
}
