package TimeSurfaceEncoder

import chisel3._
import chisel3.util._
import firrtl.options.TargetDirAnnotation
import chisel3.util._
import tool._
import _root_.circt.stage.ChiselStage

import TimeSurfaceMemory.TimeSurfaceMemory

class TimeSurfaceEncoder(
    val GRID_SIZE: Int   = 16,
    val ADDR_BITS: Int   = 8,
    val TS_BITS: Int     = 16,
    val VALUE_BITS: Int  = 8,
    val MAX_VALUE: Int   = 255,
    val DECAY_SHIFT: Int = 6
) extends Module {

    val io = IO(new Bundle {

    val t_now = Input(UInt(TS_BITS.W))

    // Event write
    val event_valid = Input(Bool())
    val event_x     = Input(UInt(log2Ceil(GRID_SIZE).W))
    val event_y     = Input(UInt(log2Ceil(GRID_SIZE).W))
    val event_ts    = Input(UInt(TS_BITS.W))

    // Read interface
    val read_enable = Input(Bool())
    val read_addr   = Input(UInt(ADDR_BITS.W))

    val read_value  = Output(UInt(VALUE_BITS.W))
    val read_ts_raw = Output(UInt(TS_BITS.W))
})

    // Instantiate memory (storage only)
    val tsMem = Module(new TimeSurfaceMemory(
        GRID_SIZE,
        ADDR_BITS,
        TS_BITS,
        VALUE_BITS,
        MAX_VALUE,
        DECAY_SHIFT
    ))

    tsMem.io.t_now       := io.t_now
    tsMem.io.event_valid := io.event_valid
    tsMem.io.event_x     := io.event_x
    tsMem.io.event_y     := io.event_y
    tsMem.io.event_ts    := io.event_ts

    tsMem.io.read_enable := io.read_enable
    tsMem.io.read_addr   := io.read_addr

    val raw_ts       = tsMem.io.read_ts_raw
    val cell_valid   = tsMem.io.read_cell_valid

    io.read_ts_raw := raw_ts

    // Stage 1: compute Delta t and decay steps
    val delta_t_r1     = Reg(UInt(TS_BITS.W))
    val decay_steps_r1 = Reg(UInt(TS_BITS.W))
    val cell_valid_r1  = Reg(Bool())

    when (reset.asBool) {
        delta_t_r1     := 0.U
        decay_steps_r1 := 0.U
        cell_valid_r1  := false.B
    }.elsewhen (io.read_enable) {

        val delta = io.t_now - raw_ts

        delta_t_r1     := delta
        decay_steps_r1 := delta >> DECAY_SHIFT
        cell_valid_r1  := cell_valid
    }

    // Stage 2: exponential decay
    // value = MAX_VALUE >> decay_steps
    val next_value = Wire(UInt(VALUE_BITS.W))

    when (!cell_valid_r1) {
        next_value := 0.U
    }.elsewhen (decay_steps_r1 >= VALUE_BITS.U) {
        next_value := 0.U
    }.otherwise {
        next_value := (MAX_VALUE.U >> decay_steps_r1(VALUE_BITS-1,0))(VALUE_BITS-1,0)
    }

    io.read_value := RegNext(next_value, 0.U)
}

object TimeSurfaceEncoder extends App {
    ChiselStage.emitSystemVerilogFile(
        new TimeSurfaceEncoder,
        Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
    )
}
