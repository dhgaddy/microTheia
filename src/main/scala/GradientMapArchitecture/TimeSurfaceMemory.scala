package GradientMapArchitecture

import chisel3._
import chisel3.util._
import firrtl.options.TargetDirAnnotation
import chisel3.util._
import tool._
import _root_.circt.stage.ChiselStage

class TimeSurfaceMemory(
    val GRID_SIZE: Int   = 16,
    val ADDR_BITS: Int   = 8,
    val TS_BITS: Int     = 16,
    val VALUE_BITS: Int  = 8,
    val MAX_VALUE: Int   = 255,
    val DECAY_SHIFT: Int = 6
) extends Module {

    val NUM_CELLS = GRID_SIZE * GRID_SIZE
    require(NUM_CELLS == (1 << ADDR_BITS))

    val io = IO(new Bundle {
        val t_now = Input(UInt(TS_BITS.W))

    // Event write
    val event_valid = Input(Bool())
    val event_x     = Input(UInt(log2Ceil(GRID_SIZE).W))
    val event_y     = Input(UInt(log2Ceil(GRID_SIZE).W))
    val event_ts    = Input(UInt(TS_BITS.W))

    // Read interface
    val read_enable     = Input(Bool())
    val read_addr       = Input(UInt(ADDR_BITS.W))
    val read_value      = Output(UInt(VALUE_BITS.W))
    val read_ts_raw     = Output(UInt(TS_BITS.W))
    val read_cell_valid = Output(Bool())
})

    // Memory
    val mem = SyncReadMem(NUM_CELLS, UInt(TS_BITS.W))
    val cellValid = RegInit(VecInit(Seq.fill(NUM_CELLS)(false.B)))

    // Write logic
    //   val writeAddr = io.event_y * GRID_SIZE.U + io.event_x
    val writeAddr = (io.event_y << log2Ceil(GRID_SIZE)) | io.event_x

    when (reset.asBool) {
        cellValid.foreach(_ := false.B)
    }.elsewhen (io.event_valid) {
        mem.write(writeAddr, io.event_ts)
        cellValid(writeAddr) := true.B
    }

    // Read stage 1 (BRAM read)
    val bramData  = mem.read(io.read_addr, io.read_enable)
    val bramValid = RegNext(cellValid(io.read_addr), false.B)

    val bramDataReg  = RegNext(bramData)
    val bramValidReg = RegNext(bramValid)

    io.read_ts_raw     := bramDataReg
    io.read_cell_valid := bramValidReg

    // Read stage 2 (decay compute)
    // Total latency = 2 cycles
    val delta_t = io.t_now - bramDataReg
    val decaySteps = (delta_t >> DECAY_SHIFT)(7,0)

    val decayValue = Wire(UInt(VALUE_BITS.W))

    when (!bramValidReg) {
        decayValue := 0.U
    }.otherwise {
        when (decaySteps >= 8.U) {
        decayValue := 0.U
        }.otherwise {
        decayValue := (MAX_VALUE.U >> decaySteps)(VALUE_BITS-1, 0)
        }
    }

    io.read_value := RegNext(decayValue, 0.U)
}

object TimeSurfaceMemory extends App {
    ChiselStage.emitSystemVerilogFile(
        new TimeSurfaceMemory,
        Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
    )
}