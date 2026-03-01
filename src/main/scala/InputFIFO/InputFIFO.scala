package InputFIFO

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import firrtl.options.TargetDirAnnotation
import chisel3.util._
import tool._
import _root_.circt.stage.ChiselStage

class InputFIFO(
    val depth: Int = 256,
    val ptrBits: Int = 8,
    val dataWidth: Int = 32
) extends Module {

    require(isPow2(depth), "FIFO depth must be power of 2")
    require(depth == (1 << ptrBits), "ptrBits must equal log2(depth)")

    val io = IO(new Bundle {
    val clk     = Input(Clock())
    val rst     = Input(Bool())

    // Write interface
    val wr_en   = Input(Bool())
    val wr_data = Input(UInt(dataWidth.W))

    // Read interface
    val rd_en   = Input(Bool())
    val rd_data = Output(UInt(dataWidth.W))

    // Status
    val empty   = Output(Bool())
    val full    = Output(Bool())
    val count   = Output(UInt((ptrBits + 1).W))
})

    // Memory (infers BRAM)
    val mem = SyncReadMem(depth, UInt(dataWidth.W))

    // Pointers (extra MSB for wrap)
    val wr_ptr = RegInit(0.U((ptrBits + 1).W))
    val rd_ptr = RegInit(0.U((ptrBits + 1).W))

    val wr_addr = wr_ptr(ptrBits - 1, 0)
    val rd_addr = rd_ptr(ptrBits - 1, 0)

    // Status logic
    val full =
        (wr_ptr(ptrBits) =/= rd_ptr(ptrBits)) &&
        (wr_ptr(ptrBits - 1, 0) === rd_ptr(ptrBits - 1, 0))

    val empty = wr_ptr === rd_ptr

    io.full  := full
    io.empty := empty
    io.count := wr_ptr - rd_ptr

    // Write logic
    val doWrite = io.wr_en && !full

    when (doWrite) {
        mem.write(wr_addr, io.wr_data)
        wr_ptr := wr_ptr + 1.U
    }

    // Read logic (synchronous BRAM)
    val doRead = io.rd_en && !empty

    val rdDataReg = RegInit(0.U(dataWidth.W))

    // SyncReadMem requires explicit read enable
    val memReadData = mem.read(rd_addr, doRead)

    rdDataReg := memReadData

    when (doRead) {
        rd_ptr := rd_ptr + 1.U
    }

    io.rd_data := rdDataReg
}

object InputFIFO extends App {
    ChiselStage.emitSystemVerilogFile(
        new InputFIFO,
        Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
    )
}