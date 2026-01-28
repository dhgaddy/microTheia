package sync_matmul

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import firrtl.options.TargetDirAnnotation
import tool._

// _root_ disambiguates from package chisel3.util.circt if user imports chisel3.util._
import _root_.circt.stage.ChiselStage

class Sync_MatMul(val n: Int = 4, val bits: Int = 8) extends Module {
  val io = IO(new Bundle { 
    val A = Input(Vec(n, UInt(bits.W)))          // A(:,k) each cycle
    val B = Input(Vec(n, UInt(bits.W)))          // B(k,:) each cycle
    val start = Input(Bool())                        // clear accumulators
    val C = Output(Vec(n*n, UInt((2*bits).W)))
  })

  // Flattened (row, col)
  def idx(r: Int, c: Int): Int = r * n + c

  // Cell storage
  val acc = RegInit(VecInit(Seq.fill(n * n)(0.U((2*bits).W))))
  val aPipe = RegInit(VecInit(Seq.fill(n * n)(0.U(bits.W))))
  val bPipe = RegInit(VecInit(Seq.fill(n * n)(0.U(bits.W))))

  when(io.start) {
    // clear everything when starting a new multiplication
    for (k <- 0 until n * n) {
      acc(k) := 0.U
      aPipe(k) := 0.U
      bPipe(k) := 0.U
    }
  }.otherwise {
    // systolic update across the grid
    for (r <- 0 until n) {
      for (c <- 0 until n) {
        val id = idx(r, c)

        // choosing where A and B come from
        val aIn = if (c == 0) io.A(r) else aPipe(idx(r, c - 1))
        val bIn = if (r == 0) io.B(c) else bPipe(idx(r - 1, c))

        // accumulate MAC
        acc(id) := acc(id) + (aIn * bIn)

        // forwarding values for next input batch
        aPipe(id) := aIn
        bPipe(id) := bIn
      }
    }
  }

  io.C := acc
  
}

object Sync_MatMul extends App {
  ChiselStage.emitSystemVerilogFile(
    new Sync_MatMul,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
  )
}