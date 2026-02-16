package SpatialDownsampler

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class SpatialDownsampler extends Module {
  val io = IO(new Bundle {

    // Input from decoder
    val inValid   = Input(Bool())
    val inX       = Input(UInt(11.W))
    val inY       = Input(UInt(11.W))
    val polarity  = Input(Bool())
    val timestamp = Input(UInt(34.W))

    // Output 16x16
    val outValid  = Output(Bool())
    val outX      = Output(UInt(4.W))  // 0..15
    val outY      = Output(UInt(4.W))
    val outPolarity  = Output(Bool())
    val outTimestamp = Output(UInt(34.W))
  })

  // Parameters
  val SCALE = 20.U

  // Downscale logic
  val scaledX = (io.inX / SCALE)(3,0)
  val scaledY = (io.inY / SCALE)(3,0)

  // Register outputs (1-cycle pipeline)
  val outValidReg = RegNext(io.inValid, false.B)

  val xReg = RegNext(scaledX)
  val yReg = RegNext(scaledY)
  val polReg = RegNext(io.polarity)
  val tsReg = RegNext(io.timestamp)

  io.outValid     := outValidReg
  io.outX         := xReg
  io.outY         := yReg
  io.outPolarity  := polReg
  io.outTimestamp := tsReg
}

object SpatialDownsampler extends App {
  ChiselStage.emitSystemVerilogFile(
    new SpatialDownsampler,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
  )
}