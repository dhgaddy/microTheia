package ram

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import firrtl.options.TargetDirAnnotation
import tool._

// _root_ disambiguates from package chisel3.util.circt if user imports chisel3.util._
import _root_.circt.stage.ChiselStage

class RAM extends Module{
  val io = IO(new Bundle{
    val wr_valid = new HS_Data(1)
    val wr_data = new HS_Data(8)
    val wr_addr = new HS_Data(3)
    val rd_addr = new HS_Data(3)
    val rd_data = Flipped(new HS_Data(8))
  })

  private val ACG = Module(new ACG(Map(
    "InNum" -> 4,
    "OutNum" -> 1
  )))

  ACG.In(0) <> io.wr_valid.HS
  ACG.In(1) <> io.wr_data.HS
  ACG.In(2) <> io.wr_addr.HS
  ACG.In(3) <> io.rd_addr.HS
  ACG.Out(0) <> io.rd_data.HS

  //Chisel memory block
  val mem = Mem(8, UInt(8.W))

  AsyncClock(ACG.fire_o, reset){
    // If wr_valid.Data(0)==1, do a write on the handshake
    when(io.wr_valid.Data(0)) {
      mem(io.wr_addr.Data) := io.wr_data.Data
    }

    // Read data for rd_addr
    io.rd_data.Data := RegNext(mem(io.rd_addr.Data))
  }
}

object RAM extends App {
  ChiselStage.emitSystemVerilogFile(
    new RAM,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
  )
}
