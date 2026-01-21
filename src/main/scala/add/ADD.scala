// package add

// import chisel3._
// import chisel3.stage.ChiselGeneratorAnnotation
// import firrtl.options.TargetDirAnnotation
// import tool._

// // _root_ disambiguates from package chisel3.util.circt if user imports chisel3.util._
// import _root_.circt.stage.ChiselStage

// class ADD extends Module{
//   val io = IO(new Bundle{
//     val In0 = new HS_Data(8)
//     val In1 = new HS_Data(8)
//     val Out = Flipped(new HS_Data(8))
//   })

//   private val ACG = Module(new ACG(Map(
//     "InNum" -> 2,
//     "OutNum" -> 1
//   )))

//   ACG.In(0) <> io.In0.HS
//   ACG.In(1) <> io.In1.HS
//   ACG.Out(0) <> io.Out.HS

//   AsyncClock(ACG.fire_o, reset){
//     io.Out.Data := RegNext(io.In0.Data + io.In1.Data)
//   }
// }

// // object elaborate_ADD extends App {
// //   (new chisel3.stage.ChiselStage).execute(
// //     Array("-X", "verilog"),
// //     Seq(ChiselGeneratorAnnotation(() => new Add()),
// //       TargetDirAnnotation("Outputs/Add"))
// //   )
// // }

// object ADD extends App {
//   ChiselStage.emitSystemVerilogFile(
//     new ADD,
//     Array("--target-dir", "rtl/chisel-verilog", "--target", "systemverilog"),
//     firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable")
//   )
// }