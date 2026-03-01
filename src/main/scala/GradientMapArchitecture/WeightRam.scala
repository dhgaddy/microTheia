package GradientMapArchitecture

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import firrtl.options.TargetDirAnnotation
import chisel3.util._
import tool._
import _root_.circt.stage.ChiselStage

class WeightRam(
    val CLASS_IDX: Int = 0,
    val NUM_CLASSES: Int = 4,
    val NUM_CELLS: Int = 256,
    val GRID_SIZE: Int = 16,
    val WEIGHT_BITS: Int = 8
) extends Module {

    require(GRID_SIZE * GRID_SIZE == NUM_CELLS)

    val addrWidth = log2Ceil(NUM_CELLS)

    val io = IO(new Bundle {
    val cell_addr = Input(UInt(addrWidth.W))
    val dout      = Output(SInt(WEIGHT_BITS.W))
})

    // Generate weights in Scala (elaboration time)
    val centre = GRID_SIZE / 2

    def clamp(x: Int, min: Int, max: Int): Int =
        math.max(min, math.min(max, x))

    val weights: Seq[SInt] = Seq.tabulate(NUM_CELLS) { addr =>
        val cy = addr / GRID_SIZE
        val cx = addr % GRID_SIZE

        val raw_val = CLASS_IDX match {

        case 0 => // UP
            if (cy < centre)
            (centre - cy) * 6
            else
            -((cy - centre + 1) * 4)

        case 1 => // DOWN
            if (cy >= centre)
            (cy - centre + 1) * 6
            else
            -((centre - cy) * 4)

        case 2 => // LEFT
            if (cx < centre)
            (centre - cx) * 6
            else
            -((cx - centre + 1) * 4)

        case 3 => // RIGHT
            if (cx >= centre)
            (cx - centre + 1) * 6
            else
            -((centre - cx) * 4)

        case _ => 0
        }

        clamp(raw_val, -128, 127).S(WEIGHT_BITS.W)
    }

    // ROM (inferred as constant Vec)
    val rom = VecInit(weights)

    // Synchronous read (1-cycle latency)
    val doutReg = RegNext(rom(io.cell_addr))

    io.dout := doutReg
}

object WeightRam extends App {
    ChiselStage.emitSystemVerilogFile(
        new WeightRam,
        Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
        firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
    )
}