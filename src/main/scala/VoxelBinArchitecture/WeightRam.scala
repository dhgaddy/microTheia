package VoxelBinArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage


class WeightRam(
  val CLASS_IDX: Int = 0,
  val NUM_CELLS: Int = 1024,
  val GRID_SIZE: Int = 16,
  val WEIGHT_BITS: Int = 8
) extends Module {

  require(CLASS_IDX >= 0 && CLASS_IDX <= 3, "CLASS_IDX must be 0..3")

  private val ADDR_BITS = log2Ceil(NUM_CELLS)

  val io = IO(new Bundle {
    val we        = Input(Bool())
    val cell_addr = Input(UInt(ADDR_BITS.W))
    val din       = Input(SInt(WEIGHT_BITS.W))
    val dout      = Output(SInt(WEIGHT_BITS.W))
  })

  // Block RAM (infers SB_RAM40_4K on iCE40)
  val ram = SyncReadMem(NUM_CELLS, SInt(WEIGHT_BITS.W))

  // Precompute initialization contents (elaboration time)
  val cellsPerBin = GRID_SIZE * GRID_SIZE
  val half        = GRID_SIZE / 2

  def clampToBits(x: Int): Int = {
    val maxVal = (1 << (WEIGHT_BITS - 1)) - 1
    val minVal = -(1 << (WEIGHT_BITS - 1))
    math.max(minVal, math.min(maxVal, x))
  }

  val initVec = Seq.tabulate(NUM_CELLS) { i =>
    val spatialAddr = i % cellsPerBin
    val binIdx      = i / cellsPerBin
    val cy          = spatialAddr / GRID_SIZE
    val cx          = spatialAddr % GRID_SIZE

    val tempPhase = if (binIdx >= 2) 1 else -1

    val rawVal = CLASS_IDX match {
      case 0 => // UP
        if (cy < half)
          tempPhase * (half - cy)
        else
          -(tempPhase * (cy - half + 1))

      case 1 => // DOWN
        if (cy >= half)
          tempPhase * (cy - half + 1)
        else
          -(tempPhase * (half - cy))

      case 2 => // LEFT
        if (cx < half)
          tempPhase * (half - cx)
        else
          -(tempPhase * (cx - half + 1))

      case 3 => // RIGHT
        if (cx >= half)
          tempPhase * (cx - half + 1)
        else
          -(tempPhase * (half - cx))

      case _ => 0
    }

    clampToBits(rawVal)
  }

  // Convert to VecInit for memory load
  val initMem = VecInit(initVec.map(_.S(WEIGHT_BITS.W)))

  // Initialize memory at elaboration (synthesis-friendly)
  for (i <- 0 until NUM_CELLS) {
    ram.write(i.U, initMem(i))
  }

  // Synchronous Read / Write Port
  val readData = ram.read(io.cell_addr)

  when(io.we) {
    ram.write(io.cell_addr, io.din)
  }

  io.dout := readData
}

object WeightRam extends App {
  ChiselStage.emitSystemVerilogFile(
    new WeightRam,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
