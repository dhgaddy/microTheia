package voxel

import chisel3._
import chisel3.util._

class VoxelBinner(
    val numBins: Int = 8,
    val gridSize: Int = 16,
    val counterBits: Int = 9
) extends Module {

  val binCount = numBins + 1
  val addrBits = log2Ceil(gridSize * gridSize)
  val binBits  = log2Ceil(binCount)

  val io = IO(new Bundle {

    // Input event
    val inValid   = Input(Bool())
    val x         = Input(UInt(4.W))
    val y         = Input(UInt(4.W))
    val polarity  = Input(Bool())
    val timestamp = Input(UInt(34.W))

    // Time window control
    val advanceBin = Input(Bool())

    // Read port
    val readBin  = Input(UInt(binBits.W))
    val readAddr = Input(UInt(addrBits.W))
    val readData = Output(UInt(counterBits.W))
  })

  // ============================================================
  // Memory: [bin][pixel]
  // ============================================================

  val bins = RegInit(
    VecInit(Seq.fill(binCount)(
      VecInit(Seq.fill(gridSize * gridSize)(0.U(counterBits.W)))
    ))
  )

  // ============================================================
  // Active bin pointer
  // ============================================================

  val currentBin = RegInit(0.U(binBits.W))

  // Compute next bin explicitly (CRITICAL FIX)
  val nextBin = Mux(
    currentBin === (binCount - 1).U,
    0.U,
    currentBin + 1.U
  )

  when(io.advanceBin) {
    currentBin := nextBin

    // Clear the NEW active bin (not old one)
    for (i <- 0 until gridSize * gridSize) {
      bins(nextBin)(i) := 0.U
    }
  }

  // ============================================================
  // Spatial flatten index (CORRECT FIX)
  // ============================================================

  // 4-bit y concatenated with 4-bit x → 8-bit index
  val pixelIndex = Cat(io.y, io.x)

  // ============================================================
  // Saturating increment logic
  // ============================================================

  val maxValue = ((1 << counterBits) - 1).U

  when(io.inValid) {
    val currentValue = bins(currentBin)(pixelIndex)

    when(currentValue =/= maxValue) {
      bins(currentBin)(pixelIndex) := currentValue + 1.U
    }
  }

  // ============================================================
  // Read port
  // ============================================================

  io.readData := bins(io.readBin)(io.readAddr)
}
