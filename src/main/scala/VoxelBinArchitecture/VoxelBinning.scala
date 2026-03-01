package VoxelBinArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class VoxelBinning(
  val CLK_FREQ_HZ: Int = 12000000,
  val WINDOW_MS: Int = 400,
  val NUM_BINS: Int = 4,
  val READOUT_BINS: Int = 4,
  val GRID_SIZE: Int = 16,
  val COUNTER_BITS: Int = 6,
  val PARALLEL_READS: Int = 4,
  val CYCLES_PER_BIN: Int = 0
) extends Module {

  // Derived parameters
  val BIN_DURATION_MS     = WINDOW_MS / NUM_BINS
  val CYCLES_PER_BIN_AUTO = (CLK_FREQ_HZ / 1000) * BIN_DURATION_MS
  val CYCLES_PER_BIN_USE  =
    if (CYCLES_PER_BIN == 0) CYCLES_PER_BIN_AUTO else CYCLES_PER_BIN

  val TIMER_BITS      = log2Ceil(CYCLES_PER_BIN_USE + 1)
  val CELLS_PER_BIN   = GRID_SIZE * GRID_SIZE
  val TOTAL_CELLS     = NUM_BINS * CELLS_PER_BIN
  val CELL_ADDR_BITS  = log2Ceil(TOTAL_CELLS)
  val BIN_IDX_BITS    = log2Ceil(NUM_BINS)
  val GRID_ADDR_BITS  = log2Ceil(CELLS_PER_BIN)
  val PARALLEL_BITS   = log2Ceil(PARALLEL_READS)
  val CYCLES_PER_BIN_READ =
    (CELLS_PER_BIN + PARALLEL_READS - 1) / PARALLEL_READS

  val io = IO(new Bundle {
    val event_valid   = Input(Bool())
    val event_x       = Input(SInt(5.W))
    val event_y       = Input(SInt(5.W))
    val event_polarity= Input(Bool())

    val readout_start = Output(Bool())
    val readout_data  = Output(UInt((PARALLEL_READS * COUNTER_BITS).W))
    val readout_valid = Output(Bool())
  })

  // Memory (block RAM inferred)
  val mem = SyncReadMem(TOTAL_CELLS, UInt(COUNTER_BITS.W))

  // Bin Timer + Rotation
  val binTimer       = RegInit(0.U(TIMER_BITS.W))
  val currentBinIdx  = RegInit(0.U(BIN_IDX_BITS.W))
  val triggerReadout = RegInit(false.B)

  val states = Enum(3)
  val S_IDLE    = states(0)
  val S_READOUT = states(1)
  val S_CLEAR   = states(2)
  val state = RegInit(S_IDLE)

  val clearCellCtr = RegInit(0.U(GRID_ADDR_BITS.W))

  triggerReadout := false.B

  switch(state) {
    is(S_IDLE) {
      when(binTimer >= (CYCLES_PER_BIN_USE - 1).U) {
        binTimer := 0.U
        currentBinIdx :=
          Mux(currentBinIdx === (NUM_BINS - 1).U,
              0.U,
              currentBinIdx + 1.U)
        triggerReadout := true.B
        state := S_CLEAR
        clearCellCtr := 0.U
      }.otherwise {
        binTimer := binTimer + 1.U
      }
    }

    is(S_CLEAR) {
      when(clearCellCtr === (CELLS_PER_BIN - 1).U) {
        state := S_IDLE
      }
      clearCellCtr := clearCellCtr + 1.U
    }
  }

  // Event Mapping
  val mappedX = io.event_x + 8.S
  val mappedY = io.event_y + 8.S

  val eventCellAddr =
    Cat(mappedY(3,0).asUInt, mappedX(3,0).asUInt)

  val evtAddr = Cat(currentBinIdx, eventCellAddr)

  // Event Write (1-cycle pipelined like SV)
  val eventValidPipe = RegNext(io.event_valid)
  val evtAddrPipe    = RegNext(evtAddr)

  val readData = mem.read(evtAddr)

  when(state === S_CLEAR) {
    val clearAddr = Cat(currentBinIdx, clearCellCtr)
    mem.write(clearAddr, 0.U)
  }.elsewhen(eventValidPipe) {

    val maxVal = (1 << COUNTER_BITS) - 1
    val nextVal = Mux(readData =/= maxVal.U,
                      readData + 1.U,
                      readData)

    mem.write(evtAddrPipe, nextVal)

  }

  // Readout Engine
  val readoutBusy  = RegInit(false.B)
  val rdBinOffset  = RegInit(0.U(BIN_IDX_BITS.W))
  val rdCellCtr    = RegInit(0.U(GRID_ADDR_BITS.W))

  io.readout_start := false.B

  when(triggerReadout) {
    readoutBusy  := true.B
    io.readout_start := true.B
    rdBinOffset := 0.U
    rdCellCtr   := 0.U
  }.elsewhen(readoutBusy) {

    when(rdCellCtr >= (CYCLES_PER_BIN_READ - 1).U) {
      rdCellCtr := 0.U
      when(rdBinOffset === (READOUT_BINS - 1).U) {
        readoutBusy := false.B
      }.otherwise {
        rdBinOffset := rdBinOffset + 1.U
      }
    }.otherwise {
      rdCellCtr := rdCellCtr + 1.U
    }
  }

  // Chronological Bin Calculation
  val calcBinIdx = currentBinIdx + rdBinOffset
  val actualRdBinIdx =
    Mux(calcBinIdx >= NUM_BINS.U,
        calcBinIdx - NUM_BINS.U,
        calcBinIdx)

  // Parallel Reads
  val readVec = Wire(Vec(PARALLEL_READS, UInt(COUNTER_BITS.W)))

  for (p <- 0 until PARALLEL_READS) {
    val cellOffset = rdCellCtr * PARALLEL_READS.U + p.U
    val rdAddr = Cat(actualRdBinIdx, cellOffset(GRID_ADDR_BITS-1,0))

    readVec(p) :=
      Mux(readoutBusy && cellOffset < CELLS_PER_BIN.U,
          mem.read(rdAddr),
          0.U)
  }

  io.readout_data := readVec.asUInt

  val cellOffsetMax =
    rdCellCtr * PARALLEL_READS.U + (PARALLEL_READS - 1).U

  val readoutValidD =
    RegNext(readoutBusy && cellOffsetMax < CELLS_PER_BIN.U)

  io.readout_valid := readoutValidD
}

object VoxelBinning extends App {
  ChiselStage.emitSystemVerilogFile(
    new VoxelBinning,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
