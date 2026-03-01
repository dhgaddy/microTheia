package VoxelBinArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

import InputFIFO.InputFIFO
import EVT2Decoder.EVT2Decoder
import VoxelBinArchitecture.VoxelBinning
import VoxelBinArchitecture.WeightRam
import VoxelBinArchitecture.SystolicArray
import VoxelBinArchitecture.GestureClassifier

class VoxelBinCore(
  val CLK_FREQ_HZ: Int = 12000000,
  val WINDOW_MS: Int = 400,
  val GRID_SIZE: Int = 16,
  val FIFO_DEPTH: Int = 128,
  val MIN_EVENT_THRESH: Int = 20,
  val MOTION_THRESH: Int = 8,
  val PERSISTENCE_COUNT: Int = 2,
  val CYCLES_PER_BIN: Int = 0,
  val PARALLEL_READS: Int = 4
) extends Module {

  // Derived parameters
  val COUNTER_BITS = 6
  val NUM_CLASSES  = 4
  val NUM_BINS     = 4
  val NUM_CELLS    = NUM_BINS * GRID_SIZE * GRID_SIZE
  val WEIGHT_BITS  = 8
  val ACC_BITS     = 24
  val MIN_SCORE_THRESH = 30

  val FIFO_PTR_BITS = log2Ceil(FIFO_DEPTH)

  val io = IO(new Bundle {
    val evt_word        = Input(UInt(32.W))
    val evt_word_valid  = Input(Bool())
    val evt_word_ready  = Output(Bool())

    val gesture         = Output(UInt(2.W))
    val gesture_valid   = Output(Bool())
    val gesture_confidence = Output(UInt(4.W))

    val debug_event_count  = Output(UInt(8.W))
    val debug_state        = Output(UInt(3.W))
    val debug_fifo_empty   = Output(Bool())
    val debug_fifo_full    = Output(Bool())
    val debug_temporal_phase = Output(Bool())
  })

  // FIFO
  val fifo = Module(new InputFIFO(
    depth = FIFO_DEPTH,
    ptrBits = FIFO_PTR_BITS,
    dataWidth = 32
  ))

  fifo.io.clk := clock
  fifo.io.rst := reset

  fifo.io.wr_en  := io.evt_word_valid && !fifo.io.full
  fifo.io.wr_data:= io.evt_word
  fifo.io.rd_en  := !fifo.io.empty

  io.evt_word_ready := !fifo.io.full
  io.debug_fifo_empty := fifo.io.empty
  io.debug_fifo_full  := fifo.io.full
  io.debug_temporal_phase := false.B

  val fifoRdValid = RegNext(fifo.io.rd_en && !fifo.io.empty, false.B)

  // Event counter
  val eventCounter = RegInit(0.U(8.W))
  when(io.evt_word_valid && io.evt_word_ready) {
    eventCounter := eventCounter + 1.U
  }
  io.debug_event_count := eventCounter

  // EVT2 Decoder
  val decoder = Module(new EVT2Decoder)

  decoder.io.inWord  := fifo.io.rd_data
  decoder.io.inValid := fifoRdValid

val binnerX = decoder.io.x.asSInt - 8.S
val binnerY = decoder.io.y.asSInt - 8.S

  // Voxel Binning
  val voxel = Module(new VoxelBinning(
    CLK_FREQ_HZ = CLK_FREQ_HZ,
    WINDOW_MS = WINDOW_MS,
    NUM_BINS = NUM_BINS,
    READOUT_BINS = NUM_BINS,
    GRID_SIZE = GRID_SIZE,
    COUNTER_BITS = COUNTER_BITS,
    PARALLEL_READS = PARALLEL_READS,
    CYCLES_PER_BIN = CYCLES_PER_BIN
  ))

  voxel.io.event_valid    := decoder.io.outValid
  voxel.io.event_x        := binnerX
  voxel.io.event_y        := binnerY
  voxel.io.event_polarity := decoder.io.polarity

  // Systolic Array
  val systolic = Module(new SystolicArray(
    NUM_CLASSES = NUM_CLASSES,
    NUM_CELLS = NUM_CELLS,
    VALUE_BITS = COUNTER_BITS,
    WEIGHT_BITS = WEIGHT_BITS,
    ACC_BITS = ACC_BITS,
    PARALLEL_INPUTS = PARALLEL_READS
  ))

  systolic.io.start := voxel.io.readout_start
  systolic.io.feature_in := voxel.io.readout_data
  systolic.io.feature_valid := voxel.io.readout_valid

  // Parallel Weight RAMs
  val wAddrVec = systolic.io.w_addr_flat
  val ADDR_BITS = log2Ceil(NUM_CELLS)

  val wDataParallel = Wire(Vec(PARALLEL_READS,
    UInt((NUM_CLASSES * WEIGHT_BITS).W)))

  for (p <- 0 until PARALLEL_READS) {

    val addr =
      wAddrVec((p+1)*ADDR_BITS-1, p*ADDR_BITS)

    val classData = Wire(Vec(NUM_CLASSES, SInt(WEIGHT_BITS.W)))

    for (k <- 0 until NUM_CLASSES) {
      val wr = Module(new WeightRam(
        CLASS_IDX = k,
        NUM_CELLS = NUM_CELLS,
        GRID_SIZE = GRID_SIZE,
        WEIGHT_BITS = WEIGHT_BITS
      ))

      wr.io.we := false.B
      wr.io.cell_addr := addr
      wr.io.din := 0.S
      classData(k) := wr.io.dout
    }

    wDataParallel(p) := classData.asUInt
  }

  systolic.io.w_data_flat := wDataParallel.asUInt

  // Score Threshold Logic
  val scores = Wire(Vec(NUM_CLASSES, SInt(ACC_BITS.W)))
  for (k <- 0 until NUM_CLASSES) {
    scores(k) :=
      systolic.io.scores_flat((k+1)*ACC_BITS-1,
                               k*ACC_BITS).asSInt
  }

  val bestScore = scores(systolic.io.best_class)
  val absBestScore =
    Mux(bestScore < 0.S,
        (-bestScore).asUInt,
        bestScore.asUInt)

  val scoreAboveThresh =
    absBestScore >= MIN_SCORE_THRESH.U

  val pseudoMagX =
    Cat(0.U(2.W), absBestScore(15,0))

  val pseudoMagY = 0.U(18.W)

  // Gesture Classifier
  val classifier = Module(new GestureClassifier(
    ACC_SUM_BITS = 18,
    PERSISTENCE_COUNT = PERSISTENCE_COUNT
  ))

  classifier.io.class_gesture := systolic.io.best_class
  classifier.io.class_valid   := systolic.io.result_valid
  classifier.io.class_pass    :=
    systolic.io.result_valid && scoreAboveThresh

  classifier.io.abs_delta_x := pseudoMagX
  classifier.io.abs_delta_y := pseudoMagY

  io.gesture := classifier.io.gesture
  io.gesture_valid := classifier.io.gesture_valid
  io.gesture_confidence := classifier.io.gesture_confidence
  io.debug_state := classifier.io.debug_state
}

object VoxelBinCore extends App {
  ChiselStage.emitSystemVerilogFile(
    new VoxelBinCore,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
