package voxelbinning

import chisel3._
import chisel3.util._
import chisel3.stage.ChiselGeneratorAnnotation
import firrtl.options.TargetDirAnnotation
import tool._
import _root_.circt.stage.ChiselStage

class VoxelBinning(
  val win: Int = 400,                 // Window size W 
  val w_bins: Int = 5,                // Number of bins B stored in ring
  val r_bins: Int = 5,                // Number of bins to read out
  val grid: Int = 16,                 // Pooled x/y grid (e.g., 16x16)
  val counter: Int = 8,               // Counter bit size
  val parallelReads: Int = 1,         // How many counters output per cycle
  val TIME_W: Int = 32
) extends Module {
  val io = IO(new Bundle { 
    // Event input
    val event_valid = Input(Bool())
    val event_ready = Output(Bool())
    val event_x     = Input(SInt(5.W))
    val event_y     = Input(SInt(5.W))
    val polarity    = Input(Bool())
    val event_time  = Input(UInt(TIME_W.W))

    // Readout stream
    val readout_start = Output(Bool())
    val readout_data  = Output(Vec(parallelReads, UInt(counter.W)))
    val readout_valid = Output(Bool())
    val readout_ready = Input(Bool())
    val readout_last  = Output(Bool())
  })

  // Derived sizes
  val dt: UInt = (win / w_bins).U(TIME_W.W)            // dt = window/bins
  val cellsPerBin: Int = grid * grid                  // number of spatial cells in one bin (e.g., 16*16=256)
  val featsPerWindow: Int = r_bins * cellsPerBin      // total features in one readout window (bins*cells)

  val binIdxW: Int = log2Ceil(w_bins max 1)          // bits needed to index bins (0..w_bins-1)
  val cellIdxW: Int = log2Ceil(cellsPerBin max 1)     // bits needed to index cells within a bin (0..grid*grid-1)
  val featIdxW: Int = log2Ceil(featsPerWindow max 1)  // bits needed to index flattened feature vector
  val laneIdxW: Int = log2Ceil(parallelReads max 1)   // bits needed to index which lane (if parallelReads>1)

  // Ring buffer memory: counters for all bins
  val totalCounters = w_bins * cellsPerBin            // total counter slots across all bins
  val counterAddrW: Int = log2Ceil(totalCounters max 1) // bits needed to index counters ring storage
  val counters = RegInit(VecInit(Seq.fill(totalCounters)(0.U(counter.W)))) // actual storage: every counter starts at 0

  def ringAddr(bin: UInt, cell: UInt): UInt =
    ((bin * cellsPerBin.U) + cell)(counterAddrW - 1, 0) // convert (bin,cell) into flat index in counters[]

  // Double buffer to hold a flattened window snapshot
  val winBuf0 = RegInit(VecInit(Seq.fill(featsPerWindow)(0.U(counter.W)))) // snapshot buffer 0 (flattened features)
  val winBuf1 = RegInit(VecInit(Seq.fill(featsPerWindow)(0.U(counter.W)))) // snapshot buffer 1 (flattened features)
  val capSel  = RegInit(false.B)                      // selects which snapshot buffer we write into this time

  def winBufWrite(idx: UInt, value: UInt): Unit = {   // helper: write into whichever snapshot buffer is active
    when(!capSel) {                                   // if capSel=0 -> write to winBuf0
      winBuf0(idx) := value                           // store feature at index idx
    }.otherwise {                                     // else capSel=1 -> write to winBuf1
      winBuf1(idx) := value                           // store feature at index idx
    }
  }

  def winBufRead(idx: UInt): UInt = {                 // helper: read from whichever snapshot buffer is active
    Mux(!capSel, winBuf0(idx), winBuf1(idx))           // if capSel=0 read winBuf0 else winBuf1
  }

  // Coordinate mapping (SInt centered -> UInt 0..grid-1)
  private def clampToGrid(u: UInt): UInt =
    Mux(u >= grid.U, (grid - 1).U, u)                 // clamp: if u is too big, force it to grid-1

  val mxU = (io.event_x + (grid/2).S).asUInt          // shift signed x (e.g., -8..7) into unsigned (0..15)
  val myU = (io.event_y + (grid/2).S).asUInt          // shift signed y into unsigned
  val mx  = clampToGrid(mxU)                          // clamp x into [0..grid-1]
  val my  = clampToGrid(myU)                          // clamp y into [0..grid-1]

  val cellIdx = (my * grid.U + mx)(cellIdxW - 1, 0)   // flatten (x,y) into one cell index: y*grid + x

  // Saturating increment helper
  val satMax = ((1 << counter) - 1).U(counter.W)      // maximum counter value (e.g., 255 for 8-bit)
  def satInc(x: UInt): UInt =
    Mux(x === satMax, x, x + 1.U)                     // if already max, stay; else increment by 1

  // State + bin tracking
  val sRun :: sCapture :: sOutput :: sAdvClear :: sApplyEvt :: Nil = Enum(5) // 5 FSM states
  val state = RegInit(sRun) // start FSM in RUN state                   

  val haveBin   = RegInit(false.B)                     // false until we see first event and initialize time/bin
  val curBinIdx = RegInit(0.U(binIdxW.W))              // which bin we are currently writing into
  val curBinEnd = RegInit(0.U(TIME_W.W))               // timestamp when the current bin ends

  val binsInMem = RegInit(0.U(binIdxW.W))              // how many bins are filled/valid so far (for "skip output until enough")

  val latEvtTime = Reg(UInt(TIME_W.W))                 // latched timestamp of boundary-crossing event
  val latEvtCell = Reg(UInt(cellIdxW.W))               // latched cell index of boundary-crossing event

  val advBinsLeft  = RegInit(0.U(binIdxW.W))           // how many bins still need to be advanced/cleared
  val clearCellIdx = RegInit(0.U(cellIdxW.W))          // which cell we're clearing inside the current bin

  val capFeatIdx = RegInit(0.U(featIdxW.W))            // index while capturing flattened features into winBuf
  val outFeatIdx = RegInit(0.U(featIdxW.W))            // index while streaming features out on readout_data

  // Readout outputs default
  io.readout_valid := false.B                          // default: not outputting anything
  io.readout_start := false.B                          // default: not start-of-window
  io.readout_last  := false.B                          // default: not end-of-window
  io.readout_data  := VecInit(Seq.fill(parallelReads)(0.U(counter.W))) // default: output zeros

  // Only accept events in RUN state
  io.event_ready := (state === sRun)                  // ready only when we're in RUN state
  val eventFire = io.event_valid && io.event_ready     // true when we actually accept an event this cycle

  // Wrap helpers for ring indices (circular bin buffer)
  def wrapInc(x: UInt): UInt =
    Mux(x === (w_bins - 1).U, 0.U, x + 1.U)            // increment bin index with wrap-around

  def wrapAdd(x: UInt, add: UInt): UInt = {            // add "add" to x with wrap-around
    val sum = Wire(UInt((binIdxW + 1).W))              // one extra bit to avoid overflow on addition
    sum := x + add                                     // compute raw sum
    Mux(sum >= w_bins.U, (sum - w_bins.U)(binIdxW-1,0), sum(binIdxW-1,0)) // wrap if sum >= w_bins
  }

  // Oldest bin index among the last r_bins (oldest->newest ordering)
  def oldestBinIdx(curr: UInt): UInt = {               // curr is the newest bin; compute oldest
    val back = (r_bins - 1).U                          // how far back the oldest is from the newest
    Mux(                                                 // do (curr - back) mod w_bins
      curr >= back,
      (curr - back)(binIdxW - 1, 0),
      (curr + w_bins.U - back)(binIdxW - 1, 0)
    )
  }

  // Compute number of bins to advance when event_time is outside the current bin
  // k = floor((t - curBinEnd)/dt) + 1
  def binsToAdvance(t: UInt, end: UInt): UInt = {
    val diff = t - end                                  // how far past the current bin end we are
    val q    = diff / dt                                // how many full bins were skipped
    val k    = q + 1.U                                  // +1 because even 0 skipped means advance 1 bin
    Mux(k > w_bins.U, w_bins.U, k)(binIdxW - 1, 0)       // cap k to w_bins to keep it bounded
  }

  // FSM
  switch(state) {

    is(sRun) {
      when(eventFire) {                                 // only process when an event is accepted

        when(!haveBin) {                                // if this is the very first event ever
          haveBin   := true.B                           // mark bin system initialized
          curBinIdx := 0.U                              // start writing into bin 0
          curBinEnd := io.event_time + dt               // current bin ends dt after first event time
          binsInMem := 1.U                              // we now have 1 valid bin in memory
          val a0 = ringAddr(0.U(binIdxW.W), cellIdx)    // first event belongs to newly initialized bin 0
          counters(a0) := satInc(counters(a0))          // count the first event immediately
        }

        when(haveBin && (io.event_time < curBinEnd)) {   // if event timestamp is still inside current bin
          val a = ringAddr(curBinIdx, cellIdx)           // compute address for (current bin, this cell)
          counters(a) := satInc(counters(a))             // increment that counter (saturating)
        }.elsewhen(haveBin) {                            // else event is outside bin -> boundary crossing

          latEvtTime := io.event_time                    // latch event time to apply later
          latEvtCell := cellIdx                          // latch event cell to apply later

          val k = binsToAdvance(io.event_time, curBinEnd) // compute bins to advance/clear
          advBinsLeft := k                               // remember how many bins to advance/clear

          when(binsInMem >= r_bins.U) {                  // if we have enough bins, do window readout
            capSel     := ~capSel                        // toggle which winBuf we capture into
            capFeatIdx := 0.U                            // start capture at feature 0
            state      := sCapture                      // go capture window snapshot
          }.otherwise {                                  // not enough bins -> skip output
            clearCellIdx := 0.U                          // start clearing at cell 0
            state        := sAdvClear                  // go advance/clear bins only
          }
        }
      }
    }

    is(sCapture) {
      val oldest = oldestBinIdx(curBinIdx)               // find the oldest bin of the last r_bins window

      for (lane <- 0 until parallelReads) {              // each lane outputs/copies one feature per cycle
        val idx = capFeatIdx + lane.U                    // which feature index we're capturing now
        when(idx < featsPerWindow.U) {                   // guard: only if idx is inside window length
          val binOff  = idx / cellsPerBin.U              // which bin offset within the window (0..r_bins-1)
          val cellOff = idx % cellsPerBin.U              // which cell within that bin (0..cellsPerBin-1)
          val bIdx    = wrapAdd(oldest, binOff)          // actual ring bin index for this window bin
          val a       = ringAddr(bIdx, cellOff)          // address of that counter
          winBufWrite(idx, counters(a))                  // copy counter into snapshot buffer at feature idx
        }
      }

      capFeatIdx := capFeatIdx + parallelReads.U         // move capture pointer forward by lanes per cycle

      when(capFeatIdx + parallelReads.U >= featsPerWindow.U) { // if we finished capturing entire window
        outFeatIdx := 0.U                                // start output from feature 0
        state      := sOutput                           // go stream window out
      }
    }

    is(sOutput) {
      val fire = io.readout_ready                        // downstream says it can accept a beat this cycle

      io.readout_valid := true.B                         // we are outputting valid data in OUTPUT state
      io.readout_start := (outFeatIdx === 0.U)           // start pulse on first beat
      io.readout_last  := (outFeatIdx + parallelReads.U >= featsPerWindow.U) // last pulse on final beat

      for (lane <- 0 until parallelReads) {              // drive each lane of output
        val idx = outFeatIdx + lane.U                    // which feature index for this lane
        io.readout_data(lane) := Mux(                    // output feature if in range else 0
          idx < featsPerWindow.U,
          winBufRead(idx),
          0.U(counter.W)
        )
      }

      when(fire) {                                       // only advance when downstream is ready
        outFeatIdx := outFeatIdx + parallelReads.U       // advance output pointer
        when(outFeatIdx + parallelReads.U >= featsPerWindow.U) { // if we just sent the last beat
          clearCellIdx := 0.U                            // begin clearing at cell 0
          state        := sAdvClear                    // advance/clear bins for the new event time
        }
      }
    }

    is(sAdvClear) {
      when(clearCellIdx === 0.U) {                       // at start of clearing a bin (first cell)
        curBinIdx := wrapInc(curBinIdx)                  // move to next bin in ring (advance time bin)
        curBinEnd := curBinEnd + dt                      // extend bin end by dt (next bin end)
        when(binsInMem < w_bins.U) {                     // if we haven't filled all bins yet
          binsInMem := binsInMem + 1.U                   // increment count of bins-in-memory
        }
      }

      val clearBinIdx = Mux(clearCellIdx === 0.U, wrapInc(curBinIdx), curBinIdx) // use the advanced bin immediately on first clear cycle
      val a = ringAddr(clearBinIdx, clearCellIdx)        // address of (new current bin, cell being cleared)
      counters(a) := 0.U                                 // clear that one cell (set counter to 0)

      when(clearCellIdx === (cellsPerBin - 1).U) {       // if we cleared the last cell of this bin
        clearCellIdx := 0.U                              // reset clear cell pointer for next bin
        when(advBinsLeft === 1.U) {                      // if this was the final bin we needed to advance
          advBinsLeft := 0.U                             // done advancing
          state       := sApplyEvt                     // now apply the latched event to the correct bin
        }.otherwise {                                     // else we still have more bins to advance/clear
          advBinsLeft := advBinsLeft - 1.U               // decrement remaining bins to clear
          // stay in S_ADV_CLEAR (next cycle will advance again at clearCellIdx==0)
        }
      }.otherwise {
        clearCellIdx := clearCellIdx + 1.U               // move to next cell to clear on next cycle
      }
    }

    is(sApplyEvt) {
      val a = ringAddr(curBinIdx, latEvtCell)            // address of (current bin after advance, latched cell)
      counters(a) := satInc(counters(a))                 // apply the boundary-crossing event increment
      state := sRun                                     // go back to RUN and accept new events again
    }
  }
}

object VoxelBinning extends App {
  ChiselStage.emitSystemVerilogFile(
    new VoxelBinning,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array("-disable-all-randomization", "-strip-debug-info", "-default-layer-specialization=enable") // Disabling this gives code more similar to the old version
  )
}
