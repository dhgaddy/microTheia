package VoxelBinArchitecture

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage

class SystolicArray(
  val NUM_CLASSES: Int = 4,
  val NUM_CELLS: Int = 1024,
  val VALUE_BITS: Int = 6,
  val WEIGHT_BITS: Int = 8,
  val ACC_BITS: Int = 24,
  val PARALLEL_INPUTS: Int = 4
) extends Module {

  // Derived parameters (match SV localparams)
  val CNT_BITS      = log2Ceil(NUM_CELLS + 4)
  val ADDR_BITS     = log2Ceil(NUM_CELLS)
  val CYCLES_NEEDED = (NUM_CELLS + PARALLEL_INPUTS - 1) / PARALLEL_INPUTS

  val io = IO(new Bundle {
    val start         = Input(Bool())
    val feature_in    = Input(UInt((PARALLEL_INPUTS * VALUE_BITS).W))
    val feature_valid = Input(Bool())

    val w_addr_flat   = Output(UInt((PARALLEL_INPUTS * ADDR_BITS).W))
    val w_data_flat   = Input(UInt((PARALLEL_INPUTS * NUM_CLASSES * WEIGHT_BITS).W))

    val result_valid  = Output(Bool())
    val best_class    = Output(UInt(2.W))
    val scores_flat   = Output(UInt((NUM_CLASSES * ACC_BITS).W))
  })

  // FSM
  val states = Enum(4)
  val S_IDLE    = states(0)
  val S_RUNNING = states(1)
  val S_DRAIN   = states(2)
  val S_ARGMAX  = states(3)
  val state = RegInit(S_IDLE)

  // Registers
  val cellCnt = RegInit(0.U(CNT_BITS.W))
  val accCnt  = RegInit(0.U(CNT_BITS.W))

  val acc   = RegInit(VecInit(Seq.fill(NUM_CLASSES)(0.S(ACC_BITS.W))))
  val acc_r = RegInit(VecInit(Seq.fill(NUM_CLASSES)(0.S(ACC_BITS.W))))

  val featPipe = Reg(UInt((PARALLEL_INPUTS * VALUE_BITS).W))
  val pipeValid = RegInit(false.B)

  val resultValidReg = RegInit(false.B)
  val bestClassReg   = RegInit(0.U(2.W))

  io.result_valid := resultValidReg
  io.best_class   := bestClassReg

  // Default pulse behavior
  resultValidReg := false.B

  // Unpack helpers
  val wAddrVec = Wire(Vec(PARALLEL_INPUTS, UInt(ADDR_BITS.W)))
  val wDataVec = Wire(Vec(PARALLEL_INPUTS, UInt((NUM_CLASSES * WEIGHT_BITS).W)))

  for (p <- 0 until PARALLEL_INPUTS) {
    wAddrVec(p) := io.w_addr_flat((p+1)*ADDR_BITS-1, p*ADDR_BITS)
    wDataVec(p) := io.w_data_flat((p+1)*NUM_CLASSES*WEIGHT_BITS-1,
                                   p*NUM_CLASSES*WEIGHT_BITS)
  }

  val w = Wire(Vec(PARALLEL_INPUTS,
              Vec(NUM_CLASSES, SInt(WEIGHT_BITS.W))))

  for (p <- 0 until PARALLEL_INPUTS) {
    for (k <- 0 until NUM_CLASSES) {
      w(p)(k) := wDataVec(p)((k+1)*WEIGHT_BITS-1,
                               k*WEIGHT_BITS).asSInt
    }
  }

  val featVals = Wire(Vec(PARALLEL_INPUTS, SInt(VALUE_BITS.W)))
  for (p <- 0 until PARALLEL_INPUTS) {
    featVals(p) := featPipe((p+1)*VALUE_BITS-1,
                             p*VALUE_BITS).asSInt
  }

  // Main FSM
  resultValidReg := false.B
  io.w_addr_flat := 0.U   // <-- add this line

  switch(state) {

    is(S_IDLE) {
      pipeValid := false.B

      when(io.start) {
        acc.foreach(_ := 0.S)
        cellCnt := 0.U
        accCnt  := 0.U

        // preload first addresses
        val addrVec = Wire(Vec(PARALLEL_INPUTS, UInt(ADDR_BITS.W)))
        for (p <- 0 until PARALLEL_INPUTS)
          addrVec(p) := p.U
        io.w_addr_flat := addrVec.asUInt

        state := S_RUNNING
      }.otherwise {
        io.w_addr_flat := 0.U
      }
    }

    is(S_RUNNING) {

      featPipe := io.feature_in

      // Accumulate (1 cycle delayed)
      when(pipeValid) {
        val accTmp = Wire(Vec(NUM_CLASSES, SInt(ACC_BITS.W)))
        for (k <- 0 until NUM_CLASSES)
          accTmp(k) := acc(k)

        for (p <- 0 until PARALLEL_INPUTS) {
          when((accCnt * PARALLEL_INPUTS.U + p.U) < NUM_CELLS.U) {
            for (k <- 0 until NUM_CLASSES) {
              val prod = (featVals(p).asSInt *
                          w(p)(k)).asSInt
              accTmp(k) := accTmp(k) + prod
            }
          }
        }

        for (k <- 0 until NUM_CLASSES)
          acc(k) := accTmp(k)

        accCnt := accCnt + 1.U
      }

      when(io.feature_valid) {
        pipeValid := true.B
        cellCnt := cellCnt + 1.U

        val addrVec = Wire(Vec(PARALLEL_INPUTS, UInt(ADDR_BITS.W)))
        for (p <- 0 until PARALLEL_INPUTS) {
          addrVec(p) := cellCnt * PARALLEL_INPUTS.U + p.U
        }
        io.w_addr_flat := addrVec.asUInt

        when(cellCnt >= (CYCLES_NEEDED - 1).U) {
          state := S_DRAIN
        }

      }.otherwise {
        io.w_addr_flat := 0.U
      }
    }

    is(S_DRAIN) {

      when(pipeValid) {
        val accTmp = Wire(Vec(NUM_CLASSES, SInt(ACC_BITS.W)))
        for (k <- 0 until NUM_CLASSES)
          accTmp(k) := acc(k)

        for (p <- 0 until PARALLEL_INPUTS) {
          when((accCnt * PARALLEL_INPUTS.U + p.U) < NUM_CELLS.U) {
            for (k <- 0 until NUM_CLASSES) {
              accTmp(k) := accTmp(k) +
                (featVals(p) * w(p)(k))
            }
          }
        }

        for (k <- 0 until NUM_CLASSES)
          acc(k) := accTmp(k)
      }

      pipeValid := false.B
      io.w_addr_flat := 0.U
      state := S_ARGMAX
    }

    is(S_ARGMAX) {

      for (k <- 0 until NUM_CLASSES)
        acc_r(k) := acc(k)

      // Parallel argmax - each class checks if it beats all others
      val is_max = Wire(Vec(NUM_CLASSES, Bool()))
      for (k <- 0 until NUM_CLASSES) {
        is_max(k) := (0 until NUM_CLASSES).filter(_ != k)
                       .map(j => acc(k) >= acc(j))
                       .reduce(_ && _)
      }

      // Priority encode lowest winning index
      val bestIdx = Wire(UInt(2.W))
      bestIdx := (NUM_CLASSES - 1).U
      for (k <- (NUM_CLASSES - 2) to 0 by -1) {
        when(is_max(k)) { bestIdx := k.U }
      }

      bestClassReg   := bestIdx
      resultValidReg := true.B
      state := S_IDLE
    }
  }

  // Flatten scores
  io.scores_flat := acc_r.asUInt
}

object SystolicArray extends App {
  ChiselStage.emitSystemVerilogFile(
    new SystolicArray,
    Array("--target-dir", "src/rtl/chisel-verilog", "--target", "systemverilog"),
    firtoolOpts = Array(
      "-disable-all-randomization",
      "-strip-debug-info",
      "-default-layer-specialization=enable"
    )
  )
}
