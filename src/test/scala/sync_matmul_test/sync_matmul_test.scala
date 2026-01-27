package sync_matmul_test

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.flatspec.AnyFlatSpec
import sync_matmul.Sync_MatMul

class SyncMatMul2x2Test extends AnyFlatSpec with ChiselSim {

  it should "multiply 2x2 matrices (skewed systolic schedule)" in {
    simulate(new Sync_MatMul(n = 2, bits = 8)) { dut =>

      // clear accumulators/pipes
      dut.io.start.poke(true.B)
      dut.clock.step(1)
      dut.io.start.poke(false.B)

      def pokeCycle(a0: Int, a1: Int, b0: Int, b1: Int): Unit = {
        dut.io.A(0).poke(a0.U)
        dut.io.A(1).poke(a1.U)
        dut.io.B(0).poke(b0.U)
        dut.io.B(1).poke(b1.U)
        dut.clock.step(1)
      }

      // A = [[1,2],[3,4]]
      // B = [[5,6],[7,8]]

      // t=0
      pokeCycle(1, 0, 5, 0)
      println(s"C0 = ${dut.io.C(0).peek().litValue}")
      println(s"C1 = ${dut.io.C(1).peek().litValue}")
      println(s"C2 = ${dut.io.C(2).peek().litValue}")
      println(s"C3 = ${dut.io.C(3).peek().litValue}")
      // t=1
      pokeCycle(2, 3, 7, 6)
      println(s"C0 = ${dut.io.C(0).peek().litValue}")
      println(s"C1 = ${dut.io.C(1).peek().litValue}")
      println(s"C2 = ${dut.io.C(2).peek().litValue}")
      println(s"C3 = ${dut.io.C(3).peek().litValue}")
      // t=2
      pokeCycle(0, 4, 0, 8)
      println(s"C0 = ${dut.io.C(0).peek().litValue}")
      println(s"C1 = ${dut.io.C(1).peek().litValue}")
      println(s"C2 = ${dut.io.C(2).peek().litValue}")
      println(s"C3 = ${dut.io.C(3).peek().litValue}")

      // flush
      pokeCycle(0, 0, 0, 0)
      println(s"C0 = ${dut.io.C(0).peek().litValue}")
      println(s"C1 = ${dut.io.C(1).peek().litValue}")
      println(s"C2 = ${dut.io.C(2).peek().litValue}")
      println(s"C3 = ${dut.io.C(3).peek().litValue}")

      // Expected C = [[19,22],[43,50]]
      dut.io.C(0).expect(19.U) // C00
      dut.io.C(1).expect(22.U) // C01
      dut.io.C(2).expect(43.U) // C10
      dut.io.C(3).expect(50.U) // C11
    }
  }
}
