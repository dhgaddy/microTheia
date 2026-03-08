# Utility functions for parsing the filelist. Each module directory
# must have filelist.json with keys for "top" and "files", like so:

# {
#     "top": "hello",
#     "files":
#     ["part1/sim/hello.sv"
#     ]
# }

# Each file in the filelist is relative to the repository root.

import os
import git

import sys
import json
import cocotb

from cocotb_test.simulator import run
from cocotb.clock import Clock
from cocotb.regression import TestFactory
from cocotb.utils import get_sim_time
from cocotb.triggers import Timer, ClockCycles, RisingEdge, FallingEdge, with_timeout
from cocotb.types import LogicArray
from cocotb.utils import get_sim_time

def runner(simulator, timescale, tbpath, params, defs=[], testname=None, pymodule=None, jsonpath=None, jsonname="filelist.json", root=None):
    """Run the simulator on test n, with parameters params, and defines
    defs. If n is none, it will run all tests"""

    # if json path is none, assume that it is the same as tbpath
    if(jsonpath is None):
        jsonpath = tbpath

    assert (os.path.exists(jsonpath)), "jsonpath directory must exist"
    top = get_top(jsonpath, jsonname)

    # if pymodule is none, assume that the python module name is test+<name of the top module>.
    if(pymodule is None):
        pymodule = "test_" + top

    if(testname is None):
        testdir = "all"
    else:
        testdir=testname
    
    # Assume all paths in the json file are relative to the repository root.
    if(root is None):
        root = git.Repo(search_parent_directories=True).working_tree_dir

    assert (os.path.exists(root)), "root directory path must exist"

    sources = get_sources(root, tbpath)

    work_dir = os.path.join(tbpath, "run", testdir, get_param_string(params), simulator)
    build_dir = os.path.join(tbpath, "build", get_param_string(params))

    # Icarus doesn't build, it just runs.
    if simulator.startswith("icarus"):
        build_dir = work_dir

    if simulator.startswith("verilator"):
        compile_args=["-Wno-fatal", "-DVM_TRACE_FST=1", "-DVM_TRACE=1"]
        plus_args = ["--trace", "--trace-fst"]
        if(not os.path.exists(work_dir)):
            os.makedirs(work_dir)
    else:
        compile_args=[]
        plus_args = []

    run(verilog_sources=sources,
        simulator=simulator,
        toplevel=top,
        module=pymodule,
        compile_args=compile_args,
        plus_args=plus_args,
        sim_build=build_dir,
        timescale=timescale,
        parameters=params,
        defines=defs + ["VM_TRACE_FST=1", "VM_TRACE=1"],
        work_dir=work_dir,
        waves=True,
        testcase=testname)

# Function to build (run) the lint and style checks.
def lint(simulator, timescale, tbpath, params, defs=[], compile_args=[], pymodule=None, jsonpath=None, jsonname="filelist.json", root=None):

    # if json path is none, assume that it is the same as tbpath
    if(jsonpath is None):
        jsonpath = tbpath

    assert (os.path.exists(jsonpath)), "jsonpath directory must exist"
    top = get_top(jsonpath, jsonname)


    # Assume all paths in the json file are relative to the repository root.
    if(root is None):
        root = git.Repo(search_parent_directories=True).working_tree_dir

    assert (os.path.exists(root)), "root directory path must exist"
    sources = get_sources(root, tbpath)

    # if pymodule is none, assume that the python module name is test+<name of the top module>.
    if(pymodule is None):
        pymodule = "test_" + top

    # Create the expected makefile so cocotb-test won't complain.
    sim_build = "lint"
    if(not os.path.exists("lint")):
       os.mkdir("lint")

    with open("lint/Vtop.mk", 'w') as fd:
        fd.write("all:")

    make_args = ["-n"]
    compile_args += ["--lint-only"]
 
    run(verilog_sources=sources,
        simulator=simulator,
        toplevel=top,
        module=pymodule,
        compile_args=compile_args,
        sim_build=sim_build,
        timescale=timescale,
        parameters=params,
        defines=defs,
        make_args=make_args,
        compile_only=True)


def get_files_from_filelist(p, n):
    """ Get a list of files from a json filelist.

    Arguments:
    p -- Path to the directory that contains the .json file
    n -- name of the .json file to read.
    """
    n = os.path.join(p, n)
    with open(n) as filelist:
        files = json.load(filelist)["files"]
    return files

def get_sources(r, p):
    """ Get a list of source file paths from a json filelist.

    Arguments:
    r -- Absolute path to the root of the repository.
    p -- Absolute path to the directory containing filelist.json
    """
    sources = get_files_from_filelist(p, "filelist.json")
    sources = [os.path.join(r, f) for f in sources]
    return sources

def get_top(p, n="filelist.json"):
    """ Get the name of the top level module from a filelist.json.

    Arguments:
    p -- Absolute path to the directory containing json filelist
    n -- Name of the json filelist, defaults to filelist.json
    """
    return get_top_from_filelist(p, "filelist.json")

def get_top_from_filelist(p, n):
    """ Get the name of the top level module a json filelist.

    Arguments:
    p -- Absolute path to the directory containing filelist.json
    n -- name of the .json file to read.
    """
    n = os.path.join(p, n)
    with open(n) as filelist:
        top = json.load(filelist)["top"]
        return top

def get_param_string(parameters):
    """ Get a string of all the parameters concatenated together.

    Arguments:
    parameters -- a list of key value pairs
    """
    return "_".join(("{}={}".format(*i) for i in parameters.items()))


def assert_resolvable(s):
    assert s.value.is_resolvable, f"Unresolvable value in {s._path} (x or z in some or all bits) at Time {get_sim_time(units='ns')}ns."

async def clock_start_sequence(clk_i, period=1, unit='ns'):
    # Set the clock to Z for 10 ns. This helps separate tests.
    clk_i.value = LogicArray(['z'])
    await Timer(10, 'ns')

    # Unrealistically fast clock, but nice for mental math (1 GHz)
    c = Clock(clk_i, period, unit)

    # Start the clock (soon). Start it low to avoid issues on the first RisingEdge
    cocotb.start_soon(c.start(start_high=False))

async def reset_sequence(clk_i, reset_i, cycles, FinishClkFalling=True, active_level=True):
    reset_i.setimmediatevalue(not active_level)

    # Always assign inputs on the falling edge
    await FallingEdge(clk_i)
    reset_i.value = active_level

    await ClockCycles(clk_i, cycles)

    # Always assign inputs on the falling edge
    await FallingEdge(clk_i)
    reset_i.value = not active_level

    reset_i._log.debug("Reset complete")

    # Always assign inputs on the falling edge
    if (not FinishClkFalling):
        await RisingEdge(clk_i)

async def delay_cycles(dut, ncyc, polarity):
    for _ in range(ncyc):
        if(polarity):
            await RisingEdge(dut.clk_i)
        else:
            await FallingEdge(dut.clk_i)
