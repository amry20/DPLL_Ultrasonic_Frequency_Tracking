"""Run hardware-independent regression checks using Zig's C++ compiler."""
import argparse
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", default="zig")
args = parser.parse_args()
repo = Path(__file__).resolve().parent.parent
output = repo / ".pio" / "native-tests"
output.mkdir(parents=True, exist_ok=True)
binary = output / ("regression.exe" if sys.platform == "win32" else "regression")
subprocess.run([
    args.compiler, "c++", "-std=c++17", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
    "-I" + str(repo / "test/native/stubs"), "-I" + str(repo / "include"),
    str(repo / "test/native/regression.cpp"), str(repo / "src/com.cpp"),
    "-o", str(binary),
], check=True)
subprocess.run([str(binary)], check=True)
