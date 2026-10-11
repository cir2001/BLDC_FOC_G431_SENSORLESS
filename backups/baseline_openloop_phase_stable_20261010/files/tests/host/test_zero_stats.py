#!/usr/bin/env python3
"""Compile current timer.c with register mocks; no hardware access or upload.
Run: python3 tests/host/test_zero_stats.py
Mocks do not model interrupt preemption, peripheral timing, or real DMA throughput.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
here = Path(__file__).resolve().parent
source = (root / 'lib/timer/timer.c').read_text()
a = source.index('void TIM1_PWM_Init(u16 arr)')
b = source.index('void SVPWM_Output_Standard(', a)
source = source[:a] + source[b:]
source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
header = (root / 'lib/timer/timer.h').read_text().replace('#include "sys.h"', '')
helpers = (here / 'zero_helpers.c').read_text()
a = helpers.index('static void fresh(')
unit = ((here / 'zero_mock.h').read_text() + header + helpers[:a] + source
        + helpers[a:] + (here / 'zero_stats_cases.c').read_text())
with tempfile.TemporaryDirectory(prefix='foc-zero-') as tmp:
    c = Path(tmp) / 'test.c'
    exe = Path(tmp) / 'test'
    c.write_text(unit)
    subprocess.run(['gcc', '-std=c11', '-Ofast', '-Wno-pointer-to-int-cast',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    str(c), '-lm', '-o', str(exe)], check=True)
    output = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
    def section(name, end):
        return output.split(name + "\n", 1)[1].split(end, 1)[0]
    def values(block, label, expected):
        line = next(line for line in block.splitlines() if label in line)
        actual = [float(v) for v in re.findall(r"(?:U|V|W|SUM)=(-?\d+\.\d+)", line)]
        assert len(actual) == 4, line
        assert all(abs(a-b) < 0.00006 for a, b in zip(actual, expected)), (line, expected)
    constant = section("REPORT_CONSTANT", "REPORT_CORRELATED")
    values(constant, "before mean", [0.75/-34.05, -0.5/-34.05, -0.75/-34.05, -0.5/-34.05])
    values(constant, "after mean", [2.75/-34.05, -1.5/-34.05, 3.25/-34.05, 4.5/-34.05])
    values(constant, "delta(after-before)", [2/-34.05, -1/-34.05, 4/-34.05, 5/-34.05])
    values(constant, "before p-p", [0, 0, 0, 0])
    correlated = section("REPORT_CORRELATED", "REPORT_PARTIAL")
    values(correlated, "before p-p", [4/34.05, 4/34.05, 0, 0])
    values(correlated, "after p-p", [4/34.05, 4/34.05, 0, 0])
    for block in (section("REPORT_PARTIAL", "REPORT_EMPTY"), section("REPORT_EMPTY", "PASS:")):
        assert "INCOMPLETE" in block and "delta(after-before)" not in block
    print("\n".join(line for line in output.splitlines() if line.startswith("PASS:")))
    print("PASS: reported means, signed deltas, phase/SUM peak-to-peak and INCOMPLETE suppression.")
