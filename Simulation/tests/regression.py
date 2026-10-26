"""JHY: stdlib-only end-to-end checks. Run after build.ps1 -RunAll."""
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

BASE = Path(__file__).resolve().parents[1]
EXE = BASE / "build" / "mecanum_sim.exe"
if not EXE.exists():
    EXE = BASE / "build" / "mecanum_sim"
WORK = Path(tempfile.mkdtemp(prefix="regression_", dir=BASE / "build"))
checks = 0


def check(condition, message):
    global checks
    checks += 1
    assert condition, message


def run(*args, expect=0):
    result = subprocess.run([str(EXE), *map(str, args)], cwd=BASE, capture_output=True)
    check(result.returncode == expect, f"returncode {result.returncode}: {args}\n{result.stdout!r}\n{result.stderr!r}")
    return result


run("--self-test")
for scene in ("default", "slip", "timeout", "overheat", "estop", "saturation"):
    out = WORK / scene
    run("--config", f"config/{scene}.cfg", "--output", out, "--quiet")
    result = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    rows = list(csv.DictReader((out / "telemetry.csv").open(encoding="utf-8", newline="")))
    check(len(rows) == result["samples"], f"{scene}: row count")
    check(abs(float(rows[-1]["time_s"]) - result["duration_s"]) < 1e-8, f"{scene}: last sample time")
    check(len(rows[0]) == 50, f"{scene}: columns")
    check(all(math.isfinite(float(v)) for row in rows for key, v in row.items() if key != "fault"), f"{scene}: finite numeric CSV")
    check(all(float(a["time_s"]) < float(b["time_s"]) for a, b in zip(rows, rows[1:])), f"{scene}: monotonic time")
    check(all(abs(float(row[f"{w}_control"])) <= 1 for row in rows for w in ("LF", "LB", "RB", "RF")), f"{scene}: bounded control")
    if scene in ("timeout", "overheat", "estop"):
        fault_rows = [row for row in rows if row["fault"] != "OK"]
        check(bool(fault_rows), f"{scene}: detects fault")
        check(all(float(row[f"{w}_control"]) == 0 and float(row[f"{w}_target_radps"]) == 0 for row in fault_rows for w in ("LF", "LB", "RB", "RF")), f"{scene}: all drives stop")
        check(result["final_max_wheel_speed_radps"] < 1e-6, f"{scene}: coast to rest")
        check(abs(result["fault_time_s"] - (2.1 if scene == "timeout" else 2)) < 1e-8, f"{scene}: detection time")
    else:
        check(result["fault"] == "OK", f"{scene}: no unexpected fault")
    if scene == "saturation":
        check(result["saturated_samples"] == result["samples"], "all saturation steps counted")
        check(all(abs(float(row[f"{w}_target_radps"])) <= 15.000001 for row in rows for w in ("LF", "LB", "RB", "RF")), "limited target wheel speed")

# Deterministic seed must reproduce every CSV byte.
repeat = WORK / "repeat"
run("--config", "config/default.cfg", "--output", repeat, "--quiet")
check(hashlib.sha256((repeat / "telemetry.csv").read_bytes()).digest() == hashlib.sha256((WORK / "default/telemetry.csv").read_bytes()).digest(), "seed reproducibility")

# Reject malformed parameters and events before any output simulation.
for index, text in enumerate(("radius=0\n", "dt=nan\n", "unknown=1\n", "dt=.2\n", "emergency_time=12\n")):
    path = WORK / f"invalid_{index}.cfg"
    path.write_text(text, encoding="utf-8")
    run("--config", path, "--output", WORK / "invalid_output", expect=1)
for index, text in enumerate(("2,estop,1\n1,estop,1\n", "1,vx,nan\n", "12,estop,1\n")):
    event = WORK / f"invalid_event_{index}.csv"
    event.write_text(text, encoding="utf-8")
    path = WORK / f"invalid_event_{index}.cfg"
    path.write_text(f"events={event.as_posix()}\n", encoding="utf-8")
    run("--config", path, "--output", WORK / "invalid_output", expect=1)

# UTF-8 arguments and Chinese filesystem paths exercise real Windows entry/path conversion.
report = WORK / "中文实验报告.md"
initial = "# 麦轮实验报告\n署名：JHY\n"
report.write_text(initial, encoding="utf-8")
run("--edit-report", report, "append", "新增实验结论")
check("新增实验结论" in report.read_text(encoding="utf-8"), "Chinese append")
run("--edit-report", report, "replace", "新增实验结论", "修改后的结果")
check("修改后的结果" in report.read_text(encoding="utf-8"), "Chinese replace")
run("--edit-report", report, "delete", "修改后的结果")
check("修改后的结果" not in report.read_text(encoding="utf-8"), "Chinese delete")
before = report.read_bytes()
run("--edit-report", report, "delete", "不存在的内容", expect=1)
check(report.read_bytes() == before, "unmatched edit preserves original")
fragment = WORK / "附加内容.txt"
fragment.write_text("从UTF8文件添加的中文", encoding="utf-8")
run("--edit-report", report, "append", "@" + str(fragment))
check("从UTF8文件添加的中文" in report.read_text(encoding="utf-8"), "UTF-8 text file argument")
invalid_utf8 = WORK / "bad_utf8.md"
invalid_utf8.write_bytes(b"\xff\xff")
run("--edit-report", invalid_utf8, "append", "safe", expect=1)
check(invalid_utf8.read_bytes() == b"\xff\xff", "invalid UTF-8 rejected without modification")

print(f"PASS: {checks} end-to-end checks; artifacts: {WORK}")
