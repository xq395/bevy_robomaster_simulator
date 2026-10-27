"""Host-build the team's UI_print.c with MSVC (no vcvars needed) and dump the UART bytes it produces.

    python tools/ui0301_host/build.py [EC_UI_DIR]      -> tools/ui0301_host/out/harness.exe, frames.bin
    python tools/ui0301_preview.py tools/ui0301_host/out/frames.bin -o ui.png

EC_UI_DIR defaults to $RM_EC_UI_DIR or the team checkout's src/modules/referee/bsp_referee_UI.
Edit harness.c to mirror the calls the firmware makes.
"""
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
EC = Path(sys.argv[1] if len(sys.argv) > 1 else os.environ.get(
    "RM_EC_UI_DIR", r"D:\HNU_RM\代码栈\EC\rm\src\modules\referee\bsp_referee_UI"))
MSVC = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133")
SDK = Path(r"C:\Program Files (x86)\Windows Kits\10")
VER = "10.0.19041.0"

env = dict(os.environ)
env["PATH"] = str(MSVC / "bin/Hostx64/x64") + ";" + r"C:\Windows\System32"
env["INCLUDE"] = ";".join(str(p) for p in [MSVC / "include", SDK / "Include" / VER / "ucrt",
                                           SDK / "Include" / VER / "um", SDK / "Include" / VER / "shared"])
env["LIB"] = ";".join(str(p) for p in [MSVC / "lib/x64", SDK / "Lib" / VER / "ucrt/x64", SDK / "Lib" / VER / "um/x64"])
OUT.mkdir(exist_ok=True)
cmd = [str(MSVC / "bin/Hostx64/x64/cl.exe"), "/nologo", "/W0", "/TC", "/I", str(HERE / "stub"), "/I", str(EC),
       "/FI", str(HERE / "stub" / "msvc_compat.h"), str(EC / "UI_print.c"), str(EC / "BSP_CRC.c"),
       str(HERE / "harness.c"), "/Fe:" + str(OUT / "harness.exe"), "/Fo:" + str(OUT) + "\\"]
r = subprocess.run(cmd, cwd=OUT, env=env, capture_output=True, text=True, encoding="gbk", errors="replace")
print(r.stdout[-3000:], r.stderr[-2000:])
if r.returncode:
    sys.exit(r.returncode)
sys.exit(subprocess.run([str(OUT / "harness.exe")], cwd=OUT).returncode)
