"""Write sources.mk for the GCC build from the Keil project file list (MDK-ARM/RosRobotControllerM4.uvprojx).

Keil-only files are swapped for their GCC equivalents:
  MDK-ARM/startup_stm32f407xx.s (armasm)        -> Core/Startup/startup_stm32f407vetx.s
  FreeRTOS portable/RVDS/ARM_CM4F/port.c        -> portable/GCC/ARM_CM4F/port.c
  Hiwonder/Misc/syscall.c (#error if not ARMCC) -> Core/Src/syscalls.c + Core/Src/sysmem.c (newlib stubs)

    python3 gen_sources.py <fw25-source-dir> > sources.mk
"""
import os
import re
import sys

root = sys.argv[1]
xml = open(os.path.join(root, "MDK-ARM", "RosRobotControllerM4.uvprojx"), encoding="utf-8", errors="replace").read()
c_files, asm_files = [], ["Core/Startup/startup_stm32f407vetx.s"]
for path in re.findall(r"<FilePath>(.*?)</FilePath>", xml):
    p = os.path.normpath(os.path.join("MDK-ARM", path.replace("\\", "/"))).replace("\\", "/")
    if not p.endswith(".c"):
        continue
    if p.endswith("Hiwonder/Misc/syscall.c"):
        continue
    p = p.replace("portable/RVDS/ARM_CM4F", "portable/GCC/ARM_CM4F")
    if not os.path.isfile(os.path.join(root, p)):
        sys.exit(f"missing source listed in the Keil project: {p}")
    c_files.append(p)
c_files += ["Core/Src/syscalls.c", "Core/Src/sysmem.c"]

inc = set()
skip = ("portable/RVDS", "portable/IAR", "portable/Keil", "Drivers/CMSIS/DSP", "Drivers/CMSIS/NN", "Backup", "USB_HOST",
        "STM32_USB_Host_Library", "Templates", "RTOS2/Template", "CMSIS/RTOS/Template")
for d in ("Core", "Drivers", "Middlewares", "Third_Party", "Hiwonder"):
    for dirpath, _, files in os.walk(os.path.join(root, d)):
        rel = os.path.relpath(dirpath, root).replace("\\", "/")
        if any(s in rel for s in skip):
            continue
        if "portable" in rel and not (rel.endswith("GCC/ARM_CM4F") or rel.endswith("MemMang")):
            continue
        if any(f.endswith(".h") for f in files):
            inc.add(rel)

print("C_SOURCES = \\")
print(" \\\n".join(f"  {f}" for f in c_files))
print("\nASM_SOURCES = " + " ".join(asm_files))
print("\nC_INCLUDES = \\")
print(" \\\n".join(f"  -I{d}" for d in sorted(inc)))
