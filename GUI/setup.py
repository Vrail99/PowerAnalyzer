from cx_Freeze import setup, Executable
import sys

includefiles = ["acspic2.jpg", "EEPROM_cfg.txt",
                "rbo.png", "volatile_cfg.txt", "requirements.txt"]
includemodules = ["tkinter"]
excludemodules = []

build_exe_options = {
    'includes': includemodules,
    'excludes': excludemodules,
    'include_files': includefiles}

base = None

if sys.platform == "win32":
    base = "Win32GUI"

setup(name='Power Analyzer',
      version=2.2,
      description="A GUI for the Power Analyzer based on the ACS71020/ACS37800",
      options={'build_exe': build_exe_options},
      executables=[Executable("main.py", base)])
