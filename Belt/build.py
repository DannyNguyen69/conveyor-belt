import os
import shutil
import subprocess

APP_NAME = "ControllConveyor"
MAIN_FILE = "main.py"
ICON_FILE = None 
for folder in ["build", "dist"]:
    if os.path.exists(folder):
        shutil.rmtree(folder)
spec_file = f"{APP_NAME}.spec"
if os.path.exists(spec_file):
    os.remove(spec_file)
cmd = [
    "pyinstaller",
    "--noconfirm",
    "--onefile",   
    "--windowed",     
    "--name", APP_NAME,
]
if ICON_FILE:
    cmd += ["--icon", ICON_FILE]
cmd.append(MAIN_FILE)
subprocess.run(cmd)
