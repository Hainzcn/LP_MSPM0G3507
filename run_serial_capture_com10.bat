@echo off
cd /d "%~dp0"
python "tools\motor_calib\serial_capture.py" --port COM10 --out "tools\motor_calib\log.txt"
pause
