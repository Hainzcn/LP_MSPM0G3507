@echo off
cd /d "%~dp0"
python "tools\serial_plot.py" --port COM10
pause
