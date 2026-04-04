@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if %errorlevel%==0 (
    py -3 motor_simulator_pyside.py
) else (
    python motor_simulator_pyside.py
)

if errorlevel 1 (
    echo.
    echo Motor simulator exited with an error.
    pause
)

endlocal
