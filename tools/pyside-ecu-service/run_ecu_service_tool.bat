@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if %errorlevel%==0 (
    py -3 ecu_service_tool.py
) else (
    python ecu_service_tool.py
)

if errorlevel 1 (
    echo.
    echo ECU service tool exited with an error.
    pause
)

endlocal
