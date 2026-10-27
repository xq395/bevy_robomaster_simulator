@echo off
chcp 65001 >nul
setlocal
title RMU sim + operator terminal
rem One-click start: Godot sim + RMU engine + terminal (http://127.0.0.1:5002/)
rem Args are passed to run_sim_rmu.ps1, e.g.
rem   run_sim_rmu.bat -NoWeb           sim + engine only
rem   run_sim_rmu.bat -NoOpen          do not open the browser
rem   run_sim_rmu.bat -CustomSize none disable the 0x0310 stream
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_sim_rmu.ps1" %*
if errorlevel 1 (
    echo.
    echo Start failed - see the error above. Usual causes: wrong Godot path, port already in use.
    pause
)
