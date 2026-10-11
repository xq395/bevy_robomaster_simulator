@echo off
chcp 65001 >nul
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_operator.ps1" %*
if errorlevel 1 pause