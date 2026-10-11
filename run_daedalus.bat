@echo off
setlocal
set TALOS_SHM_DIR=F:\talos_shm
set PATH=F:\bevy_robomaster_simulator\target\debug;%PATH%
cd /d F:\bevy_robomaster_simulator
echo Starting daedalus...
echo TALOS_SHM_DIR=%TALOS_SHM_DIR%
echo Working dir=%CD%
target\debug\daedalus.exe
echo EXIT CODE=%ERRORLEVEL%
pause