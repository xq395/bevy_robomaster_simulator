@echo off
setlocal
cd /d F:\bevy_robomaster_simulator
set PATH=F:\bevy_robomaster_simulator\target\debug;%PATH%
set RUST_LOG=info
echo CWD=%CD%
echo assets exists?
dir assets\vehicle.glb
echo Starting...
target\debug\daedalus.exe
echo EXIT=%ERRORLEVEL%
pause
