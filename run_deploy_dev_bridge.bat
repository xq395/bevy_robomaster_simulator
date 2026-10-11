@echo off
setlocal
chcp 65001 >nul

if not defined TALOS_SHM_DIR set "TALOS_SHM_DIR=F:\talos_shm"
if not defined DEPLOY_DEV_DIR set "DEPLOY_DEV_DIR=D:\HNU_RM\代码栈\deploy_dev"
if not defined DEPLOY_DEV_PYTHON set "DEPLOY_DEV_PYTHON=C:\Users\34741\AppData\Local\Programs\Python\Python311\python.exe"

if not exist "%DEPLOY_DEV_PYTHON%" (
    echo Python not found: %DEPLOY_DEV_PYTHON%
    exit /b 2
)
if not exist "%DEPLOY_DEV_DIR%\config\sp_standard.yaml" (
    echo deploy_dev not found: %DEPLOY_DEV_DIR%
    exit /b 2
)

echo Starting deploy_dev Talos bridge in AIM-ONLY mode.
echo Fire stays disabled unless both --allow-fire is passed and fire_gate.enabled=true.
echo Press F5 in Daedalus to enable command following.
"%DEPLOY_DEV_PYTHON%" "%~dp0tools\deploy_dev_talos_bridge.py" ^
    --deploy-dir "%DEPLOY_DEV_DIR%" ^
    --shm-dir "%TALOS_SHM_DIR%" ^
    --send %*

exit /b %ERRORLEVEL%