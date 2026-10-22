@echo off
chcp 65001 >nul
setlocal
rem 一键部署：安装依赖 -> 起仿真 -> 起本机自瞄 -> 起客户端 -> 开页面
rem 想分开部署请直接跑 deploy.ps1，例如：
rem   deploy.ps1 -Component sim,aim -Action start
rem   deploy.ps1 -Component client -Action start
rem   deploy.ps1 -Action stop

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy.ps1" %*
set code=%errorlevel%

if "%~1"=="" pause
exit /b %code%
