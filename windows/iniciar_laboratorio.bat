@echo off
REM ==============================================================
REM  Inicia el Laboratorio Virtual ADS (Gazebo + ROS 2 en Docker)
REM  Requisito: Docker Desktop abierto ("Engine running").
REM ==============================================================
cd /d "%~dp0.."
echo Levantando el laboratorio...
docker compose up -d
if errorlevel 1 (
  echo.
  echo No se pudo iniciar. Revisa que Docker Desktop este abierto.
  pause
  exit /b 1
)
echo.
echo Esperando a que Gazebo arranque (unos 40 s)...
timeout /t 40 /nobreak >nul
start "" http://localhost:8080/profesor
echo.
echo Listo. Panel del profesor: http://localhost:8080/profesor
echo Alumnos:                   http://localhost:8080/
echo Para compartir por internet: E:\ngrok\ngrok.exe http 8080
pause
