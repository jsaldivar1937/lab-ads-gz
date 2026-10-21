@echo off
cd /d "%~dp0.."
docker compose logs -f --tail 50 sim
