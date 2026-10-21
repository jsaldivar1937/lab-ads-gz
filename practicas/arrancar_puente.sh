#!/bin/bash
# Puente de solo lectura: dominio del laboratorio (7) → dominio de prácticas (8)
source /opt/ros/jazzy/setup.bash
cd /lab/practicas
exec python3 puente_lectura.py
