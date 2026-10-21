#!/bin/bash
# Terminal web para los alumnos (ttyd). Cada conexión abre su propia sesión.
# La página abre terminal/?arg=<carpeta> para entrar a la carpeta personal del alumno.
mkdir -p /trabajo/alumnos
exec ttyd --writable --url-arg --port 7681 --base-path /terminal --max-clients 60 \
  -t fontSize=14 -t 'titleFixed=Terminal ADS' \
  -t 'theme={"background":"#0a0e13","foreground":"#d7e1ea","cursor":"#f5a524"}' \
  bash /lab/practicas/sesion.sh
