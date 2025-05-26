#!/bin/bash

# cd ./Demo/CORTEX_LM3S811_GCC/

make clean

make

echo "--- Launching QEMU ---"
qemu-system-arm -machine lm3s811evb -kernel gcc/RTOSDemo.axf -serial stdio
# qemu-system-arm -machine lm3s811evb -kernel gcc/RTOSDemo.axf -serial pty

# # Creamos una tubería temporal para capturar la salida
# PIPE=$(mktemp -u)
# mkfifo "$PIPE"

# # Lanzamos QEMU y redirigimos la salida a la pipe
# qemu-system-arm \
#   -machine lm3s811evb \
#   -kernel gcc/RTOSDemo.axf \
#   -serial pty > "$PIPE" 2>&1 &

# # Esperamos a que QEMU imprima el PTY
# PTY=""
# while read line < "$PIPE"; do
#   echo "$line"  # opcional: ver salida de QEMU
#   if [[ "$line" =~ (/dev/pts/[0-9]+) ]]; then
#     PTY="${BASH_REMATCH[1]}"
#     break
#   fi
# done

# rm "$PIPE"

# # Conectamos con picocom si encontramos el PTY
# if [[ -n "$PTY" ]]; then
#   echo "Conectate con: picocom -b 19200 $PTY"
#   picocom --echo --imap lfcrlf -b 19200 "$PTY"
# else
#   echo "No se pudo obtener el PTY 😓"
# fi
