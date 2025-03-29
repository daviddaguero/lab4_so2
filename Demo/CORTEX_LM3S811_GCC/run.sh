
#cd ./Demo/CORTEX_LM3S811_GCC/

make clean

make

echo "--- Launching QEMU ---"
qemu-system-arm -machine lm3s811evb -kernel gcc/RTOSDemo.axf -serial stdio
