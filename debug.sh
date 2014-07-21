#!/bin/sh
echo 0 > /sys/class/gpio/gpio17/value
sleep 0.20
echo 1 > /sys/class/gpio/gpio17/value
sleep 2

echo "Waiting for debugger..."
./arm-softmmu/qemu-system-arm -machine fernvale -pflash mt6260da_spinor-patched.bin -s -S
