#!/bin/sh

BMP_PORT=`ls -1 /dev/cu.usbmodem????????1 | head -1`

mkdir -p built

echo "file blackmagic" > built/debug.gdb
echo "target extended-remote $BMP_PORT" >> built/debug.gdb
echo "monitor swdp_scan" >> built/debug.gdb
echo "attach 1" >> built/debug.gdb

arm-none-eabi-gdb --command=built/debug.gdb
