#!/bin/bash
while true; do
    cat /sys/class/tacho-motor/tacho-motor1/position >> log2.txt
done

