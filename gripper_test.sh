#!/bin/bash

TOPIC="robot/gpio/cmd"

MSG1='{"mode":0,"left":{"position":0,"speed":200,"torque":0},"right":{"position":0,"speed":200,"torque":0}}'

MSG2='{"mode":0,"left":{"position":100,"speed":200,"torque":0},"right":{"position":100,"speed":200,"torque":0}}'

while true
do
    mosquitto_pub -h localhost -t "$TOPIC" -m "$MSG1"
    sleep 0.5

    mosquitto_pub -h localhost -t "$TOPIC" -m "$MSG2"
    sleep 0.5
done
