#!/bin/sh

git pull

arduino-cli compile --fqbn esp8266:esp8266:nodemcu motor-controller
arduino-cli upload -p /dev/motor-controller --fqbn esp8266:esp8266:nodemcu motor-controller
