#!/bin/sh

git pull

arduino-cli core install esp8266:esp8266 --config-file ./.arduino-cli-config.yml
arduino-cli compile --fqbn esp8266:esp8266:nodemcu imu-encoder-node
arduino-cli upload -p /dev/imu-encoder-node --fqbn esp8266:esp8266:nodemcu imu-encoder-node
