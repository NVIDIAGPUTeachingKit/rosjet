#!/bin/bash
parent_path=$( cd "$(dirname "${BASH_SOURCE}")" ; pwd -P )

cd $parent_path/../resources
platformio run --target upload --upload-port /dev/ttyACM0
