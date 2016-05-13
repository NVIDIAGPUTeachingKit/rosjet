#!/bin/bash
parent_path=$( cd "$(dirname "${BASH_SOURCE}")" ; pwd -P )

cd $parent_path/..
platformio run --target upload --upload-port /dev/ttyACM0