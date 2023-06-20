#!/bin/bash

#  Copyright (C) 2023 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

set -e

if [ -z $evc_token ];
    then 
        echo "No argument provided for evc_token, this script needs to be run with token"
        exit 1
fi

#download apt dependencies
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends firefox dbus-x11 x11-apps x11-utils x11-xserver-utils \
    xserver-xorg-video-dummy xserver-xorg-input-void xvfb libgl1-mesa-dri libgl1-mesa-glx libpulse0 \
    alsa-utils xterm wget build-essential libssl-dev libffi-dev python3-dev python3-pip curl unzip \
    python3.8 python3.8-dev python3.8-distutils python3.8-venv cmake sumo sumo-tools sumo-doc software-properties-common

sudo add-apt-repository -y ppa:sumo/stable

sudo dpkg --add-architecture i386
sudo apt-get update
sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386 -y
#update sumo version
sudo apt-get install -y sumo sumo-tools sumo-doc
sudo curl -H "Authorization: token ${evc_token}" -H 'Accept: application/vnd.github.v4.raw' -O -L https://api.github.com/repos/usdot-fhwa-stol/CARMASensitive/contents/evcfile.zip
sudo unzip evcfile.zip




cd evcfile && python3.8 -m pip install pyeos-0.1.1-py3-none-any.whl
