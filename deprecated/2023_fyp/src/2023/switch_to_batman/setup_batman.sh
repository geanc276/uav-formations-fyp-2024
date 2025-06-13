#!/bin/bash

sudo apt install libnl-3-dev libnl-genl-3-dev

sudo apt install build-essential

git clone https://git.open-mesh.org/batctl.git

cd ~/uav-formations/src/2023/switch_to_batman/batctl

sudo make install

cd ..

sed -i -e 's/add .*\/24/add '$1'\/24/' switch_batman.sh

sudo cp switch_batman.sh /usr/bin/switch_batman.sh

sudo cp switch_batman.service /etc/systemd/system/switch_batman.service

systemctl enable switch_batman.service
