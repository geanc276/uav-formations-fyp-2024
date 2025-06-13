#!/bin/bash

roslaunch px4_tf_real.launch &
cd controller || exit
python3 generatedController.py configs/"$CONFIG"/"$DRONE_NAME".json &
python3 controller.py configs/"$CONFIG"/"$DRONE_NAME".json