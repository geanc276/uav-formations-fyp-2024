# Set up B.A.T.M.A.N-adv

## B.A.T.M.A.N-adv Setup drones

To setup the ad-hoc network for the drones. Navigate to the '''uav-formations/src/2023/batman_setup''' directory and run the setup script with the desired IP.
'''bash
bash setup-batctl.sh <IP_address>
'''
Reboot the drone and the drone will now be on an ad-hoc network with the specified IP.

The IP can be changed at a later time by running the setup script with a different IP specified.


## B.A.T.M.A.N-adv Setup Laptop

To setup the ad-hoc network for the host laptop. Navigate to the '''uav-formations/src/2023/batman_setup''' directory and run the host setup script with the desired IP.
'''bash
bash setup-host-batctl.sh <IP_address>
'''
To switch the host laptop to the ad-hoc network run the switch to batman script.
'''bash
bash switch_batman.sh 
'''
To switch back to a managed network reboot the host laptop.

The IP can be changed at a later time by running the setup file with a different IP specified.
