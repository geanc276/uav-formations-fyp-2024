
# def make_files():
#     drone_id = subprocess.check_output(["hostname"]).decode().strip().split('-')[-1]
#     config_folder_path = "/home/swarm_software/config"
#     state_folder_path = "/home/swarm_software/state"

#     config_content = f"""DRONE_ID={drone_id}\nDRONE_CONFIG_FOLDER_PATH={config_folder_path}\nDRONE_STATE_FOLDER_PATH={state_folder_path}\nROS_DOMAIN_ID=0\n
#     """

#     config_file_path = Path("/etc/drone_config.conf")
#     config_file_path.write_text(config_content)

#     subprocess.run(["sudo", "chown", "root:drone_swarm", str(config_file_path)], check=True)
#     subprocess.run(["sudo", "chmod", "660", str(config_file_path)], check=True)
#!/bin/bash

# ID=${DRONE_ID:-1}
ID=$DRONE_ID
echo "ID: $ID"

CONFIG_FOLDER_PATH=/home/ros/drones/uav-formations-fyp-2024/simulation/configs
STATE_FOLDER_PATH=/home/ros/drones/uav-formations-fyp-2024/simulation/config_states

CONF_PATH=/etc/drone_config.conf

if [ -f $CONF_PATH ]; then
    sudo rm $CONF_PATH
fi

touch $CONF_PATH

echo "DRONE_ID=$ID" > $CONF_PATH
echo "DRONE_CONFIG_FOLDER_PATH=$CONFIG_FOLDER_PATH" >> $CONF_PATH
echo "DRONE_STATE_FOLDER_PATH=$STATE_FOLDER_PATH" >> $CONF_PATH
echo "ROS_DOMAIN_ID=0" >> $CONF_PATH


