import os

UPLOAD_DIRECTORY = '/home/swarm_software'
EXECUTABLES_DIR = os.path.join(UPLOAD_DIRECTORY, 'executables')
LOGS_DIR = os.path.join(UPLOAD_DIRECTORY, 'logs')
ROS_LOGS_DIR = os.path.join(LOGS_DIR, 'ros')
UORB_LOGS_DIR = os.path.join(LOGS_DIR, 'uorb')
ANY_FILE_DIR = os.path.join(UPLOAD_DIRECTORY, 'any_file')
LAUNCH_DIR = os.path.join(UPLOAD_DIRECTORY, 'launch')

META_DATA_DIR = os.path.join(UPLOAD_DIRECTORY, 'meta_data')
CONFIG_DIR = os.path.join(UPLOAD_DIRECTORY, 'config')
STATE_DIR = os.path.join(UPLOAD_DIRECTORY, 'state')


SW_INSTALL_DIR = '/home/swarm_software/pls'


# Ensure directories exist
os.makedirs(META_DATA_DIR, exist_ok=True)
os.makedirs(CONFIG_DIR, exist_ok=True)
os.makedirs(STATE_DIR, exist_ok=True)