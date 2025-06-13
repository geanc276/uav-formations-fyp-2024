"""
Launch several drones in configuration folder
"""

import subprocess
import sys
import os
import time


conf_folder = sys.argv[1]
conf_files = [os.path.join(conf_folder, f) for f in os.listdir(conf_folder) if os.path.isfile(os.path.join(conf_folder, f))]


processes = []
for conf in conf_files: 
    if conf[-5:] == '.json': 
        processes.append(subprocess.Popen(['python3', 'controller.py', conf]))
        processes.append(subprocess.Popen(['python3', 'generatedController.py', conf]))

# Spin until keyboard interrupt
while True: 
    try: 
        time.sleep(0.1)
    except KeyboardInterrupt: 
        break

# Kill all subprocesses before exiting
for process in processes: 
    process.kill()
