#!/usr/bin/env python3

import os
import subprocess
import shutil
import tempfile
import base64
import ipaddress
import json
import socket
import logging
from functools import wraps
from threading import Thread
from zipfile import ZipFile

# Flask Imports
from flask import Flask, request, jsonify, send_from_directory, send_file
from werkzeug.utils import secure_filename

# Inotify Imports (Linux-only)
# pip install inotify
import inotify.adapters


# Cryptography Imports (for RSA decryption)
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.primitives import serialization, hashes

# Directory Paths (adjust as needed)
from messaging_services.utils.directories import (
    EXECUTABLES_DIR,
    LOGS_DIR,
    ROS_LOGS_DIR,
    UORB_LOGS_DIR,
    ANY_FILE_DIR,
    LAUNCH_DIR,
    META_DATA_DIR,
    CONFIG_DIR,
    STATE_DIR,
    SW_INSTALL_DIR
)

###############################################################################
# FLASK APP CONFIG
###############################################################################

app = Flask(__name__)
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024  # 50 MB limit

HOSTNAME = socket.gethostname()
ENABLE_LOGGING = False

USERNAME = 'drone'
PASSWORD = 'pass'
DRONE_ID = os.getenv('DRONE_ID', '1')

# Ensure directories exist
os.makedirs(EXECUTABLES_DIR, exist_ok=True)
os.makedirs(META_DATA_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)
os.makedirs(ROS_LOGS_DIR, exist_ok=True)
os.makedirs(UORB_LOGS_DIR, exist_ok=True)
os.makedirs(ANY_FILE_DIR, exist_ok=True)
os.makedirs(LAUNCH_DIR, exist_ok=True)
os.makedirs(CONFIG_DIR, exist_ok=True)
os.makedirs(STATE_DIR, exist_ok=True)
os.makedirs(SW_INSTALL_DIR, exist_ok=True)

# Simple logger/Null logger
if ENABLE_LOGGING:
    log_file_path = os.path.join(LOGS_DIR, 'app.log')
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s: %(message)s',
        handlers=[logging.FileHandler(log_file_path), logging.StreamHandler()]
    )
    logger = logging.getLogger(__name__)
else:
    class NullLogger:
        def debug(self, *args, **kwargs): pass
        def info(self, *args, **kwargs): pass
        def warning(self, *args, **kwargs): pass
        def error(self, *args, **kwargs): pass
        def critical(self, *args, **kwargs): pass
    logger = NullLogger()

###############################################################################
# RESPONSE CLASS
###############################################################################

class Response:
    def __init__(self, success=True, message='', data=None, status_code=200):
        self.success = success
        self.message = message
        self.data = data if data is not None else {}
        self.status_code = status_code
        self.hostname = HOSTNAME

    def to_flask_response(self):
        response_data = {
            'success': self.success,
            'message': self.message,
            'hostname': self.hostname
        }
        if self.data:
            response_data['data'] = self.data
        return jsonify(response_data), self.status_code

###############################################################################
# AUTH HELPERS
###############################################################################

def check_auth(username, password):
    return username == USERNAME and password == PASSWORD

def authenticate():
    return Response(success=False, message='Authentication required', status_code=401).to_flask_response()

def requires_auth(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        auth = request.authorization
        if not auth or not check_auth(auth.username, auth.password):
            return authenticate()
        return f(*args, **kwargs)
    return decorated

###############################################################################
# INOTIFY-BASED WATCHER (REPLACES POLLING)
###############################################################################

def inotify_wait_for_file(filepath, timeout=30):
    """
    Use inotify to wait for a specific file to appear instead of polling.
    Returns True if the file was found before timeout, otherwise False.
    """
    i = inotify.adapters.Inotify()
    directory = os.path.dirname(filepath)
    filename = os.path.basename(filepath)
    i.add_watch(directory)

    import time
    start_time = time.time()

    for event in i.event_gen(yield_nones=False):
        if time.time() - start_time > timeout:
            logger.info("Timed out waiting for file.")
            return False
        (_, type_names, path, f_name) = event
        # We specifically wait for the file to be closed after writing
        if f_name == filename and 'IN_CLOSE_WRITE' in type_names:
            logger.info(f"File {filename} appeared in {directory}")
            return True

    return False

###############################################################################
# MONITOR ZIP FILE (USING INOTIFY)
###############################################################################

def monitor_for_zip_file(filepath):
    """
    Waits for a ZIP file to appear using inotify instead of polling.
    Once found, unzips with up to 3 retry attempts.
    """
    if not inotify_wait_for_file(filepath, timeout=30):
        logger.info(f"No zip file detected in {LAUNCH_DIR} within 30 seconds.")
        return

    zip_filename = os.path.basename(filepath)
    time_wait = 5  # short wait to ensure file is fully written
    max_retries = 3

    logger.info(f"Zip file {zip_filename} detected. Waiting {time_wait}s for complete upload.")
    
    import time
    time.sleep(time_wait)

    for attempt in range(1, max_retries + 1):
        try:
            with ZipFile(filepath, 'r') as zip_ref:
                extract_path = os.path.join(LAUNCH_DIR, zip_filename.split(".")[0])
                os.makedirs(extract_path, exist_ok=True)
                zip_ref.extractall(extract_path)
                # Make all extracted files executable
                for root, dirs, files in os.walk(extract_path):
                    for file in files:
                        os.chmod(os.path.join(root, file), 0o755)
            logger.info(f"Extracted {zip_filename} to {extract_path}")
            return
        except Exception as e:
            logger.error(f"Error extracting {zip_filename}: {str(e)} (Attempt {attempt}/{max_retries})")
            if attempt >= max_retries:
                logger.error(f"Failed to extract {zip_filename} after {max_retries} attempts.")

###############################################################################
# ROUTES
###############################################################################

@app.route('/upload_zip', methods=['POST'])
@requires_auth
def upload_launch_zip():
    """
    Provides path for uploading the zip file, then spawns a thread that 
    uses inotify to wait for the file instead of polling/sleeping.
    """
    dir_path = "drone_launch"
    folder_path = os.path.join(LAUNCH_DIR, dir_path)
    filepath = os.path.join(LAUNCH_DIR, f'{dir_path}.zip')

    # Spawn a background thread to wait for and extract the zip
    monitoring_thread = Thread(target=monitor_for_zip_file, args=(filepath,), daemon=True)
    monitoring_thread.start()

    # Handle backup logic
    backup_dir_path = folder_path + '.bak'
    if os.path.exists(backup_dir_path):
        shutil.rmtree(backup_dir_path)
    if os.path.exists(folder_path):
        os.rename(folder_path, backup_dir_path)

    return Response(
        success=True,
        message='Filepath returned successfully',
        data={'filepath': filepath}
    ).to_flask_response()


@app.route('/upload', methods=['POST'])
@requires_auth
def upload_and_process():
    """
    Handles file uploads with a 'command' parameter.
    """
    command = request.form.get('command')
    if 'file' not in request.files or not command:
        return Response(success=False, message='Command and file are required', status_code=400).to_flask_response()

    file = request.files['file']
    if file.filename == '':
        return Response(success=False, message='No selected file', status_code=400).to_flask_response()

    filename = secure_filename(file.filename)

    if command == 'upload_new_json':
        filepath = os.path.join(META_DATA_DIR, filename)
        file.save(filepath)
        process_json_file(filepath)
        return Response(success=True, message=f'JSON {filename} uploaded/processed', status_code=200).to_flask_response()

    elif command == 'upload_new_executable':
        filepath = os.path.join(EXECUTABLES_DIR, filename)
        file.save(filepath)
        os.chmod(filepath, 0o755)
        return Response(success=True, message=f'Executable {filename} uploaded', status_code=200).to_flask_response()

    elif command in ['upload_config', 'upload_state']:
        # Reuse backup logic
        directory = CONFIG_DIR if command == 'upload_config' else STATE_DIR
        return handle_upload_with_backup(directory, filename, file)

    else:
        return Response(success=False, message=f'Unknown command: {command}', status_code=400).to_flask_response()


def handle_upload_with_backup(directory, filename, file):
    backup_dir = os.path.join(directory, '.bak')
    os.makedirs(backup_dir, exist_ok=True)
    try:
        # Backup existing files
        for existing_file in os.listdir(directory):
            existing_path = os.path.join(directory, existing_file)
            if os.path.isfile(existing_path):
                os.replace(existing_path, os.path.join(backup_dir, existing_file))

        # Save the new file
        filepath = os.path.join(directory, filename)
        file.save(filepath)
        return Response(
            success=True,
            message=f'File {filename} uploaded and backup created',
            status_code=200
        ).to_flask_response()
    except Exception as e:
        return Response(success=False, message=str(e), status_code=500).to_flask_response()


def process_json_file(filepath):
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        # Add custom logic here
        # This example just appends to a JSON processing log
        log_file = os.path.join(LOGS_DIR, 'json_processing.log')
        with open(log_file, 'a') as log_f:
            log_f.write(f"Processed {os.path.basename(filepath)}: {json.dumps(data)}\n")
    except json.JSONDecodeError as e:
        log_error(f"JSON decode error: {str(e)}")


def log_error(message):
    log_file = os.path.join(LOGS_DIR, 'error.log')
    with open(log_file, 'a') as f:
        f.write(f"{message}\n")


@app.route('/start_executable', methods=['POST'])
@requires_auth
def start_executable():
    data = request.get_json()
    if not data or 'executable_name' not in data:
        return Response(success=False, message='Executable name required', status_code=400).to_flask_response()

    executable_name = data['executable_name']
    executable_path = os.path.join(EXECUTABLES_DIR, executable_name)

    if not os.path.isfile(executable_path):
        return Response(success=False, message='Executable not found', status_code=404).to_flask_response()

    params = data.get('params', [])
    if not isinstance(params, list):
        return Response(success=False, message='Params must be a list', status_code=400).to_flask_response()

    log_file = os.path.join(LOGS_DIR, f'{executable_name}.log')
    try:
        with open(log_file, 'a') as f:
            process = subprocess.Popen([executable_path] + params, stdout=f, stderr=subprocess.STDOUT)
        return Response(success=True, message=f'Started PID {process.pid}', status_code=200).to_flask_response()
    except Exception as e:
        return Response(success=False, message=str(e), status_code=500).to_flask_response()


@app.route('/upload_any_file', methods=['POST'])
@requires_auth
def upload_any_file():
    if 'file' not in request.files:
        return Response(success=False, message='No file part', status_code=400).to_flask_response()

    file = request.files['file']
    if file.filename == '':
        return Response(success=False, message='No selected file', status_code=400).to_flask_response()

    filename = secure_filename(file.filename)
    filepath = os.path.join(ANY_FILE_DIR, filename)

    try:
        file.save(filepath)
        return Response(success=True, message=f'{filename} uploaded', status_code=200).to_flask_response()
    except Exception as e:
        return Response(success=False, message=str(e), status_code=500).to_flask_response()


def get_services():
    return [
        'ros_bag_movement.service',
        'micro_xrce_dds_agent.service',
        'ros2_launch.service'
    ]

def get_target_services():
    return [
        'target_launch.service',
        'micro_xrce_dds_agent.service'
    ]

@app.route('/launch', methods=['POST'])
@requires_auth
def launch():
    try:
        data = request.get_json()
        logger.info("Received launch request with data: %s", data)
        
        if not data or data.get('command', '').upper() != 'LAUNCH':
            logger.error("Invalid or missing command in request data")
            return Response(success=False, message='Invalid or missing command', status_code=400).to_flask_response()

        drone_id = DRONE_ID
        num_drones = data.get('num_drones', 1)

        # Validate
        if not isinstance(num_drones, int) or num_drones <= 0:
            logger.error("'num_drones' must be a positive integer")
            return Response(success=False, message="'num_drones' must be positive int", status_code=400).to_flask_response()

        services = get_services()
        for service in services:
            if service.split(".")[1] not in ['service', 'target']:
                logger.error("Invalid service name: %s", service)
                return Response(success=False, message=f"Invalid service name: {service}", status_code=400).to_flask_response()

        # Create drop-in
        for service in services:
            drop_in_dir = f"/etc/systemd/system/{service}.d"
            os.makedirs(drop_in_dir, exist_ok=True)
            drop_in_file = os.path.join(drop_in_dir, 'env.conf')

        try:
            drone_id = int(drone_id)
            num_drones = int(num_drones)
        except ValueError:
            logger.error("'drone_id' or 'num_drones' must be integers")
            return Response(success=False, message="'drone_id'/'num_drones' must be int", status_code=400).to_flask_response()

        env_vars = f"Environment=DRONE_ID={drone_id}\nEnvironment=NUM_DRONES={num_drones}\n"
        with open(drop_in_file, 'w') as f:
            f.write(env_vars)
        logger.info("Drop-in file created with environment variables: %s", env_vars)

        subprocess.run(['sudo', 'systemctl', 'daemon-reload'], check=True)
        logger.info("Systemd daemon reloaded")

        for service in services:
            subprocess.run(['sudo', 'systemctl', 'start', service], check=True)
            logger.info("Started service: %s", service)

        import time
        time.sleep(3)  # short wait to check statuses

        service_status = {}
        for service in services:
            try:
                result = subprocess.run(
                    ['sudo', 'systemctl', 'is-active', service],
                    check=True, stdout=subprocess.PIPE, text=True
                )
                service_status[service] = result.stdout.strip()
                logger.info("Service %s status: %s", service, result.stdout.strip())
            except subprocess.CalledProcessError:
                service_status[service] = 'inactive'
                logger.error("Service %s is inactive", service)

        # Example check
        if (service_status.get('micro_xrce_dds_agent.service') == 'active' and
                service_status.get('ros2_launch.service') == 'active'):
            logger.info("All services launched successfully")
            return Response(success=True, message='Services launched', data=service_status, status_code=200).to_flask_response()
        else:
            logger.error("Failed to launch all services: %s", service_status)
            return Response(success=False, message='Failed to launch all services', data=service_status, status_code=500).to_flask_response()

    except Exception as e:
        logger.exception("Exception occurred during launch")
        return Response(success=False, message=str(e), status_code=500).to_flask_response()




@app.route('/launch_target', methods=['POST'])
@requires_auth
def launch_target():
    try:
        data = request.get_json()
        logger.info("Received launch target request with data: %s", data)
        
        if not data or data.get('command', '').upper() != 'LAUNCH':
            logger.error("Invalid or missing command in request data")
            return Response(success=False, message='Invalid or missing command', status_code=400).to_flask_response()

        # drone_id = DRONE_ID
        # num_drones = data.get('num_drones', 1)

      
        services = get_target_services()
        for service in services:
            if service.split(".")[1] not in ['service', 'target']:
                logger.error("Invalid service name: %s", service)
                return Response(success=False, message=f"Invalid service name: {service}", status_code=400).to_flask_response()



        drone_id = DRONE_ID

        # Create drop-in
        for service in services:
            drop_in_dir = f"/etc/systemd/system/{service}.d"
            os.makedirs(drop_in_dir, exist_ok=True)
            drop_in_file = os.path.join(drop_in_dir, 'env.conf')



        try:
            drone_id = int(drone_id)
        except ValueError:
            logger.error("'drone_id' must be integers")
            return Response(success=False, message="'drone_id'/'num_drones' must be int", status_code=400).to_flask_response()

        env_vars = f"Environment=DRONE_ID={drone_id}\n"
        with open(drop_in_file, 'w') as f:
            f.write(env_vars)
        logger.info("Drop-in file created with environment variables: %s", env_vars)

        subprocess.run(['sudo', 'systemctl', 'daemon-reload'], check=True)
        logger.info("Systemd daemon reloaded")

        for service in services:
            subprocess.run(['sudo', 'systemctl', 'start', service], check=True)
            logger.info("Started service: %s", service)

        import time
        time.sleep(3)  # short wait to check statuses

        service_status = {}
        for service in services:
            try:
                result = subprocess.run(
                    ['sudo', 'systemctl', 'is-active', service],
                    check=True, stdout=subprocess.PIPE, text=True
                )
                service_status[service] = result.stdout.strip()
                logger.info("Service %s status: %s", service, result.stdout.strip())
            except subprocess.CalledProcessError:
                service_status[service] = 'inactive'
                logger.error("Service %s is inactive", service)

        failed_services = []
        # Example check
        for service in services:
            if service_status.get(service) != 'active':
                logger.error("Failed to launch service: %s", service)
                failed_services.append(service)
            else:
                logger.info("Service %s launched successfully", service)

        if not failed_services:
            logger.info("All services launched successfully")
            return Response(success=True, message='Services launched', data=service_status, status_code=200).to_flask_response()
        else:
            logger.error("Failed to launch services: %s", failed_services)
            return Response(success=False, message='Failed to launch services', data=service_status, status_code=500).to_flask_response()

    except Exception as e:
        logger.exception("Exception occurred during launch")
        return Response(success=False, message=str(e), status_code=500).to_flask_response()




@app.route('/request/stop', methods=['POST'])
@requires_auth
def stop_services():
    services = get_services()
    services.extend(get_target_services())
    try:
        for service in services:
            subprocess.run(['sudo', 'systemctl', 'stop', service], check=True)
        return Response(success=True, message='Services stopped', status_code=200).to_flask_response()
    except Exception as e:
        return Response(success=False, message=str(e), status_code=500).to_flask_response()


@app.route('/logs/<path:filename>', methods=['GET'])
@requires_auth
def download_log(filename):
    return send_from_directory(LOGS_DIR, filename, as_attachment=True)


@app.errorhandler(401)
def custom_401(error):
    return Response(success=False, message='Authentication required', status_code=401).to_flask_response()

@app.errorhandler(404)
def not_found(error):
    return Response(success=False, message='Not Found', status_code=404).to_flask_response()

@app.errorhandler(500)
def internal_error(error):
    return Response(success=False, message='Internal Server Error', status_code=500).to_flask_response()


###############################################################################
# RSA KEY & RSYNC ENDPOINTS
###############################################################################

temporary_private_key = None

@app.route('/get_public_key', methods=['GET'])
@requires_auth
def get_public_key():
    global temporary_private_key
    private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    temporary_private_key = private_key

    public_key = private_key.public_key()
    pem_public_key = public_key.public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    return Response(success=True, message='Public key generated', data={'public_key': pem_public_key.decode()}, status_code=200).to_flask_response()


@app.route('/rsync_build', methods=['POST'])
@requires_auth
def rsync_build():
    global temporary_private_key
    data = request.get_json()
    if not data or 'ip' not in data or 'path' not in data or 'encrypted_password' not in data:
        return Response(success=False, message="Missing ip/path/encrypted_password", status_code=400).to_flask_response()

    ip = data['ip']
    path = data['path']
    user_name = data['username']
    enc_pass_b64 = data['encrypted_password']

    try:
        ipaddress.ip_address(ip)
    except ValueError:
        return Response(success=False, message='Invalid IP address', status_code=400).to_flask_response()

    if '..' in path or path.startswith('~'):
        return Response(success=False, message='Invalid path', status_code=400).to_flask_response()

    if not temporary_private_key:
        return Response(success=False, message='No private key available', status_code=500).to_flask_response()

    try:
        encrypted_password = base64.b64decode(enc_pass_b64)
        decrypted_password = temporary_private_key.decrypt(
            encrypted_password,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        ).decode('utf-8')
        temporary_private_key = None
    except Exception as e:
        return Response(success=False, message=f"Decryption failed: {str(e)}", status_code=500).to_flask_response()

    source = f"{user_name}@{ip}:{path}"
    destination = f"{SW_INSTALL_DIR}/"
    exclude_file = f"{SW_INSTALL_DIR}/swarm_ws/rsync_exclude.txt"

    if not os.path.isfile(exclude_file):
        rsync_command = [
            "sshpass", "-p", decrypted_password, "rsync",
            "-e", "ssh -o StrictHostKeyChecking=no",
            "-avz",
            "--exclude", "install/",
            "--exclude", "build/",
            "--exclude", "log/",
            source, destination
        ]
    else:
        rsync_command = [
            "sshpass", "-p", decrypted_password, "rsync",
            "-e", "ssh -o StrictHostKeyChecking=no",
            "-avz",
            f"--exclude-from={exclude_file}",
            source, destination
        ]

    try:
        result = subprocess.run(rsync_command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return Response(success=True, message='Files synchronized successfully', data={'output': result.stdout}, status_code=200).to_flask_response()
    except subprocess.CalledProcessError as e:
        return Response(success=False, message='rsync failed', data={'details': e.stderr}, status_code=500).to_flask_response()


@app.route('/start_build', methods=['POST'])
@requires_auth
def start_build():
    data = request.get_json()
    build_directory = f"{SW_INSTALL_DIR}/swarm_ws"

    if not os.path.isdir(build_directory):
        return Response(success=False, message=f"No build dir: {build_directory}", status_code=400).to_flask_response()

    source_cmd = "source /opt/ros/humble/setup.bash"
    build_cmd = "colcon build"
    command = f"{source_cmd} && {build_cmd}"
    try:
        result = subprocess.run(
            command, shell=True, executable='/bin/bash', cwd=build_directory,
            check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=600
        )
        return Response(success=True, message='Build complete', data={'output': result.stdout}, status_code=200).to_flask_response()
    except subprocess.CalledProcessError as e:
        return Response(success=False, message='colcon build failed', data={'details': e.stderr}, status_code=500).to_flask_response()


@app.route('/download_build', methods=['GET'])
@requires_auth
def download_build():
    build_dir = f"{SW_INSTALL_DIR}/swarm_ws/install"
    if not os.path.isdir(build_dir):
        return Response(success=False, message=f"No install dir: {build_dir}", status_code=400).to_flask_response()

    with tempfile.TemporaryDirectory() as tmpdir:
        zip_filename = "install.zip"
        zip_path = os.path.join(tmpdir, zip_filename)
        try:
            shutil.make_archive(
                base_name=zip_path.replace('.zip', ''),
                format='zip',
                root_dir=build_dir
            )
            return send_file(zip_path, mimetype='application/zip', as_attachment=True, download_name=zip_filename)
        except Exception as e:
            return Response(success=False, message='Zip creation failed', data={'details': str(e)}, status_code=500).to_flask_response()


@app.route('/connected', methods=['POST'])
@requires_auth
def connected():

    command = "sudo systemctl restart status_pub.service"
    try:
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except subprocess.CalledProcessError as e:
        return Response(success=False, message='Command failed', data={'details': e.stderr}, status_code=500).to_flask_response

    return Response(success=True, message='Connected', status_code=200).to_flask_response()





@app.route('/request/ros', methods=['POST'])
@requires_auth
def request_ros():
    accepted_commands = ['START', 'STOP', 'LAND', 'PAUSE', 'RESUME']
    data = request.get_json()
    if not data or 'command' not in data:
        return Response(success=False, message="Missing 'command'", status_code=400).to_flask_response()

    command = data['command'].upper()
    if command not in accepted_commands:
        return Response(success=False, message='Unsupported command', status_code=400).to_flask_response()

    return swarm_state_command(command)


def swarm_state_command(state_):
    state = state_.lower()
    src_cmd = "source /opt/ros/humble/setup.bash"
    ros_cmd = f"ros2 topic pub -1 /user_input/state std_msgs/String \"data: '{state}'\""
    full_cmd = f"{src_cmd} && {ros_cmd}"
    try:
        result = subprocess.run(full_cmd, shell=True, executable='/bin/bash',
                               check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return Response(success=True, message='State command complete', data={'output': result.stdout}, status_code=200).to_flask_response()
    except subprocess.CalledProcessError as e:
        return Response(success=False, message='Command failed', data={'details': e.stderr}, status_code=500).to_flask_response()


###############################################################################
# OPTIONS METHODS FOR ENDPOINTS
###############################################################################

@app.route('/upload_zip', methods=['OPTIONS'])
@requires_auth
def upload_zip_options():
    info = {
        "description": "Upload a zip file for drone launch. Monitors file using inotify and extracts contents.",
        "method": "POST",
        "required_parameters": "None. The zip file should be placed at the designated filepath.",
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/upload', methods=['OPTIONS'])
@requires_auth
def upload_options():
    info = {
        "description": "Handles file uploads with a 'command' parameter to process the file accordingly.",
        "method": "POST",
        "required_parameters": {
            "command": "One of 'upload_new_json', 'upload_new_executable', 'upload_config', 'upload_state'",
            "file": "The file to be uploaded."
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/start_executable', methods=['OPTIONS'])
@requires_auth
def start_executable_options():
    info = {
        "description": "Starts an executable on the server.",
        "method": "POST",
        "required_parameters": {
            "executable_name": "Name of the executable file",
            "params": "Optional list of parameters to pass to the executable"
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/upload_any_file', methods=['OPTIONS'])
@requires_auth
def upload_any_file_options():
    info = {
        "description": "Upload any file to the designated directory.",
        "method": "POST",
        "required_parameters": {
            "file": "The file to be uploaded."
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/launch', methods=['OPTIONS'])
@requires_auth
def launch_options():
    info = {
        "description": "Launch services with provided pardameters.",
        "method": "POST",
        "required_parameters": {
            "command": "Must be 'LAUNCH'",
            "num_drones": "Optional, number of drones (positive integer)"
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/launch_target', methods=['OPTIONS'])
@requires_auth
def launch_target_options():
    info = {
        "description": "Launch target services with provided command.",
        "method": "POST",
        "required_parameters": {
            "command": "Must be 'LAUNCH'"
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/request/stop', methods=['OPTIONS'])
@requires_auth
def stop_services_options():
    info = {
        "description": "Stop all running services.",
        "method": "POST",
        "required_parameters": "None",
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/logs/<path:filename>', methods=['OPTIONS'])
@requires_auth
def download_log_options(filename):
    info = {
        "description": "Download a specific log file.",
        "method": "GET",
        "required_parameters": {
            "filename": "The log file name (provided as a URL path parameter)"
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/get_public_key', methods=['OPTIONS'])
@requires_auth
def get_public_key_options():
    info = {
        "description": "Retrieve a generated RSA public key.",
        "method": "GET",
        "required_parameters": "None",
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/rsync_build', methods=['OPTIONS'])
@requires_auth
def rsync_build_options():
    info = {
        "description": "Synchronize build files from a remote server using rsync.",
        "method": "POST",
        "required_parameters": {
            "ip": "IP address of the remote server",
            "path": "Path to the build directory on the remote server",
            "username": "Username for the remote server",
            "encrypted_password": "RSA encrypted password in base64 format"
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/start_build', methods=['OPTIONS'])
@requires_auth
def start_build_options():
    info = {
        "description": "Start the build process using colcon build.",
        "method": "POST",
        "required_parameters": "None",
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/download_build', methods=['OPTIONS'])
@requires_auth
def download_build_options():
    info = {
        "description": "Download the built install directory as a zip file.",
        "method": "GET",
        "required_parameters": "None",
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/connected', methods=['OPTIONS'])
@requires_auth
def connected_options():
    info = {
        "description": "Restart the status publisher service to indicate connection.",
        "method": "POST",
        "required_parameters": "None",
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


@app.route('/request/ros', methods=['OPTIONS'])
@requires_auth
def request_ros_options():
    info = {
        "description": "Send a command to ROS to change the drone state.",
        "method": "POST",
        "required_parameters": {
            "command": "One of 'START', 'STOP', 'LAND', 'PAUSE', 'RESUME'"
        },
        "authentication": "Basic Auth required"
    }
    return jsonify(info), 200


###############################################################################
# MAIN
###############################################################################

if __name__ == '__main__':
    # IMPORTANT: Disable debug mode for lower CPU usage
    app.run(host='0.0.0.0', port=8001, debug=False)