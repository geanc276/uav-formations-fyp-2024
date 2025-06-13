#!../../../venv/bin/python3
import os
import socket
from multiprocessing import Pool

import docker
import requests
import scapy.all as scapy
from flask import Flask

app = Flask(__name__)

image_name = "controller"
registry_port = 5000
certs = "src/2023/management_interface/backend/certs"
options = f"-i {certs}/id_rsa -o StrictHostKeyChecking=no"


def get_local_ip():
    """
    Gets this computers ip address
    """

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    local_ip = s.getsockname()[0]
    s.close()

    return local_ip


def scan():
    """
    Sends a packet to the broadcast MAC address to get details of all devices connected to the network
    """
    ip = f"{get_local_ip()}/24"
    arp_r = scapy.ARP(pdst=ip)
    br = scapy.Ether(dst='ff:ff:ff:ff:ff:ff')
    request = br / arp_r
    answered, unanswered = scapy.srp(request, timeout=3)
    return set(answer[1].psrc for answer in answered)


def interrogate_ip(ip):
    """
    Sends a packet to a particular ip to determine if it is a drone. If it replies, we assume it is one.
    """
    drone_server_port = 5000
    try:
        print("Testing", ip)
        http_response = requests.get(f"http://{ip}:{drone_server_port}/isDrone", timeout=2)
        return ip, http_response
    except requests.exceptions.ConnectionError:
        return ip, None


def interrogate_ips(ip_responses):
    """
    Checks a number of IPs to see if they are drones
    """
    with Pool(4) as pool:
        pool_result = pool.map(interrogate_ip, ip_responses)

    return map(lambda x: x[0], filter(lambda x: x[1] is not None, pool_result))


def make_certificate():
    """Runs a script to make a TLS certificate for use in transfer"""
    os.system(f"IP={get_local_ip()} CERT_PATH={certs} src/2023/management_interface/backend/make_cert.sh")


def copy_cert(ip):
    """Uses SCP to transfer a certificate over to a drone"""
    target = "root@" + ip
    os.system(f"scp {options} {certs}/domain.crt {target}:/usr/local/share/ca-certificates/drone-controller.crt")
    os.system(f"ssh {options} {target} update-ca-certificates")
    os.system(f"ssh {options} {target} systemctl restart docker")


def sync_time(ip):
    """
    Synchronises the laptops clock with the drones'. Important for certificate validation, and useful for logging
    """
    target = "root@" + ip
    os.system(f"ssh {options} {target} \"date -s '$(date)'\"")


def flatten(nested):
    """Turns a list of lists into a single list"""
    return [item for sublist in nested for item in sublist]


def get_docker_images():
    """
    Gets the docker images on the local machine that have a tag starting with csse-sdff.canterbury...
    This weeds out any other random images users might have installed
    """
    client = docker.from_env()

    all_tags = flatten(map(lambda image: image.tags, client.images.list()))
    filtered_tags = filter(lambda tag: tag.startswith("csse-sdff.canterbury.ac.nz"), all_tags)
    cut_down = map(lambda tag: tag[27:], filtered_tags)
    return list(cut_down)



def start_registry():
    """
    Starts the docker registry on the laptop, so that images can be pushed to it for transfer
    """
    client = docker.from_env()
    try:
        client.containers.get("registry").remove(force=True)
    except docker.errors.NotFound:
        pass
    client.containers.run("registry",
                          detach=True,
                          volumes=[f"{os.getcwd()}/{certs}:/certs"],
                          environment={"REGISTRY_HTTP_ADDR": "0.0.0.0:443",
                                       "REGISTRY_HTTP_TLS_CERTIFICATE": "/certs/domain.crt",
                                       "REGISTRY_HTTP_TLS_KEY": "/certs/domain.key"},
                          ports={443: 443},
                          name="registry")


def push_image(image):
    """
    Pushes an image to the local docker registry
    """
    client = docker.from_env()
    local_ip = get_local_ip()
    image = "csse-sdff.canterbury.ac.nz/" + image
    client.api.tag(image, f"{local_ip}/{image_name}")
    client.images.push(f"{local_ip}/{image_name}")

def build(path, name):
    """
    Builds a docker image locally
    """
    os.system(f"docker buildx build --progress=plain --platform linux/arm64/v8 {path} -t temp --load")
    os.system(f"docker tag temp csse-sdff.canterbury.ac.nz/{name}")


def pull_image(name):
    """
    Pulls an image from the virtual machine and stores it on the local machine
    """
    client = docker.from_env()
    client.login(registry="csse-sdff.canterbury.ac.nz", username="test", password="testpassword")
    client.images.pull(f"csse-sdff.canterbury.ac.nz/{name}")


def ssh_pull(ip):
    """
    SSHs into a drone and instructs it to pull the image stored in the registry
    """
    print("pulling onto", ip)
    local_ip = get_local_ip()
    os.system(f"ssh {options} ubuntu@{ip} docker pull {local_ip}/{image_name}")
    os.system(f"ssh {options} ubuntu@{ip} docker tag {local_ip}/{image_name} controller")


def launch_drone(ip, config):
    """
    SSHs into a drone and launches the docker container.
    """
    command = (f"ssh {options} ubuntu@{ip} docker run --network=host "
               f"--env ROS_MASTER_URI=http://icos-M3401QAu:11311/ --env CONFIG={config} "
               "--env DRONE_NAME=\$\(cat drone_name\) "
               f"--privileged controller")
    command = (f"ssh {options} ubuntu@{ip} docker run -d --network=host "
               f" --env CONFIG={config} "
               "--env DRONE_NAME=\$\(cat drone_name\) "
               f"--privileged controller")
    os.system(command)