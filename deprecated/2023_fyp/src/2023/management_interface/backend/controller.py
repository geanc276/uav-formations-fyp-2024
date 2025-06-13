import flask
from flask import Flask, request
from flask_cors import CORS

import requests

import main

app = Flask(__name__)
CORS(app)

drones = []
registry_started = False


@app.route("/getDrones")
def get_drones():
    """Return the locally cached list of drones if available, otherwise force get"""
    if drones:
        response = flask.jsonify(drones)
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response
    else:
        return get_drones_force()


@app.route("/getDronesForce")
def get_drones_force():
    """
    Finds all the devices connected to the local network by sending a packet to the broadcast MAC address,
    then sends an HTTP request to determine whether they are drones.
    """
    global drones
    ip_responses = main.scan()
    # results = interrogate_IPs(ip_responses)
    results = list(ip_responses)
    drones = list(results)
    response = flask.jsonify(drones)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/getRemoteImages")
def get_remote_images():
    """
    Gets all the images that are available on the virtual machine
    """

    print("start get remote")
    data = requests.get("https://csse-sdff.canterbury.ac.nz/v2/_catalog", auth=("test", "testpassword"))
    print("end get remote")
    print(data.text)
    response = flask.jsonify(data.json())
    response.headers.add('Access-Control-Allow-Origin', '*')

    return response


@app.route("/remoteToLocal", methods=["POST"])
def remote_to_local():
    """
    Transfers an image from the virtual machine to the local machine
    """
    data = request.json
    main.pull_image(data["imageName"])
    resp = flask.jsonify(success=True)
    resp.headers.add('Access-Control-Allow-Origin', '*')
    return resp


@app.route("/getLocalImages")
def get_local_images():
    """
    Gets the names of all images currently stored on the local machine
    """
    tags = main.get_docker_images()
    response = flask.jsonify(tags)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/build", methods=["POST"])
def build_image():
    """
    Builds a docker image from the given dockerfile, with the specified name, and stores it on the local machine
    """

    data = request.json
    main.build(data["path"], data["name"])
    resp = flask.jsonify(success=True)
    resp.headers.add('Access-Control-Allow-Origin', '*')
    return resp


@app.route("/startRegistry", methods=["POST"])
def push_image():
    """
    Takes the name of an image on the local machine and pushes it to the local docker registry for transfer.
    """
    data = request.json
    image = data["image"]
    main.push_image(image)
    resp = flask.jsonify(success=True)
    resp.headers.add('Access-Control-Allow-Origin', '*')
    return resp


@app.route("/pullSoftware", methods=["POST"])
def pull():
    """
    SSHs into a drone and runs docker pull to initiate the transfer
    """
    data = request.json
    ip = data["ip"]

    main.sync_time(ip)
    main.copy_cert(ip)
    main.ssh_pull(ip)
    resp = flask.jsonify(success=True)
    resp.headers.add('Access-Control-Allow-Origin', '*')
    return resp


@app.route("/launchDrones", methods=["POST"])
def launch_drones():
    """
    SSHs into a drone and executes "docker run", which should hopefully make the drones soar gracefully into the sky
    """
    data = request.json
    ip = data["ip"]
    config = data["config"]
    number = data["number"]

    main.launch_drone(ip, config)
    resp = flask.jsonify(success=True)
    resp.headers.add('Access-Control-Allow-Origin', '*')
    return resp


main.make_certificate()
main.start_registry()
app.run("0.0.0.0", 8000)
