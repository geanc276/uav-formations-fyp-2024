# Fast Formation Configuration
### Management Interface

This sub-project is designed to make the process for deploying software on the drones faster 
and more efficient.

For usage, see [usage](./usage.md)
For details on how this project works, see [details](./details.md)

## Building

Before you start, you will need a working installation of the docker engine. 
Instructions can be found at https://docs.docker.com/engine/install/

### Backend

The backend of the interface is written as a flask webserver, using python.

First, make sure you have python3 installed

```bash
sudo apt update && sudo apt install python3
```

To install the dependencies, I recommend creating a virtual environment.

```bash
cd uav-formations
python3 -m venv ./venv
source venv/bin/activate
```

Then the dependencies can be installed to that virtual environment using

```bash
pip3 install -r src/2023/management_interface/backend/requirements.txt
```

You should then be able to launch the backend with:

```bash
sudo ./venv/bin/python3 src/2023/management-interface/backend/controller.py
```
Unfortunately sudo is required for the drone detection features.

### Frontend

The frontend is written using react and typescript. 
You will need to have npm installed. 

Instructions for installing npm can be found [here](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm)

Once you have npm installed, cd into the directory and install the dependencies
```bash
cd frontend
npm install
```

You can then launch the frontend with 
```bash
npm run start
```




