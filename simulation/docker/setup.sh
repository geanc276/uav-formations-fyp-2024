if ! command -v docker &> /dev/null
then
    echo "Docker is not installed. Installing Docker..."
    # Update the package database
    sudo apt-get update
    # Install required packages
    sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common
    # Add Docker’s official GPG key
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    # Set up the stable repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    # Update the package database again
    sudo apt-get update
    # Install Docker
    sudo apt-get install -y docker-ce
    # Start Docker service
    sudo systemctl start docker
    # Enable Docker to start on boot
    sudo systemctl enable docker
    echo "Docker installed successfully."
else
    echo "Docker is already installed."
fi


if ! dpkg -l | grep -q docker-buildx-plugin
then
    echo "docker-buildx-plugin is not installed. Installing docker-buildx-plugin..."
    sudo apt-get install -y docker-buildx-plugin
    echo "docker-buildx-plugin installed successfully."
else
    echo "docker-buildx-plugin is already installed."
fi

# Start Docker daemon if not running
if ! pgrep -x "dockerd" > /dev/null
then
    echo "Starting Docker daemon..."
    sudo dockerd &
    sleep 5
    echo "Docker daemon started."
fi

# Pull the ROS image from Docker Hub
eval "docker pull docker.io/library/ros:humble"


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Build the Docker image
chmod +x $SCRIPT_DIR/build.sh
$SCRIPT_DIR/build.sh humble

chmod +x run.sh
