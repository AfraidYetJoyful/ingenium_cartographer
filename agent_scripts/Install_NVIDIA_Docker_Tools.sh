#!/bin/bash

#FK This script installs things to let docker run containers
#   using the GPU (specifically, an NVIDIA GPU)

#FK PRECONDITION: The docker install script (Install_Docker.sh) has been run.

#FK This code comes from three places:
#   a general instruction guide for this problem, https://geekchamp.com/how-to-use-an-nvidia-gpu-with-docker-containers/
#   the GitHub for the recent version of the NVIDIA Container toolkit, https://github.com/NVIDIA/nvidia-container-toolkit?tab=readme-ov-file
#   and the instruction guide that the GitHub linked to: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

#FK Install the NVIDIA Container toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.17.8-1
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}

sudo apt-get install -y nvidia-docker2

#FK restart docker and reconfigure it
sudo systemctl restart docker #FK restarts the docker daemon
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker #FK restarts the docker daemon

#FK test that the NVIDIA container toolkit is working
echo “Testing that NVIDIA container toolkit is working!”
sudo docker run --name ubuntu-test --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
sudo docker stop ubuntu-test
sudo docker rm ubuntu-test
