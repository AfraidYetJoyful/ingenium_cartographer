#!/bin/bash

#FK Bag processing happens within a docker container (a virtual machine). This script takes the image
#   generated in Install_LIO-SAM.sh and runs it (creating the container) and executes it (starts using the
#   container).

#FK create an alias for clarity (note: only applies to current terminal session)
alias usr-please-ignore-error="echo 'Ignore the above error. Everything is fine."

#FK stop and remove any containers of the same name that still exist

#FK if the container is currently running, stop it
#FK if this results in an error (probably because the container is not currently running), do nothing
sudo docker stop liosam-humble-jammy-container || usr-please-ignore-error
#FK if the container exists, remove it
#FK if this results in an error (probably because the container has not been created), do nothing
sudo docker rm liosam-humble-jammy-container || usr-please-ignore-error

#FK make the container out of the image
#FK the mount is ESPECIALLY in flux rn
mkdir -p /mnt/c/Users/ingen/Documents #FK if the user doesn't have the directories to mount to the docker, create them
cd ~
sudo docker run --init -it -d \
  --name liosam-humble-jammy-container \
  --mount type=bind,source=/mnt/c/Users/ingen/Documents,target=/home/Documents \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --runtime=nvidia --gpus all \
  liosam-humble-jammy \
  bash

#FK launch the container
sudo docker exec -it liosam-humble-jammy-container bash

#FK print instructions for what to do once inside the container
echo -e "ONCE INSIDE OF CONTAINER: \n"
echo -e "cd /home/Documents/GitHub/ingenium_cartographer \n
git config --global --add safe.directory /home/Documents/GitHub/ingenium_cartographer \n
git switch humble \n
sudo apt update \n
sudo apt upgrade \n
./process_bag.sh /home/Documents/data/test.mcap \n"
