#!/bin/bash

#FK Installs docker correctly ("sudo apt install docker" doesn't)

#FK should theoretically work on any Ubuntu machine
#FK this has been used and tested on a Windows machine, WSL 2, Ubuntu 22.04.5 LTS

#FK most of this code is from: https://web.archive.org/web/20250805192147/https://www.techbloat.com/failed-to-start-docker-service-unit-not-found-error-fix-on-linux.html
#FK if confused, find more information there

#FK remove leftovers from previous incorrect docker installs
sudo apt-get remove --purge docker docker-engine docker.io containerd runc

#FK install and authenticate all the relevant things
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl gnupg lsb-release
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
sudo systemctl daemon-reload
sudo systemctl enable docker
sudo systemctl start docker
systemctl status docker

#FK test that Docker works; this test is from various places
sudo docker run --name hello hello-world
sudo docker rm hello
