#!/bin/bash

sudo snap install network-manager #AB add utility for managing networks
sudo snap install nano-strict #AB add a somewhat wonky version of nano--the only kind Ubuntu Core will allow.
echo 'alias nano="nano-strict"' >> ~/.bashrc #AB Add an alias to the system ~/.bashrc file so that the command "nano" runs "nano-strict".
