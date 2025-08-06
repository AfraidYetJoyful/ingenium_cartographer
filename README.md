# ingenium_cartographer

&nbsp; &nbsp; &nbsp; This branch of the repository contains bash scripts and config files for slamming data for the Wheaton College Tel Shimron lidar project. This branch is built on ROS2 Humble Hawksbill, since LIO-SAM does not yet work with ROS2 Jazzy Jalisco. As of this writing (August 2025) this branch is NOT complete, and NOT functional.


![Screenshot](blanchard.png)

## A Brief Note About Comments

Abraham's comments are denoted by #AB

Milan's comments are denoted by #MS

Finn's comments are denoted by #FK

## Installation Instructions

0. Download `Default_Apps_Installer.sh` onto a clean Ubuntu 22.04.5 LTS Jammy Jellyfish installation

1. Mark it as executable by navigating to its directory in Terminal and running `chmod +x Default_Apps_Installer.sh`

2. Next, run `./Default_Apps_Installer.sh`. This *will* prompt sudo--potentially multiple times. Among other things, this git repository will be cloned onto your device under `~/Documents/GitHub/ingenium_cartographer`. All files within it which match "\*.sh" will automatically be marked as executable. [Disabling this feature will cause irreparable installation errors].

3. If you ran `RPi_Default_Apps_Installer.sh` in step 2, after the reboot, run `./RPi_Network_Config.sh` (which will now be located in the `~` directory)

## Instructions for Gathering and Processing Data

0. Connect your LiDAR Hardware (this is tested with a Velodyne VLP-32C Ultra Puck) and IMU (this is tested with a LORD Microstrain 3DM-GX5-15/3DM-GX5-AR) to the data gathering device. Run `./record_to_bag.sh`. This should procduce a .mcap file in `~/Documents/Data` (this directory will be created automatically by `cleanup.sh` if it does not yet exist)

1. On the fastest available computer, run `./process_bag.sh /path/to/your/mcap/file.mcap`. [NOTE: The dependencies for this script are not included in the minimal installation for Raspberry Pi!]



## Instructions for Downloading a Single File from GitHub with the Command Line

1. On the GitHub website, navigate to the file you want to download and open the preview

2. On the upper right of the page, select "Raw" and copy the URL

3. On your device, run

`wget -O [new_file_name] https://raw.githubusercontent.com/[my_user_name]/[my_repository]/refs/heads/[my_branch]/[name_of_my_file]`

For example, to download `RPi_Default_Apps_Installer.sh`, run:

`wget -O RPi_Default_Apps_Installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/jazzy/RPi_Default_Apps_Installer.sh`
