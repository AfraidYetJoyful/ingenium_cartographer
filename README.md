# ingenium_cartographer

&nbsp; &nbsp; &nbsp; This repository contains bash scripts and config files for recording and slamming data for the Wheaton College Tel Shimron lidar project. This branch is being edited to use ROS2 Jazzy Jalisco. As of this writing (July 2025) this branch is NOT complete, and NOT functional.

The following scripts work as intended: `cleanup.sh`; `Default_Apps_Installer.sh`; `Install_Jazzy.sh`; `RPi-Default-Apps-Installer.sh`; `record_to_bag.sh`

The following scripts do not yet work as intended:
`Install_LIO-SAM.sh`

The following scripts are relics from the ROS1 branch, and will eventually be fully replaced: `display_bag.sh`; `install.sh`; `process_bag.sh`; `subtract.sh`

![Screenshot](blanchard.png)

## A Brief Note About the Jazzy Branch

&nbsp; &nbsp; &nbsp; This is a branch off a main codebase which was written almost 10 years ago. Much of the knowledge of how the original codebase worked has since been lost. Therefore, in the summer of 2025, Abraham began to reverse-engineer the various scripts, learning bash as he did so. In the above scripts, comments which were original to the document are denoted by a solitary hashtag: #.
Abraham's original reverse-engineering comments are denoted by a triple hashtag: ###. These represent his best guess as to what the code does, but he did not write it nor does he fully understand all of it.

Thereafter, Abraham's comments are denoted by #AB

Milan's comments are denoted by #MS

Finn's comments are denoted by #FK

## Installation Instructions

0. Download one of the two installer scripts. 

    0. Use `Default_Apps_Installer.sh` if you wish to install the repository, various developer tools, and the SLAM dependencies.
    
    1. Use `RPi_Default_Apps_Installer.sh` if you wish to install the data gathering tools and their dependencies only.

1. Navigate to the location of that file in terminal. Mark the script as executable using the relevant code snippet:

    `chmod +x Default_Apps_Installer.sh`

    `chmod +x RPi_Default_Apps_Installer.sh`

2. Run the script you just marked as executable (`./Default_Apps_Installer.sh` or `RPi_Default_Apps_Installer.sh`). This *will* prompt sudo--potentially multiple times. Among other things, this git repository will be cloned onto your device under `~/Documents/GitHub/ingenium_cartographer`. All files within it which match "\*.sh" will automatically be marked as executable. [Disabling this feature will cause irreparable installation errors].

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
