# ingenium_cartographer

Bash scripts and config files for recording and slamming data for the Wheaton College Tel Shimron lidar project. This branch is being edited to use ROS2 Jazzy Jalisco. As of this writing (June 2025) it is NOT complete, and NOT functional.

The following scripts work as intended: cleanup.sh; Default_Apps_Installer.sh; Install_Jazzy.sh; Ubuntu-Core-RPi-Default-Packages-Installer.sh; Install_LIO-SAM.sh

The following scripts do not yet work as intended: record_to_bag_dev_version.sh

The following scripts are relics from the ROS1 branch, and will eventually be fully replaced: display_bag.sh; install.sh; process_bag.sh; record_to_bag.sh; subtract.sh

![Screenshot](blanchard.png)

## A Brief Note About the Jazzy Branch

This is a branch off a main codebase which was written almost 10 years ago. Much of the knowledge of how the original codebase worked has since been lost. Therefore, in the summer of 2025, Abraham began to reverse-engineer the various scripts, learning bash as he did so. In the above scripts, comments which were original to the document are denoted by a solitary hashtag: #.
Abraham's original reverse-engineering comments are denoted by a triple hashtag: ###. These represent his best guess as to what the code does, but he did not write it nor does he fully understand all of it.

Thereafter, Abraham's comments are denoted by #AB

Milan's comments are denoted by #MS

## Installation Instructions

0. Download Default_Apps_Installer.sh if you wish to install the developer version, and RPi_Default_Apps_Installer if you wish to install critical dependencies only.

1. Navigate to the location of that file in terminal. Mark the script as executable using the relevant code snippet:

`chmod +x Default_Apps_Installer.sh`

`chmod +x RPi_Default_Apps_Installer.sh`

2. Run ./Default*Apps_Installer.sh or ./RPi_Default_Apps_Installer.sh, depending on which file you downloaded. This \_will* prompt sudo--potentially multiple times. Among other things, this git repository will be cloned onto your device under ~/Documents/GitHub/ingenium_cartographer. All files within it which match "\*.sh" will automatically be marked as executable. [ Failing to mark Install_Jazzy.sh and Install_LIO-SAM.sh as executable will cause installation errors. ].

3. If you ran RPi_Default_Apps_Installer.sh in step 2, after the reboot, run ./RPi_post_reboot_default_apps_installer.sh (which can be found in the home directory)

## Instructions for Gathering and Processing Data

0. Connect your LiDAR Hardware (this is tested with a Velodyne VLP-32C Ultra Puck) and IMU (this is tested with a LORD Microstrain 3DM-GX5-15 and a 3DM-GX5-AR) to the data gathering device (which should have these scripts installed). Run `./record_to_bag.sh`. This should procduce a .mcap file in the directory ~/Documents/Data (this will be created automatically by cleanup.sh if it does not yet exist)

1. On a powerful computer (Raspberry Pi not recommended for this step), run `./process_bag.sh /path/to/your/mcap/file.mcap`

2.

## Instructions for Downloading a Single File (such as an installation script) from GitHub on the Command Line

1. On the GitHub website, navigate to the file you want to download, opening the preview

2. On the upper right of the page, select "Raw" and copy the URL it leads to

3. On your device, run

`wget -O new_file_name https://raw.githubusercontent.com/my_user_name/my_reposiroty/heads/my_branch/name_of_my_file`

For example, to install the RPi Default Apps Installer (from the Jazzy branch), run:

`wget -O installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/jazzy/RPi_Default_Apps_Installer.sh`
