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
  The order of these steps is very important, and not following this order can lead to irreparable problems with the installation.

0. Download onto your device the following scripts from this branch of the repository: Default_Apps_Installer.sh ; Install_Jazzy.sh ; Install_LIO-SAM.sh . Make sure they are in the same directory. [ NOTE: if installing on a device solely for data collection, and not on a device intended for development, use Ubuntu-Core-RPi-Default-Packages-Installer.sh instead of Default_Apps_Installer.sh ]

1. Navigate to that directory in terminal. Mark both scripts as executable (either via the "properties" dialog when you right-click or via the chmod +x /path/to/file command)

2. Run sudo ./Default_Apps_Installer.sh . It is recommended that you permit all the packages to install to prevent issues. (Note: These scripts work best when run on a fresh reinstall of Ubuntu Desktop 24.04 LTS). Among other things, this git repository will be cloned onto your device under ~/Documents/GitHub/ingenium_cartographer

3. Mark all shell scripts within it as executable files


## Instructions for Gathering and Processing Data

0. Connect your LiDAR Hardware (this is tested with a Velodyne VLP-32C Ultra Puck) and IMU (this is tested with a LORD Microstrain 3DM-GX5-15 and a 3DM-GX5-AR) to the data gathering device (which should have these scripts installed). Run ./record_to_bag.sh. This should procduce a .mcap file in the directory ~/Documents/Data (this will be created automatically by cleanup.sh if it does not yet exist)

1. On a powerful computer (Raspberry Pi not recommended for this step), run ./process_bag.sh insert_mcap_file_path_here.mcap 

2.  

## Instructions for Downloading a Single File (such as an installation script) from GitHub on the Command Line

1. On the GitHub website, navigate to the file you want to download, opening the preview

2. On the upper right of the page, select "Raw" and copy the URL it leads to

3. On your device, run wget -O new_file_name https://raw.githubusercontent.com/my_user_name/my_reposiroty/heads/my_branch/name_of_my_file

For example, to install the RPi Default Apps Installer (from the Jazzy branch) , type into the terminal: 

wget -O installer.sh https://raw.githubusercontent.com/JohannesByle/ingenium_cartographer/refs/heads/jazzy/RPi_Default_Apps_Installer.sh