#!/bin/bash

#AB This script is an extended version of the one that used to run automatically at the bottom of record_to_bag_dev_version.sh. That file now uses a cleanup alias to direct the Terminal here.

if ! [ -d /home/lidar/Documents/Data ]; then #AB If the /Documents/Data folder does not exist, make it
    echo "Data folder not found. Creating at /home/lidar/Documents/Data"
    mkdir "/home/lidar/Documents/Data"
fi


for dir in "$(pwd)"/*/     #AB Iterate through every directory in the current working directory (i.e., the directory in which the script is running)
do
    if [[ "$dir" =~ rosbag2_* ]]; then #AB Use a regular expression to determine if any of the directories starts with the phrase "rosbag2_" and has other characters after it
        mv "$dir" "/home/lidar/Documents/Data" #AB If it matches this pattern, move it to the Data folder out of the script folder
        echo "Moved $dir to /Documents/Data"
    fi

done

