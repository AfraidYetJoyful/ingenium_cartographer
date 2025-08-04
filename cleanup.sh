#!/bin/bash

#AB This script is an extended version of the one that used to run automatically at the bottom of record_to_bag_dev_version.sh. record_to_bag.sh now uses a cleanup alias to direct the Terminal here.

if ! [ -d ~/Documents/Data ]; then #AB If the ~/Documents/Data folder does not exist, make it
    echo "Data folder not found. Creating at ~/Documents/Data"
    mkdir -p "~/Documents/Data"
fi


for dir in "$(pwd)"/*/     #AB Iterate through every directory in the current working directory (i.e., the directory in which the script is running)
do
    if [[ "$dir" =~ rosbag2_* ]]; then #AB Use a regular expression to determine if any of the directories starts with the phrase "rosbag2_" and has 1 or more characters after that phrase
        mv "$dir" ~/Documents/Data/"$(basename "$dir")" #AB If the directory matches this pattern, move it to the Data folder. It will no longer be present in the same directory as the scripts.
        #AB Then print stuff
        echo "Moved $dir to ~/Documents/Data"
        echo "File data:"
        echo "—————————————————————————————————————————————————————————————————————————————————————————————————————"
        echo "Size                                        Filename"
        du -h --apparent-size ~/Documents/Data/"$(basename "$dir")/$(basename "$dir")_0.mcap" #AB Print the size and name of the file
        echo "—————————————————————————————————————————————————————————————————————————————————————————————————————"
        echo " "
    fi

done

