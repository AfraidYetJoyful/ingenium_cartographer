cwd=$(pwd)
catkin_dir=~/catkin_ws/

function catkin_make_custom() {
if [ -d "~/catkin_ws/src/cartographer" ] ### If ~/catkin_ws_/src/cartographer is a directory...
then
    catkin_make_isolated --install --use-ninja
else
    catkin_make
fi
}

function clone_at_commit() { ### This function takes 2 parameters. It clones a given git repo at a specified point in the current directory
  url=$1
  commit=$2
  git clone "$url"
  cd "$(basename "$url")" || exit
  git reset --hard "$commit"
  git pull
  cd .. || exit
}

cd $catkin_dir/src || exit

# This stage of the repo was tested to be working, but may not work with future ROS releases
clone_at_commit https://github.com/ros-drivers/velodyne f235ac6b0d1728e97de552f386b412f5fa5a092d ### Clone the Velodyne Driver. Cloning at this specific point means that the code will maintain compatibility even if the drivers update.

# This stage of the repo was tested to be working, but may not work with future ROS releases
clone_at_commit https://github.com/LORD-MicroStrain/ROS-MSCL e3703a0608536a6d226cc7e727914aed25bd83d5 ### Clone the LORD Microstrain IMU Driver. Cloning at this specific point means that the code will maintain compatibility even if the drivers update.

cd $catkin_dir/ || exit ### Navigate to ~/catkin_ws if possible, otherwise kill process.
catkin_make_custom ### Judging from the if statement immediately below, this function is designed to ensure that ~/catkin_ws/src/cartographer exists. 

if [ -d "~/catkin_ws/src/cartographer" ]  ### Note the absence of an else. catkin_make_custom must surely create this directory that the if is checking.
then
cd $catkin_dir/src || exit ### Navivate to the /src directory or exit the process. Given the if, the || exit seems hardly necessary, but it gives the program a way out without crashing.
sudo rm -r velodyne ### Delete the velodyne directory recursively (which means delete the directory and every single thing inside it)
git clone https://github.com/JohannesByle/velodyne ### Import Johannes' velodyne folder from GitHub. His commits indicate that the extant velodyne folder required some online APIs, which he removed and hard-coded the functionality for into the C++ code.

cd $catkin_dir/ || exit ### move back to the catkin directory, if this is not possible, exit
catkin_make_custom ### Still not sure what this does exactly.
fi
cd "$cwd" || exit ### return to the directory the program started in. Ff this is not possible, exit
