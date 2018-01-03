## Starting the turtlebot/xterm does not work.
# xterm -e roslaunch whs_navigation turtlebot.launch

cd `rospack find whs_navigation`/bags
rosbag record -o whs_navigation /scan /tf /tf_static
