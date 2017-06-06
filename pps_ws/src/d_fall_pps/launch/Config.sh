export ROS_MASTER_URI=http://teacher:11311
export ROS_IP=hostname -I | awk '{print $1;}'
export ROS_HOSTNAME=student$(cat /etc/StudentID)
export ROS_NAMESPACE=$(cat /etc/StudentID)

