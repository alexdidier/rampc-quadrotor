export ROS_MASTER_URI=http://teacher:11311
export ROS_IP=$(hostname -I | awk '{print $1;}')
export DFALL_DEFAULT_ID=$(cat /etc/dfall_default_id)
export ROS_NAMESPACE='dfall'

