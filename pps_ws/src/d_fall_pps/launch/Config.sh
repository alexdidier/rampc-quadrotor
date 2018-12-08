# TO RUN THE SYSTEM FULLY ON THE LOCAL COMPUTER:
export ROS_MASTER_URI=http://localhost:11311
# TO RUN THE SYSTEM ON A DEFAULT CONFIGURATION OF THE NETWROK:
# export ROS_MASTER_URI=http://teacher:11311
# OTHER NECESSARY ENVIRONMENT VARIABLES:
export ROS_IP=$(hostname -I | awk '{print $1;}')
export DFALL_DEFAULT_AGENT_ID=$(cat /etc/dfall_default_agent_id)
export DFALL_DEFAULT_COORD_ID=$(cat /etc/dfall_default_coord_id)
export ROS_NAMESPACE='dfall'