if [ "$#" -ne 4 ] ;
then echo "usage: safe_controller_setpoint <x> <y> <z> <yaw>"
else rostopic pub -1 /SafeControllerService/Setpoint d_fall_pps/Setpoint "{x: $0, y: $1, z: $2, yaw: $3}";
fi
