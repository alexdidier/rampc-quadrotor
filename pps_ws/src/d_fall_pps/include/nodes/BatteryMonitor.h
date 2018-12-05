//    Copyright (C) 2017, ETH Zurich, D-ITET, Paul Beuchat
//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    The service that manages the loading of YAML parameters
//
//    ----------------------------------------------------------------------------------





//    ----------------------------------------------------------------------------------
//    III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//     I   NN  N  C      L      U   U  D   D  E      S
//     I   N N N  C      L      U   U  D   D  EEE     SSS
//     I   N  NN  C      L      U   U  D   D  E          S
//    III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS
//    ----------------------------------------------------------------------------------

#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>
#include "std_msgs/Int32.h"
//#include "std_msgs/Float32.h"
//#include <std_msgs/String.h>

#include "d_fall_pps/Controller.h"
#include "d_fall_pps/LoadYamlFiles.h"

// Include the shared definitions
#include "nodes/ParameterServiceDefinitions.h"


//    ----------------------------------------------------------------------------------
//    DDDD   EEEEE  FFFFF  III  N   N  EEEEE   SSSS
//    D   D  E      F       I   NN  N  E      S
//    D   D  EEE    FFF     I   N N N  EEE     SSS
//    D   D  E      F       I   N  NN  E          S
//    DDDD   EEEEE  F      III  N   N  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// Battery states
#define BATTERY_LEVEL_000            0
#define BATTERY_LEVEL_010            1
#define BATTERY_LEVEL_020            2
#define BATTERY_LEVEL_030            3
#define BATTERY_LEVEL_040            4
#define BATTERY_LEVEL_050            5
#define BATTERY_LEVEL_060            6
#define BATTERY_LEVEL_070            7
#define BATTERY_LEVEL_080            8
#define BATTERY_LEVEL_090            9
#define BATTERY_LEVEL_100           10
#define BATTERY_LEVEL_UNAVAILABLE   -1

// The types, i.e., agent or coordinator
#define TYPE_INVALID      -1
#define TYPE_COORDINATOR   1
#define TYPE_AGENT         2


// CrazyRadio states:
#define CRAZY_RADIO_STATE_CONNECTED      0
#define CRAZY_RADIO_STATE_CONNECTING     1
#define CRAZY_RADIO_STATE_DISCONNECTED   2


// Flying states
#define AGENT_OPERATING_STATE_MOTORS_OFF 1
#define AGENT_OPERATING_STATE_TAKE_OFF   2
#define AGENT_OPERATING_STATE_FLYING     3
#define AGENT_OPERATING_STATE_LAND       4




//    ----------------------------------------------------------------------------------
//    V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
//    V   V   A A   R   R   I    A A   B   B  L      E      S
//    V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
//     V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//      V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
//    ----------------------------------------------------------------------------------

// The level of the battery
int m_battery_level;

// The status of the crazyradio as received via messages
//int m_crazyradio_status;

// The flying state of the agent, as received via messages
int m_agent_operating_state;

// Publisher for the filtered battery voltage
ros::Publisher filteredBatteryVoltagePublisher;

// Publisher for the battery state
ros::Publisher batteryLevelPublisher;



// VARIABLES THAT ARE LOADED FROM THE YAML FILE

// Battery thresholds while in the "motors off" state, in [Volts]
float m_battery_threshold_lower_while_standby = 3.30f;
float m_battery_threshold_upper_while_standby = 4.20f;

// Battery thresholds while in the "flying" state, in [Volts]
float m_battery_threshold_lower_while_flying = 2.60f;
float m_battery_threshold_upper_while_flying = 3.70f;



//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    PPPP   RRRR    OOO   TTTTT   OOO   TTTTT  Y   Y  PPPP   EEEEE   SSSS
//    P   P  R   R  O   O    T    O   O    T     Y Y   P   P  E      S
//    PPPP   RRRR   O   O    T    O   O    T      Y    PPPP   EEE     SSS
//    P      R  R   O   O    T    O   O    T      Y    P      E          S
//    P      R   R   OOO     T     OOO     T      Y    P      EEEEE  SSSS
//    ----------------------------------------------------------------------------------

void newBatteryVoltageCallback(const std_msgs::Float32& msg);

float fromVoltageToPercent(float voltage , float operating_state);

int level convertVoltageToLevel(float voltage , float operating_state);


