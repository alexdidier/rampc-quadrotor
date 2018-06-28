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




// DEFINES THAT ARE SHARED WITH OTHER FILES

// For which controller parameters to load from file
#define LOAD_YAML_SAFE_CONTROLLER_AGENT             1
#define LOAD_YAML_DEMO_CONTROLLER_AGENT             2
#define LOAD_YAML_STUDENT_CONTROLLER_AGENT          3
#define LOAD_YAML_MPC_CONTROLLER_AGENT              4
#define LOAD_YAML_REMOTE_CONTROLLER_AGENT           5
#define LOAD_YAML_TUNING_CONTROLLER_AGENT           6

#define LOAD_YAML_SAFE_CONTROLLER_COORDINATOR       11
#define LOAD_YAML_DEMO_CONTROLLER_COORDINATOR       12
#define LOAD_YAML_STUDENT_CONTROLLER_COORDINATOR    13
#define LOAD_YAML_MPC_CONTROLLER_COORDINATOR        14
#define LOAD_YAML_REMOTE_CONTROLLER_COORDINATOR     15
#define LOAD_YAML_TUNING_CONTROLLER_COORDINATOR     16


// For sending commands to the controller node informing
// which parameters to fetch
// > NOTE: these are identical to the #defines above, but
//         used because they have the name distinguishes
//         between:
//         - "loading" a yaml file into ram
//         - "fetching" the values that were loaded into ram
#define FETCH_YAML_SAFE_CONTROLLER_FROM_OWN_AGENT      1
#define FETCH_YAML_DEMO_CONTROLLER_FROM_OWN_AGENT      2
#define FETCH_YAML_STUDENT_CONTROLLER_FROM_OWN_AGENT   3
#define FETCH_YAML_MPC_CONTROLLER_FROM_OWN_AGENT       4
#define FETCH_YAML_REMOTE_CONTROLLER_FROM_OWN_AGENT    5
#define FETCH_YAML_TUNING_CONTROLLER_FROM_OWN_AGENT    6

#define FETCH_YAML_SAFE_CONTROLLER_FROM_COORDINATOR      11
#define FETCH_YAML_DEMO_CONTROLLER_FROM_COORDINATOR      12
#define FETCH_YAML_STUDENT_CONTROLLER_FROM_COORDINATOR   13
#define FETCH_YAML_MPC_CONTROLLER_FROM_COORDINATOR       14
#define FETCH_YAML_REMOTE_CONTROLLER_FROM_COORDINATOR    15
#define FETCH_YAML_TUNING_CONTROLLER_FROM_COORDINATOR    16