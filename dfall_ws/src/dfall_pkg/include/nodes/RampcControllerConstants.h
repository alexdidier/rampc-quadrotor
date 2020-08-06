//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
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
//    Constants that are used across the Rampc Controller
//    Service and the Flying Agent Client
//
//    ----------------------------------------------------------------------------------


#ifndef M_PI
#define M_PI 3.1415926
#endif

// TO REQUEST MANOEUVRES

#define RAMPC_CONTROLLER_REQUEST_TAKEOFF     1
#define RAMPC_CONTROLLER_REQUEST_LANDING     2


// TO NOTIFY THAT MANOEUVRES ARE COMPLETED

#define RAMPC_CONTROLLER_LANDING_COMPLETE     2


// TO IDENITFY THE STATE OF THE RAMPC CONTROLLER

#define RAMPC_CONTROLLER_STATE_LQR				11
#define RAMPC_CONTROLLER_STATE_EXCITATION_LQR	12
#define RAMPC_CONTROLLER_STATE_RAMPC			13
#define RAMPC_CONTROLLER_STATE_EXCITATION_RAMPC	14
#define RAMPC_CONTROLLER_STATE_STANDBY			99

// > The sequence of states for a LANDING manoeuvre
#define RAMPC_CONTROLLER_STATE_LANDING_MOVE_DOWN        21
#define RAMPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS      22


// OPTIMIZATION SOLVERS

#define RAMPC_CONTROLLER_SOLVER_GUROBI	0
#define RAMPC_CONTROLLER_SOLVER_OSQP	1
#define RAMPC_CONTROLLER_SOLVER_MPC		2

// EXPERIMENT
#define RAMPC_CONTROLLER_EXPERIMENT_MASS 1
#define RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS 2
#define RAMPC_CONTROLLER_EXPERIMENT_FULL_STATE 3