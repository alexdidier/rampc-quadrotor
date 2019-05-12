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
//    A Deepc Controller for students build from
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/DeepcControllerService.h"






//    ----------------------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    III M   M PPPP  L     EEEEE M   M EEEEE N   N TTTTT   A   TTTTT III  OOO  N   N
//     I  MM MM P   P L     E     MM MM E     NN  N   T    A A    T    I  O   O NN  N
//     I  M M M PPPP  L     EEE   M M M EEE   N N N   T   A   A   T    I  O   O N N N
//     I  M   M P     L     E     M   M E     N  NN   T   AAAAA   T    I  O   O N  NN
//    III M   M P     LLLLL EEEEE M   M EEEEE N   N   T   A   A   T   III  OOO  N   N
//    ----------------------------------------------------------------------------------


// DEEPC FUNCTIONS

// DEEPC THREAD MAIN
// Deepc operations run in seperate thread as they are time consuming
void deepc_thread_main()
{
	bool params_changed_local;
	bool setupDeepc_local;
	bool grb_setup_success_local;
	bool solveDeepc_local;

	while (ros::ok())
	{
		m_Deepc_mutex.lock();
        params_changed_local = m_params_changed;
        setupDeepc_local = m_setupDeepc;
        solveDeepc_local = m_solveDeepc;
        m_Deepc_mutex.unlock();

        if (params_changed_local)
        	change_Deepc_params;

        if (setupDeepc_local)
        {
        	grb_setup_success_local = setup_Deepc();

        	m_Deepc_mutex.lock();
        	m_setupDeepc = false;
        	m_grb_setup_success = grb_setup_success_local;
        	m_Deepc_mutex.unlock();
        }

        if (solveDeepc_local)
        {
        	solve_Deepc();
		  	sleep(1);

		  	m_Deepc_mutex.lock();
        	m_solveDeepc = false;
        	m_Deepc_mutex.unlock();
        }
	}
}

void change_Deepc_params()
{
	m_Deepc_mutex.lock();
	
	// > Change Gurobi optimization parameters
	if (yaml_grb_LogToFile)
		m_grb_model.set(GRB_StringParam_LogFile, m_logFolder + "gurobi.log");
	else
		m_grb_model.set(GRB_StringParam_LogFile, "");
	m_grb_model.set(GRB_IntParam_LogToConsole, yaml_grb_LogToConsole);
	
	m_params_changed = false;

	m_Deepc_mutex.unlock();
}

bool setup_Deepc()
{
	try
    {
    	// Configure environment
	    m_grb_env.set(GRB_StringParam_LogFile, m_logFolder + "gurobi.log");
	    m_grb_env.start();

	    // Create variables
	    m_grb_x = m_grb_model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
	    m_grb_y = m_grb_model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
	    m_grb_z = m_grb_model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

	    // Set objective: maximize x + y + 2 z
	    m_grb_model.setObjective(m_grb_x + m_grb_y + 2 * m_grb_z, GRB_MAXIMIZE);

	    // Add constraint: x + 2 y + 3 z <= 4
	    m_grb_model.addConstr(m_grb_x + 2 * m_grb_y + 3 * m_grb_z <= 4, "c0");

	    // Add constraint: x + y >= 1
	    m_grb_model.addConstr(m_grb_x + m_grb_y >= 1, "c1");

	    // Inform the user
	    ROS_INFO("[DEEPC CONTROLLER] Deepc optimization setup successful");

	    return true;
    }
    
    catch(GRBException e)
    {
	    ROS_INFO_STREAM("[DEEPC CONTROLLER] Deepc optimization setup failed with error code = " << e.getErrorCode());
	    ROS_INFO_STREAM("[DEEPC CONTROLLER] Error message: " << e.getMessage());

	    return false;
  	}
  	catch(...)
  	{
    	ROS_INFO("[DEEPC CONTROLLER] Deepc optimization setup failed");

    	return false;
  	}
}

bool solve_Deepc()
{
	// Solve optimization
	try
	{
		m_grb_model.optimize();
		ROS_INFO("[DEEPC CONTROLLER] Deepc optimization solved with following results:");
		ROS_INFO_STREAM(m_grb_x.get(GRB_StringAttr_VarName) << " " << m_grb_x.get(GRB_DoubleAttr_X));
    	ROS_INFO_STREAM(m_grb_y.get(GRB_StringAttr_VarName) << " " << m_grb_y.get(GRB_DoubleAttr_X));
    	ROS_INFO_STREAM(m_grb_z.get(GRB_StringAttr_VarName) << " " << m_grb_z.get(GRB_DoubleAttr_X));
    	ROS_INFO_STREAM("Objective: " << m_grb_model.get(GRB_DoubleAttr_ObjVal));
    	ROS_INFO_STREAM("Runtime: " << m_grb_model.get(GRB_DoubleAttr_Runtime));

    	return true;
	}
	catch(GRBException e)
    {
	    ROS_INFO_STREAM("[DEEPC CONTROLLER] Deepc optimization failed to solve with error code = " << e.getErrorCode());
	    ROS_INFO_STREAM("[DEEPC CONTROLLER] Error message: " << e.getMessage());

	    return false;
  	}
  	catch(...)
  	{
    	ROS_INFO("[DEEPC CONTROLLER] Deepc optimization failed to solve");

    	return false;
  	}
}

// DEEPC HELPER FUNCTIONS

// ---------- READ/WRITE CSV FILES ----------

// Read csv file into matrix
MatrixXf read_csv(const string & path)
{
	ifstream fin;
	fin.open(path);
	if(!fin)
	{
		MatrixXf M;
		return M;
	}
	string line;
	vector<float> values;
	int rows = 0;
	while (getline(fin, line))
	{
		stringstream lineStream(line);
		string cell;
		bool empty_row = true;
		while (getline(lineStream, cell, ','))
		{
			values.push_back(stof(cell));
			empty_row = false;
		}
		if(!empty_row)
			rows++;
	}
	return Map<Matrix<float, Dynamic, Dynamic, RowMajor>>(values.data(), rows, values.size()/rows);
}

// Write matrix into csv file
bool write_csv(const string & path, MatrixXf M)
{
	ofstream fout;
	fout.open(path);
	if(!fout)
		return false;
	int rows = M.rows();
	int cols = M.cols();
	for(int i = 0; i < rows; i++)
	{
		for(int j = 0; j < cols - 1; j++)
			fout << M(i,j) << ',';
		fout << M(i,cols-1) << endl;
	}
	return true;
}

//    ------------------------------------------------------------------------------
//    RRRR   EEEEE   QQQ   U   U  EEEEE   SSSS  TTTTT
//    R   R  E      Q   Q  U   U  E      S        T
//    RRRR   EEE    Q   Q  U   U  EEE     SSS     T
//    R   R  E      Q  Q   U   U  E          S    T
//    R   R  EEEEE   QQ Q   UUU   EEEEE  SSSS     T
//    
//    M   M    A    N   N   OOO   EEEEE  U   U  V   V  RRRR   EEEEE
//    MM MM   A A   NN  N  O   O  E      U   U  V   V  R   R  E
//    M M M  A   A  N N N  O   O  EEE    U   U  V   V  RRRR   EEE
//    M   M  AAAAA  N  NN  O   O  E      U   U   V V   R   R  E
//    M   M  A   A  N   N   OOO   EEEEE   UUU     V    R   R  EEEEE
//    ------------------------------------------------------------------------------

// CALLBACK FOR THE REQUEST MANOEUVRE SERVICE
bool requestManoeuvreCallback(IntIntService::Request &request, IntIntService::Response &response)
{
	// Extract the requested manoeuvre
	int requestedManoeuvre = request.data;

	// Switch between the possible manoeuvres
	switch (requestedManoeuvre)
	{
		case DEEPC_CONTROLLER_REQUEST_TAKEOFF:
		{
			// Inform the user
			ROS_INFO("[DEEPC CONTROLLER] Received request to take off. Switch to state: LQR");
			// Update the state accordingly
			m_current_state = DEEPC_CONTROLLER_STATE_LQR;
			m_current_state_changed = true;
			// Provide dummy response
			response.data = 0;
			break;
		}

		case DEEPC_CONTROLLER_REQUEST_LANDING:
		{
			// Inform the user
			ROS_INFO("[DEEPC CONTROLLER] Received request to perform landing manoeuvre. Switch to state: landing move down");
			// Update the state accordingly
			m_current_state = DEEPC_CONTROLLER_STATE_LANDING_MOVE_DOWN;
			m_current_state_changed = true;
			// Fill in the response duration in milliseconds
			response.data = int(
					1000 * (
						+ yaml_landing_move_down_time_max
						+ yaml_landing_spin_motors_time
					)
				);
			break;
		}

		default:
		{
			// Inform the user
			ROS_INFO("[DEEPC CONTROLLER] The requested manoeuvre is not recognised. Hence switching to standby state.");
			// Update the state to standby
			m_current_state = DEEPC_CONTROLLER_STATE_STANDBY;
			m_current_state_changed = true;
			// Fill in the response duration in milliseconds
			response.data = 0;
			break;
		}
	}

	// Return success
	return true;
}


//    ------------------------------------------------------------------------------
//     OOO   U   U  TTTTT  EEEEE  RRRR 
//    O   O  U   U    T    E      R   R
//    O   O  U   U    T    EEE    RRRR
//    O   O  U   U    T    E      R  R
//     OOO    UUU     T    EEEEE  R   R
//
//     CCCC   OOO   N   N  TTTTT  RRRR    OOO   L           L       OOO    OOO   PPPP
//    C      O   O  NN  N    T    R   R  O   O  L           L      O   O  O   O  P   P
//    C      O   O  N N N    T    RRRR   O   O  L           L      O   O  O   O  PPPP
//    C      O   O  N  NN    T    R  R   O   O  L           L      O   O  O   O  P
//     CCCC   OOO   N   N    T    R   R   OOO   LLLLL       LLLLL   OOO    OOO   P
//    ----------------------------------------------------------------------------------

// This function is the callback that is linked to the "DeepcController"
// service that is advertised in the main function. This must have arguments
// that match the "input-output" behaviour defined in the "Controller.srv"
// file (located in the "srv" folder)
//
// The arument "request" is a structure provided to this service with the
// following two properties:
//
// >> request.ownCrazyflie
// This property is itself a structure of type "CrazyflieData",  which is
// defined in the file "CrazyflieData.msg", and has the following properties
// string crazyflieName
//     float64 x                         The x position of the Crazyflie [metres]
//     float64 y                         The y position of the Crazyflie [metres]
//     float64 z                         The z position of the Crazyflie [metres]
//     float64 roll                      The roll component of the intrinsic Euler angles [radians]
//     float64 pitch                     The pitch component of the intrinsic Euler angles [radians]
//     float64 yaw                       The yaw component of the intrinsic Euler angles [radians]
//     float64 acquiringTime #delta t    The time elapsed since the previous "CrazyflieData" was received [seconds]
//     bool occluded                     A boolean indicted whether the Crazyflie for visible at the time of this measurement
// The values in these properties are directly the measurement taken by the
// motion capture system of the Crazyflie that is to be controlled by this
// service.
//
// >> request.otherCrazyflies
// This property is an array of "CrazyflieData" structures, what allows access
// to the motion capture measurements of other Crazyflies.
//
// The argument "response" is a structure that is expected to be filled in by
// this service by this function, it has only the following property
//
// >> response.ControlCommand
// This property is iteself a structure of type "ControlCommand", which is
// defined in the file "ControlCommand.msg", and has the following properties:
//     float32 roll                      The command sent to the Crazyflie for the body frame x-axis
//     float32 pitch                     The command sent to the Crazyflie for the body frame y-axis
//     float32 yaw                       The command sent to the Crazyflie for the body frame z-axis
//     uint16 motorCmd1                  The command sent to the Crazyflie for motor 1
//     uint16 motorCmd2                  The command sent to the Crazyflie for motor 1
//     uint16 motorCmd3                  The command sent to the Crazyflie for motor 1
//     uint16 motorCmd4                  The command sent to the Crazyflie for motor 1
//     uint8 onboardControllerType       The flag sent to the Crazyflie for indicating how to implement the command
// 
// IMPORTANT NOTES FOR "onboardControllerType"  AND AXIS CONVENTIONS
// > The allowed values for "onboardControllerType" are in the "Defines"
//   section in the header file, they are:
//   - CF_COMMAND_TYPE_MOTORS
//   - CF_COMMAND_TYPE_RATE
//   - CF_COMMAND_TYPE_ANGLE
//
// > For most common option to use is CF_COMMAND_TYPE_RATE option.
//
// > For the CF_COMMAND_TYPE_RATE optoin:
//   1) the ".roll", ".ptich", and ".yaw" properties of
//      "response.ControlCommand" specify the angular rate in
//      [radians/second] that will be requested from the PID controllers
//      running in the Crazyflie 2.0 firmware.
//   2) the ".motorCmd1" to ".motorCmd4" properties of
//      "response.ControlCommand" are the baseline motor commands
//      requested from the Crazyflie, with the adjustment for body rates
//      being added on top of this in the firmware (i.e., as per the
//      code of the "distribute_power" found in the firmware).
//   3) the axis convention for the roll, pitch, and yaw body rates
//      returned in "response.ControlCommand" should use right-hand
//      coordinate axes with x-forward and z-upwards (i.e., the positive
//      z-axis is aligned with the direction of positive thrust). To
//      assist, the following is an ASCII art of this convention.
//
// ASCII ART OF THE CRAZYFLIE 2.0 LAYOUT
//
//  > This is a top view,
//  > M1 to M4 stand for Motor 1 to Motor 4,
//  > "CW"  indicates that the motor rotates Clockwise,
//  > "CCW" indicates that the motor rotates Counter-Clockwise,
//  > By right-hand axis convention, the positive z-direction points out
//    of the screen,
//  > This being a "top view" means that the direction of positive thrust
//    produced by the propellers is also out of the screen.
//
//        ____                         ____
//       /    \                       /    \
//  (CW) | M4 |           x           | M1 | (CCW)
//       \____/\          ^          /\____/
//            \ \         |         / /
//             \ \        |        / /
//              \ \______ | ______/ /
//               \        |        /
//                |       |       |
//        y <-------------o       |
//                |               |
//               / _______________ \
//              / /               \ \
//             / /                 \ \
//        ____/ /                   \ \____
//       /    \/                     \/    \
// (CCW) | M3 |                       | M2 | (CW)
//       \____/                       \____/
//
//
//
//

// THE MAIN CONTROL FUNCTION CALLED FROM THE FLYING AGENT CLIENT
bool calculateControlOutput(Controller::Request &request, Controller::Response &response)
{

	// This is the "start" of the outer loop controller, add all your control
	// computation here, or you may find it convienient to create functions
	// to keep you code cleaner

	// Switch between the possible states
	switch (m_current_state)
	{
		case DEEPC_CONTROLLER_STATE_LQR:
			computeResponse_for_LQR(request, response);
			break;

        case DEEPC_CONTROLLER_STATE_EXCITATION:
            computeResponse_for_excitation(request, response);
            break;

        case DEEPC_CONTROLLER_STATE_DEEPC:
            computeResponse_for_Deepc(request, response);
            break;

		case DEEPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
			computeResponse_for_landing_move_down(request, response);
			break;

		case DEEPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
			computeResponse_for_landing_spin_motors(request, response);
			break;

		case DEEPC_CONTROLLER_STATE_STANDBY:
		default:
			computeResponse_for_standby(request, response);
			break;
	}


	// Return "true" to indicate that the control computation was performed successfully
	return true;
}


void computeResponse_for_standby(Controller::Request &request, Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Nothing to perform for this state
		// Set the change flag back to false
		m_current_state_changed = false;
        // Publish the change
        publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[DEEPC CONTROLLER] State \"standby\" started");
	}

	// PREPARE AND RETURN THE VARIABLE "response"
	// Specify that using a "motor type" of command
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS;

	// Fill in zero for the angle parts
	response.controlOutput.roll  = 0.0;
	response.controlOutput.pitch = 0.0;
	response.controlOutput.yaw   = 0.0;

	// Fill in all motor thrusts as zero
	response.controlOutput.motorCmd1 = 0.0;
	response.controlOutput.motorCmd2 = 0.0;
	response.controlOutput.motorCmd3 = 0.0;
	response.controlOutput.motorCmd4 = 0.0;

	// DEBUG INFO
	if (yaml_shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("output.thrust = " << 0.0);
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);
		ROS_INFO_STREAM("controlOutput.motorCmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.motorCmd2 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.motorCmd3 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.motorCmd4 = " << response.controlOutput.motorCmd4);
	}
}

void computeResponse_for_LQR(Controller::Request &request, Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		for (int i = 0; i < 9; i++)
			m_previous_stateErrorInertial[i] = 0.0;
        m_thrustExcEnable = false;
		m_rollRateExcEnable = false;
        m_pitchRateExcEnable = false;
        m_yawRateExcEnable = false;
		// Set the change flag back to false
		m_current_state_changed = false;

		// If just coming from excitation state, write data collected
		if (m_write_data)
		{
			ROS_INFO_STREAM("[DEEPC CONTROLLER] Writing input data to: " << m_outputFolder << "m_u_data.csv");
            if (write_csv(m_outputFolder + "m_u_data.csv", m_u_data))
            	ROS_INFO("[DEEPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[DEEPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[DEEPC CONTROLLER] Writing output data to: " << m_outputFolder << "m_y_data.csv");
            if (write_csv(m_outputFolder + "m_y_data.csv", m_y_data))
            	ROS_INFO("[DEEPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[DEEPC CONTROLLER] Write file failed");
		}

		// Publish the change
		publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[DEEPC CONTROLLER] State \"LQR\" started");
	}

	m_setpoint_for_controller[0] = m_setpoint[0];
	m_setpoint_for_controller[1] = m_setpoint[1];
	m_setpoint_for_controller[2] = m_setpoint[2];
	m_setpoint_for_controller[3] = m_setpoint[3];
	
	// Call the LQR control function
	control_output output;
	calculateControlOutput_viaLQR(request, output);

	// PREPARE AND RETURN THE VARIABLE "response"
	// Specify that using a "rate type" of command
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;

	// Put the computed body rate commands into the "response" variable
	response.controlOutput.roll = output.rollRate;
	response.controlOutput.pitch = output.pitchRate;
	response.controlOutput.yaw = output.yawRate;

	// Put the thrust commands into the "response" variable.
	// . NOTE: The thrust is commanded per motor, so divide by 4.0
	// > NOTE: The function "computeMotorPolyBackward" converts the input argument
	//         from Newtons to the 16-bit command expected by the Crazyflie.
	float thrust_request_per_motor = output.thrust / 4.0;
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_request_per_motor);

	// DEBUG INFO
	if (yaml_shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("output.thrust = " << output.thrust);
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);
		ROS_INFO_STREAM("controlOutput.motorCmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.motorCmd2 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.motorCmd3 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.motorCmd4 = " << response.controlOutput.motorCmd4);
	}
}

void computeResponse_for_excitation(Controller::Request &request, Controller::Response &response)
{
    // Check if the state "just recently" changed
    if (m_current_state_changed)
    {
        // PERFORM "ONE-OFF" OPERATIONS HERE
		m_time_in_seconds = 0.0;
        for (int i = 0; i < 9; i++)
			m_previous_stateErrorInertial[i] = 0.0;
        m_thrustExcIndex = 0;
        m_rollRateExcIndex = 0;
        m_pitchRateExcIndex = 0;
        m_yawRateExcIndex = 0;
        m_dataIndex = 0;
        // Set the change flag back to false
        m_current_state_changed = false;
        // Publish the change
        publishCurrentSetpointAndState();
        // Inform the user
        ROS_INFO_STREAM("[DEEPC CONTROLLER] State \"excitation\" started");
    }

    m_setpoint_for_controller[0] = m_setpoint[0];
    m_setpoint_for_controller[1] = m_setpoint[1];
    m_setpoint_for_controller[2] = m_setpoint[2];
    m_setpoint_for_controller[3] = m_setpoint[3];

    // Call the LQR control function
    control_output output;
    calculateControlOutput_viaLQR(request, output);

    // Output excitation
    if (m_thrustExcEnable && m_time_in_seconds > yaml_exc_start_time - m_control_deltaT)
    {
        //output.thrust += m_thrustExcAmp_in_newtons * sin(2 * PI * yaml_thrustExcFreq * m_thrustExcTime_in_seconds);
        //m_thrustExcTime_in_seconds += m_control_deltaT;
        if (m_thrustExcIndex < m_thrustExcSignal.size())
        {
        	output.thrust += m_thrustExcAmp_in_newtons * m_thrustExcSignal(m_thrustExcIndex);
        	m_thrustExcIndex++;
        }
        else
        {
        	if (m_rollRateExcEnable || m_pitchRateExcEnable || m_yawRateExcEnable)
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Thrust excitation signal ended. State stays at: excitation");
                m_thrustExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Thrust excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = DEEPC_CONTROLLER_STATE_LQR;
                m_current_state_changed = true;
                m_write_data = true;
            }
        }
    }

    if (m_rollRateExcEnable && m_time_in_seconds > yaml_exc_start_time - m_control_deltaT)
    {
        //output.rollRate += m_rollRateExcAmp_in_rad * sin(2 * PI * yaml_rollRateExcFreq * m_rollRateExcTime_in_seconds);
        //m_rollRateExcTime_in_seconds += m_control_deltaT;
        if (m_rollRateExcIndex < m_rollRateExcSignal.size())
        {
        	output.rollRate += m_rollRateExcAmp_in_rad * m_rollRateExcSignal(m_rollRateExcIndex);
        	m_rollRateExcIndex++;
        }
        else
        {
        	if (m_thrustExcEnable || m_pitchRateExcEnable || m_yawRateExcEnable)
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Roll rate excitation signal ended. State stays at: excitation");
                m_rollRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Roll rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = DEEPC_CONTROLLER_STATE_LQR;
                m_current_state_changed = true;
                m_write_data = true;
            }
        }
    }

    if (m_pitchRateExcEnable && m_time_in_seconds > yaml_exc_start_time - m_control_deltaT)
    {
        //output.pitchRate += m_pitchRateExcAmp_in_rad * sin(2 * PI * yaml_pitchRateExcFreq * m_pitchRateExcTime_in_seconds);
        //m_pitchRateExcTime_in_seconds += m_control_deltaT;
        if (m_pitchRateExcIndex < m_pitchRateExcSignal.size())
        {
        	output.pitchRate += m_pitchRateExcAmp_in_rad * m_pitchRateExcSignal(m_pitchRateExcIndex);
        	m_pitchRateExcIndex++;
        }
        else
        {
        	if (m_thrustExcEnable || m_rollRateExcEnable || m_yawRateExcEnable)
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Pitch rate excitation signal ended. State stays at: excitation");
                m_pitchRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Pitch rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = DEEPC_CONTROLLER_STATE_LQR;
                m_current_state_changed = true;
                m_write_data = true;
            }
        }
    }

    if (m_yawRateExcEnable  && m_time_in_seconds > yaml_exc_start_time - m_control_deltaT)
    {
        //output.yawRate += m_yawRateExcAmp_in_rad * sin(2 * PI * yaml_yawRateExcFreq * m_yawRateExcTime_in_seconds);
        //m_yawRateExcTime_in_seconds += m_control_deltaT;
        if (m_yawRateExcIndex < m_yawRateExcSignal.size())
        {
        	output.yawRate += m_yawRateExcAmp_in_rad * m_yawRateExcSignal(m_yawRateExcIndex);
        	m_yawRateExcIndex++;
        }
        else
        {
        	if (m_thrustExcEnable || m_rollRateExcEnable || m_pitchRateExcEnable)
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Yaw rate excitation signal ended. State stays at: excitation");
                m_yawRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[DEEPC CONTROLLER] Yaw rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = DEEPC_CONTROLLER_STATE_LQR;
                m_current_state_changed = true;
                m_write_data = true;
            }
        }
    }

    // PREPARE AND RETURN THE VARIABLE "response"
    // Specify that using a "rate type" of command
    response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;

    // Put the computed body rate commands into the "response" variable
    response.controlOutput.roll = output.rollRate;
    response.controlOutput.pitch = output.pitchRate;
    response.controlOutput.yaw = output.yawRate;

    // Put the thrust commands into the "response" variable.
    // . NOTE: The thrust is commanded per motor, so divide by 4.0
    // > NOTE: The function "computeMotorPolyBackward" converts the input argument
    //         from Newtons to the 16-bit command expected by the Crazyflie.
    float thrust_request_per_motor = output.thrust / 4.0;
    response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_request_per_motor);
    response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_request_per_motor);
    response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_request_per_motor);
    response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_request_per_motor);

    // Capture data
    if (m_dataIndex < m_u_data.rows())
    {
    	// Input data
    	m_u_data(m_dataIndex,0) = output.thrust;
    	m_u_data(m_dataIndex,1) = output.rollRate;
    	m_u_data(m_dataIndex,2) = output.pitchRate;
    	m_u_data(m_dataIndex,3) = output.yawRate;

    	// Output data
    	m_y_data(m_dataIndex,0) = request.ownCrazyflie.x;
    	m_y_data(m_dataIndex,1) = request.ownCrazyflie.y;
    	m_y_data(m_dataIndex,2) = request.ownCrazyflie.z;
    	m_y_data(m_dataIndex,3) = request.ownCrazyflie.roll;
    	m_y_data(m_dataIndex,4) = request.ownCrazyflie.pitch;
    	m_y_data(m_dataIndex,5) = request.ownCrazyflie.yaw;
    }
    m_dataIndex++;

	// Increment time
    m_time_in_seconds += m_control_deltaT;

    // DEBUG INFO
    if (yaml_shouldDisplayDebugInfo)
    {
        ROS_INFO_STREAM("output.thrust = " << output.thrust);
        ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
        ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
        ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);
        ROS_INFO_STREAM("controlOutput.motorCmd1 = " << response.controlOutput.motorCmd1);
        ROS_INFO_STREAM("controlOutput.motorCmd2 = " << response.controlOutput.motorCmd2);
        ROS_INFO_STREAM("controlOutput.motorCmd3 = " << response.controlOutput.motorCmd3);
        ROS_INFO_STREAM("controlOutput.motorCmd4 = " << response.controlOutput.motorCmd4);
    }
}

void computeResponse_for_Deepc(Controller::Request &request, Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		for (int i = 0; i < 9; i++)
			m_previous_stateErrorInertial[i] = 0.0;

		// Set the change flag back to false
		m_current_state_changed = false;

		// Publish the change
		publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[DEEPC CONTROLLER] State \"Deepc\" started");
	}

	m_setpoint_for_controller[0] = m_setpoint[0];
	m_setpoint_for_controller[1] = m_setpoint[1];
	m_setpoint_for_controller[2] = m_setpoint[2];
	m_setpoint_for_controller[3] = m_setpoint[3];
	
	// Call the LQR control function
	control_output output;
	calculateControlOutput_viaLQR(request, output);

	// PREPARE AND RETURN THE VARIABLE "response"
	// Specify that using a "rate type" of command
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;

	// Put the computed body rate commands into the "response" variable
	response.controlOutput.roll = output.rollRate;
	response.controlOutput.pitch = output.pitchRate;
	response.controlOutput.yaw = output.yawRate;

	// Put the thrust commands into the "response" variable.
	// . NOTE: The thrust is commanded per motor, so divide by 4.0
	// > NOTE: The function "computeMotorPolyBackward" converts the input argument
	//         from Newtons to the 16-bit command expected by the Crazyflie.
	float thrust_request_per_motor = output.thrust / 4.0;
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_request_per_motor);

	// DEBUG INFO
	if (yaml_shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("output.thrust = " << output.thrust);
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);
		ROS_INFO_STREAM("controlOutput.motorCmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.motorCmd2 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.motorCmd3 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.motorCmd4 = " << response.controlOutput.motorCmd4);
	}

	// Set flag to solve Deepc optimization
	m_Deepc_mutex.lock();
	m_solveDeepc = true;
	m_Deepc_mutex.unlock();
}

void computeResponse_for_landing_move_down(Controller::Request &request, Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Set the current (x,y,z,yaw) location as the setpoint
		m_setpoint_for_controller[0] = request.ownCrazyflie.x;
		m_setpoint_for_controller[1] = request.ownCrazyflie.y;
		m_setpoint_for_controller[2] = yaml_landing_move_down_end_height_setpoint;
		m_setpoint_for_controller[3] = request.ownCrazyflie.yaw;
		// Set the change flag back to false
		m_current_state_changed = false;
        // Publish the change
        publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[DEEPC CONTROLLER] State \"landing move-down\" started with \"m_setpoint_for_controller\" (x,y,z,yaw) =  ( " << m_setpoint_for_controller[0] << ", " << m_setpoint_for_controller[1] << ", " << m_setpoint_for_controller[2] << ", " << m_setpoint_for_controller[3] << ")");
	}

	// Check if within the threshold of zero
	if (request.ownCrazyflie.z < yaml_landing_move_down_end_height_threshold)
	{
		// Inform the user
		ROS_INFO("[DEEPC CONTROLLER] Switch to state: landing spin motors");
		// Update the state accordingly
		m_current_state = DEEPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS;
		m_current_state_changed = true;
	}

	// Change to landing spin motors if the timeout is reached
	if (m_time_in_seconds > yaml_landing_move_down_time_max)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Did not reach the setpoint within the \"landing move down\" allowed time. Switch to state: landing spin motors");
		// Update the state accordingly
		m_current_state = DEEPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS;
		m_current_state_changed = true;
	}
	
	// Call the LQR control function
	control_output output;
	calculateControlOutput_viaLQR(request, output);

	// PREPARE AND RETURN THE VARIABLE "response"
	// Specify that using a "rate type" of command
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_RATE;

	// Put the computed body rate commands into the "response" variable
	response.controlOutput.roll = output.rollRate;
	response.controlOutput.pitch = output.pitchRate;
	response.controlOutput.yaw = output.yawRate;

	// Put the thrust commands into the "response" variable.
	// . NOTE: The thrust is commanded per motor, so divide by 4.0
	// > NOTE: The function "computeMotorPolyBackward" converts the input argument
	//         from Newtons to the 16-bit command expected by the Crazyflie.
	float thrust_request_per_motor = output.thrust / 4.0;
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_request_per_motor);

    // Increment time
    m_time_in_seconds += m_control_deltaT;

	// DEBUG INFO
	if (yaml_shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("output.thrust = " << output.thrust);
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);
		ROS_INFO_STREAM("controlOutput.motorcmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.motorcmd3 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.motorcmd2 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.motorcmd4 = " << response.controlOutput.motorCmd4);
	}
}

void computeResponse_for_landing_spin_motors(Controller::Request &request, Controller::Response &response)
{
	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		// Reset the time variable
		m_time_in_seconds = 0.0;
		// Set the change flag back to false
		m_current_state_changed = false;
        // Publish the change
        publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[DEEPC CONTROLLER] state \"landing spin motors\" started");
	}

	// Change to next state after specified time
	if (m_time_in_seconds > 0.7 * yaml_landing_spin_motors_time)
	{
		// Inform the user
		ROS_INFO("[DEEPC CONTROLLER] Publish message that landing is complete, and switch to state: standby");
		// Update the state accordingly
		m_current_state = DEEPC_CONTROLLER_STATE_STANDBY;
		m_current_state_changed = true;
		// Publish a message that the landing is complete
		IntWithHeader msg;
		msg.data = DEEPC_CONTROLLER_LANDING_COMPLETE;
		m_manoeuvreCompletePublisher.publish(msg);
		// Publish the change
		publishCurrentSetpointAndState();
	}

	// PREPARE AND RETURN THE VARIABLE "response"
	// Specify that using a "motor type" of command
	response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS;

	// Fill in zero for the angle parts
	response.controlOutput.roll  = 0.0;
	response.controlOutput.pitch = 0.0;
	response.controlOutput.yaw   = 0.0;

	// Fill in all motor thrusts as landing sping thrust
	response.controlOutput.motorCmd1 = yaml_landing_spin_motors_thrust;
	response.controlOutput.motorCmd2 = yaml_landing_spin_motors_thrust;
	response.controlOutput.motorCmd3 = yaml_landing_spin_motors_thrust;
	response.controlOutput.motorCmd4 = yaml_landing_spin_motors_thrust;

    // Increment time
    m_time_in_seconds += m_control_deltaT;

	// DEBUG INFO
	if (yaml_shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("output.thrust = " << 0.0);
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);
		ROS_INFO_STREAM("controlOutput.motorCmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.motorCmd2 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.motorCmd3 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.motorCmd4 = " << response.controlOutput.motorCmd4);
	}
}


void calculateControlOutput_viaLQR(Controller::Request &request, control_output &output)
{
	// Define a local array to fill in with the state error
	float stateErrorInertial[9];

	// Fill in the (x,y,z) position error
	stateErrorInertial[0] = request.ownCrazyflie.x - m_setpoint_for_controller[0];
	stateErrorInertial[1] = request.ownCrazyflie.y - m_setpoint_for_controller[1];
	stateErrorInertial[2] = request.ownCrazyflie.z - m_setpoint_for_controller[2];

	// Compute an estimate of the velocity
	// > As simply the derivative between the current and previous position
	stateErrorInertial[3] = (stateErrorInertial[0] - m_previous_stateErrorInertial[0]) * yaml_control_frequency;
	stateErrorInertial[4] = (stateErrorInertial[1] - m_previous_stateErrorInertial[1]) * yaml_control_frequency;
	stateErrorInertial[5] = (stateErrorInertial[2] - m_previous_stateErrorInertial[2]) * yaml_control_frequency;

	// Fill in the roll and pitch angle measurements directly
	stateErrorInertial[6] = request.ownCrazyflie.roll;
	stateErrorInertial[7] = request.ownCrazyflie.pitch;

	// Fill in the yaw angle error
	// > This error should be "unwrapped" to be in the range
	//   ( -pi , pi )
	// > First, get the yaw error into a local variable
	float yawError = request.ownCrazyflie.yaw - m_setpoint_for_controller[3];
	// > Second, "unwrap" the yaw error to the interval ( -pi , pi )
	while(yawError > PI) {yawError -= 2 * PI;}
	while(yawError < -PI) {yawError += 2 * PI;}
	// > Third, put the "yawError" into the "stateError" variable
	stateErrorInertial[8] = yawError;


	// CONVERSION INTO BODY FRAME
	// Conver the state erorr from the Inertial frame into the Body frame
	// > Note: the function "convertIntoBodyFrame" is implemented in this file
	//   and by default does not perform any conversion. The equations to convert
	//   the state error into the body frame should be implemented in that function
	//   for successful completion of the classroom exercise
	float stateErrorBody[9];
	convertIntoBodyFrame(stateErrorInertial, stateErrorBody, request.ownCrazyflie.yaw);


	// SAVE THE STATE ERROR TO BE USED NEXT TIME THIS FUNCTION IS CALLED
	// > as we have already used previous error we can now update it update it
	for(int i = 0; i < 9; ++i)
	{
		m_previous_stateErrorInertial[i] = stateErrorInertial[i];
	}


	// PERFORM THE "u=-Kx" CONTROLLER COMPUTATIONS

	// Initialize control output
	output.thrust = 0;
	output.rollRate = 0;
	output.pitchRate = 0;
	output.yawRate = 0;

	// Perform the "-Kx" LQR computation
	for(int i = 0; i < 9; ++i)
	{
		// For the z-controller
		output.thrust -= yaml_gainMatrixThrust_NineStateVector[i] * stateErrorBody[i];
		// For the x-controller
		output.pitchRate -= yaml_gainMatrixPitchRate[i] * stateErrorBody[i];
		// For the y-controller
		output.rollRate -= yaml_gainMatrixRollRate[i] * stateErrorBody[i];
		// For the yaw-controller
		output.yawRate -= yaml_gainMatrixYawRate[i] * stateErrorBody[i];
	}

	// Feedforward thrust command
	output.thrust += m_cf_weight_in_newtons;

	// DEBUG INFO
	if (yaml_shouldDisplayDebugInfo)
	{
		ROS_INFO_STREAM("x-coordinates: " << request.ownCrazyflie.x);
		ROS_INFO_STREAM("y-coordinates: " << request.ownCrazyflie.y);
		ROS_INFO_STREAM("z-coordinates: " << request.ownCrazyflie.z);
		ROS_INFO_STREAM("roll: " << request.ownCrazyflie.roll);
		ROS_INFO_STREAM("pitch: " << request.ownCrazyflie.pitch);
		ROS_INFO_STREAM("yaw: " << request.ownCrazyflie.yaw);
		ROS_INFO_STREAM("Delta t: " << request.ownCrazyflie.acquiringTime);
	}
}

//    ------------------------------------------------------------------------------
//    RRRR    OOO   TTTTT    A    TTTTT  EEEEE       III  N   N  TTTTT   OOO
//    R   R  O   O    T     A A     T    E            I   NN  N    T    O   O
//    RRRR   O   O    T    A   A    T    EEE          I   N N N    T    O   O
//    R  R   O   O    T    AAAAA    T    E            I   N  NN    T    O   O
//    R   R   OOO     T    A   A    T    EEEEE       III  N   N    T     OOO
//
//    BBBB    OOO   DDDD   Y   Y       FFFFF  RRRR     A    M   M  EEEEE
//    B   B  O   O  D   D   Y Y        F      R   R   A A   MM MM  E
//    BBBB   O   O  D   D    Y         FFF    RRRR   A   A  M M M  EEE
//    B   B  O   O  D   D    Y         F      R  R   AAAAA  M   M  E
//    BBBB    OOO   DDDD     Y         F      R   R  A   A  M   M  EEEEE
//    ----------------------------------------------------------------------------------

// The arguments for this function are as follows:
// stateInertial
// This is an array of length 9 with the estimates the error of of the following values
// relative to the sepcifed setpoint:
//     stateInertial[0]    x position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[1]    y position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[2]    z position of the Crazyflie relative to the inertial frame origin [meters]
//     stateInertial[3]    x-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[4]    y-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[5]    z-axis component of the velocity of the Crazyflie in the inertial frame [meters/second]
//     stateInertial[6]    The roll  component of the intrinsic Euler angles [radians]
//     stateInertial[7]    The pitch component of the intrinsic Euler angles [radians]
//     stateInertial[8]    The yaw   component of the intrinsic Euler angles [radians]
// 
// stateBody
// This is an empty array of length 9, this function should fill in all elements of this
// array with the same ordering as for the "stateInertial" argument, expect that the (x,y)
// position and (x,y) velocities are rotated into the body frame.
//
// yaw_measured
// This is the yaw component of the intrinsic Euler angles in [radians] as measured by
// the Vicon motion capture system
//
void convertIntoBodyFrame(float stateInertial[9], float (&stateBody)[9], float yaw_measured)
{
	float sinYaw = sin(yaw_measured);
	float cosYaw = cos(yaw_measured);

	// Fill in the (x,y,z) position estimates to be returned
	stateBody[0] = stateInertial[0] * cosYaw  +  stateInertial[1] * sinYaw;
	stateBody[1] = stateInertial[1] * cosYaw  -  stateInertial[0] * sinYaw;
	stateBody[2] = stateInertial[2];

	// Fill in the (x,y,z) velocity estimates to be returned
	stateBody[3] = stateInertial[3] * cosYaw  +  stateInertial[4] * sinYaw;
	stateBody[4] = stateInertial[4] * cosYaw  -  stateInertial[3] * sinYaw;
	stateBody[5] = stateInertial[5];

	// Fill in the (roll,pitch,yaw) estimates to be returned
	stateBody[6] = stateInertial[6];
	stateBody[7] = stateInertial[7];
	stateBody[8] = stateInertial[8];
}





//    ------------------------------------------------------------------------------
//    N   N  EEEEE  W     W   TTTTT   OOO   N   N        CCCC  M   M  DDDD
//    NN  N  E      W     W     T    O   O  NN  N       C      MM MM  D   D
//    N N N  EEE    W     W     T    O   O  N N N  -->  C      M M M  D   D
//    N  NN  E       W W W      T    O   O  N  NN       C      M   M  D   D
//    N   N  EEEEE    W W       T     OOO   N   N        CCCC  M   M  DDDD
//
//     CCCC   OOO   N   N  V   V  EEEEE  RRRR    SSSS  III   OOO   N   N
//    C      O   O  NN  N  V   V  E      R   R  S       I   O   O  NN  N
//    C      O   O  N N N  V   V  EEE    RRRR    SSS    I   O   O  N N N
//    C      O   O  N  NN   V V   E      R  R       S   I   O   O  N  NN
//     CCCC   OOO   N   N    V    EEEEE  R   R  SSSS   III   OOO   N   N
//    ----------------------------------------------------------------------------------

float computeMotorPolyBackward(float thrust)
{
	// Compute the 16-but command that would produce the requested
	// "thrust" based on the quadratic mapping that is described
	// by the coefficients in the "yaml_motorPoly" variable.
	float cmd_16bit = (-yaml_motorPoly[1] + sqrt(yaml_motorPoly[1] * yaml_motorPoly[1] - 4 * yaml_motorPoly[2] * (yaml_motorPoly[0] - thrust))) / (2 * yaml_motorPoly[2]);

	// Saturate the signal to be 0 or in the range [1000,65000]
	if (cmd_16bit < yaml_command_sixteenbit_min)
	{
		cmd_16bit = 0;
	}
	else if (cmd_16bit > yaml_command_sixteenbit_max)
	{
		cmd_16bit = yaml_command_sixteenbit_max;
	}
	// Return the result
	return cmd_16bit;
}





//    ----------------------------------------------------------------------------------
//    N   N  EEEEE  W     W        SSSS  EEEEE  TTTTT  PPPP    OOO   III  N   N  TTTTT
//    NN  N  E      W     W       S      E        T    P   P  O   O   I   NN  N    T
//    N N N  EEE    W     W        SSS   EEE      T    PPPP   O   O   I   N N N    T
//    N  NN  E       W W W            S  E        T    P      O   O   I   N  NN    T
//    N   N  EEEEE    W W         SSSS   EEEEE    T    P       OOO   III  N   N    T
//
//     CCCC    A    L      L      BBBB     A     CCCC  K   K
//    C       A A   L      L      B   B   A A   C      K  K
//    C      A   A  L      L      BBBB   A   A  C      KKK
//    C      AAAAA  L      L      B   B  AAAAA  C      K  K
//     CCCC  A   A  LLLLL  LLLLL  BBBB   A   A   CCCC  K   K
//    ----------------------------------------------------------------------------------


// REQUEST SETPOINT CHANGE CALLBACK
void requestSetpointChangeCallback(const SetpointWithHeader& newSetpoint)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , newSetpoint.shouldCheckForAgentID , newSetpoint.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Check if the request if for the default setpoint
		if (newSetpoint.buttonID == REQUEST_DEFAULT_SETPOINT_BUTTON_ID)
		{
			setNewSetpoint(
					yaml_default_setpoint[0],
					yaml_default_setpoint[1],
					yaml_default_setpoint[2],
					yaml_default_setpoint[3]
				);
		}
		else
		{
			// Call the function for actually setting the setpoint
			setNewSetpoint(
					newSetpoint.x,
					newSetpoint.y,
					newSetpoint.z,
					newSetpoint.yaw
				);
		}
	}
}


// CHANGE SETPOINT FUNCTION
void setNewSetpoint(float x, float y, float z, float yaw)
{
	// Put the new setpoint into the class variable
	m_setpoint[0] = x;
	m_setpoint[1] = y;
	m_setpoint[2] = z;
	m_setpoint[3] = yaw;

	// Publish the change so that the network is updated
	// (mainly the "flying agent GUI" is interested in
	// displaying this change to the user)
	publishCurrentSetpointAndState();
}


// GET CURRENT SETPOINT SERVICE CALLBACK
bool getCurrentSetpointCallback(GetSetpointService::Request &request, GetSetpointService::Response &response)
{
	// Directly put the current setpoint into the response
	response.setpointWithHeader.x   = m_setpoint[0];
	response.setpointWithHeader.y   = m_setpoint[1];
	response.setpointWithHeader.z   = m_setpoint[2];
	response.setpointWithHeader.yaw = m_setpoint[3];
	// Return
	return true;
}


// PUBLISH THE CURRENT SETPOINT SO THAT THE NETWORK IS UPDATED
void publishCurrentSetpointAndState()
{
	// Instantiate a local variable of type "SetpointWithHeader"
	SetpointWithHeader msg;
	// Fill in the setpoint
	msg.x   = m_setpoint[0];
	msg.y   = m_setpoint[1];
	msg.z   = m_setpoint[2];
	msg.yaw = m_setpoint[3];
	// Put the current state into the "buttonID" field
	msg.buttonID = m_current_state;
	// Publish the message
	m_setpointChangedPublisher.publish(msg);
}



//    ----------------------------------------------------------------------------------
//     CCCC  U   U   SSSS  TTTTT   OOO   M   M
//    C      U   U  S        T    O   O  MM MM
//    C      U   U   SSS     T    O   O  M M M
//    C      U   U      S    T    O   O  M   M
//     CCCC   UUU   SSSS     T     OOO   M   M
//
//     CCCC   OOO   M   M  M   M    A    N   N  DDDD
//    C      O   O  MM MM  MM MM   A A   NN  N  D   D
//    C      O   O  M M M  M M M  A   A  N N N  D   D
//    C      O   O  M   M  M   M  AAAAA  N  NN  D   D
//     CCCC   OOO   M   M  M   M  A   A  N   N  DDDD
//    ----------------------------------------------------------------------------------

// CUSTOM COMMAND RECEIVED CALLBACK
void customCommandReceivedCallback(const CustomButtonWithHeader& commandReceived)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , commandReceived.shouldCheckForAgentID , commandReceived.agentIDs );

	if (isRevelant)
	{
		// Extract the data from the message
		int custom_button_index = commandReceived.button_index;
		float float_data        = commandReceived.float_data;
		int int_data = int(float_data);
		bool bool_data[32];
		for (int i = 0; i < 32; i++)
		{
			bool_data[i] = (int_data >> i) & 1;
		}

		// Switch between the button pressed
		switch(custom_button_index)
		{

			// > FOR CUSTOM BUTTON 1 - EXCITATION
			case 1:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[DEEPC CONTROLLER] Button 1 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 1
				processCustomButton1(float_data, int_data, bool_data);

                break;

			// > FOR CUSTOM BUTTON 2 - SETUP GUROBI OPTIMIZATION
			case 2:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[DEEPC CONTROLLER] Button 2 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 2
                processCustomButton2(float_data, int_data, bool_data);

				break;

			// > FOR CUSTOM BUTTON 3 - DEEPC
			case 3:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[DEEPC CONTROLLER] Button 3 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 3
                processCustomButton3(float_data, int_data, bool_data);

				break;

			// > FOR CUSTOM BUTTON 4 - SPARE
			case 4:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[DEEPC CONTROLLER] Button 4 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 4
                
				break;

			// > FOR CUSTOM BUTTON 5 - SPARE
			case 5:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[DEEPC CONTROLLER] Button 5 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 5
                
                break;

			default:
				// Let the user know that the command was not recognised
				ROS_INFO_STREAM("[DEEPC CONTROLLER] A button clicked command was received in the controller but not recognised, message.button_index = " << custom_button_index << ", and message.float_data = " << float_data );
				break;
		}
	}
}

// CUSTOM BUTTON 1 - EXCITATION
void processCustomButton1(float float_data, int int_data, bool* bool_data)
{
	// Button data decoding:
	// int_data		== 0 => Excite all
	// bool_data[0]	== 1 => Excite thrust
	// bool_data[1]	== 1 => Excite roll rate
	// bool_data[2]	== 1 => Excite pitch rate
	// bool_data[3]	 == 1 => Excite yaw rate 
	
    // Switch between the possible states
    switch (m_current_state)
    {
        case DEEPC_CONTROLLER_STATE_LQR:
            // Inform the user
            ROS_INFO("[DEEPC CONTROLLER] Received request to start excitation. Switch to state: excitation");
            // Update the state accordingly
            m_current_state = DEEPC_CONTROLLER_STATE_EXCITATION;
            m_current_state_changed = true;
            if (!int_data)
            {
            	m_thrustExcEnable = true;
            	m_rollRateExcEnable = true;
            	m_pitchRateExcEnable = true;
            	m_yawRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[DEEPC CONTROLLER] Exciting all");
            }
            if (bool_data[0])
            {
            	m_thrustExcEnable = true;
            	// Inform the user
            	ROS_INFO("[DEEPC CONTROLLER] Exciting thrust");
            }
            if (bool_data[1])
            {
            	m_rollRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[DEEPC CONTROLLER] Exciting roll rate");
            }
            if (bool_data[2])
            {
            	m_pitchRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[DEEPC CONTROLLER] Exciting pitch rate");
            }
            if (bool_data[3])
            {
            	m_yawRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[DEEPC CONTROLLER] Exciting yaw rate");
            }
            break;

        case DEEPC_CONTROLLER_STATE_EXCITATION:
            // Inform the user
            ROS_INFO("[DEEPC CONTROLLER] Received request to stop excitation. Switch to state: LQR");
            // Update the state accordingly
            m_current_state = DEEPC_CONTROLLER_STATE_LQR;
            m_current_state_changed = true;
            m_write_data = true;
            break;

        case DEEPC_CONTROLLER_STATE_DEEPC:
        case DEEPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
        case DEEPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
        case DEEPC_CONTROLLER_STATE_STANDBY:
        default:
            // Inform the user
            ROS_INFO("[DEEPC CONTROLLER] Received request to start excitation in invalid state. Request ignored");
            break;
    }
}

// CUSTOM BUTTON 2 - SETUP GUROBI OPTIMIZATION
void processCustomButton2(float float_data, int int_data, bool* bool_data)
{
	// Inform the user
    ROS_INFO("[DEEPC CONTROLLER] Received request to setup Deepc optimization");

    m_Deepc_mutex.lock();
    m_setupDeepc = true;
    m_Deepc_mutex.unlock();
}

// CUSTOM BUTTON 3 - DEEPC
void processCustomButton3(float float_data, int int_data, bool* bool_data)
{	
    // Check if Deepc optimization was setup successfully
    if (!m_grb_setup_success)
    {
    	// Inform the user
        ROS_INFO("[DEEPC CONTROLLER] Received request to start Deepc but optimization is not setup successfully. Request ignored");
        return;
    }

    // Switch between the possible states
    switch (m_current_state)
    {
        case DEEPC_CONTROLLER_STATE_LQR:
            // Inform the user
            ROS_INFO("[DEEPC CONTROLLER] Received request to start Deepc. Switch to state: Deepc");
            // Update the state accordingly
            m_current_state = DEEPC_CONTROLLER_STATE_DEEPC;
            m_current_state_changed = true;
            break;

        case DEEPC_CONTROLLER_STATE_DEEPC:
            // Inform the user
            ROS_INFO("[DEEPC CONTROLLER] Received request to stop Deepc. Switch to state: LQR");
            // Update the state accordingly
            m_current_state = DEEPC_CONTROLLER_STATE_LQR;
            m_current_state_changed = true;
            break;

        case DEEPC_CONTROLLER_STATE_EXCITATION:
        case DEEPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
        case DEEPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
        case DEEPC_CONTROLLER_STATE_STANDBY:
        default:
            // Inform the user
            ROS_INFO("[DEEPC CONTROLLER] Received request to start Deepc in invalid state. Request ignored");
            break;
    }
}

//    ----------------------------------------------------------------------------------
//    L       OOO     A    DDDD
//    L      O   O   A A   D   D
//    L      O   O  A   A  D   D
//    L      O   O  AAAAA  D   D
//    LLLLL   OOO   A   A  DDDD
//
//    PPPP     A    RRRR     A    M   M  EEEEE  TTTTT  EEEEE  RRRR    SSSS
//    P   P   A A   R   R   A A   MM MM  E        T    E      R   R  S
//    PPPP   A   A  RRRR   A   A  M M M  EEE      T    EEE    RRRR    SSS
//    P      AAAAA  R  R   AAAAA  M   M  E        T    E      R  R       S
//    P      A   A  R   R  A   A  M   M  EEEEE    T    EEEEE  R   R  SSSS
//    ----------------------------------------------------------------------------------


// CALLBACK NOTIFYING THAT THE YAML PARAMETERS ARE READY TO BE LOADED
void isReadyDeepcControllerYamlCallback(const IntWithHeader & msg)
{
	// Check whether the message is relevant
	bool isRevelant = checkMessageHeader( m_agentID , msg.shouldCheckForAgentID , msg.agentIDs );

	// Continue if the message is relevant
	if (isRevelant)
	{
		// Extract the data
		int parameter_service_to_load_from = msg.data;
		// Initialise a local variable for the namespace
		string namespace_to_use;
		// Load from the respective parameter service
		switch(parameter_service_to_load_from)
		{
			// > FOR FETCHING FROM THE AGENT'S OWN PARAMETER SERVICE
			case LOAD_YAML_FROM_AGENT:
			{
				ROS_INFO("[DEEPC CONTROLLER] Now fetching the DeepcController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[DEEPC CONTROLLER] Now fetching the DeepcController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[DEEPC CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchDeepcControllerYamlParameters(nodeHandle_to_use);
	}
}


// LOADING OF THE YAML PARAMTERS
void fetchDeepcControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// DeepcController.yaml

	// Add the "DeepcController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "DeepcController");

	// GET THE PARAMETERS:

	// ------------------------------------------------------
	// PARAMTERS FOR THE LANDING MANOEUVRE

	// Height change for the landing move-down
	yaml_landing_move_down_end_height_setpoint  = getParameterFloat(nodeHandle_for_paramaters , "landing_move_down_end_height_setpoint");
	yaml_landing_move_down_end_height_threshold = getParameterFloat(nodeHandle_for_paramaters , "landing_move_down_end_height_threshold");
	// The time for: landing move-down
	yaml_landing_move_down_time_max = getParameterFloat(nodeHandle_for_paramaters , "landing_move_down_time_max");

	// The thrust for landing spin motors
	yaml_landing_spin_motors_thrust = getParameterFloat(nodeHandle_for_paramaters , "landing_spin_motors_thrust");
	// The time for: landing spin motors
	yaml_landing_spin_motors_time = getParameterFloat(nodeHandle_for_paramaters , "landing_spin_motors_time");


	// ------------------------------------------------------
	// PARAMTERS THAT ARE STANDARD FOR A "CONTROLLER SERVICE"

	// > The mass of the crazyflie
	yaml_cf_mass_in_grams = getParameterFloat(nodeHandle_for_paramaters , "mass");

	// > The frequency at which the "computeControlOutput" is being called,
	//   as determined by the frequency at which the motion capture system
	//   provides position and attitude data
	yaml_control_frequency = getParameterFloat(nodeHandle_for_paramaters, "control_frequency");

	// > The co-efficients of the quadratic conversation from 16-bit motor
	//   command to thrust force in Newtons
	getParameterFloatVector(nodeHandle_for_paramaters, "motorPoly", yaml_motorPoly, 3);

	// > The min and max for saturating 16 bit thrust commands
	yaml_command_sixteenbit_min = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_min");
	yaml_command_sixteenbit_max = getParameterFloat(nodeHandle_for_paramaters, "command_sixteenbit_max");

	// The default setpoint, the ordering is (x,y,z,yaw),
	// with unit [meters,meters,meters,radians]
	getParameterFloatVector(nodeHandle_for_paramaters, "default_setpoint", yaml_default_setpoint, 4);

	// Boolean indiciating whether the "Debug Message" of this agent
	// should be published or not
	yaml_shouldPublishDebugMessage = getParameterBool(nodeHandle_for_paramaters, "shouldPublishDebugMessage");

	// Boolean indiciating whether the debugging ROS_INFO_STREAM should
	// be displayed or not
	yaml_shouldDisplayDebugInfo = getParameterBool(nodeHandle_for_paramaters, "shouldDisplayDebugInfo");

	// The LQR Controller parameters
	// The LQR Controller parameters for "LQR_MODE_RATE"
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixThrust_NineStateVector", yaml_gainMatrixThrust_NineStateVector, 9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixRollRate",               yaml_gainMatrixRollRate,               9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixPitchRate",              yaml_gainMatrixPitchRate,              9);
	getParameterFloatVector(nodeHandle_for_paramaters, "gainMatrixYawRate",                yaml_gainMatrixYawRate,                9);

	// Thrust excitation parameters
	yaml_thrustExcAmp_in_grams = getParameterFloat(nodeHandle_for_paramaters, "thrustExcAmp");
	yaml_thrustExcSignalFile = getParameterString(nodeHandle_for_paramaters, "thrustExcSignalFile");

	// Roll rate excitation parameters
	yaml_rollRateExcAmp_in_deg = getParameterFloat(nodeHandle_for_paramaters, "rollRateExcAmp");
	yaml_rollRateExcSignalFile = getParameterString(nodeHandle_for_paramaters, "rollRateExcSignalFile");

	// Pitch rate excitation parameters
	yaml_pitchRateExcAmp_in_deg = getParameterFloat(nodeHandle_for_paramaters, "pitchRateExcAmp");
	yaml_pitchRateExcSignalFile = getParameterString(nodeHandle_for_paramaters, "pitchRateExcSignalFile");

	// Yaw rate perturbation parameters
	yaml_yawRateExcAmp_in_deg = getParameterFloat(nodeHandle_for_paramaters, "yawRateExcAmp");
	yaml_yawRateExcSignalFile = getParameterString(nodeHandle_for_paramaters, "yawRateExcSignalFile");

	// Excitation start time, in s. Used to collect steady-state data before excitation
	yaml_exc_start_time = getParameterFloat(nodeHandle_for_paramaters, "exc_start_time");


	// > DEBUGGING: Print out one of the parameters that was loaded to
	//   debug if the fetching of parameters worked correctly
	ROS_INFO_STREAM("[DEEPC CONTROLLER] DEBUGGING: the fetched DeepcController/mass = " << yaml_cf_mass_in_grams);


	// PROCESS THE PARAMTERS

	// > Compute the feed-forward force that we need to counteract
	//   gravity (i.e., mg) in units of [Newtons]
	m_cf_weight_in_newtons = yaml_cf_mass_in_grams * 9.81/1000.0;

	// > Convert the control frequency to a delta T
	m_control_deltaT = 1.0 / yaml_control_frequency;

	// > Compute the thrust excitation force in units of [Newtons]
	m_thrustExcAmp_in_newtons = yaml_thrustExcAmp_in_grams * 9.81/1000.0;

	// > Compute the roll rate excitation in units of [rad/s]
	m_rollRateExcAmp_in_rad = yaml_rollRateExcAmp_in_deg * PI/180.0;

	// > Compute the pitch rate excitation in units of [rad/s]
	m_pitchRateExcAmp_in_rad = yaml_pitchRateExcAmp_in_deg * PI/180.0;

	// > Compute the yaw rate excitation in units of [rad/s]
	m_yawRateExcAmp_in_rad = yaml_yawRateExcAmp_in_deg * PI/180.0;

	// > Get the excitation signals from files
	m_thrustExcSignal = read_csv(HOME + yaml_dataFolder + yaml_thrustExcSignalFile);
	if (m_thrustExcSignal.size() <= 0)
		ROS_INFO("[DEEPC CONTROLLER] Failed to read thrust excitation signal file");
	else
	{
		int exc_start_time_d = int(yaml_exc_start_time / m_control_deltaT);
		m_u_data.setZero(exc_start_time_d + m_thrustExcSignal.size(), 4);
		m_y_data.setZero(exc_start_time_d +m_thrustExcSignal.size(), 6);
	}
	
	m_rollRateExcSignal = read_csv(HOME + yaml_dataFolder + yaml_rollRateExcSignalFile);
	if (m_rollRateExcSignal.size() <= 0)
		ROS_INFO("[DEEPC CONTROLLER] Failed to read roll rate excitation signal file");
	
	m_pitchRateExcSignal = read_csv(HOME + yaml_dataFolder + yaml_pitchRateExcSignalFile);
	if (m_pitchRateExcSignal.size() <= 0)
		ROS_INFO("[DEEPC CONTROLLER] Failed to read pitch rate excitation signal file");
	
	m_yawRateExcSignal = read_csv(HOME + yaml_dataFolder + yaml_yawRateExcSignalFile);
	if (m_yawRateExcSignal.size() <= 0)
		ROS_INFO("[DEEPC CONTROLLER] Failed to read yaw rate excitation signal file");

	// PARAMETERS ACCESSED BY DEEPC THREAD
	m_Deepc_mutex.lock();
	
	// Data folder locations
	yaml_dataFolder = getParameterString(nodeHandle_for_paramaters, "dataFolder");
	yaml_outputFolder = getParameterString(nodeHandle_for_paramaters, "outputFolder");
	yaml_logFolder = getParameterString(nodeHandle_for_paramaters, "logFolder");
	
	// Gurobi optimization parameters
	yaml_grb_LogToFile = getParameterBool(nodeHandle_for_paramaters, "grb_LogToFile");
	yaml_grb_LogToConsole = getParameterBool(nodeHandle_for_paramaters, "grb_LogToConsole");

	// > Get absolute output data folder location
	m_outputFolder = HOME + yaml_dataFolder + yaml_outputFolder;

	// > Get absolute log files folder location
	m_logFolder = HOME + yaml_dataFolder + yaml_logFolder;

	// > Set flag for Deepc thread to update parameters
	m_params_changed = true;

	m_Deepc_mutex.lock();

	// DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[DEEPC CONTROLLER] DEBUGGING: thus the weight of this agent in [Newtons] = " << m_cf_weight_in_newtons);
}


//    ----------------------------------------------------------------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ----------------------------------------------------------------------------------


// This function does NOT need to be edited 
int main(int argc, char* argv[]) {

	// Starting the ROS-node
	ros::init(argc, argv, "DeepcControllerService");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "DeepcControllerService" node
	string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[DEEPC CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



	// AGENT ID AND COORDINATOR ID

	// NOTES:
	// > If you look at the "Agent.launch" file in the "launch" folder,
	//   you will see the following line of code:
	//   <param name="agentID" value="$(optenv ROS_NAMESPACE)" />
	//   This line of code adds a parameter named "agentID" to the
	//   "FlyingAgentClient" node.
	// > Thus, to get access to this "agentID" paremeter, we first
	//   need to get a handle to the "FlyingAgentClient" node within which this
	//   controller service is nested.


	// Get the ID of the agent and its coordinator
	bool isValid_IDs = getAgentIDandCoordIDfromClientNode( m_namespace + "/FlyingAgentClient" , &m_agentID , &m_coordID);

	// Stall the node IDs are not valid
	if ( !isValid_IDs )
	{
		ROS_ERROR("[DEEPC CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[DEEPC CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
	}



	// PARAMETER SERVICE NAMESPACE AND NODEHANDLES:

	// NOTES:
	// > The parameters that are specified thorugh the *.yaml files
	//   are managed by a separate node called the "Parameter Service"
	// > A separate node is used for reasons of speed and generality
	// > To allow for a distirbuted architecture, there are two
	//   "ParamterService" nodes that are relevant:
	//   1) the one that is nested under the "m_agentID" namespace
	//   2) the one that is nested under the "m_coordID" namespace
	// > The following lines of code create the namespace (as strings)
	//   to there two relevant "ParameterService" nodes.
	// > The node handles are also created because they are needed
	//   for the ROS Subscriptions that follow.

	// Set the class variable "m_namespace_to_own_agent_parameter_service",
	// i.e., the namespace of parameter service for this agent
	m_namespace_to_own_agent_parameter_service = m_namespace + "/ParameterService";

	// Set the class variable "m_namespace_to_coordinator_parameter_service",
	// i.e., the namespace of parameter service for this agent's coordinator
	constructNamespaceForCoordinatorParameterService( m_coordID, m_namespace_to_coordinator_parameter_service );

	// Inform the user of what namespaces are being used
	ROS_INFO_STREAM("[DEEPC CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[DEEPC CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber safeContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "DeepcController", 1, isReadyDeepcControllerYamlCallback);
	ros::Subscriber safeContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("DeepcController", 1, isReadyDeepcControllerYamlCallback);



	// GIVE YAML VARIABLES AN INITIAL VALUE
	// This can be done either here or as part of declaring the
	// variables in the header file




	// FETCH ANY PARAMETERS REQUIRED FROM THE "PARAMETER SERVICES"

	// The yaml files for the controllers are not added to
	// "Parameter Service" as part of launching.
	// The process for loading the yaml parameters is to send a
	// service call containing the filename of the *.yaml file,
	// and then a message will be received on the above subscribers
	// when the paramters are ready.
	// > NOTE IMPORTANTLY that by using a serice client
	//   we stall the availability of this node until the
	//   paramter service is ready

	// Create the service client as a local variable
	ros::ServiceClient requestLoadYamlFilenameServiceClient = nodeHandle_to_own_agent_parameter_service.serviceClient<LoadYamlFromFilename>("requestLoadYamlFilename", false);
	// Create the service call as a local variable
	LoadYamlFromFilename loadYamlFromFilenameCall;
	// Specify the Yaml filename as a string
	loadYamlFromFilenameCall.request.stringWithHeader.data = "DeepcController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForAgentID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyDeepcControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
		// Inform the user
		ROS_ERROR("[DEEPC CONTROLLER] The request load yaml file service call failed.");
	}




    // PUBLISHERS AND SUBSCRIBERS

    // Instantiate the class variable "m_debugPublisher" to be a
    // "ros::Publisher". This variable advertises under the name
    // "DebugTopic" and is a message with the structure defined
    //  in the file "DebugMsg.msg" (located in the "msg" folder).
    m_debugPublisher = nodeHandle.advertise<DebugMsg>("DebugTopic", 1);

	// Instantiate the local variable "requestSetpointChangeSubscriber"
	// to be a "ros::Subscriber" type variable that subscribes to the
	// "RequestSetpointChange" topic and calls the class function
	// "requestSetpointChangeCallback" each time a messaged is received
	// on this topic and the message is passed as an input argument to
	// the callback function. This subscriber will mainly receive
	// messages from the "flying agent GUI" when the setpoint is changed
	// by the user.
	ros::Subscriber requestSetpointChangeSubscriber = nodeHandle.subscribe("RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	string namespace_to_coordinator;
	constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("DeepcControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

	// Instantiate the class variable "m_setpointChangedPublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "SetpointChanged" and is a message with the structure defined
	// in the file "SetpointWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// field that displays the current setpoint for this controller.
	m_setpointChangedPublisher = nodeHandle.advertise<SetpointWithHeader>("SetpointChanged", 1);

	// Instantiate the local variable "getCurrentSetpointService" to be
	// a "ros::ServiceServer" type variable that advertises the service
	// called "GetCurrentSetpoint". This service has the input-output
	// behaviour defined in the "GetSetpointService.srv" file (located
	// in the "srv" folder). This service, when called, is provided with
	// an integer (that is essentially ignored), and is expected to respond
	// with the current setpoint of the controller. When a request is made
	// of this service the "getCurrentSetpointCallback" function is called.
	ros::ServiceServer getCurrentSetpointService = nodeHandle.advertiseService("GetCurrentSetpoint", getCurrentSetpointCallback);



    // Instantiate the local variable "service" to be a "ros::ServiceServer" type
    // variable that advertises the service called "DeepcController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("DeepcController", calculateControlOutput);

    // Instantiate the local variable "customCommandSubscriber" to be a "ros::Subscriber"
    // type variable that subscribes to the "GUIButton" topic and calls the class
    // function "customCommandReceivedCallback" each time a messaged is received on this topic
    // and the message received is passed as an input argument to the callback function.
    ros::Subscriber customCommandReceivedSubscriber = nodeHandle.subscribe("CustomButtonPressed", 1, customCommandReceivedCallback);

    // Same again but instead for changes requested by the coordinator.
	// For this we need to first create a node handle to the coordinator:
	//string namespace_to_coordinator;
	//constructNamespaceForCoordinator( m_coordID, namespace_to_coordinator );
	//ros::NodeHandle nodeHandle_to_coordinator(namespace_to_coordinator);
	// And now we can instantiate the subscriber:
	ros::Subscriber customCommandReceivedSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("DeepcControllerService/CustomButtonPressed", 1, customCommandReceivedCallback);


	// Instantiate the local variable "service" to be a "ros::ServiceServer"
	// type variable that advertises the service called:
	// >> "RequestManoeuvre"
	// This service has the input-output behaviour defined in the
	// "IntIntService.srv" file (located in the "srv" folder).
	// This service, when called, is provided with what manoeuvre
	// is requested and responds with the duration that menoeuvre
	// will take to perform (in milliseconds)
	ros::ServiceServer requestManoeuvreService = nodeHandle.advertiseService("RequestManoeuvre", requestManoeuvreCallback);

	// Instantiate the class variable "m_manoeuvreCompletePublisher" to
	// be a "ros::Publisher". This variable advertises under the name
	// "ManoeuvreComplete" and is a message with the structure defined
	// in the file "IntWithHeader.msg" (located in the "msg" folder).
	// This publisher is used by the "flying agent GUI" to update the
	// flying state once the manoeuvre is complete.
	m_manoeuvreCompletePublisher = nodeHandle.advertise<IntWithHeader>("ManoeuvreComplete", 1);

	// Create thread for solving Deepc optimization
	boost::thread deepc_thread(deepc_thread_main);

    // Print out some information to the user.
    ROS_INFO("[DEEPC CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Wait for Deepc thread to finish
    deepc_thread.join();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
