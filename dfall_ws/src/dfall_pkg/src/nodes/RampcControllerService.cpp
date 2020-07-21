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
//    A RAMPC Controller
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/RampcControllerService.h"









void setup_Rampc_mpc()
{
	try
    {
    	d_num_outputs = 3;

    	s_Rampc_mutex.lock();
		d_setpoint = s_setpoint;
		s_Rampc_mutex.unlock();

    	// COMPUTE THE NUMBER OF DECICION VARIABLES
    	int num_inputs = 3;
    	int num_states = 8;
    	int num_input_by_state = num_inputs + num_states;
    	int num_input_dec_vars = d_N     * num_inputs;
    	int num_state_dec_vars = (d_N+1) * num_states;

    	int num_mpc_dec_vars = num_input_dec_vars + num_state_dec_vars;


    	// INFORM THE USER
    	ROS_INFO_STREAM("[MPC CONTROLLER] now setting up with num input = " << num_input_dec_vars << ", num state = " << num_state_dec_vars << ", total num dec vars = " << num_mpc_dec_vars );

    	// SPECIFY THE Q, R, P MATRICES
		MatrixXf Q_cost_mat = MatrixXf::Zero(num_states, num_states);
		Q_cost_mat(0,0) = 40.0f;
		Q_cost_mat(1,1) = 40.0f;
		Q_cost_mat(2,2) = 40.0f;
		Q_cost_mat(3,3) = 0.1f;
		Q_cost_mat(3,4) = 0.1f;
		Q_cost_mat(5,5) = 0.1f;
		Q_cost_mat(6,6) = 0.1f;
		Q_cost_mat(7,7) = 0.1f;

		MatrixXf R_cost_mat = MatrixXf::Zero(num_inputs, num_inputs);
		R_cost_mat(0,0) = 160.0f;
		R_cost_mat(1,1) = 4.0f;
		R_cost_mat(2,2) = 4.0f;

		MatrixXf P_cost_mat = MatrixXf::Zero(num_states, num_states);
		P_cost_mat(0,0) = 661.21f;
		P_cost_mat(1,1) = 661.21f;
		P_cost_mat(2,2) = 382.33f;
		P_cost_mat(3,3) = 100.92f;
		P_cost_mat(4,4) = 100.92f;
		P_cost_mat(5,5) = 26.93f;
		P_cost_mat(6,6) = 633.60f;
		P_cost_mat(7,7) = 633.60f;

		P_cost_mat(3,0) =  202.82f;
		P_cost_mat(0,3) =  202.82f;

		P_cost_mat(7,0) =  317.06f;
		P_cost_mat(0,7) =  317.06f;

		P_cost_mat(1,4) =  202.82f;
		P_cost_mat(4,1) =  202.82f;

		P_cost_mat(1,6) = -317.06f;
		P_cost_mat(6,1) = -317.06f;

		P_cost_mat(2,5) =   64.00f;
		P_cost_mat(5,2) =   64.00f;

		P_cost_mat(3,7) =  201.77f;
		P_cost_mat(7,3) =  201.77f;

		P_cost_mat(4,6) =  -201.77f;
		P_cost_mat(6,4) =  -201.77f;

//       0       1       2       3       4       5       6       7
// 0:  657.21    0.00   -0.00  202.82    0.00   -0.00    0.00  317.06
// 1:    0.00  657.21   -0.00    0.00  202.82   -0.00 -317.06   -0.00
// 2:   -0.00   -0.00  378.33   -0.00   -0.00   64.00    0.00   -0.00
// 3:  202.82    0.00   -0.00   96.92   -0.00   -0.00    0.00  201.77
// 4:    0.00  202.82   -0.00   -0.00   96.92   -0.00 -201.77   -0.00
// 5:   -0.00   -0.00   64.00   -0.00   -0.00   22.93    0.00   -0.00
// 6:    0.00 -317.06    0.00    0.00 -201.77    0.00  629.60    0.00
// 7:  317.06   -0.00   -0.00  201.77   -0.00   -0.00    0.00  629.60


    	// INITIALISE THE QUADRATIC COST MATRIX
		MatrixXf osqp_P = MatrixXf::Zero(num_mpc_dec_vars, num_mpc_dec_vars);

		// ADD THE Q AND R BLOCK DIAGONAL TERMS
		// ROS_INFO("[MPC CONTROLLER] DEBUG 1");
		for (int i = 0; i < d_N; i++)
		{
			osqp_P.block(i * num_input_by_state             , i * num_input_by_state             , num_states, num_states) = Q_cost_mat;
			osqp_P.block(i * num_input_by_state + num_states, i * num_input_by_state + num_states, num_inputs, num_inputs) = R_cost_mat;
		}
		// ROS_INFO("[MPC CONTROLLER] DEBUG 2");

		// ADD THE TERMINAL COST
		//quad_cost_mat_yt = d_P;
		// osqp_P.block(d_N * num_input_by_state, d_N * num_input_by_state, num_states, num_states) = P_cost_mat;
		osqp_P.block(d_N * num_input_by_state, d_N * num_input_by_state, num_states, num_states) = Q_cost_mat;
		// osqp_P.block(d_N * num_input_by_state, d_N * num_input_by_state, num_states, num_states) = 0.1 * MatrixXf::Identity(num_states, num_states);
		// ROS_INFO("[MPC CONTROLLER] DEBUG 3");


		// RESET THE LINEAR COST VECTOR TO ZERO
		d_mpc_q = MatrixXf::Zero(d_num_opt_vars, 1);



		// SPECIFY THE A, B LTI DYNAMICS MATRICES
		MatrixXf A_dynamics = MatrixXf::Zero(num_states, num_states);
		A_dynamics(0,0) = 1.0f;
		A_dynamics(1,1) = 1.0f;
		A_dynamics(2,2) = 1.0f;
		A_dynamics(3,3) = 1.0f;
		A_dynamics(4,4) = 1.0f;
		A_dynamics(5,5) = 1.0f;
		A_dynamics(6,6) = 1.0f;
		A_dynamics(7,7) = 1.0f;

		A_dynamics(0,3) = 0.04f;
		A_dynamics(1,4) = 0.04f;
		A_dynamics(2,5) = 0.04f;

		A_dynamics(0,7) = 0.0078f;
		A_dynamics(1,6) =-0.0078f;

		A_dynamics(3,7) = 0.3924;
		A_dynamics(4,6) =-0.3924;

//     0         1         2         3         4         5         6         7
// 0:  1.0000    0         0         0.0400    0         0         0         0.0078
// 1:  0         1.0000    0         0         0.0400    0        -0.0078    0
// 2:  0         0         1.0000    0         0         0.0400    0         0
// 3:  0         0         0         1.0000    0         0         0         0.3924
// 4:  0         0         0         0         1.0000    0        -0.3924    0
// 5:  0         0         0         0         0         1.0000    0         0
// 6:  0         0         0         0         0         0         1.0000    0
// 7:  0         0         0         0         0         0         0         1.0000

		MatrixXf B_dynamics = MatrixXf::Zero(num_states, num_inputs);
		B_dynamics(0,2) = 0.0001f;
		B_dynamics(1,1) =-0.0001f;

		B_dynamics(2,0) = 0.0250f;
		B_dynamics(5,0) = 1.2500f;

		B_dynamics(3,2) = 0.0078;
		B_dynamics(4,1) =-0.0078;

		B_dynamics(6,1) = 0.0400;
		B_dynamics(7,2) = 0.0400;

//    0         1         2
//0:  0         0         0.0001
//1:  0        -0.0001    0
//2:  0.0250    0         0
//3:  0         0         0.0078
//4:  0        -0.0078    0
//5:  1.2500    0         0
//6:  0         0.0400    0
//7:  0         0         0.0400


		// BUILD THE [A,B,-I] BLOCK FOR THE DYNAMICS CONSTRAIN
		MatrixXf ABI_mat = MatrixXf::Zero(num_states,(num_states+num_inputs+num_states));
		// ROS_INFO("[MPC CONTROLLER] DEBUG 4");
		ABI_mat.block(0,0                 ,num_states,num_states) = A_dynamics;
		// ROS_INFO("[MPC CONTROLLER] DEBUG 5");
		ABI_mat.block(0,num_states        ,num_states,num_inputs) = B_dynamics;
		// ROS_INFO("[MPC CONTROLLER] DEBUG 6");
		ABI_mat.block(0,num_input_by_state,num_states,num_states) = -MatrixXf::Identity(num_states,num_states);
		// ROS_INFO("[MPC CONTROLLER] DEBUG 7");


		// INITIALISE THE EQUALITY CONSTRAINTS MATRIX
		MatrixXf A_cons_eq_mat = MatrixXf::Zero(num_states*(d_N+1),num_mpc_dec_vars);

		// FILL IN THE IDENTIY FOR SETTING THE INITAL STATE
		A_cons_eq_mat.topLeftCorner(num_states,num_states) = MatrixXf::Identity(num_states,num_states);

		// ROS_INFO("[MPC CONTROLLER] DEBUG 8");
		// FILL IN ALL THE DYNAMINCS BLOCKS
		for (int i = 0; i < d_N; i++)
		{
			A_cons_eq_mat.block((i+1) * num_states, i * num_input_by_state, num_states, (num_states+num_inputs+num_states)) = ABI_mat;
		}
		// ROS_INFO("[MPC CONTROLLER] DEBUG 9");

		// INITIALISE THE RHS OF THE EQUALITY CONSTRAINTS
		MatrixXf b_cons_eq_vec = MatrixXf::Zero(num_states*(d_N+1),1);


		
		// BULD THE CONSTRAINTS
		int num_cons_ineq_per_time_step = 6;

		// INITIALISE AND SPECIFY THE MATRX FOR ONE TIME STEP
		MatrixXf A_cons_ineq_mat_for_one_time_step = MatrixXf::Zero(num_cons_ineq_per_time_step,num_input_by_state);

		A_cons_ineq_mat_for_one_time_step.topLeftCorner(3,3)     = MatrixXf::Identity(3,3);
		A_cons_ineq_mat_for_one_time_step.bottomRightCorner(3,3) = MatrixXf::Identity(3,3);

		// INITIALISE AND SPECIFY THE VECTOR FOR ONE TIME STEP
		MatrixXf l_cons_ineq_vec_for_one_time_step = MatrixXf::Zero(num_cons_ineq_per_time_step,1);
		MatrixXf u_cons_ineq_vec_for_one_time_step = MatrixXf::Zero(num_cons_ineq_per_time_step,1);

		l_cons_ineq_vec_for_one_time_step(0,0) = -4.0f - d_setpoint(0);
		l_cons_ineq_vec_for_one_time_step(1,0) = -4.0f - d_setpoint(1);
		l_cons_ineq_vec_for_one_time_step(2,0) =  0.1f - d_setpoint(2);

		l_cons_ineq_vec_for_one_time_step(3,0) =  0.1597f - 0.028 * 9.81;
		l_cons_ineq_vec_for_one_time_step(4,0) = -1.5708f;
		l_cons_ineq_vec_for_one_time_step(5,0) = -1.5708f;

		u_cons_ineq_vec_for_one_time_step(0,0) =  4.0f - d_setpoint(0);
		u_cons_ineq_vec_for_one_time_step(1,0) =  4.0f - d_setpoint(1);
		u_cons_ineq_vec_for_one_time_step(2,0) =  4.0f - d_setpoint(2);

		u_cons_ineq_vec_for_one_time_step(3,0) =  0.4791f - 0.028 * 9.81;
		u_cons_ineq_vec_for_one_time_step(4,0) =  1.5708f;
		u_cons_ineq_vec_for_one_time_step(5,0) =  1.5708f;

		// INITIALISE THE MATRX AND VECTORS FOR ALL TIME STEPS
		MatrixXf A_cons_ineq_mat = MatrixXf::Zero((3+d_N*num_cons_ineq_per_time_step),num_mpc_dec_vars);
		MatrixXf l_cons_ineq_vec = MatrixXf::Zero((3+d_N*num_cons_ineq_per_time_step),1);
		MatrixXf u_cons_ineq_vec = MatrixXf::Zero((3+d_N*num_cons_ineq_per_time_step),1);

		// ROS_INFO("[MPC CONTROLLER] DEBUG 10");
		for (int i = 0; i < d_N; i++)
		{
			A_cons_ineq_mat.block(i * num_cons_ineq_per_time_step, i * num_input_by_state, num_cons_ineq_per_time_step, num_input_by_state) = A_cons_ineq_mat_for_one_time_step;

			l_cons_ineq_vec.middleRows(i * num_cons_ineq_per_time_step, num_cons_ineq_per_time_step) = l_cons_ineq_vec_for_one_time_step;
			u_cons_ineq_vec.middleRows(i * num_cons_ineq_per_time_step, num_cons_ineq_per_time_step) = u_cons_ineq_vec_for_one_time_step;
		}
		// ROS_INFO("[MPC CONTROLLER] DEBUG 11");


		// OSQP CONSTRAINTS
		// Concatenate all constraints in single matrix/vectors
		MatrixXf osqp_A = MatrixXf::Zero(A_cons_eq_mat.rows() + A_cons_ineq_mat.rows(), num_mpc_dec_vars);
		MatrixXf osqp_l = MatrixXf::Zero(A_cons_eq_mat.rows() + A_cons_ineq_mat.rows(), 1);
		MatrixXf osqp_u = MatrixXf::Zero(A_cons_eq_mat.rows() + A_cons_ineq_mat.rows(), 1);
		// ROS_INFO("[MPC CONTROLLER] DEBUG 12");
		
		osqp_A.topRows(A_cons_eq_mat.rows()) = A_cons_eq_mat;
		// ROS_INFO("[MPC CONTROLLER] DEBUG 13");
		osqp_l.topRows(A_cons_eq_mat.rows()) = b_cons_eq_vec;
		osqp_u.topRows(A_cons_eq_mat.rows()) = b_cons_eq_vec;
		// ROS_INFO("[MPC CONTROLLER] DEBUG 14");

		osqp_A.bottomRows(A_cons_ineq_mat.rows()) = A_cons_ineq_mat;
		// ROS_INFO("[MPC CONTROLLER] DEBUG 15");
		osqp_l.bottomRows(A_cons_ineq_mat.rows()) = l_cons_ineq_vec;
		osqp_u.bottomRows(A_cons_ineq_mat.rows()) = u_cons_ineq_vec;
		// ROS_INFO("[MPC CONTROLLER] DEBUG 16");

		// OSQP MODEL SETUP
		// Follows 'Setup and solve' example (https://osqp.org/docs/examples/setup-and-solve.html)

		osqp_extended_cleanup();

		// Convert Eigen matrices to CSC format
		csc* osqp_P_csc = eigen2csc(osqp_P);
		csc* osqp_A_csc = eigen2csc(osqp_A);

		// Convert Eigen vectors to c_float arrays
		// One copy is used for initial setup, the other is used during runtime in update function
		//c_float* osqp_q_cfloat = (c_float*) c_malloc(d_mpc_q.rows() * sizeof(c_float));
		c_float* osqp_l_cfloat = (c_float*) c_malloc(osqp_l.rows() * sizeof(c_float));
		c_float* osqp_u_cfloat = (c_float*) c_malloc(osqp_u.rows() * sizeof(c_float));

		//d_mpc_q_new = (c_float*) c_malloc(d_mpc_q.rows() * sizeof(c_float));
		d_mpc_l_new = (c_float*) c_malloc(osqp_l.rows() * sizeof(c_float));
		d_mpc_u_new = (c_float*) c_malloc(osqp_u.rows() * sizeof(c_float));

		//Matrix<c_float, Dynamic, Dynamic>::Map(osqp_q_cfloat, d_mpc_q.rows(), d_mpc_q.cols()) = d_mpc_q.cast<c_float>();
		//Matrix<c_float, Dynamic, Dynamic>::Map(d_mpc_q_new, d_mpc_q.rows(), d_mpc_q.cols()) = d_mpc_q.cast<c_float>();

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_l_cfloat, osqp_l.rows(), osqp_l.cols()) = osqp_l.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(d_mpc_l_new, osqp_l.rows(), osqp_l.cols()) = osqp_l.cast<c_float>();

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_u_cfloat, osqp_u.rows(), osqp_u.cols()) = osqp_u.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(d_mpc_u_new, osqp_u.rows(), osqp_u.cols()) = osqp_u.cast<c_float>();

		// Populate data
	    OSQPData* osqp_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_data->n = osqp_A.cols();
	    osqp_data->m = osqp_A.rows();
	    osqp_data->P = osqp_P_csc;
	    //osqp_data->q = osqp_q_cfloat;
	    osqp_data->A = osqp_A_csc;
	    osqp_data->l = osqp_l_cfloat;
	    osqp_data->u = osqp_u_cfloat;

		// Problem settings
	    d_osqp_settings = (OSQPSettings*) c_malloc(sizeof(OSQPSettings));

	    // Define Solver settings as default, and change settings as desired
	    osqp_set_default_settings(d_osqp_settings);
	    d_osqp_settings->verbose = d_opt_verbose;

	    // Setup workspace
	    d_osqp_work = osqp_setup(osqp_data, d_osqp_settings);

	    // Clear data after setting up to allow subseqeuent setups
	    osqp_cleanup_data(osqp_data);

	    if (!d_osqp_work)
	    {
	    	clear_setupRampc_success_flag();

	    	ROS_INFO("[MPC CONTROLLER] Rampc optimization setup failed with OSQP");
	    	ROS_INFO("[MPC CONTROLLER] Rampc must be (re-)setup");

	    	return;
	    }

	    // Some steps to finish setup
		//finish_Rampc_setup();
		// Setup output variables
    	d_u_f = MatrixXf::Zero(d_N * num_inputs, 1);
    	d_y_f = MatrixXf::Zero((d_N+1) * d_num_outputs, 1);

    	d_u_f(0) = 0.028 * 9.81;

	    // Setup successful flag
	    d_setupRampc_success = true;

	    s_Rampc_mutex.lock();
	    // ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1285");
		// s_num_inputs = num_inputs;
		// s_num_outputs = d_num_outputs;
		// s_Nuini = d_Tini * num_inputs;
		// s_Nyini = d_Tini * d_num_outputs;
		// s_uini = MatrixXf::Zero(s_Nuini, 1);
		// s_yini = MatrixXf::Zero(s_Nyini, 1);
		s_setupRampc_success = d_setupRampc_success;
		s_Rampc_mutex.unlock();
		// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1294");

	    // Inform the user
	    ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup successful with OSQP");
    }

  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization setup exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup exception with OSQP");
    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}





void change_Rampc_setpoint_mpc()
{
	try
	{
		// No need to change the MPC optimization problem.
		// Just shift the initial state given to MPC

		s_Rampc_mutex.lock();
		// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1225");
		d_setpoint = s_setpoint;
		s_Rampc_mutex.unlock();
		// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1225");


		int num_inputs = 3;
    	int num_states = 8;
    	int num_input_by_state = num_inputs + num_states;
    	int num_input_dec_vars = d_N     * num_inputs;
    	int num_state_dec_vars = (d_N+1) * num_states;

    	int num_mpc_dec_vars = num_input_dec_vars + num_state_dec_vars;

    	// COMPUTE THE NUMBER OF EQUALITY CONSTRAINTS
    	int ineq_start = (d_N+1) * num_states;

    	// BULD THE CONSTRAINTS
		int num_cons_ineq_per_time_step = 6;

		// Update equality constraint vectors
		for (int t = 0; t < d_N; t++)
		{
			d_mpc_l_new[ineq_start + (t*num_cons_ineq_per_time_step) + 0] = -4.0f - d_setpoint(0);
			d_mpc_u_new[ineq_start + (t*num_cons_ineq_per_time_step) + 0] =  4.0f - d_setpoint(0);

			d_mpc_l_new[ineq_start + (t*num_cons_ineq_per_time_step) + 1] = -4.0f - d_setpoint(1);
			d_mpc_u_new[ineq_start + (t*num_cons_ineq_per_time_step) + 1] =  4.0f - d_setpoint(1);

			d_mpc_l_new[ineq_start + (t*num_cons_ineq_per_time_step) + 2] =  0.1f - d_setpoint(2);
			d_mpc_u_new[ineq_start + (t*num_cons_ineq_per_time_step) + 2] =  4.0f - d_setpoint(2);
		}

	    // Inform the user
	    ROS_INFO("[MPC CONTROLLER] MPC setpoint updated successfully");
	}

  	catch(exception& e)
    {
    	//clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[MPC CONTROLLER] MPC setpoint update exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[MPC CONTROLLER] MPC must be (re-)setup");
  	}
  	catch(...)
  	{
  		//clear_setupRampc_success_flag();

  		ROS_INFO("[MPC CONTROLLER] MPC setpoint update exception with OSQP");
  		ROS_INFO("[MPC CONTROLLER] MPC must be (re-)setup");
  	}
}



void change_Rampc_setpoint_mpc_changing_ref()
{
	try
	{
		// DO NOTHING
		// THIS IS FOR A TIME-VARYING SETPOINT, eg., FIGURE 8
	}

  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[MPC CONTROLLER] Rampc setpoint update exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[MPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

  		ROS_INFO("[MPC CONTROLLER] Rampc setpoint update exception with OSQP");
  		ROS_INFO("[MPC CONTROLLER] Rampc must be (re-)setup");
  	}
}




void solve_Rampc_mpc()
{
	try
	{
		s_Rampc_mutex.lock();
		d_current_state_estimate = s_current_state_estimate;
		d_setpoint = s_setpoint;
		s_Rampc_mutex.unlock();


		int num_states = 8;
		int num_inputs = 3;
		int num_input_by_state = num_inputs + num_states;

		// Update reference if reference is changing
		if (d_changing_ref_enable)
			change_Rampc_setpoint_mpc_changing_ref();


		// Get the input that was applied at this time step
		MatrixXf u_current = MatrixXf::Zero(num_inputs, 1);
		u_current(0) = d_u_f(0) - 0.028 * 9.81;
		u_current(1) = d_u_f(1);
		u_current(2) = d_u_f(2);


		
		// SPECIFY THE A, B LTI DYNAMICS MATRICES
		MatrixXf A_dynamics = MatrixXf::Zero(num_states, num_states);
		A_dynamics(0,0) = 1.0f;
		A_dynamics(1,1) = 1.0f;
		A_dynamics(2,2) = 1.0f;
		A_dynamics(3,3) = 1.0f;
		A_dynamics(4,4) = 1.0f;
		A_dynamics(5,5) = 1.0f;
		A_dynamics(6,6) = 1.0f;
		A_dynamics(7,7) = 1.0f;

		A_dynamics(0,3) = 0.04f;
		A_dynamics(1,4) = 0.04f;
		A_dynamics(2,5) = 0.04f;

		A_dynamics(0,7) = 0.0078f;
		A_dynamics(1,6) =-0.0078f;

		A_dynamics(3,7) = 0.3924;
		A_dynamics(4,6) =-0.3924;

//     0         1         2         3         4         5         6         7
// 0:  1.0000    0         0         0.0400    0         0         0         0.0078
// 1:  0         1.0000    0         0         0.0400    0        -0.0078    0
// 2:  0         0         1.0000    0         0         0.0400    0         0
// 3:  0         0         0         1.0000    0         0         0         0.3924
// 4:  0         0         0         0         1.0000    0        -0.3924    0
// 5:  0         0         0         0         0         1.0000    0         0
// 6:  0         0         0         0         0         0         1.0000    0
// 7:  0         0         0         0         0         0         0         1.0000

		MatrixXf B_dynamics = MatrixXf::Zero(num_states, num_inputs);
		B_dynamics(0,2) = 0.0001f;
		B_dynamics(1,1) =-0.0001f;

		B_dynamics(2,0) = 0.0250f;
		B_dynamics(5,0) = 1.2500f;

		B_dynamics(3,2) = 0.0078;
		B_dynamics(4,1) =-0.0078;

		B_dynamics(6,1) = 0.0400;
		B_dynamics(7,2) = 0.0400;

//    0         1         2
//0:  0         0         0.0001
//1:  0        -0.0001    0
//2:  0.0250    0         0
//3:  0         0         0.0078
//4:  0        -0.0078    0
//5:  1.2500    0         0
//6:  0         0.0400    0
//7:  0         0         0.0400


		// ROS_INFO("[MPC CONTROLLER] DEBUG 1");

		MatrixXf x_current = d_current_state_estimate;

		// ROS_INFO_STREAM("[MPC CONTROLLER] u delta current (f,wx,wy) = ( " << u_current(0) << " , " << u_current(1) << " , " << u_current(2) << " )" );
		// ROS_INFO_STREAM("[MPC CONTROLLER] current estimate (x,y,z)  = ( " << x_current(0) << " , " << x_current(1) << " , " << x_current(2) << " )" );
		

		// Subtract the setpoint from the (x,y,z) positions
		x_current(0) = x_current(0) - d_setpoint(0);
		x_current(1) = x_current(1) - d_setpoint(1);
		x_current(2) = x_current(2) - d_setpoint(2);
		// ROS_INFO("[MPC CONTROLLER] DEBUG 2");


		// Compute the initial state as x_initial = A x_current + B * u_current
		MatrixXf x_initial = A_dynamics * x_current + B_dynamics * u_current;


		// ROS_INFO_STREAM("[MPC CONTROLLER] initial state (x,y,z)     = ( " << x_initial(0) << " , " << x_initial(1) << " , " << x_initial(2) << " )" );



		// Update equality constraint vectors for the initial state

		for (int i = 0; i < num_states; i++)
		{
			d_mpc_l_new[i] = x_initial(i);
			d_mpc_u_new[i] = x_initial(i);
		}
		// ROS_INFO("[MPC CONTROLLER] DEBUG 3");
		osqp_update_bounds(d_osqp_work, d_mpc_l_new, d_mpc_u_new);

		// Solve optimization
		osqp_solve(d_osqp_work);
		d_RampcOpt_status = d_osqp_work->info->status_val;

		if (d_RampcOpt_status > 0)
		{	

			// ROS_INFO_STREAM("[MPC CONTROLLER] d_mpc_l_new (x,y,z)         = ( " << d_mpc_l_new[0] << " , " << d_mpc_l_new[1] << " , " << d_mpc_l_new[2]  << " )" );
			// ROS_INFO_STREAM("[MPC CONTROLLER] d_mpc_u_new (x,y,z)         = ( " << d_mpc_u_new[0] << " , " << d_mpc_u_new[1] << " , " << d_mpc_u_new[2]  << " )" );
			// ROS_INFO_STREAM("[MPC CONTROLLER] OSQP initial l (x,y,z)      = ( " << d_osqp_work->data->l[0] << " , " << d_osqp_work->data->l[1] << " , " << d_osqp_work->data->l[2]  << " )" );
			// ROS_INFO_STREAM("[MPC CONTROLLER] OSQP initial state (x,y,z)  = ( " << d_osqp_work->solution->x[0] << " , " << d_osqp_work->solution->x[1] << " , " << d_osqp_work->solution->x[2]  << " )" );
			// ROS_INFO_STREAM("[MPC CONTROLLER] OSQP new input (f,wx,wy)    = ( " << d_osqp_work->solution->x[8] << " , " << d_osqp_work->solution->x[9] << " , " << d_osqp_work->solution->x[10] << " )" );


			// With sparse formulation can get uf and yf directly
			for (int t = 0; t < d_N; t++)
			{
				for (int i = 0; i < num_inputs; i++)
				{
					d_u_f((t*num_inputs + i)) = d_osqp_work->solution->x[t*num_input_by_state + num_states + i];
					if (i == 0)
						d_u_f((t*num_inputs + i)) += 0.028 * 9.81;
				}
			}
			
			for (int t = 0; t < d_N + 1; t++)
			{
				for (int i = 0; i < d_num_outputs; i++)
				{
					if (i<3)
						d_y_f((t*d_num_outputs + i)) = d_osqp_work->solution->x[t*num_input_by_state + i] + d_setpoint(i);
					else
						d_y_f((t*d_num_outputs + i)) = d_osqp_work->solution->x[t*num_input_by_state + 3 + i];
				}
			}

			d_solve_time = d_osqp_work->info->run_time;

			s_Rampc_mutex.lock();
			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 649");
			s_u_f = d_u_f;
			s_y_f = d_y_f;
			s_solve_time = d_solve_time;
			s_Rampc_mutex.unlock();
			//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 649");

			ROS_INFO_STREAM("[MPC CONTROLLER] Rampc found optimal solution with OSQP status: " << d_osqp_work->info->status);
			ROS_INFO_STREAM("Thrust: "     << d_u_f(0) );
			ROS_INFO_STREAM("Roll Rate: "  << d_u_f(1) );
			ROS_INFO_STREAM("Pitch Rate: " << d_u_f(2) );
			//if (d_Rampc_yaw_control)
			//	ROS_INFO_STREAM("Yaw Rate: " << d_u_f(3));
			ROS_INFO_STREAM("Objective: " << d_osqp_work->info->obj_val);
			ROS_INFO_STREAM("Runtime: " << d_solve_time);
		}
		else
		{
			ROS_INFO_STREAM("[MPC CONTROLLER] Rampc failed to find optimal solution with OSQP status: " << d_osqp_work->info->status);
			s_Rampc_mutex.lock();
			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 649");
			s_u_f = d_u_f;
			s_y_f = d_y_f;
			s_solve_time = 0.0;
			s_Rampc_mutex.unlock();
		}
	}

  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[MPC CONTROLLER] Rampc optimization exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[MPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

    	ROS_INFO("[MPC CONTROLLER] Rampc optimization exception with OSQP");
    	ROS_INFO("[MPC CONTROLLER] Rampc must be (re-)setup");
  	}
}


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


// RAMPC FUNCTIONS

// RAMPC THREAD MAIN
// Rampc operations run in seperate thread as they are time consuming
void Rampc_thread_main()
{
	bool params_changed;
	bool setpoint_changed;
	bool setupRampc;
	bool solveRampc;
	bool changing_ref_enable_prev = false;
	bool theta_changed;

	// Create thread for gs matrix inversion
	//boost::thread Rampc_gs_inversion_thread(Rampc_gs_inversion_thread_main);

	while (ros::ok())
	{

		s_Rampc_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 72");
        params_changed = s_params_changed;
        setpoint_changed = s_setpoint_changed;
        setupRampc = s_setupRampc;
        solveRampc = s_solveRampc;
        d_changing_ref_enable = s_changing_ref_enable;
        s_Rampc_mutex.unlock();
        //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 72");

        // Detect changing_ref_enable rising edge to reset time
        if (!changing_ref_enable_prev && d_changing_ref_enable)
        	d_time_in_seconds = 0.0;
        
        // Detect changing_ref_enable falling edge to reset setpoint to before it was enabled
        if (changing_ref_enable_prev && !d_changing_ref_enable)
        	setpoint_changed = true;

        changing_ref_enable_prev = d_changing_ref_enable;
        

        if (params_changed)
        {
        	ROS_INFO_STREAM("[RAMPC CONTROLLER] Params Changed.");
        	change_Rampc_params();
        	
        	s_Rampc_mutex.lock();
        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 85");
        	s_params_changed = false;
        	s_Rampc_mutex.unlock();
        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 85");
        }

        if (setpoint_changed)
        {
        	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Change setpoint 2");
        	if (d_setupRampc_success)
        	{
        		// Switch between the possible solvers
        		ROS_INFO_STREAM("[RAMPC CONTROLLER] Change setpoint");
				switch (d_solver)
				{
					case RAMPC_CONTROLLER_SOLVER_OSQP:
						//change_Rampc_setpoint_osqp();
						//MatrixXf d_z_setpoint_Rampc=MatrixXf::Zero(d_num_outputs,1);
					
						
						d_z_setpoint_Rampc<< 	d_setpoint(2)-0.8,
														0.0;
					
						
						for(int i=0;i<d_N;i++){
							for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
								temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
								d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
							}
						}


						s_Rampc_mutex.lock();
	        			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 96");
	        			s_Rampc_active_setpoint = d_setpoint;
	        			s_setpoint_changed = false;
	        			s_Rampc_mutex.unlock();
						break;

					case RAMPC_CONTROLLER_SOLVER_MPC:
						change_Rampc_setpoint_mpc();
						s_Rampc_active_setpoint = d_setpoint;
		        		s_setpoint_changed = false;
		        		s_Rampc_mutex.unlock();
						break;

					case RAMPC_CONTROLLER_SOLVER_GUROBI:
					default:
						change_Rampc_setpoint_gurobi();
						break;
				}
	        	/*
				// If optimizing over steady state gs & us, no need to perform gs inversion logic
				//if (d_opt_sparse && d_opt_steady_state)
				//{
					s_Rampc_mutex.lock();
		        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 96");
		        	s_Rampc_active_setpoint = d_setpoint;
		        	s_setpoint_changed = false;
		        	s_Rampc_mutex.unlock();
		        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 96");
				//}
				// Wait for gs inversion to complete before exiting this mode
		        	/*
				else if (d_gs_inversion_complete)
				{
					d_gs_inversion_complete = false;

					ds_Rampc_gs_inversion_mutex.lock();
					//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 117");
			        ds_gs_inversion_complete = d_gs_inversion_complete;
			        ds_Rampc_gs_inversion_mutex.unlock();
			        //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 117");

		        	s_Rampc_mutex.lock();
		        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 96");
		        	s_Rampc_active_setpoint = d_setpoint;
		        	s_setpoint_changed = false;
		        	s_Rampc_mutex.unlock();
		        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 96");
	        	}
	        	*/
        	}
        	else
        	{
           		s_Rampc_mutex.lock();
	        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 96");
	        	s_Rampc_active_setpoint = s_setpoint;
	        	s_setpoint_changed = false;
	        	s_Rampc_mutex.unlock();
	        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 96");
        	}
        }

        if (setupRampc)
        {
        	ROS_INFO_STREAM("[RAMPC CONTROLLER] Setup Rampc.");
        	// Switch between the possible solvers
			switch (d_solver)
			{
				case RAMPC_CONTROLLER_SOLVER_OSQP:
					setup_Rampc_osqp();
					break;

				case RAMPC_CONTROLLER_SOLVER_MPC:
					setup_Rampc_mpc();
					break;

				case RAMPC_CONTROLLER_SOLVER_GUROBI:
				default:
					setup_Rampc_gurobi();
					break;
			}

        	s_Rampc_mutex.lock();
        	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 107");
        	s_setupRampc = false;
        	s_Rampc_mutex.unlock();
        	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 107");
        }

        if (solveRampc)
        {
        	ROS_INFO_STREAM("[RAMPC CONTROLLER] Solve Rampc.");
        	// Switch between the possible solvers
			switch (d_solver)
			{
				case RAMPC_CONTROLLER_SOLVER_OSQP:
					solve_Rampc_osqp();
					break;

				case RAMPC_CONTROLLER_SOLVER_MPC:
					solve_Rampc_mpc();
					break;

				case RAMPC_CONTROLLER_SOLVER_GUROBI:
				default:
					solve_Rampc_gurobi();
					break;
			}

		  	s_Rampc_mutex.lock();
		  	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 118");
        	s_solveRampc = false;
        	s_Rampc_mutex.unlock();
        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 118");
        }
	}

	// Cleanup for memory allocation etc.
	gurobi_cleanup();
	osqp_extended_cleanup();

	// Wait for gs matrix inversion thread to finish
    //Rampc_gs_inversion_thread.join();
}



void change_Rampc_params()
{
	s_Rampc_mutex.lock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 133");
	d_cf_weight_in_newtons = s_cf_weight_in_newtons;
	d_dataFolder = s_dataFolder;
	d_logFolder = s_logFolder;
	d_Rampc_measure_roll_pitch = s_Rampc_measure_roll_pitch;
	d_Rampc_yaw_control = s_Rampc_yaw_control;
	d_Tini = s_yaml_Tini;
	d_N = s_yaml_N;
	d_Q_vec = s_yaml_Q;
	d_R_vec = s_yaml_R;
	d_P_vec = s_yaml_P;
	d_K_vec = s_yaml_K;
	d_G_vec = s_yaml_G;
	d_F_vec = s_yaml_F;
	d_lambda2_g = s_yaml_lambda2_g;
	d_lambda2_s = s_yaml_lambda2_s;
	d_input_min_vec = s_yaml_input_min;
	d_input_max_vec = s_yaml_input_max;
	d_output_min_vec = s_yaml_output_min;
	d_output_max_vec = s_yaml_output_max;
	d_reference_difference=s_yaml_reference_difference;
	if (s_yaml_solver == "osqp")
		d_solver = RAMPC_CONTROLLER_SOLVER_OSQP;
	else if (s_yaml_solver == "mpc")
		d_solver = RAMPC_CONTROLLER_SOLVER_MPC;
	else
	{
		// Default solver is Gurobi
		d_solver = RAMPC_CONTROLLER_SOLVER_GUROBI;
	}

	d_opt_sparse = s_yaml_opt_sparse;
	d_opt_verbose = s_yaml_opt_verbose;
	d_opt_steady_state = s_yaml_opt_steady_state;
	d_grb_LogToFile = s_yaml_grb_LogToFile;
	d_grb_presolve_at_setup = s_yaml_grb_presolve_at_setup;

	// Variables used for changing reference
	d_changing_ref_enable = s_changing_ref_enable;
	d_figure_8_amplitude = s_figure_8_amplitude;
	d_figure_8_frequency_rad = s_figure_8_frequency_rad;
	d_z_sine_amplitude = s_z_sine_amplitude;
	d_z_sine_frequency_rad = s_z_sine_frequency_rad;
	d_control_deltaT = s_control_deltaT;
	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 133");

	// Rampc setup must be re-run after changes
	clear_setupRampc_success_flag();

	// Inform the user
	ROS_INFO("[RAMPC CONTROLLER] Rampc parameters change successful");
	ROS_INFO("[RAMPC CONTROLLER] (Re-)setup Rampc to apply changes");
}

void change_Rampc_setpoint_gurobi()
{
	try
	{
		// Update linear cost vectors
		update_lin_cost_vectors();

		// If optimizing over steady state gs & us, no need to perform gs inversion logic
		if (d_opt_sparse && d_opt_steady_state)
		{
			// Update linear objective terms
		    d_grb_lin_obj_r = 0;
		    for (d_i = 0; d_i < d_Nyf + d_num_outputs; d_i++)
		    	d_grb_lin_obj_r += d_lin_cost_vec_r(d_i) * d_grb_vars[d_yf_start_i + d_i];

		    // Update objective
		    d_grb_model.setObjective(d_grb_quad_obj + d_grb_lin_obj_us + d_grb_lin_obj_r + d_grb_lin_obj_gs);

			// Update equality constraints RHS
			for (d_i = 0; d_i < d_Nyini + d_Nyf + d_num_outputs; d_i++)
				d_grb_dyn_constrs[d_r_gs_start_i + d_i].set(GRB_DoubleAttr_RHS, d_r_gs(d_i));

		    // Inform the user
		    ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update successful with Gurobi");
		}
		// Wait for gs inversion to complete before proceeding
		else if (d_gs_inversion_complete)
		{
			// Update linear objective terms
		    d_grb_lin_obj_r = 0;
		    d_grb_lin_obj_gs = 0;
		    if (d_opt_sparse)
		    {
		    	for (d_i = 0; d_i < d_Ng; d_i++)
			    	d_grb_lin_obj_gs += d_lin_cost_vec_gs(d_i) * d_grb_vars[d_i];
			    for (d_i = 0; d_i < d_Nyf + d_num_outputs; d_i++)
			    	d_grb_lin_obj_r += d_lin_cost_vec_r(d_i) * d_grb_vars[d_yf_start_i + d_i];
		    }
		    else
		    {
		    	for (d_i = 0; d_i < d_Ng; d_i++)
			    {
			    	d_grb_lin_obj_r += d_lin_cost_vec_r(d_i) * d_grb_vars[d_i];
			    	d_grb_lin_obj_gs += d_lin_cost_vec_gs(d_i) * d_grb_vars[d_i];
			    }
		    }

		    // Update objective
		    // It was observed that objective of pre-solved model is same as original model
		    if (d_opt_sparse || !d_grb_presolve_at_setup)
		    	d_grb_model.setObjective(d_grb_quad_obj + d_grb_lin_obj_us + d_grb_lin_obj_r + d_grb_lin_obj_gs);
		    else
		    	d_grb_model_presolved->setObjective(d_grb_quad_obj + d_grb_lin_obj_us + d_grb_lin_obj_r + d_grb_lin_obj_gs);

		    // Inform the user
		    ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update successful with Gurobi");
	    }
	}

	catch(GRBException e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc setpoint update exception with Gurobi error code = " << e.getErrorCode());
	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Error message: " << e.getMessage());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc setpoint update exception with Gurobi with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

  		ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update exception with Gurobi");
  		ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}

void change_Rampc_setpoint_gurobi_changing_ref()
{
	try
	{
		// Update linear cost vectors
		update_lin_cost_vectors_changing_ref();

		// Update linear cost vector, depending on optimization formulation
		d_grb_lin_obj_r = 0;
		if (d_opt_sparse)
		{
			// Update linear objective terms
		    for (d_i = 0; d_i < d_Nyf + d_num_outputs; d_i++)
		    	d_grb_lin_obj_r += d_lin_cost_vec_r(d_i) * d_grb_vars[d_yf_start_i + d_i];

		    // Update equality constraint RHS if optimizing over gs
			if (d_opt_steady_state)
				for (d_i = 0; d_i < d_Nyini + d_Nyf + d_num_outputs; d_i++)
					d_grb_dyn_constrs[d_r_gs_start_i + d_i].set(GRB_DoubleAttr_RHS, d_r_gs(d_i));
		}
		else
		{
			// Update linear objective terms
	    	for (d_i = 0; d_i < d_Ng; d_i++)
		    {
		    	d_grb_lin_obj_r += d_lin_cost_vec_r(d_i) * d_grb_vars[d_i];
		    	d_grb_lin_obj_gs += d_lin_cost_vec_gs(d_i) * d_grb_vars[d_i];
		    }
		}

	    // Update objective
	    // It was observed that objective of pre-solved model is same as original model
	    if (d_opt_sparse || !d_grb_presolve_at_setup)
	    	d_grb_model.setObjective(d_grb_quad_obj + d_grb_lin_obj_us + d_grb_lin_obj_r + d_grb_lin_obj_gs);
	    else
	    	d_grb_model_presolved->setObjective(d_grb_quad_obj + d_grb_lin_obj_us + d_grb_lin_obj_r + d_grb_lin_obj_gs);
	}

	catch(GRBException e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc setpoint update exception with Gurobi error code = " << e.getErrorCode());
	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Error message: " << e.getMessage());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc setpoint update exception with Gurobi with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

  		ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update exception with Gurobi");
  		ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}

void change_Rampc_setpoint_osqp()
{
	try
	{	





		/*
		// Update linear cost vector
		update_lin_cost_vectors();

		// If optimizing over steady state gs & us, no need to perform gs inversion logic
		if (d_opt_sparse && d_opt_steady_state)
		{
			// Update linear cost vector
			d_osqp_q.middleRows(d_yf_start_i, d_Nyf + d_num_outputs) = d_lin_cost_vec_r;

			// Convert Eigen vector to c_float array
			Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_q_new, d_osqp_q.rows(), d_osqp_q.cols()) = d_osqp_q.cast<c_float>();

			// Update OSQP linear cost
			osqp_update_lin_cost(d_osqp_work, d_osqp_q_new);

			// Update equality constraint vectors
			for (d_i = 0; d_i < d_Nyini + d_Nyf + d_num_outputs; d_i++)
			{
				d_osqp_l_new[d_r_gs_start_i + d_i] = d_r_gs(d_i);
				d_osqp_u_new[d_r_gs_start_i + d_i] = d_osqp_l_new[d_r_gs_start_i + d_i];
			}

		    // Inform the user
		    ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update successful with OSQP");
		}
		// Wait for gs inversion to complete before proceeding
		else if (d_gs_inversion_complete)
		{
			// Update linear cost vector
			if (d_opt_sparse)
			{
				d_osqp_q.topRows(d_Ng) = d_lin_cost_vec_gs;
				d_osqp_q.bottomRows(d_Nyf + d_num_outputs) = d_lin_cost_vec_r;
			}
			else
				d_osqp_q.topRows(d_Ng) = d_lin_cost_vec_us + d_lin_cost_vec_r + d_lin_cost_vec_gs;

			// Convert Eigen vector to c_float array
			Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_q_new, d_osqp_q.rows(), d_osqp_q.cols()) = d_osqp_q.cast<c_float>();

			// Update OSQP linear cost
			osqp_update_lin_cost(d_osqp_work, d_osqp_q_new);

		    // Inform the user
		    ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update successful with OSQP");
		 */
		//}

	}

  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc setpoint update exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

  		ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update exception with OSQP");
  		ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}


void change_Rampc_setpoint_osqp_changing_ref()
{
	try
	{
		// Update linear cost vector
		update_lin_cost_vectors_changing_ref();

		// Update linear cost vector, depending on optimization formulation
		if (d_opt_sparse)
		{
			d_osqp_q.middleRows(d_yf_start_i, d_Nyf + d_num_outputs) = d_lin_cost_vec_r;

			// Update equality constraint vectors if optimizing over gs
			if (d_opt_steady_state)
				for (d_i = 0; d_i < d_Nyini + d_Nyf + d_num_outputs; d_i++)
				{
					d_osqp_l_new[d_r_gs_start_i + d_i] = d_r_gs(d_i);
					d_osqp_u_new[d_r_gs_start_i + d_i] = d_osqp_l_new[d_r_gs_start_i + d_i];
				}
		}
		else
			d_osqp_q.topRows(d_Ng) = d_lin_cost_vec_us + d_lin_cost_vec_r + d_lin_cost_vec_gs;

		// Convert Eigen vector to c_float array
		Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_q_new, d_osqp_q.rows(), d_osqp_q.cols()) = d_osqp_q.cast<c_float>();

		// Update OSQP linear cost
		osqp_update_lin_cost(d_osqp_work, d_osqp_q_new);
	}

  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc setpoint update exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

  		ROS_INFO("[RAMPC CONTROLLER] Rampc setpoint update exception with OSQP");
  		ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}

void setup_Rampc_gurobi()
{
	/*
	try
    {
		// Get u_data & y_data from files
		MatrixXf u_data = get_u_data();
		MatrixXf y_data = get_y_data();

		// Get variable lengths
		get_variable_lengths(u_data, y_data);

		// Get Hankel matrices
		get_hankel_matrices(u_data, y_data);

		// Get cost matrices
		get_cost_matrices();

		// Input/output constraint vectors
		get_input_output_constr_vectors();

		// GUROBI QUADRATIC COST MATRIX
		MatrixXf grb_Q = get_quad_cost_matrix();

		// GUROBI LINEAR COST VECTORS
		get_lin_cost_vectors();

		// GUROBI LINEAR INEQUALITY CONSTRAINT MATRIX
		// Only used in 'dense' formulation
		MatrixXf grb_A_g;
		if (d_opt_sparse)
			grb_A_g = MatrixXf::Zero(0, 0);
		else
		{
			grb_A_g = MatrixXf::Zero(2 * (d_Nuf + d_Nyf + d_num_outputs), d_Ng);
			grb_A_g.topRows(d_Nuf) = -d_U_f;
			grb_A_g.middleRows(d_Nuf, d_Nuf) = d_U_f;
			grb_A_g.middleRows(2 * d_Nuf, d_Nyf + d_num_outputs) = -d_Y_f;
			grb_A_g.bottomRows(d_Nyf + d_num_outputs) = d_Y_f;
		}

		// GUROBI LINEAR INEQUALITY CONSTRAINT VECTOR
		// Only used in 'dense' formulation
		MatrixXf grb_b_g;
		if (d_opt_sparse)
			grb_b_g = MatrixXf::Zero(0, 0);
		else
		{
			grb_b_g = MatrixXf::Zero(2 * (d_Nuf + d_Nyf + d_num_outputs), 1);
			grb_b_g.topRows(d_Nuf) = -d_input_min;
			grb_b_g.middleRows(d_Nuf, d_Nuf) = d_input_max;
			grb_b_g.middleRows(2 * d_Nuf, d_Nyf + d_num_outputs) = -d_output_min;
			grb_b_g.bottomRows(d_Nyf + d_num_outputs) = d_output_max;
		}

		// GUROBI BOUNDS VECTOR
		MatrixXf lb = -GRB_INFINITY * MatrixXf::Ones(d_num_opt_vars, 1);
		MatrixXf ub = GRB_INFINITY * MatrixXf::Ones(d_num_opt_vars, 1);
		if (d_opt_sparse)
		{
			lb.middleRows(d_uf_start_i, d_Nuf) = d_input_min;
			lb.middleRows(d_yf_start_i, d_Nyf + d_num_outputs) = d_output_min;

			ub.middleRows(d_uf_start_i, d_Nuf) = d_input_max;
			ub.middleRows(d_yf_start_i, d_Nyf + d_num_outputs) = d_output_max;
		}

		// GUROBI LINEAR EQUALITY CONSTRAINTS
		// Static equality constraints ([uf; yf; yt]), and ([gs; us]) if optimizing over steady state gs & us
		MatrixXf grb_A_eq_stat = get_static_eq_constr_matrix();
		MatrixXf grb_b_eq_stat = get_static_eq_constr_vector();
		
		// Dynamic equality constraints that change every time ([uini; uini]), and ([gs; r]) if optimizing over steady state gs & us
		MatrixXf grb_A_eq_dyn = get_dynamic_eq_constr_matrix();
		MatrixXf grb_b_eq_dyn = get_dynamic_eq_constr_vector();
		if (d_opt_sparse && d_opt_steady_state)
			d_r_gs_start_i = d_Nuini + d_Nyini;

		// GUROBI MODEL SETUP
		// Follows 'dense_c++.cpp' example (https://www.gurobi.com/documentation/8.1/examples/dense_cpp_cpp.html)
    	// (Re-)Configure environment
	    d_grb_env.set(GRB_StringParam_LogFile, d_logFolder + "gurobi.log");
	    d_grb_env.start();

	    // Clear variables if previously created
	    int num_vars = d_grb_model.get(GRB_IntAttr_NumVars);
	    d_grb_vars = d_grb_model.getVars();
	    for (int i = 0; i < num_vars; i++)
	    	d_grb_model.remove(d_grb_vars[i]);
	    d_grb_model.update();
	    delete[] d_grb_vars;

	    // Create variables
	    double grb_lb[d_num_opt_vars];
	    double grb_ub[d_num_opt_vars];
	    for (int i = 0; i < d_num_opt_vars; i++)
	    {
	    	grb_lb[i] = lb(i);
	    	grb_ub[i] = ub(i);
	    }
	    d_grb_vars = d_grb_model.addVars(grb_lb, grb_ub, NULL, NULL, NULL, d_num_opt_vars);

	    // Set quadratic objective term
	    d_grb_quad_obj = 0;
	    for (int i = 0; i < d_num_opt_vars; i++)
	    	for (int j = 0; j < d_num_opt_vars; j++)
	    		d_grb_quad_obj += grb_Q(i,j) * d_grb_vars[i] * d_grb_vars[j];

	    // Set linear objective term
	    d_grb_lin_obj_us = 0;
	    d_grb_lin_obj_r = 0;
	    d_grb_lin_obj_gs = 0;
	    if (d_opt_sparse)
	    {
	    	for (int i = 0; i < d_Ng; i++)
		    	d_grb_lin_obj_gs += d_lin_cost_vec_gs(i) * d_grb_vars[i];
		    for (int i = 0; i < d_Nuf; i++)
		    	d_grb_lin_obj_us += d_lin_cost_vec_us(i) * d_grb_vars[d_uf_start_i + i];
		    for (int i = 0; i < d_Nyf + d_num_outputs; i++)
		    	d_grb_lin_obj_r += d_lin_cost_vec_r(i) * d_grb_vars[d_yf_start_i + i];
	    }
	    else
	    {
	    	for (int i = 0; i < d_Ng; i++)
		    {
		    	d_grb_lin_obj_us += d_lin_cost_vec_us(i) * d_grb_vars[i];
		    	d_grb_lin_obj_r += d_lin_cost_vec_r(i) * d_grb_vars[i];
		    	d_grb_lin_obj_gs += d_lin_cost_vec_gs(i) * d_grb_vars[i];
		    }
	    }
	    
	    // Set objective
	    d_grb_model.setObjective(d_grb_quad_obj + d_grb_lin_obj_us + d_grb_lin_obj_r + d_grb_lin_obj_gs);

	    // Clear constraints if previously created
	    int num_constrs = d_grb_model.get(GRB_IntAttr_NumConstrs);
    	GRBConstr* grb_constrs = d_grb_model.getConstrs();
    	for (int i = 0; i < num_constrs; i++)
    		d_grb_model.remove(grb_constrs[i]);
    	d_grb_model.update();
    	delete[] grb_constrs;
    	delete[] d_grb_dyn_constrs;

	    // Add inequality constraints
	    // Note that grb_A_g is empty when using sparse formulation and for loop below does not run
	    for (int i = 0; i < grb_A_g.rows(); i++)
	    {
	    	GRBLinExpr lhs = 0;
	    	for (int j = 0; j < d_Ng; j++)
	    		lhs += grb_A_g(i,j) * d_grb_vars[j];
	    	d_grb_model.addConstr(lhs, '<', grb_b_g(i));
	    }

	    // Add static equality constraints
	    // Note that grb_A_eq_stat is empty when using dense formulation and for loop below does not run
	    for (int i = 0; i < grb_A_eq_stat.rows(); i++)
	    {
	    	GRBLinExpr lhs = 0;
	    	for (int j = 0; j < d_num_opt_vars; j++)
	    		lhs += grb_A_eq_stat(i,j) * d_grb_vars[j];
	    	d_grb_model.addConstr(lhs, '=', grb_b_eq_stat(i));
	    }

	    // Add dynamic equality constraints and store in memory for quick change
	    d_grb_dyn_constrs = new GRBConstr[d_num_dyn_eq_constr];
	    for (int i = 0; i < d_num_dyn_eq_constr; i++)
	    {
	    	GRBLinExpr lhs = 0;
	    	for (int j = 0; j < d_num_opt_vars; j++)
	    		lhs += grb_A_eq_dyn(i,j) * d_grb_vars[j];
	    	d_grb_dyn_constrs[i] = d_grb_model.addConstr(lhs, '=', grb_b_eq_dyn(i));
	    }

	    // Set model parameters
	    if (d_grb_LogToFile)
			d_grb_model.set(GRB_StringParam_LogFile, d_logFolder + "gurobi.log");
		else
			d_grb_model.set(GRB_StringParam_LogFile, "");
		d_grb_model.set(GRB_IntParam_LogToConsole, d_opt_verbose);
		// Setting Aggregate to 0 was found to speed up optimization using Gurobi auto-tune
	    // This is only relevant when Presolve is on (dense formulation)
	    d_grb_model.set(GRB_IntParam_Aggregate, 0);
	    // Skip Presolve when using sparse model, as it returns same model (presolving sparsifies)
	    if (d_opt_sparse)
	    	d_grb_model.set(GRB_IntParam_Presolve, 0);
	    else
	    	d_grb_model.set(GRB_IntParam_Presolve, -1);
	    

	    // Presolve - only applicable for dense formulation
	    if (!d_opt_sparse && d_grb_presolve_at_setup)
	    {
		    static GRBModel grb_model_presolved = d_grb_model.presolve();
		    
		    // Update variables to presolved model variables
		    d_grb_vars = grb_model_presolved.getVars();

		    // Update dynamic equality constraints to presolved model constraints. They are last set of constraints in presolved model
		    num_constrs = grb_model_presolved.get(GRB_IntAttr_NumConstrs);
		    GRBConstr* grb_constrs_presolved = grb_model_presolved.getConstrs();
		    int j = 0;
	    	for (int i = num_constrs - d_num_dyn_eq_constr; i < num_constrs; i++)
	    	{
	    		d_grb_dyn_constrs[j] = grb_constrs_presolved[i];
	    		j++;
	    	}
	    	delete[] grb_constrs_presolved;

	    	// Set Presolve parameter of presolved model to 0 to avoid re-presolving
		    grb_model_presolved.set(GRB_IntParam_Presolve, 0);

		    // Global variable is pointer to model
		    d_grb_model_presolved = &grb_model_presolved;
		}

	    // Some steps to finish setup
		finish_Rampc_setup();

	    // Inform the user
	    ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup successful with Gurobi");
    }
    
    catch(GRBException e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization setup exception with Gurobi error code = " << e.getErrorCode());
	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Error message: " << e.getMessage());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization setup exception with Gurobi with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup exception with Gurobi");
    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	*/
}

void setup_Rampc_osqp()
{
	try
    {
    	
		// Get u_data & y_data from files
		//MatrixXf u_data = get_u_data();
		//MatrixXf y_data = get_y_data();

		// Get variable lengths
		get_variable_lengths();
		get_control_feedback();
	
		// Get Hankel matrices
		//get_hankel_matrices(u_data, y_data);
		
		// Get cost matrices
		//d_num_inputs=1;
		//d_num_states=2;
		get_cost_matrices();

		
		// OSQP QUADRATIC COST MATRIX
		d_osqp_P = get_quad_cost_matrix();

		
		// Input/output constraint vectors
		get_input_output_constr();
		get_dynamics_matrix();

		get_tube_params();



		MatrixXf x_hat_0;
		x_hat_0=MatrixXf::Zero(d_num_outputs,1);
		x_hat_0(0,0)=-0.5;
		

		MatrixXf x_bar_0=MatrixXf::Zero(d_num_outputs,1);
		x_bar_0=x_hat_0;
		


		// INITIAL STATE constraints
		MatrixXf osqp_A_x_hat_ini=MatrixXf::Zero(d_num_outputs,d_col_num);
		MatrixXf osqp_l_x_hat_ini=MatrixXf::Zero(d_num_outputs,1);
		MatrixXf osqp_u_x_hat_ini=MatrixXf::Zero(d_num_outputs,1);

		osqp_A_x_hat_ini.block(0,0,d_num_outputs,d_num_outputs)=MatrixXf::Identity(d_num_outputs,d_num_outputs);
		osqp_l_x_hat_ini=x_hat_0;
		osqp_u_x_hat_ini=x_hat_0;


		MatrixXf osqp_A_x_bar_ini=MatrixXf::Zero(d_num_outputs,d_col_num);
		MatrixXf osqp_l_x_bar_ini=MatrixXf::Zero(d_num_outputs,1);
		MatrixXf osqp_u_x_bar_ini=MatrixXf::Zero(d_num_outputs,1);

		osqp_A_x_bar_ini.block(0,d_num_outputs*(d_N+1),d_num_outputs,d_num_outputs)=MatrixXf::Identity(d_num_outputs,d_num_outputs);
		osqp_l_x_bar_ini=x_bar_0;
		osqp_u_x_bar_ini=x_bar_0;


		MatrixXf osqp_A_s_ini=MatrixXf::Zero(1,d_col_num);
		MatrixXf osqp_l_s_ini=MatrixXf::Zero(1,1);
		MatrixXf osqp_u_s_ini=MatrixXf::Zero(1,1);

		osqp_A_s_ini(0,d_num_outputs*(d_N+1)*2+d_num_inputs*d_N)=1.0;
		osqp_l_s_ini(0,0)=0.0;
		osqp_u_s_ini(0,0)=0.0;


		MatrixXf osqp_A_hat_dynamics_constr=MatrixXf::Zero(d_num_outputs*d_N,d_col_num);
		MatrixXf osqp_l_hat_dynamics_constr=MatrixXf::Zero(d_num_outputs*d_N,1);
		MatrixXf osqp_u_hat_dynamics_constr=MatrixXf::Zero(d_num_outputs*d_N,1);


		MatrixXf dynamics_constr_A_hat_temp=d_A+(d_B_0+d_B_1*d_theta_hat_k)*d_K;
		MatrixXf dynamics_constr_B_hat_temp=(d_B_0+d_B_1*d_theta_hat_k);
		for(int i=0;i<d_N;i++){
			osqp_A_hat_dynamics_constr.block(i*d_num_outputs,i*d_num_outputs,d_num_outputs,d_num_outputs)=dynamics_constr_A_hat_temp;
			osqp_A_hat_dynamics_constr.block(i*d_num_outputs,i*d_num_outputs+d_num_outputs,d_num_outputs,d_num_outputs)=-MatrixXf::Identity(d_num_outputs,d_num_outputs);
			osqp_A_hat_dynamics_constr.block(i*d_num_outputs,d_num_outputs*(d_N+1)*2+i*d_num_inputs,d_num_outputs,d_num_inputs)=dynamics_constr_B_hat_temp;
			osqp_l_hat_dynamics_constr.block(i*d_num_outputs,0,d_num_outputs,1)=MatrixXf::Zero(d_num_outputs,1);
			osqp_u_hat_dynamics_constr.block(i*d_num_outputs,0,d_num_outputs,1)=MatrixXf::Zero(d_num_outputs,1);
		}



		MatrixXf osqp_A_bar_dynamics_constr=MatrixXf::Zero(d_num_outputs*d_N,d_col_num);
		MatrixXf osqp_l_bar_dynamics_constr=MatrixXf::Zero(d_num_outputs*d_N,1);
		MatrixXf osqp_u_bar_dynamics_constr=MatrixXf::Zero(d_num_outputs*d_N,1);

		MatrixXf dynamics_constr_A_bar_temp=d_A+(d_B_0+d_B_1*d_theta_bar_k)*d_K;
		MatrixXf dynamics_constr_B_bar_temp=(d_B_0+d_B_1*d_theta_bar_k);
		for(int i=0;i<d_N;i++){
			osqp_A_bar_dynamics_constr.block(i*d_num_outputs,d_num_outputs*(d_N+1)+i*d_num_outputs,d_num_outputs,d_num_outputs)=dynamics_constr_A_bar_temp;
			osqp_A_bar_dynamics_constr.block(i*d_num_outputs,d_num_outputs*(d_N+1)+i*d_num_outputs+d_num_outputs,d_num_outputs,d_num_outputs)=-MatrixXf::Identity(d_num_outputs,d_num_outputs);
			osqp_A_bar_dynamics_constr.block(i*d_num_outputs,d_num_outputs*(d_N+1)*2+i*d_num_inputs,d_num_outputs,d_num_inputs)=dynamics_constr_B_bar_temp;
			osqp_l_bar_dynamics_constr.block(i*d_num_outputs,0,d_num_outputs,1)=MatrixXf::Zero(d_num_outputs,1);
			osqp_u_bar_dynamics_constr.block(i*d_num_outputs,0,d_num_outputs,1)=MatrixXf::Zero(d_num_outputs,1);
		}


		MatrixXf osqp_A_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),d_col_num);
		MatrixXf osqp_l_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
		MatrixXf osqp_u_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
	    d_z_setpoint_Rampc=MatrixXf::Zero(d_num_outputs,1);
		d_z_setpoint_Rampc << 0.4-0.8,		
							   0.0;

		for(int i=0;i<d_N;i++){
			for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
				osqp_A_state_input_constr.block(i*(d_num_outputs*2+d_num_inputs*2)+j,d_num_outputs*(d_N+1)+i*d_num_outputs,1,d_num_outputs)=d_F.row(j)+d_G.row(j)*d_K;
				osqp_A_state_input_constr.block(i*(d_num_outputs*2+d_num_inputs*2)+j,d_num_outputs*(d_N+1)*2+i*d_num_inputs,1,d_num_inputs)=d_G.row(j);
				osqp_A_state_input_constr.block(i*(d_num_outputs*2+d_num_inputs*2)+j,d_num_outputs*(d_N+1)*2+d_num_inputs*d_N+i,1,1)=d_c.row(j);
				osqp_u_state_input_constr.block(i*(d_num_outputs*2+d_num_inputs*2)+j,0,1,1)=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
				osqp_l_state_input_constr(i*(d_num_outputs*2+d_num_inputs*2)+j,0)=Inf_min;
			}
		}


		MatrixXf osqp_A_tube_constr=MatrixXf::Zero(d_N*d_H_x.rows()*d_e_l.rows(),d_col_num);
		MatrixXf osqp_l_tube_constr=MatrixXf::Zero(d_N*d_H_x.rows()*d_e_l.rows(),1);
		MatrixXf osqp_u_tube_constr=MatrixXf::Zero(d_N*d_H_x.rows()*d_e_l.rows(),1);
		
		for(int i=0;i<d_N;i++){
			for(int j=0;j<d_H_x.rows();j++){
				for(int l=0;l<d_e_l.rows();l++){
					osqp_A_tube_constr.block(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)+i*d_num_outputs,1,d_num_outputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_K*d_e_l(l,0);
					osqp_A_tube_constr.block(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+i*d_num_inputs,1,d_num_inputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_e_l.row(l);
					osqp_A_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+d_num_inputs*d_N+i)=-d_rho_theta_k;
					osqp_A_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+d_num_inputs*d_N+i+1)=1.0;
					osqp_u_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,0)=Inf_max;
					osqp_l_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,0)=d_w_bar;
				}
			}
		}
		

		MatrixXf osqp_A_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),d_col_num);
		MatrixXf osqp_l_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);
		MatrixXf osqp_u_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);

		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*d_N,d_H_xf_x.rows(),d_num_outputs)=d_H_xf_x;
		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*(d_N+1)+d_num_inputs*d_N+d_N,d_H_xf_x.rows(),1)=d_H_xf_s;
		osqp_u_term_constr=MatrixXf::Ones(d_H_xf_x.rows(),1);
		osqp_l_term_constr=Inf_min*MatrixXf::Ones(d_H_xf_x.rows(),1);






		MatrixXf osqp_A=MatrixXf::Zero(osqp_A_x_hat_ini.rows()+osqp_A_x_bar_ini.rows()+osqp_A_s_ini.rows()+osqp_A_hat_dynamics_constr.rows()+osqp_A_bar_dynamics_constr.rows()+osqp_A_state_input_constr.rows()+osqp_A_tube_constr.rows()+osqp_A_term_constr.rows(),d_col_num);
		MatrixXf osqp_u=MatrixXf::Zero(osqp_u_x_hat_ini.rows()+osqp_u_x_bar_ini.rows()+osqp_u_s_ini.rows()+osqp_u_hat_dynamics_constr.rows()+osqp_u_bar_dynamics_constr.rows()+osqp_u_state_input_constr.rows()+osqp_u_tube_constr.rows()+osqp_u_term_constr.rows(),1);
		MatrixXf osqp_l=MatrixXf::Zero(osqp_l_x_hat_ini.rows()+osqp_l_x_bar_ini.rows()+osqp_l_s_ini.rows()+osqp_l_hat_dynamics_constr.rows()+osqp_l_bar_dynamics_constr.rows()+osqp_l_state_input_constr.rows()+osqp_l_tube_constr.rows()+osqp_l_term_constr.rows(),1);

		osqp_A.topRows(osqp_A_x_hat_ini.rows())=osqp_A_x_hat_ini;
		osqp_u.topRows(osqp_u_x_hat_ini.rows())=osqp_u_x_hat_ini;
		osqp_l.topRows(osqp_l_x_hat_ini.rows())=osqp_l_x_hat_ini;

		osqp_A.middleRows(osqp_A_x_hat_ini.rows(),osqp_A_x_bar_ini.rows())=osqp_A_x_bar_ini;
		osqp_u.middleRows(osqp_u_x_hat_ini.rows(),osqp_u_x_bar_ini.rows())=osqp_u_x_bar_ini;
		osqp_l.middleRows(osqp_l_x_hat_ini.rows(),osqp_l_x_bar_ini.rows())=osqp_l_x_bar_ini;

		osqp_A.middleRows(osqp_A_x_hat_ini.rows()+osqp_A_x_bar_ini.rows(),osqp_A_s_ini.rows())=osqp_A_s_ini;
		osqp_u.middleRows(osqp_u_x_hat_ini.rows()+osqp_u_x_bar_ini.rows(),osqp_u_s_ini.rows())=osqp_u_s_ini;
		osqp_l.middleRows(osqp_l_x_hat_ini.rows()+osqp_l_x_bar_ini.rows(),osqp_l_s_ini.rows())=osqp_l_s_ini;

		osqp_A.middleRows(osqp_A_x_hat_ini.rows()+osqp_A_x_bar_ini.rows()+osqp_A_s_ini.rows(),osqp_A_hat_dynamics_constr.rows())=osqp_A_hat_dynamics_constr;
		osqp_u.middleRows(osqp_u_x_hat_ini.rows()+osqp_u_x_bar_ini.rows()+osqp_u_s_ini.rows(),osqp_u_hat_dynamics_constr.rows())=osqp_u_hat_dynamics_constr;
		osqp_l.middleRows(osqp_l_x_hat_ini.rows()+osqp_l_x_bar_ini.rows()+osqp_l_s_ini.rows(),osqp_l_hat_dynamics_constr.rows())=osqp_l_hat_dynamics_constr;

		osqp_A.middleRows(osqp_A_x_hat_ini.rows()+osqp_A_x_bar_ini.rows()+osqp_A_s_ini.rows()+osqp_A_hat_dynamics_constr.rows(),osqp_A_bar_dynamics_constr.rows())=osqp_A_bar_dynamics_constr;
		osqp_u.middleRows(osqp_u_x_hat_ini.rows()+osqp_u_x_bar_ini.rows()+osqp_u_s_ini.rows()+osqp_u_hat_dynamics_constr.rows(),osqp_u_bar_dynamics_constr.rows())=osqp_u_bar_dynamics_constr;
		osqp_l.middleRows(osqp_l_x_hat_ini.rows()+osqp_l_x_bar_ini.rows()+osqp_l_s_ini.rows()+osqp_l_hat_dynamics_constr.rows(),osqp_l_bar_dynamics_constr.rows())=osqp_l_bar_dynamics_constr;

		osqp_A.middleRows(osqp_A_x_hat_ini.rows()+osqp_A_x_bar_ini.rows()+osqp_A_s_ini.rows()+osqp_A_hat_dynamics_constr.rows()+osqp_A_bar_dynamics_constr.rows(),osqp_A_state_input_constr.rows())=osqp_A_state_input_constr;
		osqp_u.middleRows(osqp_u_x_hat_ini.rows()+osqp_u_x_bar_ini.rows()+osqp_u_s_ini.rows()+osqp_u_hat_dynamics_constr.rows()+osqp_u_bar_dynamics_constr.rows(),osqp_u_state_input_constr.rows())=osqp_u_state_input_constr;
		osqp_l.middleRows(osqp_l_x_hat_ini.rows()+osqp_l_x_bar_ini.rows()+osqp_l_s_ini.rows()+osqp_l_hat_dynamics_constr.rows()+osqp_l_bar_dynamics_constr.rows(),osqp_l_state_input_constr.rows())=osqp_l_state_input_constr;

		osqp_A.middleRows(osqp_A_x_hat_ini.rows()+osqp_A_x_bar_ini.rows()+osqp_A_s_ini.rows()+osqp_A_hat_dynamics_constr.rows()+osqp_A_bar_dynamics_constr.rows()+osqp_A_state_input_constr.rows(),osqp_A_tube_constr.rows())=osqp_A_tube_constr;
		osqp_u.middleRows(osqp_u_x_hat_ini.rows()+osqp_u_x_bar_ini.rows()+osqp_u_s_ini.rows()+osqp_u_hat_dynamics_constr.rows()+osqp_u_bar_dynamics_constr.rows()+osqp_u_state_input_constr.rows(),osqp_u_tube_constr.rows())=osqp_u_tube_constr;
		osqp_l.middleRows(osqp_l_x_hat_ini.rows()+osqp_l_x_bar_ini.rows()+osqp_l_s_ini.rows()+osqp_l_hat_dynamics_constr.rows()+osqp_l_bar_dynamics_constr.rows()+osqp_l_state_input_constr.rows(),osqp_l_tube_constr.rows())=osqp_l_tube_constr;

		osqp_A.bottomRows(osqp_A_term_constr.rows()) = osqp_A_term_constr;
		osqp_l.bottomRows(osqp_A_term_constr.rows()) = osqp_l_term_constr;
		osqp_u.bottomRows(osqp_A_term_constr.rows()) = osqp_u_term_constr;




		/*
		d_osqp_q = MatrixXf::Zero(d_num_opt_vars, 1);
		if (d_opt_sparse)
		{
			d_osqp_q.topRows(d_Ng) = d_lin_cost_vec_gs;
			d_osqp_q.middleRows(d_uf_start_i, d_Nuf) = d_lin_cost_vec_us;
			d_osqp_q.middleRows(d_yf_start_i, d_Nyf + d_num_outputs) = d_lin_cost_vec_r;
		}
		else
			d_osqp_q.topRows(d_Ng) = d_lin_cost_vec_us + d_lin_cost_vec_r + d_lin_cost_vec_gs;
		*/

		// OSQP LINEAR INEQUALITY CONSTRAINT MATRIX
		/*
		MatrixXf osqp_A_ineq = MatrixXf::Zero(d_Nuf + d_Nyf + d_num_outputs, d_num_opt_vars);
		if (d_opt_sparse)
			osqp_A_ineq.middleCols(d_uf_start_i, d_Nuf + d_Nyf + d_num_outputs) = MatrixXf::Identity(d_Nuf + d_Nyf + d_num_outputs, d_Nuf + d_Nyf + d_num_outputs);
		else
		{
			osqp_A_ineq.topLeftCorner(d_Nuf, d_Ng) = d_U_f;
			osqp_A_ineq.bottomLeftCorner(d_Nyf + d_num_outputs, d_Ng) = d_Y_f;
		}

		// OSQP LINEAR INEQUALITY CONSTRAINT VECTORS
		MatrixXf osqp_l_ineq = MatrixXf::Zero(d_Nuf + d_Nyf + d_num_outputs, 1);
		MatrixXf osqp_u_ineq = MatrixXf::Zero(d_Nuf + d_Nyf + d_num_outputs, 1);
		osqp_l_ineq.topRows(d_Nuf) = d_input_min;
		osqp_l_ineq.bottomRows(d_Nyf + d_num_outputs) = d_output_min;
		osqp_u_ineq.topRows(d_Nuf) = d_input_max;
		osqp_u_ineq.bottomRows(d_Nyf + d_num_outputs) = d_output_max;

		// OSQP LINEAR EQUALITY CONSTRAINTS
		// Static equality constraints ([uf; yf; yt]), and ([gs; us]) if optimizing over steady state gs & us
		MatrixXf osqp_A_eq_stat = get_static_eq_constr_matrix();
		MatrixXf osqp_l_eq_stat = get_static_eq_constr_vector();
		MatrixXf osqp_u_eq_stat = osqp_l_eq_stat;

		// Dynamic equality constraints that change every time ([uini; uini]), and ([gs; r]) if optimizing over steady state gs & us
		MatrixXf osqp_A_eq_dyn = get_dynamic_eq_constr_matrix();
		MatrixXf osqp_l_eq_dyn = get_dynamic_eq_constr_vector();
		MatrixXf osqp_u_eq_dyn = osqp_l_eq_dyn;
		d_uini_start_i = osqp_A_ineq.rows() + osqp_A_eq_stat.rows();
		d_yini_start_i = d_uini_start_i + d_Nuini;
		if (d_opt_sparse && d_opt_steady_state)
			d_r_gs_start_i = d_yini_start_i + d_Nyini;

		// OSQP CONSTRAINTS
		// Concatenate all constraints in single matrix/vectors
		MatrixXf osqp_A = MatrixXf::Zero(osqp_A_ineq.rows() + osqp_A_eq_stat.rows() + osqp_A_eq_dyn.rows(), d_num_opt_vars);
		MatrixXf osqp_l = MatrixXf::Zero(osqp_A_ineq.rows() + osqp_A_eq_stat.rows() + osqp_A_eq_dyn.rows(), 1);
		MatrixXf osqp_u = osqp_l;
		
		osqp_A.topRows(osqp_A_ineq.rows()) = osqp_A_ineq;
		osqp_l.topRows(osqp_A_ineq.rows()) = osqp_l_ineq;
		osqp_u.topRows(osqp_A_ineq.rows()) = osqp_u_ineq;

		// Static equality constraints only present in sparse formulation
		if (d_opt_sparse)
		{
			osqp_A.middleRows(osqp_A_ineq.rows(), osqp_A_eq_stat.rows()) = osqp_A_eq_stat;
			osqp_l.middleRows(osqp_A_ineq.rows(), osqp_A_eq_stat.rows()) = osqp_l_eq_stat;
			osqp_u.middleRows(osqp_A_ineq.rows(), osqp_A_eq_stat.rows()) = osqp_u_eq_stat;
		}

		osqp_A.bottomRows(osqp_A_eq_dyn.rows()) = osqp_A_eq_dyn;
		osqp_l.bottomRows(osqp_A_eq_dyn.rows()) = osqp_l_eq_dyn;
		osqp_u.bottomRows(osqp_A_eq_dyn.rows()) = osqp_u_eq_dyn;
	*/
		// OSQP MODEL SETUP
		// Follows 'Setup and solve' example (https://osqp.org/docs/examples/setup-and-solve.html)
		MatrixXf osqp_q=MatrixXf::Zero(d_col_num,1);


		osqp_extended_cleanup();
		

		// Convert Eigen matrices to CSC format
		csc* osqp_P_csc = eigen2csc(d_osqp_P);
		csc* osqp_A_csc = eigen2csc(osqp_A);
		

		// Convert Eigen vectors to c_float arrays
		// One copy is used for initial setup, the other is used during runtime in update function
		c_float* osqp_q_cfloat = (c_float*) c_malloc(d_osqp_q.rows() * sizeof(c_float));
		c_float* osqp_l_cfloat = (c_float*) c_malloc(osqp_l.rows() * sizeof(c_float));
		c_float* osqp_u_cfloat = (c_float*) c_malloc(osqp_u.rows() * sizeof(c_float));
		

		d_osqp_q_new = (c_float*) c_malloc(d_osqp_q.rows() * sizeof(c_float));
		d_osqp_l_new = (c_float*) c_malloc(osqp_l.rows() * sizeof(c_float));
		d_osqp_u_new = (c_float*) c_malloc(osqp_u.rows() * sizeof(c_float));
	

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_q_cfloat, d_osqp_q.rows(), d_osqp_q.cols()) = d_osqp_q.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_q_new, d_osqp_q.rows(), d_osqp_q.cols()) = d_osqp_q.cast<c_float>();
	

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_l_cfloat, osqp_l.rows(), osqp_l.cols()) = osqp_l.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_l_new, osqp_l.rows(), osqp_l.cols()) = osqp_l.cast<c_float>();
	

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_u_cfloat, osqp_u.rows(), osqp_u.cols()) = osqp_u.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_u_new, osqp_u.rows(), osqp_u.cols()) = osqp_u.cast<c_float>();
	

		// Populate data
	    OSQPData* osqp_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_data->n = d_col_num;
	    osqp_data->m = osqp_A.rows();
	    osqp_data->P = osqp_P_csc;
	    osqp_data->q = osqp_q_cfloat;
	    osqp_data->A = osqp_A_csc;
	    osqp_data->l = osqp_l_cfloat;
	    osqp_data->u = osqp_u_cfloat;
		

		// Problem settings
	    d_osqp_settings = (OSQPSettings*) c_malloc(sizeof(OSQPSettings));
		

	    // Define Solver settings as default, and change settings as desired
	    osqp_set_default_settings(d_osqp_settings);
	    d_osqp_settings->verbose = d_opt_verbose;
		

	    // Setup workspace
	    d_osqp_work = osqp_setup(osqp_data, d_osqp_settings);
		
		
	    osqp_solve(d_osqp_work);


	    //d_RampcOpt_status = d_osqp_work->info->status_val;
	    //float sol_u=d_osqp_work->solution->x[2*(d_N+1)*d_num_outputs]; 
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Status: "<< d_RampcOpt_status);
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Solution: "<< sol_u);


	    // Clear data after setting up to allow subseqeuent setups
	    osqp_cleanup_data(osqp_data);
		


	    if (!d_osqp_work)
	    {
	    	clear_setupRampc_success_flag();

	    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup failed with OSQP");
	    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");

	    	return;
	    }

	    // Some steps to finish setup
		finish_Rampc_setup();

		setup_theta_update_osqp();


	    // Inform the user
	    ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup successful with OSQP");
    }

  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization setup exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup exception with OSQP");
    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}

void solve_Rampc_gurobi()
{
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 622");
	d_uini = s_uini;
	d_yini = s_yini;
	s_Rampc_mutex.unlock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 622");

	try
	{
		// Update reference if reference is changing
		if (d_changing_ref_enable)
			change_Rampc_setpoint_gurobi_changing_ref();

		// Update equality constraints RHS
		for (d_i = 0; d_i < d_Nuini; d_i++)
			d_grb_dyn_constrs[d_i].set(GRB_DoubleAttr_RHS, d_uini(d_i));
		for (d_i = 0; d_i < d_Nyini; d_i++)
			d_grb_dyn_constrs[d_Nuini + d_i].set(GRB_DoubleAttr_RHS, d_yini(d_i));

		// Solve optimization - presolve only applies if using dense formulation
		if (d_opt_sparse || !d_grb_presolve_at_setup)
		{
			d_grb_model.optimize();
			d_RampcOpt_status = d_grb_model.get(GRB_IntAttr_Status);
		}
		else
		{
			d_grb_model_presolved->optimize();
			d_RampcOpt_status = d_grb_model_presolved->get(GRB_IntAttr_Status);
		}
		

		if (d_RampcOpt_status == GRB_OPTIMAL)
		{	
			// With sparse formulation can get uf and yf directly
			if (d_opt_sparse)
			{
				for (d_i = 0; d_i < d_Nuf; d_i++)
					d_u_f(d_i) = d_grb_vars[d_uf_start_i + d_i].get(GRB_DoubleAttr_X);

				for (d_i = 0; d_i < d_Nyf + d_num_outputs; d_i++)
					d_y_f(d_i) = d_grb_vars[d_yf_start_i + d_i].get(GRB_DoubleAttr_X);
			}
			// With dense formulation get uf and y_f through g
			else
			{
				for (d_i = 0; d_i < d_Ng; d_i++)
					d_g(d_i) = d_grb_vars[d_i].get(GRB_DoubleAttr_X);

				d_u_f = d_U_f * d_g;
				d_y_f = d_Y_f * d_g;
			}

			if (d_opt_sparse || !d_grb_presolve_at_setup)
				d_solve_time = d_grb_model.get(GRB_DoubleAttr_Runtime);
			else
				d_solve_time = d_grb_model_presolved->get(GRB_DoubleAttr_Runtime);

			s_Rampc_mutex.lock();
			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 649");
			s_u_f = d_u_f;
			s_y_f = d_y_f;
			s_solve_time = d_solve_time;
			s_Rampc_mutex.unlock();
			//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 649");

			ROS_INFO("[RAMPC CONTROLLER] Rampc found optimal solution with Gurobi:");
			ROS_INFO_STREAM("Thrust: " << d_u_f(0));
			ROS_INFO_STREAM("Roll Rate: " << d_u_f(1));
			ROS_INFO_STREAM("Pitch Rate: " << d_u_f(2));
			if (d_Rampc_yaw_control)
				ROS_INFO_STREAM("Yaw Rate: " << d_u_f(3));

			if (d_opt_sparse || !d_grb_presolve_at_setup)
				ROS_INFO_STREAM("Objective: " << d_grb_model.get(GRB_DoubleAttr_ObjVal));
			else
				ROS_INFO_STREAM("Objective: " << d_grb_model_presolved->get(GRB_DoubleAttr_ObjVal));

			ROS_INFO_STREAM("Runtime: " << d_solve_time);
		}
		else
		{
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc failed to find optimal solution with Gurobi status code = " << d_RampcOpt_status);
		}
	}

	catch(GRBException e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization exception with Gurobi error code = " << e.getErrorCode());
	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Error message: " << e.getMessage());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization exception with Gurobi with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization exception with Gurobi");
    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}

void solve_Rampc_osqp()
{
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 622");
	//d_uini = s_uini;
	//d_yini = s_yini;

		d_current_state_estimate = s_current_state_estimate;
		d_setpoint = s_setpoint;
		
	s_Rampc_mutex.unlock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 622");

	try
	{
		// Update reference if reference is changing
		//if (d_changing_ref_enable)
		//	change_Rampc_setpoint_osqp_changing_ref();

		// Update equality constraint vectors

		//ROS_INFO_STREAM("[RAMPC CONTRLLER] Current height: "<<d_setpoint(2));
		/*
		if (d_current_state_estimate(2,0)-d_setpoint(2)>d_reference_difference){
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Set setpoint difference to "<<d_reference_difference<<".");
			d_osqp_l_new[0]=d_reference_difference;
			d_osqp_u_new[0]=d_reference_difference;
			d_osqp_l_new[2]=d_reference_difference;
			d_osqp_u_new[2]=d_reference_difference;

			d_z_setpoint_Rampc<< d_current_state_estimate(2,0)-d_reference_difference,
								 0.0;



			for(int i=0;i<d_N;i++){
				for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
					temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
					d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
				}
			}

		}
		else{
			if (d_current_state_estimate(2,0)-d_setpoint(2)<-d_reference_difference){
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Set setpoint difference to "<<-d_reference_difference<<".");
				d_osqp_l_new[0]=-d_reference_difference;
				d_osqp_u_new[0]=-d_reference_difference;
				d_osqp_l_new[2]=-d_reference_difference;
				d_osqp_u_new[2]=-d_reference_difference;


				d_z_setpoint_Rampc<< d_current_state_estimate(2,0)+d_reference_difference,
								 0.0;



				for(int i=0;i<d_N;i++){
					for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
						temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
						d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
					}
				}

			}
			else{
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Did not set setpoint difference.");
				d_osqp_l_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_u_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_l_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_u_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
			}
		}
		*/
		d_osqp_l_new[1]=d_current_state_estimate(5,0);
		d_osqp_u_new[1]=d_current_state_estimate(5,0);
		
		d_osqp_l_new[3]=d_current_state_estimate(5,0);
		d_osqp_u_new[3]=d_current_state_estimate(5,0);
		
		d_osqp_l_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
		d_osqp_u_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
		d_osqp_l_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
		d_osqp_u_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
		

		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Delta_X: " << )


		/*

		for (d_i = 0; d_i < d_Nuini; d_i++)
		{
			d_osqp_l_new[d_uini_start_i + d_i] = d_uini(d_i);
			d_osqp_u_new[d_uini_start_i + d_i] = d_osqp_l_new[d_uini_start_i + d_i];
		}
		for (d_i = 0; d_i < d_Nyini; d_i++)
		{
			d_osqp_l_new[d_yini_start_i + d_i] = d_yini(d_i);
			d_osqp_u_new[d_yini_start_i + d_i] = d_osqp_l_new[d_yini_start_i + d_i];
		}
		*/
		osqp_update_bounds(d_osqp_work, d_osqp_l_new, d_osqp_u_new);





		// Solve optimization
		osqp_solve(d_osqp_work);
		d_RampcOpt_status = d_osqp_work->info->status_val;

		if (d_RampcOpt_status > 0)
		{	
			MatrixXf pred_x=MatrixXf::Zero(2,1);
			MatrixXf uplus=MatrixXf::Zero(1,1);
			for(int d_i=0;d_i<d_Nuf;d_i++){
				pred_x << d_osqp_work->solution->x[d_Nyf+d_num_outputs*d_i],
							 d_osqp_work->solution->x[d_Nyf+1+d_num_outputs*d_i];
				MatrixXf u_plus=d_K*pred_x;
				d_u_f(d_i) = d_osqp_work->solution->x[d_uf_start_i + d_i]+u_plus(0)+m_cf_weight_in_newtons;
			}
			// With sparse formulation can get uf and yf directly
			/*
			if (d_opt_sparse)
			{
				for (d_i = 0; d_i < d_Nuf; d_i++)
					d_u_f(d_i) = d_osqp_work->solution->x[d_uf_start_i + d_i];
				
				for (d_i = 0; d_i < d_Nyf + d_num_outputs; d_i++)
					d_y_f(d_i) = d_osqp_work->solution->x[d_yf_start_i + d_i];
			}
			// With dense formulation get uf and yf through g
			else
			{
				for (d_i = 0; d_i < d_Ng; d_i++)
					d_g(d_i) = d_osqp_work->solution->x[d_i];

				d_u_f = d_U_f * d_g;
				d_y_f = d_Y_f * d_g;
			}
			*/
			d_solve_time = d_osqp_work->info->run_time;

			s_Rampc_mutex.lock();
			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 649");
			s_u_f = d_u_f;
			s_y_f = d_y_f;
			s_solve_time = d_solve_time;
			s_Rampc_mutex.unlock();
			//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 649");
			
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc found optimal solution with OSQP status: " << d_osqp_work->info->status);
			ROS_INFO_STREAM("Thrust: " << d_u_f(0));
			//ROS_INFO_STREAM("Roll Rate: " << d_u_f(1));
			//ROS_INFO_STREAM("Pitch Rate: " << d_u_f(2));
			//if (d_Rampc_yaw_control)
			//	ROS_INFO_STREAM("Yaw Rate: " << d_u_f(3));
			//ROS_INFO_STREAM("Objective: " << d_osqp_work->info->obj_val);
			//ROS_INFO_STREAM("Runtime: " << d_solve_time);
			
		}
		else
		{
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc failed to find optimal solution with OSQP status: " << d_osqp_work->info->status);
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Current state estimate: " << d_current_state_estimate(2,0) << ", "<<d_current_state_estimate(5,0));
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Current setpoint: " << d_setpoint(2));
			
		}
	}

  	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization exception with OSQP");
    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}

// RAMPC HELPER FUNCTIONS

// Update uini yini
// This function is called by main thread
void update_uini_yini(Controller::Request &request, control_output &output)
{
	// If Rampc was not setup yet don't do anything as uini and yini matrices are not setup yet
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 741");
	// ROS_INFO("[MPC CONTROLLER] DEBUG 17");
	bool setupRampc_success = s_setupRampc_success;
	m_uini = s_uini;
	m_yini = s_yini;
	m_num_inputs = s_num_inputs;
	int num_outputs = s_num_outputs;
	int Nuini = s_Nuini;
	int Nyini = s_Nyini;
	// ROS_INFO("[MPC CONTROLLER] DEBUG 18");
	bool mpc_enabled = false;
	if (s_yaml_solver == "mpc")
		mpc_enabled = true;
	s_Rampc_mutex.unlock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 750");

	if (!setupRampc_success)
		return;

	try
	{
		//if (!mpc_enabled)
		//{	
			
			//ROS_INFO("[MPC CONTROLLER] DEBUG 19");
			/*
			// Update uini

			int u_shift = Nuini - m_num_inputs;
			m_uini.topRows(u_shift) = m_uini.bottomRows(u_shift);
			m_uini(u_shift) = output.thrust;
			m_uini(u_shift + 1) = output.rollRate;
			m_uini(u_shift + 2) = output.pitchRate;
			if (yaml_Rampc_yaw_control)
				m_uini(u_shift + 3) = output.yawRate;

			// Update yini
			int y_shift = Nyini - num_outputs;
			m_yini.topRows(y_shift) = m_yini.bottomRows(y_shift);
			m_yini(y_shift) = request.ownCrazyflie.x;
			m_yini(y_shift + 1) = request.ownCrazyflie.y;
			m_yini(y_shift + 2) = request.ownCrazyflie.z;
			if (yaml_Rampc_measure_roll_pitch)
			{
				m_yini(y_shift + 3) = request.ownCrazyflie.roll;
				m_yini(y_shift + 4) = request.ownCrazyflie.pitch;
			}
			if (yaml_Rampc_yaw_control)
				m_yini(Nyini - 1) = request.ownCrazyflie.yaw;
				*/
		//}
		//else
		//{
			 //ROS_INFO_STREAM("[MPC CONTROLLER] DEBUG 20");
			// Update current_state_estimate
			m_current_state_estimate(0) = request.ownCrazyflie.x;
			m_current_state_estimate(1) = request.ownCrazyflie.y;
			m_current_state_estimate(2) = request.ownCrazyflie.z;
			// ROS_INFO_STREAM("[MPC CONTROLLER] DEBUG 21");


			m_current_state_estimate(3) = (request.ownCrazyflie.x - m_previous_xyz(0)) * yaml_control_frequency;
			m_current_state_estimate(4) = (request.ownCrazyflie.y - m_previous_xyz(1)) * yaml_control_frequency;
			m_current_state_estimate(5) = (request.ownCrazyflie.z - m_previous_xyz(2)) * yaml_control_frequency;
			// ROS_INFO("[MPC CONTROLLER] DEBUG 22");


			m_current_state_estimate(6) = request.ownCrazyflie.roll;
			m_current_state_estimate(7) = request.ownCrazyflie.pitch;
			//ROS_INFO("[MPC CONTROLLER] DEBUG 23");


			for(int i = 0; i < 3; ++i)
			{
				m_previous_xyz(i) = m_current_state_estimate(i);
			}
			//ROS_INFO("[MPC CONTROLLER] DEBUG 24");

		//}



		s_Rampc_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 786");
		s_uini = m_uini;
		s_yini = m_yini;
		s_current_state_estimate = m_current_state_estimate;
		s_Rampc_mutex.unlock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 786");
	}

	catch(exception& e)
    {
	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Update uini yini exception with standard error message: " << e.what());
  	}
  	catch(...)
  	{
    	ROS_INFO("[RAMPC CONTROLLER] Update uini yini exception");
  	}
}

void setup_theta_update_osqp(){
	try{
		MatrixXf osqp_theta_P;
		MatrixXf osqp_theta_A;
		MatrixXf osqp_theta_l;
		MatrixXf osqp_theta_u;
		MatrixXf osqp_theta_q_min;
		MatrixXf osqp_theta_q_max;

		osqp_theta_P=MatrixXf::Zero(d_N_theta,d_N_theta);
		osqp_theta_A=MatrixXf::Zero(2*d_N_theta+d_num_outputs,d_N_theta);
		osqp_theta_l=Inf_min*MatrixXf::Ones(2*d_N_theta+d_num_outputs,1);
		osqp_theta_u=MatrixXf::Ones(2*d_N_theta+d_num_outputs,1);
		osqp_theta_q_min=MatrixXf::Ones(d_N_theta,d_N_theta);
		osqp_theta_q_max=-1*MatrixXf::Ones(d_N_theta,d_N_theta);

		osqp_theta_extended_cleanup();
		

		// Convert Eigen matrices to CSC format
		csc* osqp_theta_P_csc = eigen2csc(osqp_theta_P);
		csc* osqp_theta_A_csc = eigen2csc(osqp_theta_A);
		
		// Convert Eigen vectors to c_float arrays
		// One copy is used for initial setup, the other is used during runtime in update function
		c_float* osqp_theta_q_min_cfloat = (c_float*) c_malloc(osqp_theta_q_min.rows() * sizeof(c_float));
		c_float* osqp_theta_q_max_cfloat = (c_float*) c_malloc(osqp_theta_q_max.rows() * sizeof(c_float));
		c_float* osqp_theta_l_cfloat = (c_float*) c_malloc(osqp_theta_l.rows() * sizeof(c_float));
		c_float* osqp_theta_u_cfloat = (c_float*) c_malloc(osqp_theta_u.rows() * sizeof(c_float));
		

		//d_osqp_theta_q_new = (c_float*) c_malloc(d_osqp_theta_q.rows() * sizeof(c_float));
		d_osqp_theta_l_new = (c_float*) c_malloc(osqp_theta_l.rows() * sizeof(c_float));
		d_osqp_theta_u_new = (c_float*) c_malloc(osqp_theta_u.rows() * sizeof(c_float));
	
		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_theta_q_max_cfloat, osqp_theta_q_max.rows(), osqp_theta_q_max.cols()) = osqp_theta_q_max.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_theta_q_min_cfloat, osqp_theta_q_min.rows(), osqp_theta_q_min.cols()) = osqp_theta_q_min.cast<c_float>();
		//Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_theta_q_new, d_osqp_theta_q.rows(), d_osqp_theta_q.cols()) = d_osqp_theta_q.cast<c_float>();
	

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_theta_l_cfloat, osqp_theta_l.rows(), osqp_theta_l.cols()) = osqp_theta_l.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_theta_l_new, osqp_theta_l.rows(), osqp_theta_l.cols()) = osqp_theta_l.cast<c_float>();
	

		Matrix<c_float, Dynamic, Dynamic>::Map(osqp_theta_u_cfloat, osqp_theta_u.rows(), osqp_theta_u.cols()) = osqp_theta_u.cast<c_float>();
		Matrix<c_float, Dynamic, Dynamic>::Map(d_osqp_theta_u_new, osqp_theta_u.rows(), osqp_theta_u.cols()) = osqp_theta_u.cast<c_float>();
	
		// Populate data
	    OSQPData* osqp_theta_max_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_theta_max_data->n = osqp_theta_A.cols();
	    osqp_theta_max_data->m = osqp_theta_A.rows();
	    osqp_theta_max_data->P = osqp_theta_P_csc;
	    osqp_theta_max_data->q = osqp_theta_q_max_cfloat;
	    osqp_theta_max_data->A = osqp_theta_A_csc;
	    osqp_theta_max_data->l = osqp_theta_l_cfloat;
	    osqp_theta_max_data->u = osqp_theta_u_cfloat;

	    OSQPData* osqp_theta_min_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_theta_min_data->n = osqp_theta_A.cols();
	    osqp_theta_min_data->m = osqp_theta_A.rows();
	    osqp_theta_min_data->P = osqp_theta_P_csc;
	    osqp_theta_min_data->q = osqp_theta_q_min_cfloat;
	    osqp_theta_min_data->A = osqp_theta_A_csc;
	    osqp_theta_min_data->l = osqp_theta_l_cfloat;
	    osqp_theta_min_data->u = osqp_theta_u_cfloat;
		
		// Problem settings
	    d_osqp_theta_settings = (OSQPSettings*) c_malloc(sizeof(OSQPSettings));
		

	    // Define Solver settings as default, and change settings as desired
	    osqp_set_default_settings(d_osqp_theta_settings);
	    d_osqp_theta_settings->verbose = d_opt_verbose;
		

	    // Setup workspace
	    d_osqp_theta_max_work = osqp_setup(osqp_theta_max_data, d_osqp_settings);
	    d_osqp_theta_min_work = osqp_setup(osqp_theta_min_data, d_osqp_settings);
		
	    osqp_solve(d_osqp_theta_max_work);
	    osqp_solve(d_osqp_theta_min_work);

	    //d_RampcOpt_status = d_osqp_work->info->status_val;
	    //float sol_u=d_osqp_work->solution->x[2*(d_N+1)*d_num_outputs]; 
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Status: "<< d_RampcOpt_status);
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Solution: "<< sol_u);

	    // Clear data after setting up to allow subseqeuent setups
	    osqp_cleanup_data(osqp_theta_min_data);
	    c_free(osqp_theta_max_data->q);
	    osqp_solve(d_osqp_theta_max_work);
	    osqp_solve(d_osqp_theta_min_work);

	    if ((!d_osqp_theta_max_work) || (!d_osqp_theta_min_work))
	    {
	    	clear_setupRampc_success_flag();

	    	ROS_INFO("[RAMPC CONTROLLER] Rampc theta update setup failed with OSQP");
	    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");

	    	return;
	    }



	}
	catch(exception& e)
    {
    	clear_setupRampc_success_flag();

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc optimization exception with OSQP with standard error message: " << e.what());
	    ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
  	catch(...)
  	{
  		clear_setupRampc_success_flag();

    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization exception with OSQP");
    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");
  	}
}





void get_tube_params(){

	int H_x_size=8;
	d_H_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_s=MatrixXf::Zero(H_x_size,1);
	d_c=MatrixXf::Zero(6,1);
	d_e_l=MatrixXf::Zero(2,1);




	/*
	d_H_x << 	-4.5984, -1.2755,
				6.0957, 1.6908,
				5.5192, 0.7645,
				3.1222, 0.0996,
				-7.3163, -1.0134,
				-4.1389, -0.1321;

	d_H_xf_x <<	-4.5984, -1.2755,
				6.0957, 1.6908,
				5.5192, 0.7645,
				3.1222, 0.0996,
				-7.3163, -1.0134,
				-4.1389, -0.1321;
	*/

	d_H_x << 	-4.4652, -1.1794,
				5.9192, 1.5634,
				2.2426, -0.1519,
				-2.9729, 0.2013,
				5.2618, 0.5232,
				4.9847, 0.4500,
				-6.9751, -0.6936,
				-6.6078, -0.5966;

	d_H_xf_x=d_H_x;

	d_H_xf_s=MatrixXf::Ones(H_x_size,1);
	/*
	d_H_xf_s << 1.0,
				1.0,
				1.0,
				1.0,
				1.0,
				1.0;
	*/

	/*
	d_c <<	0.5574,
			0.4205,
			0.1653,
			0.2191,
			1.0,
			1.0;
	*/

	d_c <<	0.4868,
			0.3672,
			0.1523,
			0.2019,
			1.0,
			1.0;

	d_w_bar=0.0714;

	d_theta_bar_k=33.67;
	d_theta_hat_k=d_theta_bar_k;
	d_eta_k=6.7340;
	d_rho_theta_k=0.6;
	d_e_l << 0.5,
			 - 0.5;


}

void get_dynamics_matrix(){
	d_A=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	d_B_0=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	d_B_1=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	MatrixXf d_A_quad=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	
	MatrixXf d_eye=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	
	d_A_quad<< 	0.0, 1.0,
		   		0.0, 0.0;

	d_eye = MatrixXf::Identity(d_num_outputs,d_num_outputs);
 	d_A=d_eye+d_T_s*d_A_quad;

 	d_B_0 << 0.0,
 			 0.0;
 			
 	MatrixXf B_1=MatrixXf::Zero(d_num_outputs,d_num_inputs);
 	B_1 << 0.0,
 		   1.0;
 		
 	d_B_1=d_T_s*B_1;
 	
}


// Get u_data from file
MatrixXf get_u_data()
{
	MatrixXf u_data_in = read_csv(d_dataFolder + "/output/m_u_data.csv");
	if (u_data_in.size() <= 0)
	{
		clear_setupRampc_success_flag();

    	ROS_INFO("[RAMPC CONTROLLER] Failed to read u data file");

		return MatrixXf::Zero(0, 0);
	}

	MatrixXf u_data;
	if (d_Rampc_yaw_control)
		u_data = u_data_in;
	else
		u_data = u_data_in.topRows(3);

	return u_data;
}

// Get y_data from file
MatrixXf get_y_data()
{
	MatrixXf y_data_in = read_csv(d_dataFolder + "/output/m_y_data.csv");
	if (y_data_in.size() <= 0)
	{	
		clear_setupRampc_success_flag();
		
		ROS_INFO("[RAMPC CONTROLLER] Failed to read y data file");

		return MatrixXf::Zero(0, 0);
	}

	MatrixXf y_data;
	if (d_Rampc_measure_roll_pitch && d_Rampc_yaw_control)
		y_data = y_data_in;
	else if (d_Rampc_measure_roll_pitch)
		y_data = y_data_in.topRows(5);
	else if (d_Rampc_yaw_control)
	{
		y_data = MatrixXf::Zero(4, y_data_in.cols());
		y_data.topRows(3) = y_data_in.topRows(3);
		y_data.bottomRows(1) = y_data_in.bottomRows(1);
	}
	else
		y_data = y_data_in.topRows(3);

	return y_data;
}

// Get variable lengths
void get_variable_lengths()
{

	d_T_s=0.1; //yaml_control_frequency;
	ROS_INFO_STREAM("[RAMPC CONTROLLER] delta T: "<<d_T_s);
	// Number of inputs m
	//d_num_inputs = u_data.rows();
	d_num_inputs=1;
	// Number of outputs p
	//d_num_outputs = y_data.rows();
	d_num_outputs=2;
	// Number of block rows for Hankel matrices
	//d_num_block_rows = d_Tini + d_N + 1;
	// Previous inputs vector (uini) length
	d_Nuini = d_num_inputs * d_Tini;
	// Previous outputs vector (yini) length
	d_Nyini = d_num_outputs * d_Tini;

	d_uini = MatrixXf::Zero(d_Nuini, 1);
    d_yini = MatrixXf::Zero(d_Nyini, 1);

    d_N_theta=1;
	// Trajectory mapper g vector size
	//d_Ng = u_data.cols() - d_num_block_rows + 1;
	// Slack variable length
	//d_Ns = d_Nyini;
	// Future inputs vector (uf) length
	d_Nuf = d_num_inputs * d_N;
	// Future output vector (yf) length 
	d_Nyf = d_num_outputs * (d_N+1);
	//Number of tube dilation coefficients
	d_Nsf= d_N+1;
	// Optimization decision vector size
	// Optimization variables in all formulations: [g; slack]
	d_num_opt_vars = 2*d_Nyf+d_Nuf+d_Nsf;

	// NUMBER OF DECISION VARIABLES
		d_col_num=d_num_outputs*(d_N+1)+d_num_outputs*(d_N+1)+d_num_inputs*d_N+(d_N+1);

		d_uf_start_i=d_Nyf*2;
	/*
	// In sparse formulation, add [uf; yf; yt], where yt is terminal output
	if (d_opt_sparse)
	{
		d_num_opt_vars += d_Nuf + d_Nyf + d_num_outputs;

		d_uf_start_i = d_Ng + d_Ns;
    	d_yf_start_i = d_uf_start_i + d_Nuf;

		// If optimizing over steady state values, add [gs; us]. This is available in sparse formulation only
		if (d_opt_steady_state)
		{
			d_num_opt_vars += d_Ng + d_num_inputs;

			d_gs_start_i = d_yf_start_i + d_Nyf + d_num_outputs;
			d_us_start_i = d_gs_start_i + d_Ng;
		}
	}
	*/
}

// Get Hankel matrices
void get_hankel_matrices(const MatrixXf& u_data, const MatrixXf& y_data)
{
	MatrixXf H_u = data2hankel(u_data, d_num_block_rows);
	MatrixXf H_y = data2hankel(y_data, d_num_block_rows);
	d_U_p = H_u.topRows(d_Nuini);
	d_U_f = H_u.middleRows(d_Nuini, d_Nuf);
	d_Y_p = H_y.topRows(d_Nyini);
	d_Y_f = H_y.bottomRows(d_Nyf + d_num_outputs);
}

// Get cost matrices
void get_cost_matrices()
{
	// Output cost and terminal output cost matrix
	d_Q = MatrixXf::Zero(d_num_outputs, d_num_outputs);
	d_P = MatrixXf::Zero(d_num_outputs, d_num_outputs);
	for (int i = 0; i < 2; i++)
	{
		d_Q(i,i) = d_Q_vec[i];
		//d_P(i,i) = d_P_vec[i];
	}
	d_P(0,0)=d_P_vec[0];
	d_P(0,1)=d_P_vec[1];
	d_P(1,0)=d_P_vec[1];
	d_P(1,1)=d_P_vec[2];
	/*
	if (d_Rampc_measure_roll_pitch)
		for (int i = 3; i < 5; i++)
		{
			d_Q(i,i) = d_Q_vec[i+3];
			d_P(i,i) = d_P_vec[i+3];
		}
	if (d_Rampc_yaw_control)
	{
		d_Q(d_num_outputs-1,d_num_outputs-1) = d_Q_vec[8];
		d_P(d_num_outputs-1,d_num_outputs-1) = d_P_vec[8];
	}
	*/
	// Input cost matrix
	d_R = MatrixXf::Zero(d_num_inputs, d_num_inputs);
	for (int i = 0; i < d_num_inputs; i++)
		d_R(i,i) = d_R_vec[i];
}

// Get input/output constraint vectors
void get_input_output_constr()
{
	d_F=MatrixXf::Zero(d_num_outputs*2+d_num_inputs*2,d_num_outputs);
	d_G=MatrixXf::Zero(d_num_outputs*2+d_num_inputs*2,d_num_inputs);
	d_z=MatrixXf::Zero(d_num_outputs*2+d_num_inputs,1);
	for(int i=0;i<d_num_outputs;i++){
		d_F(2*i,i)=1.0/d_F_vec[2*i];
		d_F(2*i+1,i)=1.0/d_F_vec[2*i+1];
	}

	for(int i=0;i<d_num_inputs;i++){
		d_G(2*i+2*d_num_outputs,i)=1.0/(d_G_vec[2*i]-m_cf_weight_in_newtons);
		d_G(2*i+1+2*d_num_outputs,i)=1.0/(d_G_vec[2*i+1]-m_cf_weight_in_newtons);
	}
	for(int i=0;i<d_num_outputs*2+d_num_inputs;i++){
		d_z(i,0)=1;
	}
	/*
	// Input constraints
	MatrixXf input_min = MatrixXf::Zero(d_num_inputs, 1);
	MatrixXf input_max = MatrixXf::Zero(d_num_inputs, 1);
	for (int i = 0; i < d_num_inputs; i++)
	{
		input_min(i) = d_input_min_vec[i];
		input_max(i) = d_input_max_vec[i];
	}
	d_input_min = input_min.replicate(d_N, 1);
	d_input_max = input_max.replicate(d_N, 1);

	// Output constraints
	MatrixXf output_min = MatrixXf::Zero(d_num_outputs, 1);
	MatrixXf output_max = MatrixXf::Zero(d_num_outputs, 1);
	for (int i = 0; i < 3; i++)
	{
		output_min(i) = d_output_min_vec[i];
		output_max(i) = d_output_max_vec[i];
	}
	if (d_Rampc_measure_roll_pitch)
		for (int i = 3; i < 5; i++)
		{
			output_min(i) = d_output_min_vec[i+3];
			output_max(i) = d_output_max_vec[i+3];
		}
	if (d_Rampc_yaw_control)
	{
		output_min(d_num_outputs-1) = d_output_min_vec[8];
		output_max(d_num_outputs-1) = d_output_max_vec[8];
	}
	d_output_min = output_min.replicate(d_N + 1, 1);
	d_output_max = output_max.replicate(d_N + 1, 1);
	*/
}

void get_control_feedback(){
	d_K = MatrixXf::Zero(d_num_inputs,d_num_outputs);
	for(int i=0;i<d_K_vec.size();i++){
		d_K(0,i)=d_K_vec[i];
	}
}

// Get optimization quadratic cost matrix
// Gurobi refers to this matrix as 'Q'
// OSQP refers to this matrix as 'P'
MatrixXf get_quad_cost_matrix()
{

	MatrixXf quad_cost_mat_xf = d_Q+d_K.transpose()*d_R*d_K;
	MatrixXf quad_cost_mat_uxf = d_K.transpose()*d_R;
	MatrixXf quad_cost_mat_xuf = d_R*d_K;
	MatrixXf quad_cost_mat_uf = d_R;





	/*
	MatrixXf quad_cost_mat_g = d_lambda2_g * MatrixXf::Identity(d_Ng, d_Ng);
	MatrixXf quad_cost_mat_s = d_lambda2_s * MatrixXf::Identity(d_Ns, d_Ns);
	MatrixXf quad_cost_mat_uf;
	MatrixXf quad_cost_mat_yf;
	MatrixXf quad_cost_mat_yt;
	*/
	MatrixXf quad_cost_mat = MatrixXf::Zero(d_col_num, d_col_num);


	for(int i=0;i<d_N;i++){
		quad_cost_mat.block(i*d_num_outputs,i*d_num_outputs,d_num_outputs,d_num_outputs)=quad_cost_mat_xf;
		quad_cost_mat.block(i*d_num_outputs,2*(d_N+1)*d_num_outputs+d_num_inputs*i,d_num_outputs,d_num_inputs)=quad_cost_mat_uxf;
		quad_cost_mat.block(2*(d_N+1)*d_num_outputs+i*d_num_inputs,i*d_num_outputs,d_num_inputs,d_num_outputs)=quad_cost_mat_xuf;
		quad_cost_mat.block(2*(d_N+1)*d_num_outputs+i*d_num_inputs,2*(d_N+1)*d_num_outputs+i*d_num_inputs,d_num_inputs,d_num_inputs)=quad_cost_mat_uf;
	}

	quad_cost_mat.block(d_N*d_num_outputs,d_N*d_num_outputs,d_num_outputs,d_num_outputs)=d_P;	

	quad_cost_mat=2*quad_cost_mat;
	/*
	if (d_opt_sparse)
	{
		quad_cost_mat_uf = MatrixXf::Zero(d_Nuf, d_Nuf);
		quad_cost_mat_yf = MatrixXf::Zero(d_Nyf, d_Nyf);
		quad_cost_mat_yt = d_P;
		for (int i = 0; i < d_N; i++)
		{
			quad_cost_mat_uf.block(i * d_num_inputs, i * d_num_inputs, d_num_inputs, d_num_inputs) = d_R;
			quad_cost_mat_yf.block(i * d_num_outputs, i * d_num_outputs, d_num_outputs, d_num_outputs) = d_Q;
		}
	}
	else
	{
		for (int i = 0; i < d_N; i++)
		{
			quad_cost_mat_g += d_U_f.middleRows(i * d_num_inputs, d_num_inputs).transpose() * d_R * d_U_f.middleRows(i * d_num_inputs, d_num_inputs);
			quad_cost_mat_g += d_Y_f.middleRows(i * d_num_outputs, d_num_outputs).transpose() * d_Q * d_Y_f.middleRows(i * d_num_outputs, d_num_outputs);
		}
		quad_cost_mat_g += d_Y_f.bottomRows(d_num_outputs).transpose() * d_P * d_Y_f.bottomRows(d_num_outputs);
	}
	quad_cost_mat.topLeftCorner(d_Ng, d_Ng) = quad_cost_mat_g;
	quad_cost_mat.block(d_Ng, d_Ng, d_Ns, d_Ns) = quad_cost_mat_s;
	if (d_opt_sparse)
	{
		quad_cost_mat.block(d_Ng + d_Ns, d_Ng + d_Ns, d_Nuf, d_Nuf) = quad_cost_mat_uf;
		quad_cost_mat.block(d_Ng + d_Ns + d_Nuf, d_Ng + d_Ns + d_Nuf, d_Nyf, d_Nyf) = quad_cost_mat_yf;
		quad_cost_mat.block(d_Ng + d_Ns + d_Nuf + d_Nyf, d_Ng + d_Ns + d_Nuf + d_Nyf, d_num_outputs, d_num_outputs) = quad_cost_mat_yt;

		if (d_opt_steady_state)
		{
			MatrixXf quad_cost_mat_gs = quad_cost_mat_g + MatrixXf::Identity(d_Ng, d_Ng);
			MatrixXf quad_cost_mat_g_gs = -quad_cost_mat_g;
			MatrixXf quad_cost_mat_us = d_N * d_R;
			MatrixXf quad_cost_mat_uf_us = -d_R.replicate(d_N, 1);

			quad_cost_mat.block(d_gs_start_i, d_gs_start_i, d_Ng, d_Ng) = quad_cost_mat_gs;
			quad_cost_mat.block(0, d_gs_start_i, d_Ng, d_Ng) = quad_cost_mat_g_gs;
			quad_cost_mat.block(d_gs_start_i, 0, d_Ng, d_Ng) = quad_cost_mat_g_gs.transpose();
			
			quad_cost_mat.bottomRightCorner(d_num_inputs, d_num_inputs) = quad_cost_mat_us;
			quad_cost_mat.block(d_uf_start_i, d_us_start_i, d_Nuf, d_num_inputs) = quad_cost_mat_uf_us;
			quad_cost_mat.block(d_us_start_i, d_uf_start_i, d_num_inputs, d_Nuf) = quad_cost_mat_uf_us.transpose();
		}
	}
	*/
	return quad_cost_mat;
}

// Get optimization linear cost vectors
// Gurobi refers to this as 'c'. It is multiplied by 2 since Gurobi minimizes (x^T * Q * x + c^t * x)
// OSQP refers to this as 'q'. It is not multiplied by 2 since OSQP minimizes (1/2 * x^T * P * x + q^t * x)
void get_lin_cost_vectors()
{
	s_Rampc_mutex.lock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1225");
	d_setpoint = s_setpoint;
	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1225");

	// Reference
	d_r = MatrixXf::Zero(d_num_outputs, 1);
	d_r.topRows(3) = d_setpoint.topRows(3);
	if (d_Rampc_yaw_control)
		d_r.bottomRows(1) = d_setpoint.bottomRows(1);

	if (d_opt_sparse)
	{
		d_lin_cost_vec_r = MatrixXf::Zero(d_Nyf + d_num_outputs, 1);
		d_lin_cost_vec_r.topRows(d_Nyf) = (-d_Q * d_r).replicate(d_N, 1);
		d_lin_cost_vec_r.bottomRows(d_num_outputs) = -d_P * d_r;
	}
	else
	{
		d_lin_cost_vec_r = MatrixXf::Zero(d_Ng, 1);

		for (int i = 0; i < d_N; i++)
			d_lin_cost_vec_r -= d_Y_f.middleRows(i * d_num_outputs, d_num_outputs).transpose() * d_Q * d_r;

		d_lin_cost_vec_r -= d_Y_f.bottomRows(d_num_outputs).transpose() * d_P * d_r;
	}
	if (d_solver == RAMPC_CONTROLLER_SOLVER_GUROBI)
		d_lin_cost_vec_r *= 2.0;

	d_r_gs = d_r.replicate(d_Tini + d_N + 1, 1);

	// Steady state input us and trajectory mapper gs
	// Only execute this if not optimizing over gs & us
	if (d_opt_sparse && d_opt_steady_state)
	{
		d_lin_cost_vec_gs = MatrixXf::Zero(d_Ng, 1);
		d_lin_cost_vec_us = MatrixXf::Zero(d_Nuf, 1);
	}
	else
	{
		// Steady state input
		MatrixXf us = MatrixXf::Zero(d_num_inputs, 1);
		us(0) = d_cf_weight_in_newtons;

		// Steady state trajectory mapper
		MatrixXf u_gs = us.replicate(d_Tini + d_N, 1);
		d_A_gs = MatrixXf::Zero(d_U_p.rows() + d_U_f.rows() + d_Y_p.rows() + d_Y_f.rows(), d_Ng);
		d_b_gs = MatrixXf::Zero(d_A_gs.rows(), 1);
		d_A_gs.topRows(d_U_p.rows()) = d_U_p;
		d_A_gs.middleRows(d_U_p.rows(), d_U_f.rows()) = d_U_f;
		d_A_gs.middleRows(d_U_p.rows() + d_U_f.rows(), d_Y_p.rows()) = d_Y_p;
		d_A_gs.bottomRows(d_Y_f.rows()) = d_Y_f;
		d_b_gs.topRows(u_gs.rows()) = u_gs;
		d_b_gs.bottomRows(d_r_gs.rows()) = d_r_gs;
		d_gs = d_A_gs.bdcSvd(ComputeThinU | ComputeThinV).solve(d_b_gs);

		d_lin_cost_vec_gs = -d_lambda2_g * d_gs;
		if (d_opt_sparse)
			d_lin_cost_vec_us = (-d_R * us).replicate(d_N, 1);
		else
		{
			d_lin_cost_vec_us = MatrixXf::Zero(d_Ng, 1);

			for (int i = 0; i < d_N; i++)
				d_lin_cost_vec_us -= d_U_f.middleRows(i * d_num_inputs, d_num_inputs).transpose() * d_R * us;
		}
	}

	if (d_solver == RAMPC_CONTROLLER_SOLVER_GUROBI)
	{
		d_lin_cost_vec_gs *= 2.0;
		d_lin_cost_vec_us *= 2.0;
	}
}

// Update optimization linear cost vectors
// Used during runtime to update objective on reference r and steady-state trajectory mapper gs
// us is hovering steady state input and is constant so does not get updated
// Gurobi refers to this as 'c'. It is multiplied by 2 since Gurobi minimizes (x^T * Q * x + c^t * x)
// OSQP refers to this as 'q'. It is not multiplied by 2 since OSQP minimizes (1/2 * x^T * P * x + q^t * x)
void update_lin_cost_vectors()
{
	s_Rampc_mutex.lock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1312");
	d_setpoint = s_setpoint;
	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1312");

    // Reference
	d_r.topRows(3) = d_setpoint.topRows(3);
	if (d_Rampc_yaw_control)
		d_r.bottomRows(1) = d_setpoint.bottomRows(1);

	if (d_opt_sparse)
	{
		d_lin_cost_vec_r.topRows(d_Nyf) = (-d_Q * d_r).replicate(d_N, 1);
		d_lin_cost_vec_r.bottomRows(d_num_outputs) = -d_P * d_r;
	}
	else
	{
		d_lin_cost_vec_r = MatrixXf::Zero(d_Ng, 1);

		for (int i = 0; i < d_N; i++)
			d_lin_cost_vec_r -= d_Y_f.middleRows(i * d_num_outputs, d_num_outputs).transpose() * d_Q * d_r;

		d_lin_cost_vec_r -= d_Y_f.bottomRows(d_num_outputs).transpose() * d_P * d_r;
	}
	if (d_solver == RAMPC_CONTROLLER_SOLVER_GUROBI)
		d_lin_cost_vec_r *= 2.0;

	d_r_gs = d_r.replicate(d_Tini + d_N + 1, 1);

	// If optimizing over steady state gs & us, no need to perform gs inversion logic
	if (d_opt_sparse && d_opt_steady_state)
		return;

	ds_Rampc_gs_inversion_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1327");
    d_gs_inversion_complete = ds_gs_inversion_complete;
    ds_Rampc_gs_inversion_mutex.unlock();
    //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1327");

	if (!d_get_gs)
	{
		// Steady state trajectory mapper
		d_b_gs.bottomRows(d_r_gs.rows()) = d_r_gs;

		// Get gs from gs matrix inversion thread
		d_get_gs = true;

		ds_Rampc_gs_inversion_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1320");
		ds_A_gs = d_A_gs;
		ds_b_gs = d_b_gs;
        ds_get_gs = d_get_gs;
        ds_Rampc_gs_inversion_mutex.unlock();
        //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1320");
	}

    if (d_gs_inversion_complete)
    {
    	d_get_gs = false;

    	ds_Rampc_gs_inversion_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1337");
	    d_gs = ds_gs;
	    ds_get_gs = d_get_gs;
	    ds_Rampc_gs_inversion_mutex.unlock();
	    //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1337");

	    d_lin_cost_vec_gs = -d_lambda2_g * d_gs;

		if (d_solver == RAMPC_CONTROLLER_SOLVER_GUROBI)
			d_lin_cost_vec_gs *= 2.0;
    }
}

// Update optimization linear cost vectors when reference is changing
// Used during runtime to update objective on reference r only steady-state trajectory mapper is not updated since it takes too long for inversion to do in real-time
// us is hovering steady state input and is constant so does not get updated
// Gurobi refers to this as 'c'. It is multiplied by 2 since Gurobi minimizes (x^T * Q * x + c^t * x)
// OSQP refers to this as 'q'. It is not multiplied by 2 since OSQP minimizes (1/2 * x^T * P * x + q^t * x)
void update_lin_cost_vectors_changing_ref()
{
    // Reference
	d_r.topRows(3) = d_setpoint.topRows(3);
	if (d_Rampc_yaw_control)
		d_r.bottomRows(1) = d_setpoint.bottomRows(1);

	d_figure_8_scale = 2 / (3 - cos(2 * d_figure_8_frequency_rad * (d_time_in_seconds - PI/2))) * d_figure_8_amplitude;
	d_r(0) += d_figure_8_scale * cos(d_figure_8_frequency_rad * (d_time_in_seconds - PI/2));
	d_r(1) += d_figure_8_scale * sin(2 * d_figure_8_frequency_rad * (d_time_in_seconds - PI/2)) / 2;
	d_r(2) += d_z_sine_amplitude * sin(d_z_sine_frequency_rad * d_time_in_seconds);

	d_time_in_seconds += d_control_deltaT;

	if (d_opt_sparse)
	{
		d_lin_cost_vec_r.topRows(d_Nyf) = (-d_Q * d_r).replicate(d_N, 1);
		d_lin_cost_vec_r.bottomRows(d_num_outputs) = -d_P * d_r;
	}
	else
	{
		d_lin_cost_vec_r = MatrixXf::Zero(d_Ng, 1);

		for (int i = 0; i < d_N; i++)
			d_lin_cost_vec_r -= d_Y_f.middleRows(i * d_num_outputs, d_num_outputs).transpose() * d_Q * d_r;

		d_lin_cost_vec_r -= d_Y_f.bottomRows(d_num_outputs).transpose() * d_P * d_r;
	}
	if (d_solver == RAMPC_CONTROLLER_SOLVER_GUROBI)
		d_lin_cost_vec_r *= 2.0;

	d_r_gs = d_r.replicate(d_Tini + d_N + 1, 1);
}

// Get static equality constraints matrix
// Static equality constraints don't change in runtime ([uf; yf; yt]), and ([gs; us]) if optimizing over steady state gs & us
// This is part of Gurobi/OSQP A matrix
MatrixXf get_static_eq_constr_matrix()
{
	MatrixXf A_eq_stat;
	if (d_opt_sparse)
	{
		d_num_stat_eq_constr = d_Nuf + d_Nyf + d_num_outputs;
		if (d_opt_steady_state)
			d_num_stat_eq_constr += d_Nuini + d_Nuf;

		A_eq_stat = MatrixXf::Zero(d_num_stat_eq_constr, d_num_opt_vars);
		A_eq_stat.topLeftCorner(d_Nuf, d_Ng) = d_U_f;
		A_eq_stat.block(0, d_uf_start_i, d_Nuf, d_Nuf) = -MatrixXf::Identity(d_Nuf, d_Nuf);
		A_eq_stat.block(d_Nuf, 0, d_Nyf + d_num_outputs, d_Ng) = d_Y_f;
		A_eq_stat.block(d_Nuf, d_yf_start_i, d_Nyf + d_num_outputs, d_Nyf + d_num_outputs) = -MatrixXf::Identity(d_Nyf + d_num_outputs, d_Nyf + d_num_outputs);

		if (d_opt_steady_state)
		{
			MatrixXf Up_Uf = MatrixXf::Zero(d_Nuini + d_Nuf, d_Ng);
			Up_Uf.topRows(d_Nuini) = d_U_p;
			Up_Uf.bottomRows(d_Nuf) = d_U_f;

			A_eq_stat.block(d_Nuf + d_Nyf + d_num_outputs, d_gs_start_i, d_Nuini + d_Nuf, d_Ng) = Up_Uf;
			A_eq_stat.bottomRightCorner(d_Nuini + d_Nuf, d_num_inputs) = -MatrixXf::Identity(d_num_inputs, d_num_inputs).replicate(d_Tini + d_N, 1);
		}
	}
	else
	{
		// There are no static equality constraints in dense formulation (they are subtituted)
		d_num_stat_eq_constr = 0;

		A_eq_stat = MatrixXf::Zero(0, 0);
	}

	return A_eq_stat;
}

// Get static equality constraints vector
// Static equality constraints don't change in runtime ([uf; yf; yt]), and ([gs; us]) if optimizing over steady state gs & us
// This is part of Gurobi b vector
// This is part of OSQP l/u vectors
MatrixXf get_static_eq_constr_vector()
{
	MatrixXf stat_eq_constr_vec;
	if (d_num_stat_eq_constr > 0)
		stat_eq_constr_vec = MatrixXf::Zero(d_num_stat_eq_constr, 1);
	else
	{
		// There are no static equality constraints in dense formulation (they are subtituted)
		stat_eq_constr_vec = MatrixXf::Zero(0, 0);
	}

	return stat_eq_constr_vec;
}

// Get dynamic equality constraints matrix
// Dynamic equality constraints change in runtime ([uini; yini]), and ([gs; r]) if optimizing over steady state gs & us
// This is part of Gurobi/OSQP A matrix
MatrixXf get_dynamic_eq_constr_matrix()
{
	d_num_dyn_eq_constr = d_Nuini + d_Nyini;
	if (d_opt_sparse && d_opt_steady_state)
		d_num_dyn_eq_constr += d_Nyini + d_Nyf + d_num_outputs;

	MatrixXf A_eq_dyn = MatrixXf::Zero(d_num_dyn_eq_constr, d_num_opt_vars);
	A_eq_dyn.topLeftCorner(d_Nuini, d_Ng) = d_U_p;
	A_eq_dyn.block(d_Nuini, 0, d_Nyini, d_Ng) = d_Y_p;
	A_eq_dyn.block(d_Nuini, d_Ng, d_Ns, d_Ns) = -MatrixXf::Identity(d_Ns, d_Ns);

	if (d_opt_sparse && d_opt_steady_state)
	{
		MatrixXf Yp_Yf = MatrixXf::Zero(d_Nyini + d_Nyf + d_num_outputs, d_Ng);
		Yp_Yf.topRows(d_Nyini) = d_Y_p;
		Yp_Yf.bottomRows(d_Nyf + d_num_outputs) = d_Y_f;

		A_eq_dyn.block(d_Nuini + d_Nyini, d_gs_start_i, d_Nyini + d_Nyf + d_num_outputs, d_Ng) = Yp_Yf;
	}

	return A_eq_dyn;
}

// Get dynamic equality constraints vector
// Dynamic equality constraints change in runtime ([uini; yini]), and ([gs; r]) if optimizing over steady state gs & us
// This is part of Gurobi b vector
// This is part of OSQP l/u vectors
MatrixXf get_dynamic_eq_constr_vector()
{
	MatrixXf dyn_eq_constr_vec = MatrixXf::Zero(d_num_dyn_eq_constr, 1);
	d_uini = MatrixXf::Zero(d_Nuini, 1);
    d_yini = MatrixXf::Zero(d_Nyini, 1);
	dyn_eq_constr_vec.topRows(d_Nuini) = d_uini;
	dyn_eq_constr_vec.middleRows(d_Nuini, d_Nyini) = d_yini;

	if (d_opt_sparse && d_opt_steady_state)
		dyn_eq_constr_vec.bottomRows(d_Nyini + d_Nuf + d_num_outputs) = d_r_gs;

	return dyn_eq_constr_vec;
}

// Some steps to finish Rampc setup
void finish_Rampc_setup()
{
	// Setup output variables
    //d_g = MatrixXf::Zero(d_Ng, 1);
    //if (d_opt_sparse)
    //{
    	d_u_f = MatrixXf::Zero(d_Nuf, 1);
    //	d_y_f = MatrixXf::Zero(d_Nyf + d_num_outputs, 1);
    //}

    // Setup successful flag
    d_setupRampc_success = true;

    s_Rampc_mutex.lock();
    // ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1285");
	s_num_inputs = d_num_inputs;
	s_num_outputs = d_num_outputs;
	s_Nuini = d_Nuini;
	s_Nyini = d_Nyini;
	s_uini = d_uini;
	s_yini = d_yini;
	s_setupRampc_success = d_setupRampc_success;
	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1294");
}

// Clear setup Rampc success flag
// This function was written because following code is re-curring
void clear_setupRampc_success_flag()
{
	d_setupRampc_success = false;
	s_Rampc_mutex.lock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 586");
	s_setupRampc_success = d_setupRampc_success;
	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1294");
}

// Gurobi cleanup
// Deletes any heap allocated variables
void gurobi_cleanup()
{
	delete[] d_grb_vars;
	delete[] d_grb_dyn_constrs;
}

// OSQP extended cleanup
// Frees any data structures to which memory was allocated
void osqp_extended_cleanup()
{
	osqp_cleanup(d_osqp_work);
	c_free(d_osqp_settings);
	c_free(d_osqp_q_new);
	c_free(d_osqp_l_new);
	c_free(d_osqp_u_new);
}

void osqp_theta_extended_cleanup()
{
	osqp_cleanup(d_osqp_theta_max_work);
	osqp_cleanup(d_osqp_theta_min_work);
	c_free(d_osqp_theta_settings);
	c_free(d_osqp_theta_l_new);
	c_free(d_osqp_theta_u_new);
}



void osqp_cleanup_data(OSQPData* data)
{
	if (!data)
	{
		return;
	}
	// Public members
	csc_spfree(data->P);
	csc_spfree(data->A);
	c_free(data->q);
	c_free(data->l);
	c_free(data->u);

	c_free(data);
}

// Data to Hankel function
MatrixXf data2hankel(const MatrixXf& data, int num_block_rows)
{
	// data =  Data matrix of size [dimension of data point x number of data points]
	// num_block_rows = number of block rows wanted in Hankel matrix

	int dim = data.rows();
	int num_data_pts = data.cols();
	int H_rows = dim * num_block_rows;
	int H_cols = num_data_pts - num_block_rows + 1;
	
	MatrixXf H = MatrixXf::Zero(H_rows, H_cols);

	for (int i = 0; i < num_block_rows; i++)
	    for (int j = 0; j < H_cols; j++)
	        H.block(dim*i,j,dim,1) = data.col(i+j);

    return H;
}

// Convert Eigen Dense matrix to CSC format used in OSQP
csc* eigen2csc(const MatrixXf& eigen_dense_mat)
{
	// First convert Eigen Dense matrix to Eigen Sparse
	SparseMatrix<float> eigen_sparse_mat = eigen_dense_mat.sparseView();

	// Second convert Eigen Sparse matrix to TRIPLET format, because it is not as mind bending as CSC format
	csc* trip_mat = csc_spalloc(eigen_sparse_mat.rows(), eigen_sparse_mat.cols(), eigen_sparse_mat.nonZeros(), 1, 1);
	trip_mat->nz = eigen_sparse_mat.nonZeros();
	// The following code was found under 'Iterating over the nonzero coefficients' section here: https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
	int i = 0;
	for (int j = 0; j < eigen_sparse_mat.outerSize(); j++)
		for (SparseMatrix<float>::InnerIterator it(eigen_sparse_mat, j); it; ++it)
			{
				trip_mat->x[i] = it.value();	// Value
				trip_mat->i[i] = it.row();		// Row index
				trip_mat->p[i] = it.col();		// Column index
				
				i++;
			}

	// Third convert TRIPLET matrix to CSC format, using OSQP built in function
	csc* csc_mat = triplet_to_csc(trip_mat, OSQP_NULL);

	// triplet_to_csc makes new copy of matrix, so intermediate TRIPLET matrix must be de-allocated
	csc_spfree(trip_mat);

	return csc_mat;
}


// ---------- READ/WRITE CSV FILES ----------

// Read csv file into matrix
MatrixXf read_csv(const string& path)
{
	ifstream fin;
	fin.open(path);
	if(!fin)
	{
		return MatrixXf::Zero(0, 0);
	}
	string line;
	vector<float> values;
	int rows = 0;
	try
	{
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

	catch(exception& e)
    {
	    ROS_INFO_STREAM("[RAMPC CONTROLLER] CSV read exception with standard error message: " << e.what());

		return MatrixXf::Zero(0, 0);
  	}
  	catch(...)
  	{
    	ROS_INFO("[RAMPC CONTROLLER] CSV read exception");

		return MatrixXf::Zero(0, 0);
  	}
}

// Write matrix into csv file
bool write_csv(const string& path, const MatrixXf& M)
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

// RAMPC GS MATRIX INVERSION THREAD MAIN
// Matrix inversion for steady state trajectory mapper gs takes long so it is performed in seperate thread
void Rampc_gs_inversion_thread_main()
{
	bool get_gs;
	bool gs_inversion_complete = false;
	MatrixXf gs;
	MatrixXf A_gs;
	MatrixXf b_gs;

	while (ros::ok())
	{
		ds_Rampc_gs_inversion_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1593");
        get_gs = ds_get_gs;
        ds_Rampc_gs_inversion_mutex.unlock();
        //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1593");
        
        if (!get_gs)
        	gs_inversion_complete = false;

        if (get_gs && !gs_inversion_complete)
        {
        	ds_Rampc_gs_inversion_mutex.lock();
			//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1593");
	        A_gs = ds_A_gs;
	        b_gs = ds_b_gs;
	        ds_Rampc_gs_inversion_mutex.unlock();
	        //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1593");

	        // This line is why this thread was created. It takes looong
        	gs = A_gs.bdcSvd(ComputeThinU | ComputeThinV).solve(b_gs);
        	
        	gs_inversion_complete = true;

        	ds_Rampc_gs_inversion_mutex.lock();
        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1609");
        	ds_gs = gs;
        	ds_gs_inversion_complete = gs_inversion_complete;
        	ds_Rampc_gs_inversion_mutex.unlock();
        	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1609");

        	ROS_INFO("[RAMPC CONTROLLER] Rampc gs matrix inversion complete");
        }
    }
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
		case RAMPC_CONTROLLER_REQUEST_TAKEOFF:
		{
			// Inform the user
			ROS_INFO("[RAMPC CONTROLLER] Received request to take off. Switch to state: LQR");
			// Update the state accordingly
			m_current_state = RAMPC_CONTROLLER_STATE_LQR;
			m_current_state_changed = true;
			// Provide dummy response
			response.data = 0;
			break;
		}

		case RAMPC_CONTROLLER_REQUEST_LANDING:
		{
			// Inform the user
			ROS_INFO("[RAMPC CONTROLLER] Received request to perform landing manoeuvre. Switch to state: landing move down");
			// Update the state accordingly
			m_current_state = RAMPC_CONTROLLER_STATE_LANDING_MOVE_DOWN;
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
			ROS_INFO("[RAMPC CONTROLLER] The requested manoeuvre is not recognised. Hence switching to standby state.");
			// Update the state to standby
			m_current_state = RAMPC_CONTROLLER_STATE_STANDBY;
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

// This function is the callback that is linked to the "RampcController"
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
		case RAMPC_CONTROLLER_STATE_LQR:
			computeResponse_for_LQR(request, response);
			break;

        case RAMPC_CONTROLLER_STATE_EXCITATION_LQR:
            computeResponse_for_excitation_LQR(request, response);
            break;

        case RAMPC_CONTROLLER_STATE_RAMPC:
            computeResponse_for_Rampc(request, response);
            break;

        case RAMPC_CONTROLLER_STATE_EXCITATION_RAMPC:
            computeResponse_for_excitation_Rampc(request, response);
            break;

		case RAMPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
			computeResponse_for_landing_move_down(request, response);
			break;

		case RAMPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
			computeResponse_for_landing_spin_motors(request, response);
			break;

		case RAMPC_CONTROLLER_STATE_STANDBY:
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
		ROS_INFO_STREAM("[RAMPC CONTROLLER] State \"standby\" started");
	}

	// Create dummy control output variable
	control_output output = {0.0, 0.0, 0.0, 0.0};

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

	// Update uini yini
	update_uini_yini(request, output);
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
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing input data to: " << m_outputFolder << "m_u_data.csv");
            if (write_csv(m_outputFolder + "m_u_data.csv", m_u_data.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing output data to: " << m_outputFolder << "m_y_data.csv");
            if (write_csv(m_outputFolder + "m_y_data.csv", m_y_data.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");
            
            // Make copies of Hankel matrix data if collecting data
            if (m_collect_data)
            {
            	m_num_hankels++;

            	ROS_INFO_STREAM("[RAMPC CONTROLLER] Making copy of input data to: " << m_outputFolder << "m_u_data_" << m_num_hankels << ".csv");
	            if (write_csv(m_outputFolder + "m_u_data_" + to_string(m_num_hankels) + ".csv", m_u_data.transpose()))
	            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
	            else
	            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

	            ROS_INFO_STREAM("[RAMPC CONTROLLER] Making copy of output data to: " << m_outputFolder << "m_y_data_" << m_num_hankels << ".csv");
	            if (write_csv(m_outputFolder + "m_y_data_" + to_string(m_num_hankels) + ".csv", m_y_data.transpose()))
	            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
	            else
	            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");
            }

            m_write_data = false;
		}

		// Publish the change
		publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[RAMPC CONTROLLER] State \"LQR\" started");
	}

	m_setpoint_for_controller[0] = m_setpoint[0];
	m_setpoint_for_controller[1] = m_setpoint[1];
	m_setpoint_for_controller[2] = m_setpoint[2];
	m_setpoint_for_controller[3] = m_setpoint[3];
	
	// Add 'Figure 8' (found here: "https://gamedev.stackexchange.com/questions/43691/how-can-i-move-an-object-in-an-infinity-or-figure-8-trajectory", as "Lemniscate of Bernoulli")
	/*
	if (m_changing_ref_enable)
	{
		float figure_8_scale = 2 / (3 - cos(2 * m_figure_8_frequency_rad * (m_time_in_seconds - PI/2))) * yaml_figure_8_amplitude;
		m_setpoint_for_controller[0] += figure_8_scale * cos(m_figure_8_frequency_rad * (m_time_in_seconds - PI/2));
		m_setpoint_for_controller[1] += figure_8_scale * sin(2 * m_figure_8_frequency_rad * (m_time_in_seconds - PI/2)) / 2;
		m_setpoint_for_controller[2] += yaml_z_sine_amplitude * sin(m_z_sine_frequency_rad * m_time_in_seconds);

		m_time_in_seconds += m_control_deltaT;
	}
	*/
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
	float thrust_request_per_motor = output.thrust/ 4.0;
	response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_request_per_motor);
	response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_request_per_motor);
	// Capture data
	if (m_collect_data)
	{

		ROS_INFO_STREAM("[RAMPC CONTROLLER] Collect data.");
		if (m_dataIndex_lqr < m_u_data_lqr.rows())
	    {
	    	// Input data
	    	m_u_data_lqr(m_dataIndex_lqr,0) = output.thrust;
	    	m_u_data_lqr(m_dataIndex_lqr,1) = output.rollRate;
	    	m_u_data_lqr(m_dataIndex_lqr,2) = output.pitchRate;
	    	m_u_data_lqr(m_dataIndex_lqr,3) = output.yawRate;

	    	// Output data
	    	m_y_data_lqr(m_dataIndex_lqr,0) = request.ownCrazyflie.x;
	    	m_y_data_lqr(m_dataIndex_lqr,1) = request.ownCrazyflie.y;
	    	m_y_data_lqr(m_dataIndex_lqr,2) = request.ownCrazyflie.z;
	    	m_y_data_lqr(m_dataIndex_lqr,3) = request.ownCrazyflie.roll;
	    	m_y_data_lqr(m_dataIndex_lqr,4) = request.ownCrazyflie.pitch;
	    	m_y_data_lqr(m_dataIndex_lqr,5) = request.ownCrazyflie.yaw;


	    	// Reference data
	    	m_r_data_lqr(m_dataIndex_lqr,0) = m_setpoint_for_controller[0];
	    	m_r_data_lqr(m_dataIndex_lqr,1) = m_setpoint_for_controller[1];
	    	m_r_data_lqr(m_dataIndex_lqr,2) = m_setpoint_for_controller[2];
	    	m_r_data_lqr(m_dataIndex_lqr,3) = m_setpoint_for_controller[3];

	    	m_dataIndex_lqr++;
	    }
	    else
	    {
	    	// Inform the user
	    	ROS_INFO("[RAMPC CONTROLLER] LQR data collection timeout expired.");

	    	ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing input data to: " << m_outputFolder << "m_u_data_lqr.csv");
            if (write_csv(m_outputFolder + "m_u_data_lqr.csv", m_u_data_lqr.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing output data to: " << m_outputFolder << "m_y_data_lqr.csv");
            if (write_csv(m_outputFolder + "m_y_data_lqr.csv", m_y_data_lqr.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing reference data to: " << m_outputFolder << "m_r_data_lqr.csv");
            if (write_csv(m_outputFolder + "m_r_data_lqr.csv", m_r_data_lqr.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            m_collect_data = false;
	    }
	}

	// DEBUG INFO
	if (yaml_shouldDisplayDebugInfo)
	{	
		ROS_INFO_STREAM("controlOutput.roll = " << response.controlOutput.roll);
		ROS_INFO_STREAM("controlOutput.pitch = " << response.controlOutput.pitch);
		ROS_INFO_STREAM("controlOutput.yaw = " << response.controlOutput.yaw);
		
		ROS_INFO_STREAM("controlOutput.motorCmd1 = " << response.controlOutput.motorCmd1);
		ROS_INFO_STREAM("controlOutput.motorCmd2 = " << response.controlOutput.motorCmd2);
		ROS_INFO_STREAM("controlOutput.motorCmd3 = " << response.controlOutput.motorCmd3);
		ROS_INFO_STREAM("controlOutput.motorCmd4 = " << response.controlOutput.motorCmd4);
	}

	// Update uini yini
	update_uini_yini(request, output);
}

void computeResponse_for_excitation_LQR(Controller::Request &request, Controller::Response &response)
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
        ROS_INFO_STREAM("[RAMPC CONTROLLER] State \"Excitation LQR\" started");
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
                ROS_INFO("[RAMPC CONTROLLER] Thrust excitation signal ended. State stays at: Excitation LQR");
                m_thrustExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Thrust excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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
                ROS_INFO("[RAMPC CONTROLLER] Roll rate excitation signal ended. State stays at: Excitation LQR");
                m_rollRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Roll rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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
                ROS_INFO("[RAMPC CONTROLLER] Pitch rate excitation signal ended. State stays at: Excitation LQR");
                m_pitchRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Pitch rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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
                ROS_INFO("[RAMPC CONTROLLER] Yaw rate excitation signal ended. State stays at: Excitation LQR");
                m_yawRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Yaw rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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

    // Update uini yini
	update_uini_yini(request, output);
}

void computeResponse_for_Rampc(Controller::Request &request, Controller::Response &response)
{
	bool Rampc_first_pass = false;

	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		for (int i = 0; i < 9; i++)
			m_previous_stateErrorInertial[i] = 0.0;
		if (!m_write_data)
			Rampc_first_pass = true;
		m_Rampc_solving_first_opt = false;
		m_Rampc_cycles_since_solve = 0;
		// Set the change flag back to false
		m_current_state_changed = false;

		// If just coming from excitation state, write data collected
		if (m_write_data)
		{
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing input data to: " << m_outputFolder << "m_u_data.csv");
            if (write_csv(m_outputFolder + "m_u_data.csv", m_u_data.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing output data to: " << m_outputFolder << "m_y_data.csv");
            if (write_csv(m_outputFolder + "m_y_data.csv", m_y_data.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            // Make copies of Hankel matrix data if collecting data
            if (m_collect_data)
            {
            	m_num_hankels++;

            	ROS_INFO_STREAM("[RAMPC CONTROLLER] Making copy of input data to: " << m_outputFolder << "m_u_data_" << m_num_hankels << ".csv");
	            if (write_csv(m_outputFolder + "m_u_data_" + to_string(m_num_hankels) + ".csv", m_u_data.transpose()))
	            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
	            else
	            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

	            ROS_INFO_STREAM("[RAMPC CONTROLLER] Making copy of output data to: " << m_outputFolder << "m_y_data_" << m_num_hankels << ".csv");
	            if (write_csv(m_outputFolder + "m_y_data_" + to_string(m_num_hankels) + ".csv", m_y_data.transpose()))
	            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
	            else
	            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");
            }

            m_write_data = false;
		}

		// Publish the change
		publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[RAMPC CONTROLLER] State \"Rampc\" started");
	}

	// Check if Rampc is not setup and exit Rampc control mode
	// Rampc control is not allowed to start unless setup, but on exceptions setup success flag is reset
	// Rampc must be (re-)setup in this case to allow restart
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1460");
	bool setupRampc_success = s_setupRampc_success;
	bool solveRampc = s_solveRampc;
	m_u_f = s_u_f;
	m_y_f = s_y_f;
	m_solve_time = s_solve_time;
	m_Rampc_active_setpoint = s_Rampc_active_setpoint;
	s_Rampc_mutex.unlock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1460");

	bool use_LQR = false;
	control_output output;

	if (!setupRampc_success)
	{
		// Inform the user
        ROS_INFO("[RAMPC CONTROLLER] Rampc control error. Rampc must be (re-)setup. Switch to state: LQR");
        // Update the state accordingly
        m_current_state = RAMPC_CONTROLLER_STATE_LQR;
        m_current_state_changed = true;
        use_LQR = true;
	}

	if (m_Rampc_solving_first_opt && !solveRampc)
		m_Rampc_solving_first_opt = false;

	if (Rampc_first_pass || m_Rampc_solving_first_opt)
		use_LQR = true;

	if (solveRampc)
	{
		m_Rampc_cycles_since_solve++;
		if (m_Rampc_cycles_since_solve >= yaml_N)
			use_LQR = true;
	}
	else if (!Rampc_first_pass)
	{
		ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc solving optimization took " << m_Rampc_cycles_since_solve + 1 << " cycles");
		m_Rampc_cycles_since_solve = 0;
	}

	// Set reference for LQR controller in case it is used
	m_setpoint_for_controller[0] = m_setpoint[0];
	m_setpoint_for_controller[1] = m_setpoint[1];
	m_setpoint_for_controller[2] = m_setpoint[2];
	m_setpoint_for_controller[3] = m_setpoint[3];
	
	// Add 'Figure 8' (found here: "https://gamedev.stackexchange.com/questions/43691/how-can-i-move-an-object-in-an-infinity-or-figure-8-trajectory", as "Lemniscate of Bernoulli")
	// Note that this computation is repeated in the Rampc thread to update reference in optimization problem
	if (m_changing_ref_enable)
	{
		float figure_8_scale = 2 / (3 - cos(2 * m_figure_8_frequency_rad * (m_time_in_seconds - PI/2))) * yaml_figure_8_amplitude;
		m_setpoint_for_controller[0] += figure_8_scale * cos(m_figure_8_frequency_rad * (m_time_in_seconds - PI/2));
		m_setpoint_for_controller[1] += figure_8_scale * sin(2 * m_figure_8_frequency_rad * (m_time_in_seconds - PI/2)) / 2;
		m_setpoint_for_controller[2] += yaml_z_sine_amplitude * sin(m_z_sine_frequency_rad * m_time_in_seconds);

		m_time_in_seconds += m_control_deltaT;
	}

	if (use_LQR)
	{	 
		// Call the LQR control function
		calculateControlOutput_viaLQR(request, output);
	}
	else
	{
		calculateControlOutput_viaLQR(request, output);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] LQR thrust output: "<<output.thrust);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] index number: "<<input_number * m_num_inputs);
		int input_number= m_Rampc_cycles_since_solve/20;
		ROS_INFO_STREAM("[RAMPC CONTROLLER] cycle: "<< m_Rampc_cycles_since_solve<< "; index number: "<<input_number * m_num_inputs);
		output.thrust = m_u_f(input_number * m_num_inputs);
		ROS_INFO_STREAM("[RAMPC CONTROLLER] Actual output: "<<output.thrust);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] RAMPC thrust output: "<<output.thrust);
		//output.rollRate = m_u_f(m_Rampc_cycles_since_solve * m_num_inputs + 1);
		//output.pitchRate = m_u_f(m_Rampc_cycles_since_solve * m_num_inputs + 2);
		//if (yaml_Rampc_yaw_control)
		//	output.yawRate = m_u_f(m_Rampc_cycles_since_solve * m_num_inputs + 3);
		//else
		//{
		//float yawError = request.ownCrazyflie.yaw - m_setpoint[3];
		//	while(yawError > PI) {yawError -= 2 * PI;}
		//	while(yawError < -PI) {yawError += 2 * PI;}
		//	output.yawRate = -yaml_gainMatrixYawRate[8] * yawError;
		//}
		
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
	if (m_collect_data && !use_LQR)
	{
		if (m_dataIndex_Rampc < m_u_data_Rampc.rows())
	    {
	    	// Input data
	    	m_u_data_Rampc(m_dataIndex_Rampc,0) = output.thrust;
	    	m_u_data_Rampc(m_dataIndex_Rampc,1) = output.rollRate;
	    	m_u_data_Rampc(m_dataIndex_Rampc,2) = output.pitchRate;
	    	m_u_data_Rampc(m_dataIndex_Rampc,3) = output.yawRate;

	    	// Predicted input data
	    	m_uf_data_Rampc.row(m_dataIndex_Rampc) = m_u_f.transpose();

	    	// Output data
	    	m_y_data_Rampc(m_dataIndex_Rampc,0) = request.ownCrazyflie.x;
	    	m_y_data_Rampc(m_dataIndex_Rampc,1) = request.ownCrazyflie.y;
	    	m_y_data_Rampc(m_dataIndex_Rampc,2) = request.ownCrazyflie.z;
	    	m_y_data_Rampc(m_dataIndex_Rampc,3) = request.ownCrazyflie.roll;
	    	m_y_data_Rampc(m_dataIndex_Rampc,4) = request.ownCrazyflie.pitch;
	    	m_y_data_Rampc(m_dataIndex_Rampc,5) = request.ownCrazyflie.yaw;

	    	// Predicted output data
	    	m_yf_data_Rampc.row(m_dataIndex_Rampc) = m_y_f.transpose();

	    	// Reference data
	    	m_r_data_Rampc.row(m_dataIndex_Rampc) = m_Rampc_active_setpoint.transpose();

	    	// Solve time data
	    	m_solveTime_data_Rampc(m_dataIndex_Rampc) = m_solve_time;

	    	m_dataIndex_Rampc++;
	    }
	    else
	    {
	    	// Inform the user
	    	ROS_INFO("[RAMPC CONTROLLER] Rampc data collection timeout expired.");

	    	ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing input data to: " << m_outputFolder << "m_u_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_u_data_Rampc.csv", m_u_data_Rampc.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing predicted input data to: " << m_outputFolder << "m_uf_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_uf_data_Rampc.csv", m_uf_data_Rampc.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing output data to: " << m_outputFolder << "m_y_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_y_data_Rampc.csv", m_y_data_Rampc.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing predicted output data to: " << m_outputFolder << "m_yf_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_yf_data_Rampc.csv", m_yf_data_Rampc.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing reference data to: " << m_outputFolder << "m_r_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_r_data_Rampc.csv", m_r_data_Rampc.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing solve time data to: " << m_outputFolder << "m_solveTime_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_solveTime_data_Rampc.csv", m_solveTime_data_Rampc.transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            m_collect_data = false;
	    }
	}

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
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Update uini.");
	// Update uini yini BEFORE CALLING OPTIMIZATION
	update_uini_yini(request, output);
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Update uini done.");
	if (!solveRampc)
	{
		// Set flag to solve Rampc optimization
		s_Rampc_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1520");
		s_solveRampc = true;
		s_Rampc_mutex.unlock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1520");
	}

	if (Rampc_first_pass)
			m_Rampc_solving_first_opt = true;
}

void computeResponse_for_excitation_Rampc(Controller::Request &request, Controller::Response &response)
{
    // Check if the state "just recently" changed
    if (m_current_state_changed)
    {
        // PERFORM "ONE-OFF" OPERATIONS HERE
        for (int i = 0; i < 9; i++)
			m_previous_stateErrorInertial[i] = 0.0;
		m_time_in_seconds = 0.0;
        m_thrustExcIndex = 0;
        m_rollRateExcIndex = 0;
        m_pitchRateExcIndex = 0;
        m_yawRateExcIndex = 0;
        m_Rampc_cycles_since_solve = 0;
        m_dataIndex = 0;
        // Set the change flag back to false
        m_current_state_changed = false;
        // Publish the change
        publishCurrentSetpointAndState();
        // Inform the user
        ROS_INFO_STREAM("[RAMPC CONTROLLER] State \"Excitation Rampc\" started");
    }

    // Check if Rampc is not setup and exit Rampc control mode
	// Rampc control is not allowed to start unless setup, but on exceptions setup success flag is reset
	// Rampc must be (re-)setup in this case to allow restart
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1460");
	bool setupRampc_success = s_setupRampc_success;
	bool solveRampc = s_solveRampc;
	m_u_f = s_u_f;
	s_Rampc_mutex.unlock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1460");

	bool use_LQR = false;
	control_output output;

	if (!setupRampc_success)
	{
		// Inform the user
        ROS_INFO("[RAMPC CONTROLLER] Rampc control error. Rampc must be (re-)setup. Switch to state: LQR");
        // Update the state accordingly
        m_current_state = RAMPC_CONTROLLER_STATE_LQR;
        m_current_state_changed = true;
        use_LQR = true;
	}

	if (solveRampc)
		m_Rampc_cycles_since_solve++;
	else
	{
		ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc solving optimization took " << m_Rampc_cycles_since_solve + 1 << " cycles");
		m_Rampc_cycles_since_solve = 0;
	}

	if (use_LQR)
	{
		m_setpoint_for_controller[0] = m_setpoint[0];
		m_setpoint_for_controller[1] = m_setpoint[1];
		m_setpoint_for_controller[2] = m_setpoint[2];
		m_setpoint_for_controller[3] = m_setpoint[3];
		
		// Call the LQR control function
		calculateControlOutput_viaLQR(request, output);
	}
	else
	{
		output.thrust = m_u_f(m_Rampc_cycles_since_solve * m_num_inputs);
		output.rollRate = m_u_f(m_Rampc_cycles_since_solve * m_num_inputs + 1);
		output.pitchRate = m_u_f(m_Rampc_cycles_since_solve * m_num_inputs + 2);
		if (yaml_Rampc_yaw_control)
			output.yawRate = m_u_f(m_Rampc_cycles_since_solve * m_num_inputs + 3);
		else
		{
			float yawError = request.ownCrazyflie.yaw - m_setpoint[3];
			while(yawError > PI) {yawError -= 2 * PI;}
			while(yawError < -PI) {yawError += 2 * PI;}
			output.yawRate = -yaml_gainMatrixYawRate[8] * yawError;
		}
		
	}

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
                ROS_INFO("[RAMPC CONTROLLER] Thrust excitation signal ended. State stays at: Excitation Rampc");
                m_thrustExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Thrust excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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
                ROS_INFO("[RAMPC CONTROLLER] Roll rate excitation signal ended. State stays at: Excitation Rampc");
                m_rollRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Roll rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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
                ROS_INFO("[RAMPC CONTROLLER] Pitch rate excitation signal ended. State stays at: Excitation Rampc");
                m_pitchRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Pitch rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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
                ROS_INFO("[RAMPC CONTROLLER] Yaw rate excitation signal ended. State stays at: Excitation Rampc");
                m_yawRateExcEnable = false;
            }
            else
            {
                // Inform the user
                ROS_INFO("[RAMPC CONTROLLER] Yaw rate excitation signal ended. Switch to state: LQR");
                // Update the state accordingly
                m_current_state = RAMPC_CONTROLLER_STATE_LQR;
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
    if (true)
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

    // Update uini yini BEFORE CALLING OPTIMIZATION
	update_uini_yini(request, output);

	if (!solveRampc)
	{
		// Set flag to solve Rampc optimization
		s_Rampc_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1520");
		s_solveRampc = true;
		s_Rampc_mutex.unlock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1520");
	}
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
		ROS_INFO_STREAM("[RAMPC CONTROLLER] State \"landing move-down\" started with \"m_setpoint_for_controller\" (x,y,z,yaw) =  ( " << m_setpoint_for_controller[0] << ", " << m_setpoint_for_controller[1] << ", " << m_setpoint_for_controller[2] << ", " << m_setpoint_for_controller[3] << ")");
	}

	// Check if within the threshold of zero
	if (request.ownCrazyflie.z < yaml_landing_move_down_end_height_threshold)
	{
		// Inform the user
		ROS_INFO("[RAMPC CONTROLLER] Switch to state: landing spin motors");
		// Update the state accordingly
		m_current_state = RAMPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS;
		m_current_state_changed = true;
	}

	// Change to landing spin motors if the timeout is reached
	if (m_time_in_seconds > yaml_landing_move_down_time_max)
	{
		// Inform the user
		ROS_INFO("[DEFAULT CONTROLLER] Did not reach the setpoint within the \"landing move down\" allowed time. Switch to state: landing spin motors");
		// Update the state accordingly
		m_current_state = RAMPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS;
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

	// Update uini yini
	update_uini_yini(request, output);
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
		ROS_INFO_STREAM("[RAMPC CONTROLLER] state \"landing spin motors\" started");
	}

	// Change to next state after specified time
	if (m_time_in_seconds > 0.7 * yaml_landing_spin_motors_time)
	{
		// Inform the user
		ROS_INFO("[RAMPC CONTROLLER] Publish message that landing is complete, and switch to state: standby");
		// Update the state accordingly
		m_current_state = RAMPC_CONTROLLER_STATE_STANDBY;
		m_current_state_changed = true;
		// Publish a message that the landing is complete
		IntWithHeader msg;
		msg.data = RAMPC_CONTROLLER_LANDING_COMPLETE;
		m_manoeuvreCompletePublisher.publish(msg);
		// Publish the change
		publishCurrentSetpointAndState();
	}

	// Create dummy control output variable
	control_output output = {0.0, 0.0, 0.0, 0.0};

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

	// Update uini yini
	update_uini_yini(request, output);
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
		stateErrorBody[i]=stateErrorInertial[i];
		m_previous_stateErrorInertial[i] = stateErrorInertial[i];
	}


	// PERFORM THE "u=-Kx" CONTROLLER COMPUTATIONS

	// Initialize control output
	output.thrust=0;
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
		
		/*
		ROS_INFO_STREAM("Debug_info: " <<yaml_shouldDisplayDebugInfo);
		ROS_INFO_STREAM("Control frequency: " <<yaml_control_frequency);
		ROS_INFO_STREAM("Setpoint x: "<< m_setpoint[0]);
		ROS_INFO_STREAM("Setpoint y: "<< m_setpoint[1]);
		ROS_INFO_STREAM("Setpoint z: "<< m_setpoint[2]);
		ROS_INFO_STREAM("Setpoint yaw: "<< m_setpoint[3]);
		*/
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

	// Tell Rampc thread that setpoint changed
	s_Rampc_mutex.lock();
	ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1927");
	for (int i = 0; i < 4; i++)
	{
		s_setpoint(i) = m_setpoint[i];
	}
	s_setpoint_changed = true;
	s_Rampc_mutex.unlock();
	 ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1927");

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
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Button 1 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 1
				processCustomButton1(float_data, int_data, bool_data);

                break;

			// > FOR CUSTOM BUTTON 2 - SETUP GUROBI OPTIMIZATION
			case 2:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Button 2 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 2
                processCustomButton2(float_data, int_data, bool_data);

				break;

			// > FOR CUSTOM BUTTON 3 - RAMPC
			case 3:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Button 3 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 3
                processCustomButton3(float_data, int_data, bool_data);

				break;

			// > FOR CUSTOM BUTTON 4 - COLLECT DATA
			case 4:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Button 4 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 4
				processCustomButton4(float_data, int_data, bool_data);
                
				break;

			// > FOR CUSTOM BUTTON 5 - CHANGING REFERENCE
			case 5:
				// Let the user know that this part of the code was triggered
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Button 5 received in controller, with message.float_data = " << float_data );
				// Code here to respond to custom button 5
				processCustomButton5(float_data, int_data, bool_data);
                
                break;

			default:
				// Let the user know that the command was not recognised
				ROS_INFO_STREAM("[RAMPC CONTROLLER] A button clicked command was received in the controller but not recognised, message.button_index = " << custom_button_index << ", and message.float_data = " << float_data );
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
        case RAMPC_CONTROLLER_STATE_LQR:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to start excitation while in LQR. Switch to state: Excitation LQR");
            // Update the state accordingly
            m_current_state = RAMPC_CONTROLLER_STATE_EXCITATION_LQR;
            m_current_state_changed = true;
            if (!int_data)
            {
            	m_thrustExcEnable = true;
            	m_rollRateExcEnable = true;
            	m_pitchRateExcEnable = true;
            	m_yawRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting all");
            }
            if (bool_data[0])
            {
            	m_thrustExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting thrust");
            }
            if (bool_data[1])
            {
            	m_rollRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting roll rate");
            }
            if (bool_data[2])
            {
            	m_pitchRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting pitch rate");
            }
            if (bool_data[3])
            {
            	m_yawRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting yaw rate");
            }
            break;

        case RAMPC_CONTROLLER_STATE_EXCITATION_LQR:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to stop excitation while in LQR. Switch to state: LQR");
            // Update the state accordingly
            m_current_state = RAMPC_CONTROLLER_STATE_LQR;
            m_current_state_changed = true;
            m_write_data = true;
            break;

        case RAMPC_CONTROLLER_STATE_RAMPC:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to start excitation while in Rampc. Switch to state: Excitation Rampc");
            // Update the state accordingly
            m_current_state = RAMPC_CONTROLLER_STATE_EXCITATION_RAMPC;
            m_current_state_changed = true;
            if (!int_data)
            {
            	m_thrustExcEnable = true;
            	m_rollRateExcEnable = true;
            	m_pitchRateExcEnable = true;
            	m_yawRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting all");
            }
            if (bool_data[0])
            {
            	m_thrustExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting thrust");
            }
            if (bool_data[1])
            {
            	m_rollRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting roll rate");
            }
            if (bool_data[2])
            {
            	m_pitchRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting pitch rate");
            }
            if (bool_data[3])
            {
            	m_yawRateExcEnable = true;
            	// Inform the user
            	ROS_INFO("[RAMPC CONTROLLER] Exciting yaw rate");
            }
            break;

        case RAMPC_CONTROLLER_STATE_EXCITATION_RAMPC:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to stop excitation while in Rampc. Switch to state: Rampc");
            // Update the state accordingly
            m_current_state = RAMPC_CONTROLLER_STATE_RAMPC;
            m_current_state_changed = true;
            m_write_data = true;
            break;

        case RAMPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
        case RAMPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
        case RAMPC_CONTROLLER_STATE_STANDBY:
        default:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to start excitation in invalid state. Request ignored");
            break;
    }
}

// CUSTOM BUTTON 2 - SETUP RAMPC OPTIMIZATION
void processCustomButton2(float float_data, int int_data, bool* bool_data)
{
	// Inform the user
    ROS_INFO("[RAMPC CONTROLLER] Received request to setup Rampc optimization");

    s_Rampc_mutex.lock();
    // ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 2142");
    s_setupRampc = true;
    s_Rampc_mutex.unlock();
    // ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 2142");
}

// CUSTOM BUTTON 3 - RAMPC
void processCustomButton3(float float_data, int int_data, bool* bool_data)
{	
	s_Rampc_mutex.lock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 2152");
	bool setupRampc_success = s_setupRampc_success;
	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 2152");

    // Check if Rampc optimization was setup successfully
    if (!setupRampc_success)
    {
    	// Inform the user
        ROS_INFO("[RAMPC CONTROLLER] Received request to start Rampc but optimization is not setup successfully. Request ignored");
        return;
    }

    // Switch between the possible states
    switch (m_current_state)
    {
        case RAMPC_CONTROLLER_STATE_LQR:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to start Rampc. Switch to state: Rampc");
            // Update the state accordingly
            m_current_state = RAMPC_CONTROLLER_STATE_RAMPC;
            m_current_state_changed = true;
            break;

        case RAMPC_CONTROLLER_STATE_RAMPC:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to stop Rampc. Switch to state: LQR");
            // Update the state accordingly
            m_current_state = RAMPC_CONTROLLER_STATE_LQR;
            m_current_state_changed = true;
            break;

        case RAMPC_CONTROLLER_STATE_EXCITATION_LQR:
        case RAMPC_CONTROLLER_STATE_EXCITATION_RAMPC:
        case RAMPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
        case RAMPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
        case RAMPC_CONTROLLER_STATE_STANDBY:
        default:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to start Rampc in invalid state. Request ignored");
            break;
    }
}

// CUSTOM BUTTON 4 - COLLECT DATA
void processCustomButton4(float float_data, int int_data, bool* bool_data)
{	
	if (!m_collect_data)
	{
		// Inform the user
        ROS_INFO("[RAMPC CONTROLLER] Received request to start data collection");

		m_dataIndex_lqr = 0;
		m_dataIndex_Rampc = 0;
		m_num_hankels = 0;
		m_collect_data = true;
	}
	else
	{
		// Inform the user
        ROS_INFO("[RAMPC CONTROLLER] Received request to stop data collection");

		if (m_dataIndex_lqr > 0)
		{
			// Inform the user
	    	ROS_INFO("[RAMPC CONTROLLER] LQR data found");

	    	ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing input data to: " << m_outputFolder << "m_u_data_lqr.csv");
            if (write_csv(m_outputFolder + "m_u_data_lqr.csv", m_u_data_lqr.topRows(m_dataIndex_lqr).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing output data to: " << m_outputFolder << "m_y_data_lqr.csv");
            if (write_csv(m_outputFolder + "m_y_data_lqr.csv", m_y_data_lqr.topRows(m_dataIndex_lqr).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing reference data to: " << m_outputFolder << "m_r_data_lqr.csv");
            if (write_csv(m_outputFolder + "m_r_data_lqr.csv", m_r_data_lqr.topRows(m_dataIndex_lqr).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");
		}
		if (m_dataIndex_Rampc > 0)
		{
			// Inform the user
	    	ROS_INFO("[RAMPC CONTROLLER] Rampc data found");

	    	ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing input data to: " << m_outputFolder << "m_u_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_u_data_Rampc.csv", m_u_data_Rampc.topRows(m_dataIndex_Rampc).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing predicted input data to: " << m_outputFolder << "m_uf_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_uf_data_Rampc.csv", m_uf_data_Rampc.topRows(m_dataIndex_Rampc).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing output data to: " << m_outputFolder << "m_y_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_y_data_Rampc.csv", m_y_data_Rampc.topRows(m_dataIndex_Rampc).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing predicted output data to: " << m_outputFolder << "m_yf_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_yf_data_Rampc.csv", m_yf_data_Rampc.topRows(m_dataIndex_Rampc).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing reference data to: " << m_outputFolder << "m_r_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_r_data_Rampc.csv", m_r_data_Rampc.topRows(m_dataIndex_Rampc).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
            else
            	ROS_INFO("[RAMPC CONTROLLER] Write file failed");

            ROS_INFO_STREAM("[RAMPC CONTROLLER] Writing solve time data to: " << m_outputFolder << "m_solveTime_data_Rampc.csv");
            if (write_csv(m_outputFolder + "m_solveTime_data_Rampc.csv", m_solveTime_data_Rampc.topRows(m_dataIndex_Rampc).transpose()))
            	ROS_INFO("[RAMPC CONTROLLER] Write file successful");
		}

		m_collect_data = false;
	}
}

// CUSTOM BUTTON 5 - CHANGING REFERENCE
void processCustomButton5(float float_data, int int_data, bool* bool_data)
{	
	// If already following changing reference, disable it and return
	if (m_changing_ref_enable)
	{
		m_changing_ref_enable = false;

		s_Rampc_mutex.lock();
		// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 3876");
		s_changing_ref_enable = m_changing_ref_enable;
		s_Rampc_mutex.unlock();
		// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 3876");

		return;
	}

    // Switch between the possible states
    switch (m_current_state)
    {
        case RAMPC_CONTROLLER_STATE_LQR:
        case RAMPC_CONTROLLER_STATE_RAMPC:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to follow changing reference");
            // Reset time
            m_time_in_seconds = 0.0;
            // Set the flag
            m_changing_ref_enable = true;

            s_Rampc_mutex.lock();
			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 3896");
			s_changing_ref_enable = m_changing_ref_enable;
			s_Rampc_mutex.unlock();
			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 3896");
            break;

        case RAMPC_CONTROLLER_STATE_EXCITATION_LQR:
        case RAMPC_CONTROLLER_STATE_EXCITATION_RAMPC:
        case RAMPC_CONTROLLER_STATE_LANDING_MOVE_DOWN:
        case RAMPC_CONTROLLER_STATE_LANDING_SPIN_MOTORS:
        case RAMPC_CONTROLLER_STATE_STANDBY:
        default:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to follow changing reference in invalid state. Request ignored");
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
void isReadyRampcControllerYamlCallback(const IntWithHeader & msg)
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
				ROS_INFO("[RAMPC CONTROLLER] Now fetching the RampcController YAML parameter values from this agent.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
			// > FOR FETCHING FROM THE COORDINATOR'S PARAMETER SERVICE
			case LOAD_YAML_FROM_COORDINATOR:
			{
				ROS_INFO("[RAMPC CONTROLLER] Now fetching the RampcController YAML parameter values from this agent's coordinator.");
				namespace_to_use = m_namespace_to_coordinator_parameter_service;
				break;
			}

			default:
			{
				ROS_ERROR("[RAMPC CONTROLLER] Paramter service to load from was NOT recognised.");
				namespace_to_use = m_namespace_to_own_agent_parameter_service;
				break;
			}
		}
		// Create a node handle to the selected parameter service
		ros::NodeHandle nodeHandle_to_use(namespace_to_use);
		// Call the function that fetches the parameters
		fetchRampcControllerYamlParameters(nodeHandle_to_use);
	}
}


// LOADING OF THE YAML PARAMTERS
void fetchRampcControllerYamlParameters(ros::NodeHandle& nodeHandle)
{
	// Here we load the parameters that are specified in the file:
	// RampcController.yaml

	// Add the "RampcController" namespace to the "nodeHandle"
	ros::NodeHandle nodeHandle_for_paramaters(nodeHandle, "RampcController");

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

	// Data collection max time, in minutes
	yaml_data_collection_max_time = getParameterFloat(nodeHandle_for_paramaters, "data_collection_max_time");

	// Data folder locations
	yaml_dataFolder = getParameterString(nodeHandle_for_paramaters, "dataFolder");
	yaml_outputFolder = getParameterString(nodeHandle_for_paramaters, "outputFolder");
	yaml_logFolder = getParameterString(nodeHandle_for_paramaters, "logFolder");

	// Rampc flag to use roll and pitch angle measurements
	yaml_Rampc_measure_roll_pitch = getParameterBool(nodeHandle_for_paramaters, "Rampc_measure_roll_pitch");
	
	// Rampc flag to control yaw
	yaml_Rampc_yaw_control = getParameterBool(nodeHandle_for_paramaters, "Rampc_yaw_control");

	// Rampc prediction horizon
	yaml_N = getParameterInt(nodeHandle_for_paramaters, "N");

	// Changing reference parameters
	// Figure 8 amplitude, in m
	yaml_figure_8_amplitude = getParameterFloat(nodeHandle_for_paramaters, "figure_8_amplitude");
	
	// Figure 8 frequency, in Hz
	yaml_figure_8_frequency = getParameterFloat(nodeHandle_for_paramaters, "figure_8_frequency");
	
	// z sine amplitude, in m
	yaml_z_sine_amplitude = getParameterFloat(nodeHandle_for_paramaters, "z_sine_amplitude");
	
	// z sine frequency, in Hz
	yaml_z_sine_frequency = getParameterFloat(nodeHandle_for_paramaters, "z_sine_frequency");

	yaml_reference_difference=getParameterFloat(nodeHandle_for_paramaters, "reference_difference");

	// PARAMETERS ACCESSED BY RAMPC THREAD
	s_Rampc_mutex.lock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 2352");
	
	// Rampc parameters
	s_yaml_Tini = getParameterInt(nodeHandle_for_paramaters, "Tini");
	getParameterFloatVector(nodeHandle_for_paramaters, "Q", s_yaml_Q, 9);
	getParameterFloatVector(nodeHandle_for_paramaters, "R", s_yaml_R, 4);
	getParameterFloatVector(nodeHandle_for_paramaters, "P", s_yaml_P, 9);
	getParameterFloatVector(nodeHandle_for_paramaters, "K", s_yaml_K, 2);
	getParameterFloatVector(nodeHandle_for_paramaters, "F", s_yaml_F, 4);
	getParameterFloatVector(nodeHandle_for_paramaters, "G", s_yaml_G, 2);

	s_yaml_lambda2_g = getParameterFloat(nodeHandle_for_paramaters, "lambda2_g");
	s_yaml_lambda2_s = getParameterFloat(nodeHandle_for_paramaters, "lambda2_s");
	getParameterFloatVector(nodeHandle_for_paramaters, "output_min", s_yaml_output_min, 9);
	getParameterFloatVector(nodeHandle_for_paramaters, "output_max", s_yaml_output_max, 9);
	getParameterFloatVector(nodeHandle_for_paramaters, "input_min", s_yaml_input_min, 4);
	getParameterFloatVector(nodeHandle_for_paramaters, "input_max", s_yaml_input_max, 4);


	// Optimization parameters
	s_yaml_solver = getParameterString(nodeHandle_for_paramaters, "solver");
	s_yaml_opt_sparse = getParameterBool(nodeHandle_for_paramaters, "opt_sparse");
	s_yaml_opt_verbose = getParameterBool(nodeHandle_for_paramaters, "opt_verbose");
	s_yaml_opt_steady_state = getParameterBool(nodeHandle_for_paramaters, "opt_steady_state");

	// Parameters specific to Gurobi
	s_yaml_grb_LogToFile = getParameterBool(nodeHandle_for_paramaters, "grb_LogToFile");
	s_yaml_grb_presolve_at_setup = getParameterBool(nodeHandle_for_paramaters, "grb_presolve_at_setup");

	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 2352");


	// > DEBUGGING: Print out one of the parameters that was loaded to
	//   debug if the fetching of parameters worked correctly
	ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUGGING: the fetched RampcController/mass = " << yaml_cf_mass_in_grams);


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

	// > Initialize data collection matrices
	int num_rows = ceil(yaml_data_collection_max_time * 60.0 * yaml_control_frequency);
	if (num_rows != m_u_data_lqr.rows())
	{
		m_u_data_lqr = MatrixXf::Zero(num_rows, 4);
		m_u_data_Rampc = MatrixXf::Zero(num_rows, 4);
		if (yaml_Rampc_yaw_control)
			m_uf_data_Rampc = MatrixXf::Zero(num_rows, 4 * yaml_N);
		else
			m_uf_data_Rampc = MatrixXf::Zero(num_rows, 3 * yaml_N);

		m_y_data_lqr = MatrixXf::Zero(num_rows, 6);
		m_y_data_Rampc = MatrixXf::Zero(num_rows, 6);
		if (yaml_Rampc_measure_roll_pitch && yaml_Rampc_yaw_control)
			m_yf_data_Rampc = MatrixXf::Zero(num_rows, 6 * (yaml_N + 1));
		else if(yaml_Rampc_measure_roll_pitch)
			m_yf_data_Rampc = MatrixXf::Zero(num_rows, 5 * (yaml_N + 1));
		else if(yaml_Rampc_yaw_control)
			m_yf_data_Rampc = MatrixXf::Zero(num_rows, 4 * (yaml_N + 1));
		else
			m_yf_data_Rampc = MatrixXf::Zero(num_rows, 3 * (yaml_N + 1));

		m_r_data_lqr = MatrixXf::Zero(num_rows, 4);
		m_r_data_Rampc = MatrixXf::Zero(num_rows, 4);

		m_solveTime_data_Rampc = MatrixXf::Zero(num_rows, 1);
	}

	// > Get absolute data folder location
	m_dataFolder = HOME + yaml_dataFolder;

	// > Get absolute output data folder location
	m_outputFolder = m_dataFolder + yaml_outputFolder;

	// > Get the excitation signals from files
	m_thrustExcSignal = read_csv(m_dataFolder + yaml_thrustExcSignalFile);
	if (m_thrustExcSignal.size() <= 0)
		ROS_INFO("[RAMPC CONTROLLER] Failed to read thrust excitation signal file");
	else
	{
		int exc_start_time_d = int(yaml_exc_start_time / m_control_deltaT);
		m_u_data.setZero(exc_start_time_d + m_thrustExcSignal.size(), 4);
		m_y_data.setZero(exc_start_time_d + m_thrustExcSignal.size(), 6);
	}
	
	m_rollRateExcSignal = read_csv(m_dataFolder + yaml_rollRateExcSignalFile);
	if (m_rollRateExcSignal.size() <= 0)
		ROS_INFO("[RAMPC CONTROLLER] Failed to read roll rate excitation signal file");
	
	m_pitchRateExcSignal = read_csv(m_dataFolder + yaml_pitchRateExcSignalFile);
	if (m_pitchRateExcSignal.size() <= 0)
		ROS_INFO("[RAMPC CONTROLLER] Failed to read pitch rate excitation signal file");
	
	m_yawRateExcSignal = read_csv(m_dataFolder + yaml_yawRateExcSignalFile);
	if (m_yawRateExcSignal.size() <= 0)
		ROS_INFO("[RAMPC CONTROLLER] Failed to read yaw rate excitation signal file");

	// > Compute the Figure 8 frequency in units of rad/s
	m_figure_8_frequency_rad = 2 * PI * yaml_figure_8_frequency;
	
	// > Compute the z sine frequency in units of rad/s
	m_z_sine_frequency_rad = 2 * PI * yaml_z_sine_frequency;

	// PARAMETERS ACCESSED BY RAMPC THREAD
	s_Rampc_mutex.lock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 2432");

	// Share feed-forward with Rampc thread
	s_cf_weight_in_newtons = m_cf_weight_in_newtons;

	// Share absolute data folder location with Rampc thread
	s_dataFolder = m_dataFolder;

	// > Get absolute log files folder location
	s_logFolder = m_dataFolder + yaml_logFolder;

	// Share Rampc flag to use roll and pitch angle measurements with Rampc thread
	s_Rampc_measure_roll_pitch = yaml_Rampc_measure_roll_pitch;

	// Share Rampc flag to control yaw with Rampc thread
	s_Rampc_yaw_control = yaml_Rampc_yaw_control;

	// Share Rampc prediction horizon
	s_yaml_N = yaml_N;


	s_yaml_reference_difference=yaml_reference_difference;
	// Share changing reference parameters
	s_figure_8_amplitude = yaml_figure_8_amplitude;
	s_figure_8_frequency_rad = m_figure_8_frequency_rad;
	s_z_sine_amplitude = yaml_z_sine_amplitude;
	s_z_sine_frequency_rad = m_z_sine_frequency_rad;
	s_control_deltaT = m_control_deltaT;

	// > Set flag for Rampc thread to update parameters
	s_params_changed = true;

	s_Rampc_mutex.unlock();
	// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 2433");

	// Update setpoint to default
	setNewSetpoint(yaml_default_setpoint[0], yaml_default_setpoint[1], yaml_default_setpoint[2], yaml_default_setpoint[3]);

	// DEBUGGING: Print out one of the computed quantities
	ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUGGING: thus the weight of this agent in [Newtons] = " << m_cf_weight_in_newtons);
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
	ros::init(argc, argv, "RampcControllerService");

	// Create a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the
	// node handle assigned to this variable.
	ros::NodeHandle nodeHandle("~");

	// Get the namespace of this "RampcControllerService" node
	string m_namespace = ros::this_node::getNamespace();
	ROS_INFO_STREAM("[RAMPC CONTROLLER] ros::this_node::getNamespace() =  " << m_namespace);



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
		ROS_ERROR("[RAMPC CONTROLLER] Node NOT FUNCTIONING :-)");
		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("[RAMPC CONTROLLER] loaded agentID = " << m_agentID << ", and coordID = " << m_coordID);
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
	ROS_INFO_STREAM("[RAMPC CONTROLLER] m_namespace_to_own_agent_parameter_service    =  " << m_namespace_to_own_agent_parameter_service);
	ROS_INFO_STREAM("[RAMPC CONTROLLER] m_namespace_to_coordinator_parameter_service  =  " << m_namespace_to_coordinator_parameter_service);

	// Create, as local variables, node handles to the parameters services
	ros::NodeHandle nodeHandle_to_own_agent_parameter_service(m_namespace_to_own_agent_parameter_service);
	ros::NodeHandle nodeHandle_to_coordinator_parameter_service(m_namespace_to_coordinator_parameter_service);



	// SUBSCRIBE TO "YAML PARAMTERS READY" MESSAGES

	// The parameter service publishes messages with names of the form:
	// /dfall/.../ParameterService/<filename with .yaml extension>
	ros::Subscriber safeContoller_yamlReady_fromAgent = nodeHandle_to_own_agent_parameter_service.subscribe(  "RampcController", 1, isReadyRampcControllerYamlCallback);
	ros::Subscriber safeContoller_yamlReady_fromCoord = nodeHandle_to_coordinator_parameter_service.subscribe("RampcController", 1, isReadyRampcControllerYamlCallback);



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
	loadYamlFromFilenameCall.request.stringWithHeader.data = "RampcController";
	// Set for whom this applies to
	loadYamlFromFilenameCall.request.stringWithHeader.shouldCheckForAgentID = false;
	// Wait until the serivce exists
	requestLoadYamlFilenameServiceClient.waitForExistence(ros::Duration(-1));
	// Make the service call
	if(requestLoadYamlFilenameServiceClient.call(loadYamlFromFilenameCall))
	{
		// Nothing to do in this case.
		// The "isReadyRampcControllerYamlCallback" function
		// will be called once the YAML file is loaded
	}
	else
	{
		// Inform the user
		ROS_ERROR("[RAMPC CONTROLLER] The request load yaml file service call failed.");
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
	ros::Subscriber requestSetpointChangeSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("RampcControllerService/RequestSetpointChange", 1, requestSetpointChangeCallback);

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
    // variable that advertises the service called "RampcController". This service has
    // the input-output behaviour defined in the "Controller.srv" file (located in the
    // "srv" folder). This service, when called, is provided with the most recent
    // measurement of the Crazyflie and is expected to respond with the control action
    // that should be sent via the Crazyradio and requested from the Crazyflie, i.e.,
    // this is where the "outer loop" controller function starts. When a request is made
    // of this service the "calculateControlOutput" function is called.
    ros::ServiceServer service = nodeHandle.advertiseService("RampcController", calculateControlOutput);

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
	ros::Subscriber customCommandReceivedSubscriber_from_coord = nodeHandle_to_coordinator.subscribe("RampcControllerService/CustomButtonPressed", 1, customCommandReceivedCallback);


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

	// Create thread for solving Rampc optimization
	boost::thread Rampc_thread(Rampc_thread_main);

    // Print out some information to the user.
    ROS_INFO("[RAMPC CONTROLLER] Service ready :-)");

    // Enter an endless while loop to keep the node alive.
    ros::spin();

    // Wait for Rampc thread to finish
    Rampc_thread.join();

    // Return zero if the "ross::spin" is cancelled.
    return 0;
}
