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
//    A RAMPC Controller built from the Framework of the DeePC controller.
//
//    ----------------------------------------------------------------------------------





// INCLUDE THE HEADER
#include "nodes/RampcControllerService.h"







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
	// bools for the main thread
	bool params_changed;
	bool setpoint_changed;
	bool setupRampc;
	bool setupButtonPressed;
	bool solveRampc;
	bool changing_ref_enable_prev = false;
	bool theta_changed;
	bool updateTheta;
	int experiment;

	// current and past input definitions for parameter update
	MatrixXf d_current_input=MatrixXf::Zero(d_num_inputs,1);
	MatrixXf s_current_input=MatrixXf::Zero(d_num_inputs,1);
	MatrixXf m_current_input=MatrixXf::Zero(d_num_inputs,1);
	MatrixXf d_previous_input=MatrixXf::Zero(d_num_inputs,1);
	MatrixXf s_previous_input=MatrixXf::Zero(d_num_inputs,1);
	MatrixXf m_previous_input=MatrixXf::Zero(d_num_inputs,1);
	

	while (ros::ok())
	{
		// Get relevant bools at every iteration
		s_Rampc_mutex.lock();
		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 72");
		experiment=s_experiment;
        params_changed = s_params_changed;
        setpoint_changed = s_setpoint_changed;
        setupRampc = s_setupRampc;
        setupButtonPressed=s_setupButtonPressed;
        solveRampc = s_solveRampc;
        updateTheta = s_updateTheta;
        d_changing_ref_enable = s_changing_ref_enable;
        
        s_Rampc_mutex.unlock();
        //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 72");
        //ROS_INFO_STREAM("[RAMPC CONTROLLER] Update Theta: "<<updateTheta);



        if (params_changed)
        {
        	ROS_INFO_STREAM("[RAMPC CONTROLLER] Params Changed.");

        	// Change YAML parameters
        	change_Rampc_params();
        	
        	s_Rampc_mutex.lock();
        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 85");
        	s_params_changed = false;
        	s_Rampc_mutex.unlock();
        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 85");
        }
        //ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug Info 4.");
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
					
						switch(experiment){
							case RAMPC_CONTROLLER_EXPERIMENT_MASS:
								// Change decoupled setpoint
								// 0.8m offset in z direction to change from -0.7 to 0.7m to 0.1 to 1.5m
								d_z_setpoint_Rampc<< 	d_setpoint(2)-0.8,
														0.0;
								break;

							case RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS:
								// Change decoupled setpoint
								d_z_setpoint_Rampc<< 	d_setpoint(2)-0.8,
														0.0;
								break;

							case RAMPC_CONTROLLER_EXPERIMENT_FULL_STATE:
								// Change full state setpoint
								d_z_setpoint_Rampc<< 	d_setpoint(0),
														d_setpoint(1),
														d_setpoint(2)-0.8,
														0.0,
														0.0,
														0.0,
														0.0,
														0.0,
														d_setpoint(3),
														0.0,
														0.0,
														0.0;
								break;
						}
						
						// Change setpoint in upper bound
						
						for(int i=0;i<d_N;i++){
							for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
								temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
								d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
							}
						}


						s_Rampc_mutex.lock();
	        			 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 96");
	        			s_Rampc_active_setpoint = d_setpoint;
	        			s_setpoint_changed = false;
	        			s_Rampc_mutex.unlock();
	        			//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 96");
						break;

					

					
					default:
						break;
				}
	        	
        	}
        	else
        	{	
           		s_Rampc_mutex.lock();
	        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 96");
	        	s_Rampc_active_setpoint = s_setpoint;
	        	s_setpoint_changed = false;
	        	s_Rampc_mutex.unlock();
	        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 96");
        	}
        }
        //ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug Info 5.");
        if (setupRampc)
        {
        	ROS_INFO_STREAM("[RAMPC CONTROLLER] Setup Rampc.");
        	// Switch between the possible solvers
			switch (d_solver)
			{
				// Setup RAMPC. Experiment 1 for the mass 2 for rotor failure and 3 for the full state
				case RAMPC_CONTROLLER_SOLVER_OSQP:
					switch (experiment){
						case RAMPC_CONTROLLER_EXPERIMENT_MASS:
							setup_Rampc_osqp();
							break;

						case RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS:
							if(setupButtonPressed){
								setup_Rampc_all_rotors_osqp();
								setupButtonPressed=false;
								s_Rampc_mutex.lock();
								s_setupButtonPressed=false;
								s_setupRampc=false;
								s_Rampc_mutex.unlock();
							}
							break;	

						case RAMPC_CONTROLLER_EXPERIMENT_FULL_STATE:
							setup_Rampc_full_state_osqp();
							break;

						default:
						if (setupButtonPressed){
							ROS_INFO_STREAM("[RAMPC CONTROLLER] Please enter experiment number: 1 for mass, 2 for all rotors, 3 for full state.");
							setupButtonPressed=false;
							s_Rampc_mutex.lock();
							s_setupButtonPressed=false;
							s_setupRampc=false;
							s_Rampc_mutex.unlock();
						}
							break;

					}
					//setup_Rampc_osqp();
					break;


				
				default:
					
					break;
			}
			if(d_osqp_work){
				// Sometimes OSQP thinks the problem is non-convex if this happens we retry the setup
			if(!(d_osqp_work->info->status_val==-7)){
	 		   	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup successful with OSQP");

        		s_Rampc_mutex.lock();
        		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 107");
        		s_setupRampc = false;
        		s_Rampc_mutex.unlock();
        		//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 107");
        	}
        	}
        }
        //ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug Info 6.");
        if (solveRampc)
        {
        	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Solve Rampc.");
        	// Switch between the possible solvers
			switch (d_solver)
			{
				case RAMPC_CONTROLLER_SOLVER_OSQP:
					switch(experiment){
						case RAMPC_CONTROLLER_EXPERIMENT_MASS:

							// first solve the optimisation then update theta to reduce time delay
							solve_Rampc_osqp();
							update_theta_hat();
							break;
						case RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS:
							
							solve_Rampc_osqp();

							update_theta_hat();
							break;

						case RAMPC_CONTROLLER_EXPERIMENT_FULL_STATE:
							ROS_INFO_STREAM("[RAMPC CONTROLLER] Solving RAMPC.");
							solve_Rampc_full_state_osqp();
	
							ROS_INFO_STREAM("[RAMPC CONTROLLER] Updating Theta Hat.");
							update_theta_hat_full_state();
							break;

						default:
							ROS_INFO_STREAM("[RAMPC CONTROLLER] Please setup OSQP with experiment 1 for mass, 2 for all rotor failure.");
							break;
					}
					//solve_Rampc_osqp();
					//update_theta_hat();
					break;


				
				default:
					
					break;
			}

		  	s_Rampc_mutex.lock();
		  	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 118");
        	s_solveRampc = false;
        	s_Rampc_mutex.unlock();
        	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 118");
        }
        //ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug Info 7.");
        if (updateTheta){
        	// Update hyperboxes
        	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Solve Theta Update.");
        	switch(experiment){
        		case RAMPC_CONTROLLER_EXPERIMENT_MASS:
        			solve_theta_update_osqp();
        			break;
        		case RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS:
        			solve_theta_update_osqp_all_rotors();
        			break;
        		case RAMPC_CONTROLLER_EXPERIMENT_FULL_STATE:
        			ROS_INFO_STREAM("[RAMPC CONTROLLER] Updating Theta Bar.");
        			solve_theta_update_osqp_full_state();
        		default:
        			break;
        	}
        	s_Rampc_mutex.lock();
        	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG Mutex Lock 1119");
        	s_updateTheta=false;
        	s_Rampc_mutex.unlock();
        	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1119");
        }
        //ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug Info 8.");
	}

	// Cleanup for memory allocation etc.
	osqp_extended_cleanup();

	// Wait for gs matrix inversion thread to finish
    //Rampc_gs_inversion_thread.join();
}



void change_Rampc_params()
{
	// Change the parameters in the yaml file
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
	d_theta_update_num=s_yaml_theta_update_num;
	if (s_yaml_solver == "osqp")
		d_solver = RAMPC_CONTROLLER_SOLVER_OSQP;
	else
	{
		// Default solver is OSQP
		d_solver = RAMPC_CONTROLLER_SOLVER_OSQP;
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




void setup_Rampc_osqp()
{
	try
    {
    	

		// Get variables lengths of state and input
		get_variable_lengths();

		// Feedback matrix
		get_control_feedback();
	
		// Cost matrices
		get_cost_matrices();

		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 1");
		// OSQP QUADRATIC COST MATRIX
		MatrixXf d_osqp_P = get_quad_cost_matrix();
		//d_osqp_P=d_osqp_P+0.01*MatrixXf::Identity(d_osqp_P.rows(),d_osqp_P.cols());

		
		// Input/output constraint vectors
		get_input_output_constr();
		// Dynamics Matrices
		get_dynamics_matrix();

		// Parameters for the tube
		get_tube_params();

		// Initial point for setup
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

		// Initial dilation constraint
		MatrixXf osqp_A_s_ini=MatrixXf::Zero(1,d_col_num);
		MatrixXf osqp_l_s_ini=MatrixXf::Zero(1,1);
		MatrixXf osqp_u_s_ini=MatrixXf::Zero(1,1);

		osqp_A_s_ini(0,d_num_outputs*(d_N+1)*2+d_num_inputs*d_N)=1.0;
		osqp_l_s_ini(0,0)=0.0;
		osqp_u_s_ini(0,0)=0.0;


		// Dynamics constraints for x_hat
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


		// Dynamics constraints for x_bar
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


		// State and input constraints
		MatrixXf osqp_A_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),d_col_num);
		MatrixXf osqp_l_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
		MatrixXf osqp_u_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
		d_delta_uss=MatrixXf::Zero(d_num_inputs,1);
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

		// Tube inclusion constraints
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
		
		
		// Terminal constraints
		MatrixXf osqp_A_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),d_col_num);
		MatrixXf osqp_l_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);
		MatrixXf osqp_u_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);

		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*d_N,d_H_xf_x.rows(),d_num_outputs)=d_H_xf_x;
		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*(d_N+1)+d_num_inputs*d_N+d_N,d_H_xf_x.rows(),1)=d_H_xf_s;
		osqp_u_term_constr=MatrixXf::Ones(d_H_xf_x.rows(),1);
		osqp_l_term_constr=Inf_min*MatrixXf::Ones(d_H_xf_x.rows(),1);



		// Concatenate all matrices


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



		
		// OSQP MODEL SETUP
		// Follows 'Setup and solve' example (https://osqp.org/docs/examples/setup-and-solve.html)

		// q is 0 for our example
		MatrixXf osqp_q=MatrixXf::Zero(d_col_num,1);

		d_osqp_A=MatrixXf::Zero(osqp_A.rows(),osqp_A.cols());
		//osqp_extended_cleanup();
		
		d_osqp_A=osqp_A;



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
	    osqp_data = (OSQPData*) c_malloc(sizeof(OSQPData));
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





	    // Clear data after setting up to allow subseqeuent setups
	    //osqp_cleanup_data(osqp_data);
		


	    if (!d_osqp_work)
	    {
	    	clear_setupRampc_success_flag();

	    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup failed with OSQP");
	    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");

	    	return;
	    }

	    // Some steps to finish setup
		finish_Rampc_setup();
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 5");
		setup_theta_update_osqp();
		

	    // Inform the user
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




void setup_Rampc_all_rotors_osqp()
{
	try
    {
    	// Setup similar to setup_Rampc_osqp with different bounds for the parameters

		// Get variable lengths
		get_variable_lengths();
		get_cost_matrices_all_rotors();
		get_control_feedback_all_rotors();

		

		
		// OSQP QUADRATIC COST MATRIX
		MatrixXf d_osqp_P = get_quad_cost_matrix();
		//d_osqp_P=d_osqp_P+0.01*MatrixXf::Identity(d_osqp_P.rows(),d_osqp_P.cols());
		
		// Input/output constraint vectors
		get_input_output_constr();
		get_dynamics_matrix();

		get_tube_params_all_rotors();

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

		// Dynamics constraints for x_hat
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


		// dynamics constraints for x_bar
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

		// State and input constraints
		MatrixXf osqp_A_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),d_col_num);
		MatrixXf osqp_l_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
		MatrixXf osqp_u_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
		d_delta_uss=MatrixXf::Zero(d_num_inputs,1);
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

		// Tube inclusion constraints
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
		
		// Terminal constraints
		MatrixXf osqp_A_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),d_col_num);
		MatrixXf osqp_l_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);
		MatrixXf osqp_u_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);

		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*d_N,d_H_xf_x.rows(),d_num_outputs)=d_H_xf_x;
		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*(d_N+1)+d_num_inputs*d_N+d_N,d_H_xf_x.rows(),1)=d_H_xf_s;
		osqp_u_term_constr=MatrixXf::Ones(d_H_xf_x.rows(),1);
		osqp_l_term_constr=Inf_min*MatrixXf::Ones(d_H_xf_x.rows(),1);




		// Concatenate matrices

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
		osqp_u.bottomRows(osqp_u_term_constr.rows()) = osqp_u_term_constr;
		osqp_l.bottomRows(osqp_l_term_constr.rows()) = osqp_l_term_constr;




		// OSQP MODEL SETUP
		// Follows 'Setup and solve' example (https://osqp.org/docs/examples/setup-and-solve.html)
		MatrixXf osqp_q=MatrixXf::Zero(d_col_num,1);

		d_osqp_A=MatrixXf::Zero(osqp_A.rows(),osqp_A.cols());
		//osqp_extended_cleanup();
		
		d_osqp_A=osqp_A;



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
	    osqp_data = (OSQPData*) c_malloc(sizeof(OSQPData));
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



	    // Clear data after setting up to allow subseqeuent setups
	    //osqp_cleanup_data(osqp_data);
		


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









void setup_Rampc_full_state_osqp()
{
	try
    {
    	
		// Setup for the full state model, similar to setup_Rampc_osqp but for 12 states

		// Get variable lengths
		get_variable_lengths_full_state();
		get_cost_matrices_full_state();
		get_control_feedback_full_state();

		

		
		// OSQP QUADRATIC COST MATRIX
		MatrixXf d_osqp_P = get_quad_cost_matrix_full_state();
		
		// Input/output constraint vectors
		get_input_output_constr_full_state();
		get_dynamics_matrix_full_state();
		get_tube_params_full_state();



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

		// Dynamics constraints for x_hat
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


		// Dynamics constraints for x_bar
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

		// State and input constraints
		MatrixXf osqp_A_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),d_col_num);
		MatrixXf osqp_l_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
		MatrixXf osqp_u_state_input_constr=MatrixXf::Zero(d_N*(d_num_outputs*2+d_num_inputs*2),1);
		d_delta_uss=MatrixXf::Zero(d_num_inputs,1);
	    d_z_setpoint_Rampc=MatrixXf::Zero(d_num_outputs,1);
		d_z_setpoint_Rampc << 	0.0,
								0.0,
							   	0.4-0.8,		
							   	0.0,
							   	0.0,
							   	0.0,
							   	0.0,
							   	0.0,
							   	0.0,
							   	0.0,
							   	0.0,
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


		// Tube inclusion constraints
		MatrixXf osqp_A_tube_constr=MatrixXf::Zero(d_N*d_H_x.rows()*d_e_l.rows(),d_col_num);
		MatrixXf osqp_l_tube_constr=MatrixXf::Zero(d_N*d_H_x.rows()*d_e_l.rows(),1);
		MatrixXf osqp_u_tube_constr=MatrixXf::Zero(d_N*d_H_x.rows()*d_e_l.rows(),1);

		for(int i=0;i<d_N;i++){
			for(int j=0;j<d_H_x.rows();j++){
				for(int l=0;l<d_e_l.rows();l++){
					osqp_A_tube_constr.block(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)+i*d_num_outputs,1,d_num_outputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_K*d_e_l(l,0);
					osqp_A_tube_constr.block(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+i*d_num_inputs,1,d_num_inputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_e_l(l,0);
					osqp_A_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+d_num_inputs*d_N+i)=-d_rho_theta_k;
					osqp_A_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+d_num_inputs*d_N+i+1)=1.0;
					osqp_u_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,0)=Inf_max;
					osqp_l_tube_constr(i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,0)=d_w_bar;
				}
			}
		}
		
		// Terminal constraints
		MatrixXf osqp_A_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),d_col_num);
		MatrixXf osqp_l_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);
		MatrixXf osqp_u_term_constr=MatrixXf::Zero(d_H_xf_x.rows(),1);

		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*d_N,d_H_xf_x.rows(),d_num_outputs)=d_H_xf_x;
		osqp_A_term_constr.block(0,d_num_outputs*(d_N+1)+d_num_outputs*(d_N+1)+d_num_inputs*d_N+d_N,d_H_xf_x.rows(),1)=d_H_xf_s;
		osqp_u_term_constr=MatrixXf::Ones(d_H_xf_x.rows(),1);
		osqp_l_term_constr=Inf_min*MatrixXf::Ones(d_H_xf_x.rows(),1);




		// Concatenate matrices
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
		osqp_u.bottomRows(osqp_u_term_constr.rows()) = osqp_u_term_constr;
		osqp_l.bottomRows(osqp_l_term_constr.rows()) = osqp_l_term_constr;


		
		// OSQP MODEL SETUP
		// Follows 'Setup and solve' example (https://osqp.org/docs/examples/setup-and-solve.html)
		MatrixXf osqp_q=MatrixXf::Zero(d_col_num,1);

		d_osqp_A=MatrixXf::Zero(osqp_A.rows(),osqp_A.cols());
		//osqp_extended_cleanup();
		
		d_osqp_A=osqp_A;


	



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
	    osqp_data = (OSQPData*) c_malloc(sizeof(OSQPData));
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
	    //d_osqp_settings->eps_abs=0.000001;
	    //d_osqp_settings->eps_rel=0.000001;
		

	    // Setup workspace
	    d_osqp_work = osqp_setup(osqp_data, d_osqp_settings);
		
		
	    osqp_solve(d_osqp_work);

	    ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug Solve Status: "<<d_osqp_work->info->status_val);
	    for(int i=0;i<d_num_inputs;i++){
	    	ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug input "<<i<<": "<<d_osqp_work->solution->x[d_uf_start_i+i]);
		}



	    // Clear data after setting up to allow subseqeuent setups
	    //osqp_cleanup_data(osqp_data);
		


	    if (!d_osqp_work)
	    {
	    	clear_setupRampc_success_flag();

	    	ROS_INFO("[RAMPC CONTROLLER] Rampc optimization setup failed with OSQP");
	    	ROS_INFO("[RAMPC CONTROLLER] Rampc must be (re-)setup");

	    	return;
	    }

	    // Some steps to finish setup
		finish_Rampc_setup();
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 9");
		setup_theta_update_full_state_osqp();
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 10");

	    // Inform the user
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






void solve_Rampc_osqp()
{
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 622");

	// Get state estimates
		d_current_state_estimate = s_current_state_estimate;
		d_setpoint = s_setpoint;
		d_previous_state_estimate=s_previous_state_estimate;
		d_current_input=s_current_input;
		d_previous_input=s_previous_input;
	s_Rampc_mutex.unlock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 622");

	try
	{
		
		// If reference difference is too big we may experience infeasibility due to the time horizon, thus we set the reference to a max value
		
		if (d_current_state_estimate(2,0)-d_setpoint(2)>d_reference_difference){
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Set setpoint difference to "<<d_reference_difference<<".");
			d_osqp_l_new[0]=d_reference_difference;
			d_osqp_u_new[0]=d_reference_difference;
			d_osqp_l_new[2]=d_reference_difference;
			d_osqp_u_new[2]=d_reference_difference;

			d_z_setpoint_Rampc<< d_current_state_estimate(2,0)-d_reference_difference-0.8,
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
				//ROS_INFO_STREAM("[RAMPC CONTROLLER] Set setpoint difference to "<<-d_reference_difference<<".");
				d_osqp_l_new[0]=-d_reference_difference;
				d_osqp_u_new[0]=-d_reference_difference;
				d_osqp_l_new[2]=-d_reference_difference;
				d_osqp_u_new[2]=-d_reference_difference;


				d_z_setpoint_Rampc<< d_current_state_estimate(2,0)+d_reference_difference-0.8,
								 0.0;



				for(int i=0;i<d_N;i++){
					for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
						temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
						d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
					}
				}

			}
			else{
				//ROS_INFO_STREAM("[RAMPC CONTROLLER] Did not set setpoint difference.");
				d_osqp_l_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_u_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_l_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_u_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
			}
		}
		
		d_osqp_l_new[1]=d_current_state_estimate(5,0);
		d_osqp_u_new[1]=d_current_state_estimate(5,0);
		
		d_osqp_l_new[3]=d_current_state_estimate(5,0);
		d_osqp_u_new[3]=d_current_state_estimate(5,0);
		
		// update bounds in the osqp problem
		osqp_update_bounds(d_osqp_work, d_osqp_l_new, d_osqp_u_new);





		// Solve optimization
		osqp_solve(d_osqp_work);
		d_RampcOpt_status = d_osqp_work->info->status_val;

		if (d_RampcOpt_status > 0)
		{	
			MatrixXf pred_x=MatrixXf::Zero(2,1);
			MatrixXf uplus=MatrixXf::Zero(1,1);
			// get future inputs
			for(int d_i=0;d_i<d_Nuf;d_i++){
				pred_x << d_osqp_work->solution->x[d_Nyf+d_num_outputs*d_i],
							 d_osqp_work->solution->x[d_Nyf+1+d_num_outputs*d_i];
				MatrixXf u_plus=d_K*pred_x;
				d_u_f(d_i) = d_osqp_work->solution->x[d_uf_start_i + d_i]+u_plus(0)+d_cf_weight_in_newtons;
			}
			

			d_solve_time = d_osqp_work->info->run_time;

			s_Rampc_mutex.lock();
			

			s_u_f = d_u_f;
			s_y_f = d_y_f;
			s_solve_time = d_solve_time;
			s_Rampc_mutex.unlock();

			
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


void solve_Rampc_full_state_osqp()
{
	s_Rampc_mutex.lock();


		d_current_state_estimate = s_current_state_estimate;
		d_setpoint = s_setpoint;
		d_previous_state_estimate=s_previous_state_estimate;
		d_current_input=s_current_input;
		d_previous_input=s_previous_input;
		for(int i=0;i<9;i++){
			d_previous_stateErrorInertial[i]=s_previous_stateErrorInertial[i];
			d_current_stateErrorInertial[i]=s_current_stateErrorInertial[i];	
		}
		for (int i=9;i<12;i++){
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 321: "<<s_current_stateErrorInertial[i]);
			d_current_stateErrorInertial[i]=s_current_stateErrorInertial[i];
		}
	s_Rampc_mutex.unlock();




	try
	{
		// Update reference if reference is changing
		//if (d_changing_ref_enable)
		//	change_Rampc_setpoint_osqp_changing_ref();

		// Update equality constraint vectors

		//ROS_INFO_STREAM("[RAMPC CONTRLLER] Current height: "<<d_setpoint(2));
		/*
		if (d_current_state_estimate(2,0)-d_setpoint(2)>d_reference_difference){
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Set setpoint difference to "<<d_reference_difference<<".");
			d_osqp_l_new[0]=d_reference_difference;
			d_osqp_u_new[0]=d_reference_difference;
			d_osqp_l_new[2]=d_reference_difference;
			d_osqp_u_new[2]=d_reference_difference;

			d_z_setpoint_Rampc<< d_current_state_estimate(2,0)-d_reference_difference-0.8,
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
				//ROS_INFO_STREAM("[RAMPC CONTROLLER] Set setpoint difference to "<<-d_reference_difference<<".");
				d_osqp_l_new[0]=-d_reference_difference;
				d_osqp_u_new[0]=-d_reference_difference;
				d_osqp_l_new[2]=-d_reference_difference;
				d_osqp_u_new[2]=-d_reference_difference;


				d_z_setpoint_Rampc<< d_current_state_estimate(2,0)+d_reference_difference-0.8,
								 0.0;



				for(int i=0;i<d_N;i++){
					for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
						temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
						d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
					}
				}

			}
			else{
				//ROS_INFO_STREAM("[RAMPC CONTROLLER] Did not set setpoint difference.");
				d_osqp_l_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_u_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_l_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
				d_osqp_u_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
			}
		}
		*/

		/*
	float stateErrorInertial[12];

	// Fill in the (x,y,z) position error
	stateErrorInertial[0] = d_current_state_estimate(0) - m_setpoint[0];
	stateErrorInertial[1] = d_current_state_estimate(1) - m_setpoint[1];
	stateErrorInertial[2] = d_current_state_estimate(2) - m_setpoint[2];

	// Compute an estimate of the velocity
	// > As simply the derivative between the current and previous position
	stateErrorInertial[3] = (d_current_state_estimate(0) - m_previous_stateErrorInertial[0]) * yaml_control_frequency;
	stateErrorInertial[4] = (d_current_state_estimate(1) - m_previous_stateErrorInertial[1]) * yaml_control_frequency;
	stateErrorInertial[5] = (d_current_state_estimate(2) - m_previous_stateErrorInertial[2]) * yaml_control_frequency;

	// Fill in the roll and pitch angle measurements directly
	stateErrorInertial[6] = d_current_state_estimate(6);
	stateErrorInertial[7] = d_current_state_estimate(7);


	// Fill in the yaw angle error
	// > This error should be "unwrapped" to be in the range
	//   ( -pi , pi )
	// > First, get the yaw error into a local variable
	float yawError = d_current_state_estimate(8) - m_setpoint[3];
	// > Second, "unwrap" the yaw error to the interval ( -pi , pi )
	while(yawError > PI) {yawError -= 2 * PI;}
	while(yawError < -PI) {yawError += 2 * PI;}
	// > Third, put the "yawError" into the "stateError" variable
	stateErrorInertial[8] = yawError;

	stateErrorInertial[9]=(d_current_state_estimate(6) - m_previous_stateErrorInertial[6]) * yaml_control_frequency;
	stateErrorInertial[10]=(d_current_state_estimate(7) - m_previous_stateErrorInertial[7]) * yaml_control_frequency;
	stateErrorInertial[11]=(d_current_state_estimate(8) - m_previous_stateErrorInertial[8]) * yaml_control_frequency;


	// SAVE THE STATE ERROR TO BE USED NEXT TIME THIS FUNCTION IS CALLED
	// > as we have already used previous error we can now update it update it
	for(int i = 0; i < 9; ++i)
	{
		m_previous_stateErrorInertial[i] = stateErrorInertial[i];
	}



	
*/

		// Set initial state
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] RAMPC State: ");
		for(int i=0;i<d_num_outputs;i++){
			d_osqp_l_new[i]=d_current_stateErrorInertial[i];
			d_osqp_u_new[i]=d_current_stateErrorInertial[i];
			d_osqp_l_new[i+d_num_outputs]=d_current_stateErrorInertial[i];
			d_osqp_u_new[i+d_num_outputs]=d_current_stateErrorInertial[i];

			ROS_INFO_STREAM(d_current_stateErrorInertial[i]);
		}

		/*
		d_osqp_l_new[0]-=d_setpoint(0);
		d_osqp_u_new[0]-=d_setpoint(0);
		d_osqp_l_new[1]-=d_setpoint(1);
		d_osqp_u_new[1]-=d_setpoint(1);
		d_osqp_l_new[2]-=d_setpoint(2);
		d_osqp_u_new[2]-=d_setpoint(2);
		d_osqp_l_new[8]-=d_setpoint(3);
		d_osqp_u_new[8]-=d_setpoint(3);


		d_osqp_l_new[0+d_num_outputs]-=d_setpoint(0);
		d_osqp_u_new[0+d_num_outputs]-=d_setpoint(0);
		d_osqp_l_new[1+d_num_outputs]-=d_setpoint(1);
		d_osqp_u_new[1+d_num_outputs]-=d_setpoint(1);
		d_osqp_l_new[2+d_num_outputs]-=d_setpoint(2);
		d_osqp_u_new[2+d_num_outputs]-=d_setpoint(2);
		d_osqp_l_new[8+d_num_outputs]-=d_setpoint(3);
		d_osqp_u_new[8+d_num_outputs]-=d_setpoint(3);
		*/
		/*
		ROS_INFO_STREAM("[RAMPC CONTROLLER] STATE: ");
		for(int i=0;i<d_num_outputs;i++){
			ROS_INFO_STREAM(d_osqp_l_new[i]);
		}
		ROS_INFO_STREAM("[RAMPC CONTROLLER] PREVIOUS STATE: ");
		for(int i=0;i<d_num_outputs;i++){
			ROS_INFO_STREAM(d_previous_state_estimate(i));
		}
		*/
		/*
		d_osqp_l_new[1]=d_current_state_estimate(5,0);
		d_osqp_u_new[1]=d_current_state_estimate(5,0);
		
		d_osqp_l_new[3]=d_current_state_estimate(5,0);
		d_osqp_u_new[3]=d_current_state_estimate(5,0);
		
		d_osqp_l_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
		d_osqp_u_new[0]=d_current_state_estimate(2,0)-d_setpoint(2);
		d_osqp_l_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
		d_osqp_u_new[2]=d_current_state_estimate(2,0)-d_setpoint(2);
		*/

		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Delta_X: " << )

		// update bounds
		osqp_update_bounds(d_osqp_work, d_osqp_l_new, d_osqp_u_new);

		// Solve optimization
		osqp_solve(d_osqp_work);
		d_RampcOpt_status = d_osqp_work->info->status_val;
		if (d_RampcOpt_status > 0)
		{	
			MatrixXf pred_x=MatrixXf::Zero(d_num_outputs,1);
			MatrixXf uplus=MatrixXf::Zero(d_num_inputs,1);
			// Get inputs
			for(int d_i=0;d_i<d_N;d_i++){
				for (int j=0;j<d_num_outputs;j++){
					pred_x(j)=d_osqp_work->solution->x[d_Nyf+j+d_num_outputs*d_i];
				}
				MatrixXf u_plus=d_K*pred_x;
				for(int d_j=0;d_j<d_num_inputs;d_j++){
				d_u_f(d_i*d_num_inputs+d_j) = d_osqp_work->solution->x[d_uf_start_i + d_i*d_num_inputs+d_j]+u_plus(d_j)+d_cf_weight_in_newtons/4.0;
				}
				
			}
				ROS_INFO_STREAM("[RAMPC CONTROLLER] INPUT: ");

				MatrixXf predxx=MatrixXf::Zero(d_num_outputs,1);
				MatrixXf uuplus=MatrixXf::Zero(d_num_inputs,1);
			for (int j=0;j<d_num_outputs;j++){
					predxx(j)=d_osqp_work->solution->x[d_Nyf+j+d_num_outputs*0];
				}
				uuplus=d_K*predxx;
			for(int d_j=0;d_j<d_num_inputs;d_j++){
					ROS_INFO_STREAM(d_osqp_work->solution->x[d_uf_start_i + 0*d_num_inputs+d_j]+uuplus(d_j)+d_cf_weight_in_newtons/4.0);
				}
				ROS_INFO_STREAM("[RAMPC CONTROLLER] Solution:");

				for(int d_j=0;d_j<d_num_inputs;d_j++){
					ROS_INFO_STREAM(d_osqp_work->solution->x[d_uf_start_i + 0*d_num_inputs+d_j]);
				}
			
			d_solve_time = d_osqp_work->info->run_time;

			s_Rampc_mutex.lock();
			// ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 649");
			s_u_f = d_u_f;
			s_y_f = d_y_f;
			s_solve_time = d_solve_time;
			s_Rampc_mutex.unlock();
			//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 649");
			
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc found optimal solution with OSQP status: " << d_osqp_work->info->status);
			//ROS_INFO_STREAM("Thrust: " << d_u_f(0));
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
	int experiment=s_experiment;
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
		
			// Update current and previous state

			m_previous_state_estimate=m_current_state_estimate;
	
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
			m_current_state_estimate(8) = request.ownCrazyflie.yaw;

			m_current_state_estimate(9) = (request.ownCrazyflie.roll - m_previous_state_estimate(6)) * yaml_control_frequency;
			m_current_state_estimate(10) = (request.ownCrazyflie.pitch - m_previous_state_estimate(7)) * yaml_control_frequency;
			m_current_state_estimate(11) = (request.ownCrazyflie.yaw - m_previous_state_estimate(8)) * yaml_control_frequency;
			//ROS_INFO("[MPC CONTROLLER] DEBUG 23");

			switch(experiment){
				case RAMPC_CONTROLLER_EXPERIMENT_MASS:
					m_previous_input=m_current_input;
					m_current_input(0)=output.thrust;
					break;

				case RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS:
					m_previous_input=m_current_input;
					m_current_input(0)=output.thrust;
					break;

				case RAMPC_CONTROLLER_EXPERIMENT_FULL_STATE:
					m_previous_input=m_current_input;
					m_current_input(0)=output.thrust1;
					m_current_input(1)=output.thrust2;
					m_current_input(2)=output.thrust3;
					m_current_input(3)=output.thrust4;
					break;


			}
			

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
		s_previous_state_estimate=m_previous_state_estimate;
		s_previous_input=m_previous_input;
		s_current_input=m_current_input;
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


void update_theta_hat(){
	// Get current and previous state
	MatrixXf previous_state_z=MatrixXf::Zero(d_num_outputs,1);
	MatrixXf current_state_z=MatrixXf::Zero(d_num_outputs,1);
	current_state_z<< 	d_current_state_estimate(2,0),
						d_current_state_estimate(5,0);
	previous_state_z<< 	d_previous_state_estimate(2,0),
						d_previous_state_estimate(5,0);

	// Update formula for theta_tilde
	MatrixXf temp_hat=(d_B_1*(d_previous_input-d_cf_weight_in_newtons*MatrixXf::Ones(1,1))).transpose()*(current_state_z-(d_A*previous_state_z+(d_B_0+d_B_1*d_theta_hat_k)*(d_previous_input-d_cf_weight_in_newtons*MatrixXf::Ones(1,1))));
	d_theta_hat_k+=d_mu*temp_hat(0,0);

	// project onto Theta_k
	if(d_theta_hat_k<d_vertices_theta(1,0)){
		d_theta_hat_k=d_vertices_theta(1,0);
	}
	else{
		if(d_theta_hat_k>d_vertices_theta(0,0)){
			d_theta_hat_k=d_vertices_theta(0,0);
		}
	}
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta hat: "<<d_theta_hat_k);

	// Update the A matrix
	MatrixXf dynamics_constr_A_hat_temp=d_A+(d_B_0+d_B_1*d_theta_hat_k)*d_K;
	MatrixXf dynamics_constr_B_hat_temp=(d_B_0+d_B_1*d_theta_hat_k);
	for(int i=0;i<d_N;i++){
			d_osqp_A.block(2*d_num_outputs+1+i*d_num_outputs,i*d_num_outputs,d_num_outputs,d_num_outputs)=dynamics_constr_A_hat_temp;
			d_osqp_A.block(2*d_num_outputs+1+i*d_num_outputs,d_num_outputs*(d_N+1)*2+i*d_num_inputs,d_num_outputs,d_num_inputs)=dynamics_constr_B_hat_temp;
	}

	MatrixXf osqp_A_new=MatrixXf::Zero(d_osqp_A.rows(),d_osqp_A.cols());
	osqp_A_new=d_osqp_A;
	
	csc* osqp_A_new_csc = eigen2csc(osqp_A_new);
	osqp_update_A(d_osqp_work,osqp_A_new_csc->x,OSQP_NULL,osqp_A_new_csc->nzmax);

}

void update_theta_hat_full_state(){
	/*
	MatrixXf previous_state_z=MatrixXf::Zero(d_num_outputs,1);
	MatrixXf current_state_z=MatrixXf::Zero(d_num_outputs,1);
	current_state_z=	d_current_state_estimate;
						
	previous_state_z =	d_previous_state_estimate;
	
	MatrixXf temp_hat=(d_B_1*(d_previous_input-0.25*d_cf_weight_in_newtons*MatrixXf::Ones(d_num_inputs,1))).transpose()*(current_state_z-(d_A*previous_state_z+(d_B_0+d_B_1*d_theta_hat_k)*(d_previous_input-0.25*d_cf_weight_in_newtons*MatrixXf::Ones(d_num_inputs,1))));
	d_theta_hat_k+=d_mu*temp_hat(0,0);
	if(d_theta_hat_k<d_vertices_theta(1,0)){
		d_theta_hat_k=d_vertices_theta(1,0);
	}
	else{
		if(d_theta_hat_k>d_vertices_theta(0,0)){
			d_theta_hat_k=d_vertices_theta(0,0);
		}
	}
	ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta hat: "<<d_theta_hat_k);
	MatrixXf dynamics_constr_A_hat_temp=d_A+(d_B_0+d_B_1*d_theta_hat_k)*d_K;
	MatrixXf dynamics_constr_B_hat_temp=(d_B_0+d_B_1*d_theta_hat_k);
	for(int i=0;i<d_N;i++){
			d_osqp_A.block(2*d_num_outputs+1+i*d_num_outputs,i*d_num_outputs,d_num_outputs,d_num_outputs)=dynamics_constr_A_hat_temp;
			d_osqp_A.block(2*d_num_outputs+1+i*d_num_outputs,d_num_outputs*(d_N+1)*2+i*d_num_inputs,d_num_outputs,d_num_inputs)=dynamics_constr_B_hat_temp;
	}

	MatrixXf osqp_A_new=MatrixXf::Zero(d_osqp_A.rows(),d_osqp_A.cols());
	osqp_A_new=d_osqp_A;
	
	csc* osqp_A_new_csc = eigen2csc(osqp_A_new);
	//osqp_update_A(d_osqp_work,osqp_A_new_csc->x,OSQP_NULL,osqp_A_new_csc->nzmax);
	*/
}


void solve_theta_update_osqp(){
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Updating Theta.");

	// Get previous and current state
	d_h_theta(0,0)=d_vertices_theta(0,0);
	d_h_theta(1,0)=-d_vertices_theta(1,0);
	MatrixXf Delta_k=MatrixXf::Zero(2*d_num_outputs,d_N_theta);
	MatrixXf delta_k=MatrixXf::Zero(2*d_num_outputs,1);
	MatrixXf previous_state_z=MatrixXf::Zero(d_num_outputs,1);
	MatrixXf current_state_z=MatrixXf::Zero(d_num_outputs,1);
	previous_state_z<< 	d_previous_state_estimate(2,0),
						d_previous_state_estimate(5,0);
	current_state_z<< 	d_current_state_estimate(2,0),
						d_current_state_estimate(5,0);

	// non-falsified parameter set
	Delta_k=-d_H_w*(d_B_1_update*d_previous_input);

	delta_k=d_h_w+d_H_w*(d_A_const_update+d_A_update*previous_state_z+d_B_0_update*d_previous_input-current_state_z)+d_n_bar+d_n_bar_prev;

	// Update constraints for theta_min and theta_max LPs
	d_osqp_theta_A.block(0,0,d_num_outputs,d_N_theta)=d_H_theta;
	d_osqp_theta_A.block(d_num_outputs,0,d_num_outputs,d_N_theta)=Delta_k.block(d_num_outputs,0,d_num_outputs,d_N_theta);
	for(int i=0;i<2*d_N_theta;i++){
		d_osqp_theta_u_new[i]=d_h_theta(i,0);
	}
	for(int i=0;i<d_num_outputs;i++){
		d_osqp_theta_u_new[2*d_N_theta+i]=delta_k(d_num_outputs+i,0);
	}

	csc* osqp_theta_A_csc = eigen2csc(d_osqp_theta_A);
	//osqp_update_A(d_osqp_theta_max_work, osqp_theta_A_csc);
	//osqp_update_A(d_osqp_theta_min_work, osqp_theta_A_csc);

	
	//osqp_update_upper_bound(d_osqp_theta_max_work,d_osqp_u_new);
	//osqp_update_upper_bound(d_osqp_theta_min_work,d_osqp_u_new);
	osqp_update_A(d_osqp_theta_max_work,osqp_theta_A_csc->x,OSQP_NULL,osqp_theta_A_csc->nzmax);
	osqp_update_upper_bound(d_osqp_theta_max_work,d_osqp_theta_u_new);

	osqp_update_A(d_osqp_theta_min_work,osqp_theta_A_csc->x,OSQP_NULL,osqp_theta_A_csc->nzmax);
	osqp_update_upper_bound(d_osqp_theta_min_work,d_osqp_theta_u_new);

	/*
	osqp_theta_max_data->A = osqp_theta_A_csc;
	osqp_theta_max_data->u = d_osqp_theta_u_new;

	osqp_theta_min_data->A = osqp_theta_A_csc;
	osqp_theta_min_data->u = d_osqp_theta_u_new;


	d_osqp_theta_max_work = osqp_setup(osqp_theta_max_data, d_osqp_theta_settings);
	d_osqp_theta_min_work = osqp_setup(osqp_theta_min_data, d_osqp_theta_settings);
	*/

	// solve the LPs
	osqp_solve(d_osqp_theta_max_work);
	osqp_solve(d_osqp_theta_min_work);


	d_ThetaMaxOpt_status = d_osqp_theta_max_work->info->status_val;
	d_ThetaMinOpt_status = d_osqp_theta_min_work->info->status_val;
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Status min: "<<d_ThetaMinOpt_status<<". Status max: "<<d_ThetaMaxOpt_status);
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update num: "<<d_theta_update_num);


	if ((d_ThetaMaxOpt_status > 0) && (d_ThetaMinOpt_status > 0)){
		// update every d_theta_update_num by using the convex hull of the union 
		theta_max_buffer.push_back(d_osqp_theta_max_work->solution->x[0]);
		theta_min_buffer.push_back(d_osqp_theta_min_work->solution->x[0]);
		d_theta_update_it++;
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update it: "<<d_theta_update_it);

		if(d_theta_update_it>=d_theta_update_num){
		float theta_max=*std::max_element(theta_max_buffer.begin(),theta_max_buffer.end());
		float theta_min=*std::min_element(theta_min_buffer.begin(),theta_min_buffer.end());
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_max vector: "<<theta_max_buffer);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_max : "<<theta_max);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_min vector: "<<theta_min_buffer);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_min : "<<theta_min);
		//for(int i=0;i<theta_max_buffer.size();i++){
		//	ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta max "<<i<<" : "<<theta_max_buffer[i]);
		//}
		theta_max_buffer.clear();
		theta_min_buffer.clear();
		d_theta_update_it=0;

		// Update theta_bar_k and eta_k
		float theta_bar_k1=(theta_max+theta_min)*0.5;
		float eta_k1=theta_max-theta_min;
		if ((eta_k1<d_eta_k) && (eta_k1>0.0)){
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta min: "<<theta_min<<". Theta max: "<<theta_max);
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Eta: "<<d_eta_k);
			if (theta_bar_k1>d_theta_bar_k+0.5*(d_eta_k-eta_k1)){
				d_theta_bar_k=d_theta_bar_k+0.5*(d_eta_k-eta_k1);
			}
			else{
				if (theta_bar_k1<d_theta_bar_k-0.5*(d_eta_k-eta_k1)){
					d_theta_bar_k=d_theta_bar_k-0.5*(d_eta_k-eta_k1);
				}
				else d_theta_bar_k=theta_bar_k1;
			}
			d_eta_k=eta_k1;
			d_vertices_theta(0,0)=d_theta_bar_k+0.5*d_eta_k;
			d_vertices_theta(1,0)=d_theta_bar_k-0.5*d_eta_k;
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Eta: "<<d_eta_k);
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Vertices Theta: "<<d_vertices_theta);
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta bar k: "<<d_theta_bar_k);



			// Update the constraints for the main optimisation
			MatrixXf dynamics_constr_A_bar_temp=d_A+(d_B_0+d_B_1*d_theta_bar_k)*d_K;
			MatrixXf dynamics_constr_B_bar_temp=(d_B_0+d_B_1*d_theta_bar_k);
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update Debug 1.");
			for(int i=0;i<d_N;i++){
				d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs+i*d_num_outputs,d_num_outputs*(d_N+1)+i*d_num_outputs,d_num_outputs,d_num_outputs)=dynamics_constr_A_bar_temp;
				d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs+i*d_num_outputs,d_num_outputs*(d_N+1)*2+i*d_num_inputs,d_num_outputs,d_num_inputs)=dynamics_constr_B_bar_temp;
			}


			for(int i=0;i<d_N;i++){
				for(int j=0;j<d_H_x.rows();j++){
					for(int l=0;l<d_e_l.rows();l++){
						d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs*2+d_N*(d_num_outputs*2+d_num_inputs*2)+i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)+i*d_num_outputs,1,d_num_outputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_K*d_e_l(l,0);
						d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs*2+d_N*(d_num_outputs*2+d_num_inputs*2)+i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+i*d_num_inputs,1,d_num_inputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_e_l.row(l);
					}
				}
			}
		


			// Update the steady state input
			d_delta_uss(0)=d_delta_uss(0)+d_cf_weight_in_newtons-9.81/d_theta_bar_k;
			d_cf_weight_in_newtons = 9.81/d_theta_bar_k;

			for(int i=0;i<d_N;i++){
				for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
					temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
					d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
				}
			}
		
			//osqp_update_upper_bound(d_osqp_work,d_osqp_u_new);
		



		}
		}
	}

}



void solve_theta_update_osqp_all_rotors(){


	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Updating Theta.");

	// Get current and previous state
	d_h_theta(0,0)=d_vertices_theta(0,0);
	d_h_theta(1,0)=-d_vertices_theta(1,0);
	MatrixXf Delta_k=MatrixXf::Zero(2*d_num_outputs,d_N_theta);
	MatrixXf delta_k=MatrixXf::Zero(2*d_num_outputs,1);
	MatrixXf previous_state_z=MatrixXf::Zero(d_num_outputs,1);
	MatrixXf current_state_z=MatrixXf::Zero(d_num_outputs,1);
	previous_state_z<< 	d_previous_state_estimate(2,0),
						d_previous_state_estimate(5,0);
	current_state_z<< 	d_current_state_estimate(2,0),
						d_current_state_estimate(5,0);


	// non-falsified parameter set
	Delta_k=-d_H_w*(d_B_1_update*d_previous_input);

	delta_k=d_h_w+d_H_w*(d_A_const_update+d_A_update*previous_state_z+d_B_0_update*d_previous_input-current_state_z)+d_n_bar+d_n_bar_prev;

	// Update constraints for theta_min and theta_max constraints
	d_osqp_theta_A.block(0,0,d_num_outputs,d_N_theta)=d_H_theta;
	d_osqp_theta_A.block(d_num_outputs,0,d_num_outputs,d_N_theta)=Delta_k.block(d_num_outputs,0,d_num_outputs,d_N_theta);
	for(int i=0;i<2*d_N_theta;i++){
		d_osqp_theta_u_new[i]=d_h_theta(i,0);
	}
	for(int i=0;i<d_num_outputs;i++){
		d_osqp_theta_u_new[2*d_N_theta+i]=delta_k(d_num_outputs+i,0);
	}

	csc* osqp_theta_A_csc = eigen2csc(d_osqp_theta_A);
	//osqp_update_A(d_osqp_theta_max_work, osqp_theta_A_csc);
	//osqp_update_A(d_osqp_theta_min_work, osqp_theta_A_csc);

	
	//osqp_update_upper_bound(d_osqp_theta_max_work,d_osqp_u_new);
	//osqp_update_upper_bound(d_osqp_theta_min_work,d_osqp_u_new);
	osqp_update_A(d_osqp_theta_max_work,osqp_theta_A_csc->x,OSQP_NULL,osqp_theta_A_csc->nzmax);
	osqp_update_upper_bound(d_osqp_theta_max_work,d_osqp_theta_u_new);
	osqp_update_A(d_osqp_theta_min_work,osqp_theta_A_csc->x,OSQP_NULL,osqp_theta_A_csc->nzmax);
	osqp_update_upper_bound(d_osqp_theta_min_work,d_osqp_theta_u_new);
	/*
	osqp_theta_max_data->A = osqp_theta_A_csc;
	osqp_theta_max_data->u = d_osqp_theta_u_new;

	osqp_theta_min_data->A = osqp_theta_A_csc;
	osqp_theta_min_data->u = d_osqp_theta_u_new;


	d_osqp_theta_max_work = osqp_setup(osqp_theta_max_data, d_osqp_theta_settings);
	d_osqp_theta_min_work = osqp_setup(osqp_theta_min_data, d_osqp_theta_settings);
	*/
	osqp_solve(d_osqp_theta_max_work);
	osqp_solve(d_osqp_theta_min_work);
	d_ThetaMaxOpt_status = d_osqp_theta_max_work->info->status_val;
	d_ThetaMinOpt_status = d_osqp_theta_min_work->info->status_val;
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Status min: "<<d_ThetaMinOpt_status<<". Status max: "<<d_ThetaMaxOpt_status);
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update num: "<<d_theta_update_num);
	if ((d_ThetaMaxOpt_status > 0) && (d_ThetaMinOpt_status > 0)){
		// Update theta_bar every d_theta_update_num time steps with the convex hull of the union of the Delta_k sets
		theta_max_buffer.push_back(d_osqp_theta_max_work->solution->x[0]);
		theta_min_buffer.push_back(d_osqp_theta_min_work->solution->x[0]);
		d_theta_update_it++;
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update it: "<<d_theta_update_it);
		if(d_theta_update_it>=d_theta_update_num){
		float theta_max=*std::max_element(theta_max_buffer.begin(),theta_max_buffer.end());
		float theta_min=*std::min_element(theta_min_buffer.begin(),theta_min_buffer.end());

		if (d_vertices_theta(1,0)*0.7<theta_abs_min){
			d_eta_k=d_vertices_theta(0,0)-theta_abs_min;
			d_theta_bar_k=(d_vertices_theta(0,0)+theta_abs_min)*0.5;
		}
		else{
			d_eta_k=d_vertices_theta(0,0)-d_vertices_theta(1,0)*0.7;
			d_theta_bar_k=(d_vertices_theta(0,0)+d_vertices_theta(1,0)*0.7)*0.5;
		}

		if(theta_min*0.7<theta_abs_min){
			theta_min=theta_abs_min;
		}
		else{
			theta_min=0.7*theta_min;
		}
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_max vector: "<<theta_max_buffer);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_max : "<<theta_max);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_min vector: "<<theta_min_buffer);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_min : "<<theta_min);
		//for(int i=0;i<theta_max_buffer.size();i++){
		//	ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta max "<<i<<" : "<<theta_max_buffer[i]);
		//}
		theta_max_buffer.clear();
		theta_min_buffer.clear();
		d_theta_update_it=0;

		// Update theta_bar and eta
		float theta_bar_k1=(theta_max+theta_min)*0.5;
		float eta_k1=theta_max-theta_min;
		if ((eta_k1<d_eta_k) && (eta_k1>0.0)){
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta min: "<<theta_min<<". Theta max: "<<theta_max);
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Eta: "<<d_eta_k);
			if (theta_bar_k1>d_theta_bar_k+0.5*(d_eta_k-eta_k1)){
				d_theta_bar_k=d_theta_bar_k+0.5*(d_eta_k-eta_k1);
			}
			else{
				if (theta_bar_k1<d_theta_bar_k-0.5*(d_eta_k-eta_k1)){
					d_theta_bar_k=d_theta_bar_k-0.5*(d_eta_k-eta_k1);
				}
				else d_theta_bar_k=theta_bar_k1;
			}
			d_eta_k=eta_k1;
			d_vertices_theta(0,0)=d_theta_bar_k+0.5*d_eta_k;
			d_vertices_theta(1,0)=d_theta_bar_k-0.5*d_eta_k;
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Eta: "<<d_eta_k);
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Vertices Theta: "<<d_vertices_theta);
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta bar k: "<<d_theta_bar_k);



			// Update constraints
			MatrixXf dynamics_constr_A_bar_temp=d_A+(d_B_0+d_B_1*d_theta_bar_k)*d_K;
			MatrixXf dynamics_constr_B_bar_temp=(d_B_0+d_B_1*d_theta_bar_k);
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update Debug 1.");
			for(int i=0;i<d_N;i++){
				d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs+i*d_num_outputs,d_num_outputs*(d_N+1)+i*d_num_outputs,d_num_outputs,d_num_outputs)=dynamics_constr_A_bar_temp;
				d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs+i*d_num_outputs,d_num_outputs*(d_N+1)*2+i*d_num_inputs,d_num_outputs,d_num_inputs)=dynamics_constr_B_bar_temp;
			}


			for(int i=0;i<d_N;i++){
				for(int j=0;j<d_H_x.rows();j++){
					for(int l=0;l<d_e_l.rows();l++){
						d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs*2+d_N*(d_num_outputs*2+d_num_inputs*2)+i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)+i*d_num_outputs,1,d_num_outputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_K*d_e_l(l,0);
						d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs*2+d_N*(d_num_outputs*2+d_num_inputs*2)+i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+i*d_num_inputs,1,d_num_inputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_e_l.row(l);
					}
				}
			}
		


			// Update steady-state input
			d_delta_uss(0)=d_delta_uss(0)+d_cf_weight_in_newtons-9.81/d_theta_bar_k;
			d_cf_weight_in_newtons = 9.81/d_theta_bar_k;

			for(int i=0;i<d_N;i++){
				for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
					temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
					d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
				}
			}
			
			//osqp_update_upper_bound(d_osqp_work,d_osqp_u_new);



		}
		}
	}

}


void solve_theta_update_osqp_full_state(){
	/*
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Updating Theta.");
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 1");
	d_h_theta(0,0)=d_vertices_theta(0,0);
	d_h_theta(1,0)=-d_vertices_theta(1,0);
	MatrixXf Delta_k=MatrixXf::Zero(2*d_num_outputs,d_N_theta);
	MatrixXf delta_k=MatrixXf::Zero(2*d_num_outputs,1);
	MatrixXf previous_state_z=MatrixXf::Zero(d_num_outputs,1);
	MatrixXf current_state_z=MatrixXf::Zero(d_num_outputs,1);
	previous_state_z= 	d_previous_state_estimate;
						
	current_state_z=	d_current_state_estimate;
					
//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 2");

	Delta_k=-d_H_w*(d_B_1_update*d_previous_input);

	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 3");

	delta_k=d_h_w+d_H_w*(d_A_const_update+d_A_update*previous_state_z+d_B_0_update*d_previous_input-current_state_z)+d_n_bar+d_n_bar_prev;


	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DELTA K"<<Delta_k );
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DELTA K"<<delta_k );
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 4");
	d_osqp_theta_A.block(0,0,2*d_N_theta,d_N_theta)=d_H_theta;
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 5");
	d_osqp_theta_A.block(d_num_outputs,0,2,d_N_theta)=Delta_k.block(10,0,2,d_N_theta);
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 6");
	for(int i=0;i<2*d_N_theta;i++){
		d_osqp_theta_u_new[i]=d_h_theta(i,0);
	}
	for(int i=0;i<2;i++){
		d_osqp_theta_u_new[2*d_N_theta+i]=delta_k(10+i,0);
	}
//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 3");
	csc* osqp_theta_A_csc = eigen2csc(d_osqp_theta_A);
	//osqp_update_A(d_osqp_theta_max_work, osqp_theta_A_csc);
	//osqp_update_A(d_osqp_theta_min_work, osqp_theta_A_csc);

	
	//osqp_update_upper_bound(d_osqp_theta_max_work,d_osqp_u_new);
	//osqp_update_upper_bound(d_osqp_theta_min_work,d_osqp_u_new);
	osqp_update_A(d_osqp_theta_max_work,osqp_theta_A_csc->x,OSQP_NULL,osqp_theta_A_csc->nzmax);
	osqp_update_upper_bound(d_osqp_theta_max_work,d_osqp_theta_u_new);
	osqp_update_A(d_osqp_theta_min_work,osqp_theta_A_csc->x,OSQP_NULL,osqp_theta_A_csc->nzmax);
	osqp_update_upper_bound(d_osqp_theta_min_work,d_osqp_theta_u_new);
	/*
	osqp_theta_max_data->A = osqp_theta_A_csc;
	osqp_theta_max_data->u = d_osqp_theta_u_new;

	osqp_theta_min_data->A = osqp_theta_A_csc;
	osqp_theta_min_data->u = d_osqp_theta_u_new;


	d_osqp_theta_max_work = osqp_setup(osqp_theta_max_data, d_osqp_theta_settings);
	d_osqp_theta_min_work = osqp_setup(osqp_theta_min_data, d_osqp_theta_settings);
	*/
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 5");
	/*
	osqp_solve(d_osqp_theta_max_work);
	osqp_solve(d_osqp_theta_min_work);
	d_ThetaMaxOpt_status = d_osqp_theta_max_work->info->status_val;
	d_ThetaMinOpt_status = d_osqp_theta_min_work->info->status_val;
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Status min: "<<d_ThetaMinOpt_status<<". Status max: "<<d_ThetaMaxOpt_status);
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update num: "<<d_theta_update_num);
	if ((d_ThetaMaxOpt_status > 0) && (d_ThetaMinOpt_status > 0)){
		theta_max_buffer.push_back(d_osqp_theta_max_work->solution->x[0]);
		theta_min_buffer.push_back(d_osqp_theta_min_work->solution->x[0]);
		d_theta_update_it++;
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update it: "<<d_theta_update_it);
		if(d_theta_update_it>=d_theta_update_num){
		float theta_max=*std::max_element(theta_max_buffer.begin(),theta_max_buffer.end());
		float theta_min=*std::min_element(theta_min_buffer.begin(),theta_min_buffer.end());
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_max vector: "<<theta_max_buffer);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_max : "<<theta_max);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_min vector: "<<theta_min_buffer);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta_min : "<<theta_min);
		//for(int i=0;i<theta_max_buffer.size();i++){
		//	ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta max "<<i<<" : "<<theta_max_buffer[i]);
		//}
		theta_max_buffer.clear();
		theta_min_buffer.clear();
		d_theta_update_it=0;
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 6");
		float theta_bar_k1=(theta_max+theta_min)*0.5;
		float eta_k1=theta_max-theta_min;
		if ((eta_k1<d_eta_k) && (eta_k1>0.0)){
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta min: "<<theta_min<<". Theta max: "<<theta_max);
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Eta: "<<d_eta_k);
			if (theta_bar_k1>d_theta_bar_k+0.5*(d_eta_k-eta_k1)){
				d_theta_bar_k=d_theta_bar_k+0.5*(d_eta_k-eta_k1);
			}
			else{
				if (theta_bar_k1<d_theta_bar_k-0.5*(d_eta_k-eta_k1)){
					d_theta_bar_k=d_theta_bar_k-0.5*(d_eta_k-eta_k1);
				}
				else d_theta_bar_k=theta_bar_k1;
			}
			d_eta_k=eta_k1;
			d_vertices_theta(0,0)=d_theta_bar_k+0.5*d_eta_k;
			d_vertices_theta(1,0)=d_theta_bar_k-0.5*d_eta_k;
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Eta: "<<d_eta_k);
			ROS_INFO_STREAM("[RAMPC CONTROLLER] Vertices Theta: "<<d_vertices_theta);
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta bar k: "<<d_theta_bar_k);

			//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 7");


			MatrixXf dynamics_constr_A_bar_temp=d_A+(d_B_0+d_B_1*d_theta_bar_k)*d_K;
			MatrixXf dynamics_constr_B_bar_temp=(d_B_0+d_B_1*d_theta_bar_k);
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] Theta update Debug 1.");
			for(int i=0;i<d_N;i++){
				d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs+i*d_num_outputs,d_num_outputs*(d_N+1)+i*d_num_outputs,d_num_outputs,d_num_outputs)=dynamics_constr_A_bar_temp;
				d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs+i*d_num_outputs,d_num_outputs*(d_N+1)*2+i*d_num_inputs,d_num_outputs,d_num_inputs)=dynamics_constr_B_bar_temp;
			}


			for(int i=0;i<d_N;i++){
				for(int j=0;j<d_H_x.rows();j++){
					for(int l=0;l<d_e_l.rows();l++){
						d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs*2+d_N*(d_num_outputs*2+d_num_inputs*2)+i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)+i*d_num_outputs,1,d_num_outputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_K*d_e_l(l,0);
						d_osqp_A.block(2*d_num_outputs+1+d_N*d_num_outputs*2+d_N*(d_num_outputs*2+d_num_inputs*2)+i*d_H_x.rows()*d_e_l.rows()+j*d_e_l.rows()+l,d_num_outputs*(d_N+1)*2+i*d_num_inputs,1,d_num_inputs)=-d_eta_k*d_H_x.row(j)*d_B_1*d_e_l.row(l);
					}
				}
			}
		
			//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 8");

			for(int i=0;i<d_num_inputs;i++){
				d_delta_uss(i)+=(d_cf_weight_in_newtons-9.81/d_theta_bar_k)/4.0;
			}
			d_cf_weight_in_newtons = 9.81/d_theta_bar_k;

			for(int i=0;i<d_N;i++){
				for(int j=0;j<(d_num_outputs*2+d_num_inputs*2);j++){
					temp=MatrixXf::Ones(1,1)-d_F.row(j)*d_z_setpoint_Rampc-d_G.row(j)*d_delta_uss;
					d_osqp_u_new[d_num_outputs*2+1+d_N*d_num_outputs*2+i*(d_num_outputs*2+d_num_inputs*2)+j]=temp(0);
				}
			}
		
			//osqp_update_upper_bound(d_osqp_work,d_osqp_u_new);
		

			//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 9");

		}
		}
	}
	*/
}



void get_disturbance_matrix_full_state(){
	// Disturbance and noise matrices

	d_H_w=MatrixXf::Zero(2*d_num_outputs,d_num_outputs);
	d_h_w=MatrixXf::Zero(2*d_num_outputs,1);
	d_n_bar=MatrixXf::Zero(2*d_num_outputs,1);
	d_n_bar_prev=MatrixXf::Zero(2*d_num_outputs,1);

	d_H_w << 	1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
   -1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	   -1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	  520.8132402299,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	 -520.8132402299,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   26.0406620115,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	  -26.0406620115,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    1.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -1.0000000000;
    d_h_w <<	0.0,
    			0.0,
    			0.0,
    			0.0,
    			1.0,
    			1.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			1.0,
    			1.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0,
    			0.0;



	d_n_bar<<	0.010000,	
        0.010000,	
        0.010000,	
        0.010000,	
        5.208121,	
        5.208121,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.520813,	
        0.520813,	
        0.004400,	
        0.004400,	
        0.004400,	
        0.004400,	
        0.004400,	
        0.004400,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.020000;
	//d_n_bar=0.01*d_n_bar;
	d_n_bar_prev<<	0.012000,	
        0.012000,	
        0.012000,	
        0.012000,	
        6.249722,	
        6.249722,	
        0.024316,	
        0.024316,	
        0.024316,	
        0.024316,	
        0.520813,	
        0.520813,	
        0.006400,	
        0.006400,	
        0.006400,	
        0.006400,	
        0.006400,	
        0.006400,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.020000,	
        0.020000;
	//d_n_bar_prev=0.01*d_n_bar_prev;
}


void get_disturbance_matrix(){
	// Get disturbance and noise matrices
	d_H_w=MatrixXf::Zero(2*d_num_outputs,d_num_outputs);
	d_h_w=MatrixXf::Ones(2*d_num_outputs,1);
	d_n_bar=MatrixXf::Zero(2*d_num_outputs,1);
	d_n_bar_prev=MatrixXf::Zero(2*d_num_outputs,1);

	d_H_w << 	2*520.8132,	0.0,
				2*-520.8132,	0.0,
				0.0,		2*26.0407,
				0.0,		2*-26.0407;	

	d_n_bar<<	5.2081,
				5.2081,
				0.5208,
				0.5208;
	d_n_bar=0.01*d_n_bar;
	d_n_bar_prev<<	6.2497,
					6.2497,
					0.5208,
					0.5208;
	d_n_bar_prev=0.01*d_n_bar_prev;
}

void get_Theta(){
	// Initial Polytope Theta_0
	d_H_theta=MatrixXf::Zero(2*d_N_theta,d_N_theta);
	d_h_theta=MatrixXf::Zero(2*d_N_theta,1);
	d_vertices_theta=MatrixXf::Zero(2*d_N_theta,1);

	d_H_theta<< 1.0,
				-1.0;

	d_vertices_theta<<	d_theta_bar_k+0.5*d_eta_k,
						d_theta_bar_k-0.5*d_eta_k;
	theta_abs_min=d_vertices_theta(1,0);
	d_h_theta<<	d_theta_bar_k+0.5*d_eta_k,
				-(d_theta_bar_k-0.5*d_eta_k);
}

void setup_theta_update_osqp(){
	try{
		// Get relevant matrices for the Theta update LPs
		get_disturbance_matrix();
		get_Theta();
		MatrixXf osqp_theta_P;
		//d_osqp_theta_A;
		MatrixXf osqp_theta_l;
		MatrixXf osqp_theta_u;
		MatrixXf osqp_theta_q_min;
		MatrixXf osqp_theta_q_max;
		osqp_theta_P=MatrixXf::Zero(d_N_theta,d_N_theta);
		d_osqp_theta_A=MatrixXf::Ones(2*d_N_theta+d_num_outputs,d_N_theta);
		osqp_theta_l=Inf_min*MatrixXf::Ones(2*d_N_theta+d_num_outputs,1);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Lower bound: "<<osqp_theta_l);
		osqp_theta_u=MatrixXf::Ones(2*d_N_theta+d_num_outputs,1);
		osqp_theta_q_min=MatrixXf::Ones(d_N_theta,d_N_theta);
		osqp_theta_q_max=-1*MatrixXf::Ones(d_N_theta,d_N_theta);

		osqp_theta_extended_cleanup();
		

		// Convert Eigen matrices to CSC format
		csc* osqp_theta_P_csc = eigen2csc(osqp_theta_P);
		csc* osqp_theta_A_csc = eigen2csc(d_osqp_theta_A);
		
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
	    osqp_theta_max_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_theta_max_data->n = d_osqp_theta_A.cols();
	    osqp_theta_max_data->m = d_osqp_theta_A.rows();
	    osqp_theta_max_data->P = osqp_theta_P_csc;
	    osqp_theta_max_data->q = osqp_theta_q_max_cfloat;
	    osqp_theta_max_data->A = osqp_theta_A_csc;
	    osqp_theta_max_data->l = osqp_theta_l_cfloat;
	    osqp_theta_max_data->u = osqp_theta_u_cfloat;

	    osqp_theta_min_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_theta_min_data->n = d_osqp_theta_A.cols();
	    osqp_theta_min_data->m = d_osqp_theta_A.rows();
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
	    d_osqp_theta_max_work = osqp_setup(osqp_theta_max_data, d_osqp_theta_settings);
	    d_osqp_theta_min_work = osqp_setup(osqp_theta_min_data, d_osqp_theta_settings);
		
	    osqp_solve(d_osqp_theta_max_work);
	    osqp_solve(d_osqp_theta_min_work);

	    //d_RampcOpt_status = d_osqp_work->info->status_val;
	    //float sol_u=d_osqp_work->solution->x[2*(d_N+1)*d_num_outputs]; 
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Status: "<< d_RampcOpt_status);
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Solution: "<< sol_u);

	    // Clear data after setting up to allow subseqeuent setups
	    //osqp_cleanup_data(osqp_theta_min_data);
	    //c_free(osqp_theta_max_data->q);
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


void setup_theta_update_full_state_osqp(){
	try{
		// Get relevant matrices for the Theta update LPs
		get_disturbance_matrix_full_state();
		get_Theta();
		MatrixXf osqp_theta_P;
		//d_osqp_theta_A;
		MatrixXf osqp_theta_l;
		MatrixXf osqp_theta_u;
		MatrixXf osqp_theta_q_min;
		MatrixXf osqp_theta_q_max;
		osqp_theta_P=MatrixXf::Zero(d_N_theta,d_N_theta);
		d_osqp_theta_A=MatrixXf::Ones(2*d_N_theta+d_num_outputs,d_N_theta);
		osqp_theta_l=Inf_min*MatrixXf::Ones(2*d_N_theta+d_num_outputs,1);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Lower bound: "<<osqp_theta_l);
		osqp_theta_u=MatrixXf::Ones(2*d_N_theta+d_num_outputs,1);
		osqp_theta_q_min=MatrixXf::Ones(d_N_theta,d_N_theta);
		osqp_theta_q_max=-1*MatrixXf::Ones(d_N_theta,d_N_theta);

		osqp_theta_extended_cleanup();
		

		// Convert Eigen matrices to CSC format
		csc* osqp_theta_P_csc = eigen2csc(osqp_theta_P);
		csc* osqp_theta_A_csc = eigen2csc(d_osqp_theta_A);
		
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
	    osqp_theta_max_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_theta_max_data->n = d_osqp_theta_A.cols();
	    osqp_theta_max_data->m = d_osqp_theta_A.rows();
	    osqp_theta_max_data->P = osqp_theta_P_csc;
	    osqp_theta_max_data->q = osqp_theta_q_max_cfloat;
	    osqp_theta_max_data->A = osqp_theta_A_csc;
	    osqp_theta_max_data->l = osqp_theta_l_cfloat;
	    osqp_theta_max_data->u = osqp_theta_u_cfloat;

	    osqp_theta_min_data = (OSQPData*) c_malloc(sizeof(OSQPData));
	    osqp_theta_min_data->n = d_osqp_theta_A.cols();
	    osqp_theta_min_data->m = d_osqp_theta_A.rows();
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
	    d_osqp_theta_max_work = osqp_setup(osqp_theta_max_data, d_osqp_theta_settings);
	    d_osqp_theta_min_work = osqp_setup(osqp_theta_min_data, d_osqp_theta_settings);
		
	    osqp_solve(d_osqp_theta_max_work);
	    osqp_solve(d_osqp_theta_min_work);

	    //d_RampcOpt_status = d_osqp_work->info->status_val;
	    //float sol_u=d_osqp_work->solution->x[2*(d_N+1)*d_num_outputs]; 
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Status: "<< d_RampcOpt_status);
	    //ROS_INFO_STREAM("[RAMPC CONTROLLER] Solution: "<< sol_u);

	    // Clear data after setting up to allow subseqeuent setups
	    //osqp_cleanup_data(osqp_theta_min_data);
	    //c_free(osqp_theta_max_data->q);
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
	// Matrices for the tube polytopes
	int H_x_size=10;
	d_H_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_s=MatrixXf::Zero(H_x_size,1);
	d_c=MatrixXf::Zero(6,1);
	d_e_l=MatrixXf::Zero(2,1);


	/*
	d_H_x << 	-4.4652, -1.1794,
				5.9192, 1.5634,
				2.2426, -0.1519,
				-2.9729, 0.2013,
				5.2618, 0.5232,
				4.9847, 0.4500,
				-6.9751, -0.6936,
				-6.6078, -0.5966;
	*/

	d_H_x <<	-4.5506,	-1.2259,
				6.0323,		1.6251,
				1.5656,		-0.3367,
				-2.0754,	0.4463,
				6.0527,		0.6275,
				5.1221,		0.3768,
				-8.0235,	-0.8318,
				-6.7899,	-0.4994,
				-7.1644,	-1.0511,
				-7.5888,	-0.9598;

	d_H_xf_x=d_H_x;

	d_H_xf_s=MatrixXf::Ones(H_x_size,1);

	/*
	d_c <<	0.4868,
			0.3672,
			0.1523,
			0.2019,
			1.0,
			1.0;
	*/
	d_c << 	0.4404,
			0.3322,
			0.1337,
			0.1772,
			1.0,
			1.0;


	d_w_bar=0.074;

	d_theta_bar_k=32.032;
	d_theta_hat_k=d_theta_bar_k;
	d_eta_k=10.01;
	d_rho_theta_k=0.6;
	d_e_l << 0.5,
			 - 0.5;
	d_mu=1508.9;


}

void get_tube_params_all_rotors(){
	// Matrices for the tube polytopes
	int H_x_size=17;
	d_H_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_s=MatrixXf::Zero(H_x_size,1);
	d_c=MatrixXf::Zero(6,1);
	d_e_l=MatrixXf::Zero(2,1);


	/*
	d_H_x << 	-4.4652, -1.1794,
				5.9192, 1.5634,
				2.2426, -0.1519,
				-2.9729, 0.2013,
				5.2618, 0.5232,
				4.9847, 0.4500,
				-6.9751, -0.6936,
				-6.6078, -0.5966;
	*/

	d_H_x <<	-3.7183,	-1.3108,
				4.9290,		1.7376,
				-1.2007,	-0.9544,
				1.5916,		1.2652,
				1.2783,		-0.4797,
				-1.6945,	0.6359,
				5.2626,		0.7087,
				3.3308,		0.0277,
				-6.9762,	-0.9395,
				-4.4153,	-0.0367,
				5.2952,		0.9807,
				4.5598,		0.4454,
				4.6713,		0.4848,
				-7.0193,	-1.3000,
				-6.0445,	-0.5905,
				-6.1924,	-0.6426,
				-5.9503,	-1.4225;

	d_H_xf_x=d_H_x;

	d_H_xf_s=MatrixXf::Ones(H_x_size,1);

	/*
	d_c <<	0.4868,
			0.3672,
			0.1523,
			0.2019,
			1.0,
			1.0;
	*/
	d_c << 	0.4438,
			0.3348,
			0.1041,
			0.1380,
			1.0,
			1.0;


	d_w_bar=0.0762;

	d_theta_bar_k=26.6266;
	d_theta_hat_k=d_theta_bar_k;
	d_eta_k=20.8208;
	d_rho_theta_k=0.7;
	d_e_l << 0.5,
			 - 0.5;
	d_mu=1508.9;


}


void get_tube_params_full_state(){
	// matrices for the tube polytopes
	int H_x_size=77;
	d_H_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_x=MatrixXf::Zero(H_x_size,d_num_outputs);
	d_H_xf_s=MatrixXf::Zero(H_x_size,1);
	d_c=MatrixXf::Zero(2*d_num_outputs+2*d_num_inputs,1);
	d_e_l=MatrixXf::Zero(2,1);

	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 1");

	d_H_x <<	  0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.6366197724,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -0.6366197724,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.6366197724,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -0.6366197724,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.6366197724,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	   -0.6366197724,	
    1.6214000205,	   -0.8611049583,	   -5.6479557467,	    0.7548127696,	   -0.4008661818,	   -1.5556472941,	    0.7277391015,	    1.3703133568,	    0.6705102128,	    0.0627093560,	    0.1180812174,	    0.2095340497,	
   -1.5780264556,	    0.8380698089,	    5.4968690487,	   -0.7346210092,	    0.3901427359,	    1.5140326598,	   -0.7082715804,	   -1.3336565327,	   -0.6525736038,	   -0.0610318377,	   -0.1149224637,	   -0.2039288699,	
   -1.5543828930,	   -0.4768633920,	   -5.6479557567,	   -0.7236149125,	   -0.2219797797,	   -1.5556472976,	    0.4029761932,	   -1.3136762365,	   -0.6645844040,	    0.0347235292,	   -0.1132008033,	   -0.2076822361,	
    1.5128020823,	    0.4641069686,	    5.4968690583,	    0.7042577162,	    0.2160416681,	    1.5140326632,	   -0.3921963031,	    1.2785344943,	    0.6468063144,	   -0.0337946510,	    0.1101726040,	    0.2021265935,	
   -0.8283588416,	    0.5438764945,	   -5.6479557468,	   -0.3855988011,	    0.2531762702,	   -1.5556472940,	   -0.4596114631,	   -0.7000081773,	    0.6498138195,	   -0.0396038541,	   -0.0603181990,	    0.2030664368,	
    0.8061996732,	   -0.5293274245,	    5.4968690487,	    0.3752837682,	   -0.2464036310,	    1.5140326597,	    0.4473165406,	    0.6812824775,	   -0.6324308532,	    0.0385444238,	    0.0587046457,	   -0.1976342700,	
    0.7613455092,	    0.7940873738,	   -5.6479557567,	    0.3544022474,	    0.3696681252,	   -1.5556472977,	   -0.6711017572,	    0.6433727648,	   -0.6557396854,	   -0.0578289275,	    0.0554378678,	   -0.2049182654,	
   -0.7409789936,	   -0.7728449908,	    5.4968690583,	   -0.3449217437,	   -0.3597792488,	    1.5140326632,	    0.6531493240,	   -0.6261621011,	    0.6381981981,	    0.0562819640,	   -0.0539548667,	    0.1994365609,	
    0.0000435485,	   43.9007041585,	    0.0000000004,	    0.0000160273,	   20.4364955903,	    0.0000000008,	  -37.1004486373,	    0.0000209797,	    0.0007975721,	   -2.1358857709,	    0.0000011721,	    0.0002492288,	
   -0.0000435485,	  -43.9007041585,	   -0.0000000004,	   -0.0000160273,	  -20.4364955903,	   -0.0000000008,	   37.1004486373,	   -0.0000209797,	   -0.0007975721,	    2.1358857709,	   -0.0000011721,	   -0.0002492288,	
  -43.8992594415,	   -0.0000632199,	    0.0000000013,	  -20.4359669609,	   -0.0000226841,	    0.0000000014,	    0.0000301906,	  -37.0997708150,	    0.0020528526,	    0.0000016038,	   -2.1358496944,	    0.0006414833,	
   43.8992594415,	    0.0000632199,	   -0.0000000013,	   20.4359669609,	    0.0000226841,	   -0.0000000014,	   -0.0000301906,	   37.0997708150,	   -0.0020528526,	   -0.0000016038,	    2.1358496944,	   -0.0006414833,	
    0.0005138150,	   -0.0002018315,	   -0.0000000338,	    0.0001100368,	   -0.0000428408,	   -0.0000000123,	    0.0000392696,	    0.0001020632,	   -4.4954430682,	   -0.0000005572,	   -0.0000011871,	   -0.3437903636,	
   -0.0005138150,	    0.0002018315,	    0.0000000338,	   -0.0001100368,	    0.0000428408,	    0.0000000123,	   -0.0000392696,	   -0.0001020632,	    4.4954430682,	    0.0000005572,	    0.0000011871,	    0.3437903636,	
   -5.4399984566,	    2.8891288516,	    7.6748673667,	   -2.2622032461,	    1.2014240010,	    1.1726046729,	   -1.7861970552,	   -3.3633138785,	   -0.3616326784,	   -0.0891026074,	   -0.1677754745,	   -0.0012583031,	
   -5.4399976808,	    2.8891279353,	    3.0564546760,	   -2.2622029796,	    1.2014236809,	   -0.0994699553,	   -1.7861966311,	   -3.3633135293,	   -0.3616326901,	   -0.0891025862,	   -0.1677754575,	   -0.0012583062,	
    5.2944747590,	   -2.8118426691,	   -7.4695593897,	    2.2016877544,	   -1.1692850833,	   -1.1412366920,	    1.7384150563,	    3.2733429206,	    0.3519587557,	    0.0867190514,	    0.1632873652,	    0.0012246426,	
    5.2944740039,	   -2.8118417774,	   -2.9746924128,	    2.2016874951,	   -1.1692847717,	    0.0968090657,	    1.7384146436,	    3.2733425808,	    0.3519587670,	    0.0867190308,	    0.1632873487,	    0.0012246456,	
    5.2151583706,	    1.5998102584,	    7.6748674109,	    2.1687050841,	    0.6652555435,	    1.1726046837,	   -0.9890475597,	    3.2243065910,	    0.3585704727,	   -0.0493363654,	    0.1608413296,	    0.0012889971,	
    5.2151591464,	    1.5998093421,	    3.0564547097,	    2.1687053505,	    0.6652552233,	   -0.0994699475,	   -0.9890471357,	    3.2243069401,	    0.3585704610,	   -0.0493363442,	    0.1608413466,	    0.0012889940,	
   -5.0756492998,	   -1.5570142344,	   -7.4695594327,	   -2.1106907325,	   -0.6474595005,	   -1.1412367025,	    0.9625898577,	   -3.1380541736,	   -0.3489784661,	    0.0480165837,	   -0.1565387135,	   -0.0012545155,	
   -5.0756500549,	   -1.5570133427,	   -2.9746924455,	   -2.1106909918,	   -0.6474591889,	    0.0968090580,	    0.9625894450,	   -3.1380545134,	   -0.3489784547,	    0.0480165631,	   -0.1565387300,	   -0.0012545125,	
    2.7789099550,	   -1.8246389163,	    7.6748673660,	    1.1555675736,	   -0.7587497799,	    1.1726046725,	    1.1280494330,	    1.7180100678,	   -0.3511595502,	    0.0562702563,	    0.0856981001,	   -0.0014348483,	
    2.7789107308,	   -1.8246398325,	    3.0564546755,	    1.1555678400,	   -0.7587501001,	   -0.0994699557,	    1.1280498570,	    1.7180104169,	   -0.3511595619,	    0.0562702775,	    0.0856981171,	   -0.0014348514,	
   -2.7045722037,	    1.7758285712,	   -7.4695593890,	   -1.1246553468,	    0.7384527018,	   -1.1412366916,	   -1.0978733353,	   -1.6720521177,	    0.3417657909,	   -0.0547649883,	   -0.0834056170,	    0.0013964651,	
   -2.7045729588,	    1.7758294629,	   -2.9746924122,	   -1.1246556061,	    0.7384530134,	    0.0968090661,	   -1.0978737480,	   -1.6720524575,	    0.3417658022,	   -0.0547650089,	   -0.0834056335,	    0.0013964681,	
   -2.5540807434,	   -2.6642869555,	    7.6748674113,	   -1.0620732116,	   -1.1079250561,	    1.1726046839,	    1.6471888832,	   -1.5790078015,	    0.3542219399,	    0.0821684009,	   -0.0787641997,	    0.0014041984,	
   -2.5540799675,	   -2.6642878717,	    3.0564547100,	   -1.0620729452,	   -1.1079253762,	   -0.0994699473,	    1.6471893073,	   -1.5790074524,	    0.3542219282,	    0.0821684220,	   -0.0787641827,	    0.0014041954,	
    2.4857573281,	    2.5930154483,	   -7.4695594331,	    1.0336620233,	    1.0782872993,	   -1.1412367027,	   -1.6031254485,	    1.5367682576,	   -0.3447462595,	   -0.0799703397,	    0.0766572032,	   -0.0013666352,	
    2.4857565730,	    2.5930163400,	   -2.9746924458,	    1.0336617639,	    1.0782876109,	    0.0968090578,	   -1.6031258612,	    1.5367679178,	   -0.3447462481,	   -0.0799703603,	    0.0766571867,	   -0.0013666322,	
   -0.0001541502,	  -74.1208451694,	   -0.0000000068,	   -0.0000573846,	  -27.1876813030,	   -0.0000000029,	   29.2257943748,	   -0.0000774831,	   -0.0031065074,	    0.9825773438,	   -0.0000043688,	   -0.0008378327,	
    0.0001541502,	   74.1208451694,	    0.0000000068,	    0.0000573846,	   27.1876813030,	    0.0000000029,	  -29.2257943748,	    0.0000774831,	    0.0031065074,	   -0.9825773438,	    0.0000043688,	    0.0008378327,	
   74.1159186676,	    0.0002171252,	   -0.0000000131,	   27.1858715778,	    0.0000792012,	   -0.0000000051,	   -0.0001073054,	   29.2234119550,	   -0.0079956526,	   -0.0000057300,	    0.9824482393,	   -0.0021564406,	
  -74.1159186676,	   -0.0002171252,	    0.0000000131,	  -27.1858715778,	   -0.0000792012,	    0.0000000051,	    0.0001073054,	  -29.2234119550,	    0.0079956526,	    0.0000057300,	   -0.9824482393,	    0.0021564406,	
    0.0006607460,	   -0.0002658142,	    0.0000000608,	    0.0002477156,	   -0.0000997911,	    0.0000000077,	    0.0001467583,	    0.0003640801,	   -5.0647550918,	    0.0000087152,	    0.0000216344,	   -0.5635852453,	
   -0.0006607460,	    0.0002658142,	   -0.0000000608,	   -0.0002477156,	    0.0000997911,	   -0.0000000077,	   -0.0001467583,	   -0.0003640801,	    5.0647550918,	   -0.0000087152,	   -0.0000216344,	    0.5635852453,	
    2.5025878157,	   -1.3292005196,	    6.1867264760,	    0.7087095454,	   -0.3764328434,	    0.6445763745,	    0.2513233176,	    0.4730857455,	   -0.5944883528,	    0.0012430116,	    0.0023316984,	   -0.0597965377,	
    2.5025878653,	   -1.3292005782,	    5.8914196247,	    0.7087095624,	   -0.3764328639,	    0.5632383958,	    0.2513233447,	    0.4730857678,	   -0.5944883536,	    0.0012430130,	    0.0023316994,	   -0.0597965379,	
   -2.4356418716,	    1.2936434921,	   -6.0212272906,	   -0.6897510779,	    0.3663630061,	   -0.6273335135,	   -0.2446002461,	   -0.4604303766,	    0.5785853808,	   -0.0012097602,	   -0.0022693238,	    0.0581969392,	
   -2.4356419199,	    1.2936435492,	   -5.7338201006,	   -0.6897510945,	    0.3663630261,	   -0.5481713816,	   -0.2446002725,	   -0.4604303984,	    0.5785853815,	   -0.0012097615,	   -0.0022693249,	    0.0581969394,	
   -2.3991744494,	   -0.7358554173,	    6.1867264465,	   -0.6794255175,	   -0.2083863610,	    0.6445763702,	    0.1390898104,	   -0.4535432179,	    0.5889721439,	    0.0006846931,	   -0.0022358891,	    0.0592084112,	
   -2.3991743998,	   -0.7358554758,	    5.8914196186,	   -0.6794255005,	   -0.2083863815,	    0.5632383979,	    0.1390898375,	   -0.4535431955,	    0.5889721432,	    0.0006846944,	   -0.0022358880,	    0.0592084110,	
    2.3349948356,	    0.7161708361,	   -5.7338200946,	    0.6612504014,	    0.2028119025,	   -0.5481713836,	   -0.1353690887,	    0.4414106033,	   -0.5732167336,	   -0.0006663784,	    0.0021760764,	   -0.0576245453,	
   -1.2779471391,	    0.8392714889,	    6.1867264802,	   -0.3618738376,	    0.2376713563,	    0.6445763757,	   -0.1586335583,	   -0.2414535595,	   -0.5747869976,	   -0.0007805959,	   -0.0011807131,	   -0.0576433559,	
   -1.2779470894,	    0.8392714303,	    5.8914196277,	   -0.3618738206,	    0.2376713358,	    0.5632383967,	   -0.1586335312,	   -0.2414535372,	   -0.5747869983,	   -0.0007805946,	   -0.0011807120,	   -0.0576433561,	
    1.2437611747,	   -0.8168203997,	   -6.0212272947,	    0.3521934638,	   -0.2313134842,	   -0.6273335146,	    0.1543900016,	    0.2349945108,	    0.5594110503,	    0.0007597145,	    0.0011491282,	    0.0561013565,	
    1.2437611265,	   -0.8168203427,	   -5.7338201035,	    0.3521934472,	   -0.2313134642,	   -0.5481713824,	    0.1543899752,	    0.2349944891,	    0.5594110511,	    0.0007597131,	    0.0011491271,	    0.0561013567,	
    1.1745322641,	    1.2257853502,	    6.1867264444,	    0.3325893741,	    0.3471480487,	    0.6445763696,	   -0.2317797921,	    0.2219104809,	    0.5803032207,	   -0.0011471213,	    0.0010848761,	    0.0582314907,	
    1.1745323137,	    1.2257852916,	    5.8914196171,	    0.3325893912,	    0.3471480282,	    0.5632383975,	   -0.2317797650,	    0.2219105032,	    0.5803032199,	   -0.0011471200,	    0.0010848771,	    0.0582314905,	
   -1.1431127188,	   -1.1929947496,	   -6.0212272599,	   -0.3236923798,	   -0.3378615998,	   -0.6273335087,	    0.2255795234,	   -0.2159742230,	   -0.5647797107,	    0.0011164350,	   -0.0010558549,	   -0.0566737583,	
   -1.1431127670,	   -1.1929946926,	   -5.7338200931,	   -0.3236923964,	   -0.3378615799,	   -0.5481713832,	    0.2255794970,	   -0.2159742447,	   -0.5647797100,	    0.0011164337,	   -0.0010558559,	   -0.0566737581,	
    0.0001108759,	  -55.7771269226,	    0.0000000125,	    0.0000434996,	  -26.1239987589,	    0.0000000016,	   35.8996121893,	    0.0000638780,	    0.0019697625,	    1.5743779414,	    0.0000035539,	    0.0003193615,	
   -0.0001108759,	   55.7771269226,	   -0.0000000137,	   -0.0000434996,	   26.1239987589,	   -0.0000000016,	  -35.8996121893,	   -0.0000638780,	   -0.0019697625,	   -1.5743779414,	   -0.0000035539,	   -0.0003193615,	
   -0.0001108759,	   55.7771269226,	   -0.0000000210,	   -0.0000434996,	   26.1239987589,	   -0.0000000040,	  -35.8996121893,	   -0.0000638780,	   -0.0019697625,	   -1.5743779414,	   -0.0000035539,	   -0.0003193615,	
   -0.0001108759,	   55.7771269226,	   -0.0000000125,	   -0.0000434996,	   26.1239987589,	   -0.0000000016,	  -35.8996121893,	   -0.0000638780,	   -0.0019697625,	   -1.5743779414,	   -0.0000035539,	   -0.0003193615,	
   55.7800470659,	   -0.0001301399,	    0.0000000209,	   26.1251214041,	   -0.0000506145,	    0.0000000027,	    0.0000720509,	   35.9012521864,	    0.0050694657,	    0.0000038170,	    1.5744697000,	    0.0008218738,	
  -55.7800470659,	    0.0001301399,	   -0.0000000231,	  -26.1251214041,	    0.0000506145,	   -0.0000000026,	   -0.0000720509,	  -35.9012521864,	   -0.0050694657,	   -0.0000038170,	   -1.5744697000,	   -0.0008218738,	
    0.0000476841,	   15.6057522307,	    0.0000000056,	    0.0000165880,	   -2.2961786087,	    0.0000000035,	   10.7950105410,	    0.0000224137,	    0.0030002188,	    0.7011653312,	    0.0000016212,	    0.0007721837,	
   -0.0000476841,	  -15.6057522307,	   -0.0000000112,	   -0.0000165880,	    2.2961786087,	   -0.0000000033,	  -10.7950105410,	   -0.0000224137,	   -0.0030002188,	   -0.7011653312,	   -0.0000016212,	   -0.0007721837,	
  -15.6036393183,	   -0.0001102955,	    0.0000000084,	    2.2968982906,	   -0.0000396721,	    0.0000000056,	    0.0000551110,	   10.7959084380,	    0.0077225700,	    0.0000031678,	    0.7012200321,	    0.0019875783,	
   15.6036393183,	    0.0001102955,	    0.0000000119,	   -2.2968982906,	    0.0000396721,	    0.0000000002,	   -0.0000551110,	  -10.7959084380,	   -0.0077225700,	   -0.0000031678,	   -0.7012200321,	   -0.0019875783,	
   15.6036393183,	    0.0001102955,	   -0.0000000129,	   -2.2968982906,	    0.0000396721,	   -0.0000000036,	   -0.0000551110,	  -10.7959084380,	   -0.0077225700,	   -0.0000031678,	   -0.7012200321,	   -0.0019875783,	
   15.6036393183,	    0.0001102955,	   -0.0000000207,	   -2.2968982906,	    0.0000396721,	   -0.0000000058,	   -0.0000551110,	  -10.7959084380,	   -0.0077225700,	   -0.0000031678,	   -0.7012200321,	   -0.0019875783,	
    0.0000162664,	   74.3612923405,	   -0.0000000181,	    0.0000013374,	   21.2825038127,	   -0.0000000001,	  -19.1160498969,	   -0.0000067705,	    0.0004260883,	   -0.5532703780,	   -0.0000004140,	    0.0003575374,	
   -0.0000162664,	  -74.3612923405,	    0.0000000073,	   -0.0000013374,	  -21.2825038127,	   -0.0000000009,	   19.1160498969,	    0.0000067705,	   -0.0004260883,	    0.5532703780,	    0.0000004140,	   -0.0003575374,	
  -74.3599502661,	   -0.0000356402,	   -0.0000000297,	  -21.2821256130,	   -0.0000079300,	   -0.0000000002,	    0.0000054796,	  -19.1158176659,	    0.0010969815,	    0.0000003217,	   -0.5532645181,	    0.0009203320,	
   74.3599502661,	    0.0000356402,	    0.0000000294,	   21.2821256130,	    0.0000079300,	    0.0000000000,	   -0.0000054796,	   19.1158176659,	   -0.0010969815,	   -0.0000003217,	    0.5532645181,	   -0.0009203320,	
    0.0000181034,	   85.7824792499,	   -0.0000000160,	    0.0000043640,	   30.1035393972,	   -0.0000000023,	  -34.4139015966,	   -0.0000031441,	   -0.0025077280,	   -1.3297632245,	   -0.0000007586,	   -0.0003386632,	
   -0.0000181034,	  -85.7824792499,	    0.0000000160,	   -0.0000043640,	  -30.1035393972,	    0.0000000023,	   34.4139015966,	    0.0000031441,	    0.0025077280,	    1.3297632245,	    0.0000007586,	    0.0003386632,	
  -85.7819014784,	    0.0000174347,	   -0.0000000264,	  -30.1033327766,	    0.0000108227,	   -0.0000000038,	   -0.0000228307,	  -34.4138266351,	   -0.0064546171,	   -0.0000015606,	   -1.3297755338,	   -0.0008716628,	
   85.7819014784,	   -0.0000174347,	    0.0000000264,	   30.1033327766,	   -0.0000108227,	    0.0000000038,	    0.0000228307,	   34.4138266351,	    0.0064546171,	    0.0000015606,	    1.3297755338,	    0.0008716628;	


	d_H_xf_x=d_H_x;

	d_H_xf_s=MatrixXf::Ones(H_x_size,1);

		//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 2");


	d_c << 	0.1892399988,	
    0.1892421622,	
    0.1890288952,	
    0.1890280607,	
    0.4102764145,	
    0.4215527554,	
    0.0709558415,	
    0.0709555875,	
    0.0708984918,	
    0.0709007067,	
    0.1682223150,	
    0.1637222481,	
    0.2239858231,	
    0.2239871141,	
    0.2240754428,	
    0.2240769962,	
    0.2180902965,	
    0.2180912023,	
    0.9999999800,	
    0.9999999800,	
    0.9999999800,	
    0.9999999800,	
    0.9999999800,	
    0.9999999800,	
    0.9999999801,	
    0.9999999801,	
    0.9999996001,	
    0.9999999801,	
    0.9999999801,	
    0.9999996001,	
    0.9999999801,	
    0.9999996001;

		//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 3");



	d_w_bar=0.0664063671;

	d_theta_bar_k=32.0320;
	d_theta_hat_k=d_theta_bar_k;
	d_eta_k=10.0100;
	d_rho_theta_k=0.6;
	d_e_l << 0.5,
			 - 0.5;
	d_mu=2014.9;


}


void get_dynamics_matrix(){
	// Dynamics Matrices for theta update and constraints
	d_A=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	d_A_update=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	d_A_const_update=MatrixXf::Zero(d_num_outputs,1);
	d_B_0=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	d_B_0_update=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	d_B_1=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	d_B_1_update=MatrixXf::Zero(d_num_outputs,d_num_inputs);
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
 	float d_T_update=1.0/yaml_control_frequency;
 	d_A_const_update <<	0.0,
 						-9.81*d_T_update;
 	d_A_update=d_eye+d_T_update*d_A_quad;
 	d_B_0_update=d_T_update*d_B_0;
 	d_B_1_update=d_T_update*B_1;

 	
}


void get_dynamics_matrix_full_state(){
	// Full state dynamics matrices for constraints and theta update
	d_A=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	d_A_update=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	d_A_const_update=MatrixXf::Zero(d_num_outputs,1);
	d_B_0=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	d_B_0_update=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	d_B_1=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	d_B_1_update=MatrixXf::Zero(d_num_outputs,d_num_inputs);
	MatrixXf d_A_quad=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 22");

	MatrixXf d_eye=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	d_A_quad<< 	0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        9.810000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	       -9.810000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000;
	d_eye = MatrixXf::Identity(d_num_outputs,d_num_outputs);
 	d_A=d_eye+d_T_s*d_A_quad;
 	MatrixXf B_0=MatrixXf::Zero(d_num_outputs,d_num_inputs);
 	B_0 << 0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
    0.0000000000,	    0.0000000000,	    0.0000000000,	    0.0000000000,	
 -1906.733735718,	 -2028.460977411,	  1923.291292942,	  2011.903420188,	
 -1042.955644235,	  1148.125328705,	  1067.455517955,	 -1172.625202425,	
   -92.612151424,	   182.975608751,	  -316.684540780,	   226.321083453;
 	d_B_0=d_T_s*B_0;
//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 23");
 	MatrixXf B_1=MatrixXf::Zero(d_num_outputs,d_num_inputs);
 	B_1 << 0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        1.000000,	        1.000000,	        1.000000,	        1.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	
    	0.000000,	    	0.000000,	     	0.000000,	     	0.000000,	
    	0.000000,	     	0.000000,	     	0.000000,	    	0.000000,	
      	0.000000,	      	0.000000,	     	0.000000,	      	0.000000;
      //ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 26");
 	d_B_1=d_T_s*B_1;
 	float d_T_update=1.0/yaml_control_frequency;
 	d_A_const_update <<	0.0,
 						0.0,
 						0.0,
 						0.0,
 						0.0,
 						-9.81*d_T_update,
 						0.0,
 						0.0,
 						0.0,
 						0.0,
 						0.0,
 						0.0;


 	//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 25");
 	MatrixXf d_A_quad_update=MatrixXf::Zero(d_num_outputs,d_num_outputs);
	d_A_quad_update<< 	0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        1.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	
        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000,	        0.000000;
        //ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug 24");
 	d_A_update=d_eye+d_T_update*d_A_quad_update;
 	d_B_0_update=d_T_update*B_0;
 	d_B_1_update=d_T_update*B_1;

 	
}




// Get variable lengths
void get_variable_lengths()
{

	d_T_s=0.1; //time step

	// Input and output lengts
	d_num_inputs=1;
	

	d_num_outputs=2;
	
	d_Nuini = d_num_inputs * d_Tini;
	
	d_Nyini = d_num_outputs * d_Tini;

	d_uini = MatrixXf::Zero(d_Nuini, 1);
    d_yini = MatrixXf::Zero(d_Nyini, 1);

    // Number of parameters
    d_N_theta=1;
	
	// Future inputs vector (uf) length
	d_Nuf = d_num_inputs * d_N;
	// Future output vector (yf) length 
	d_Nyf = d_num_outputs * (d_N+1);
	//Number of tube dilation coefficients
	d_Nsf= d_N+1;
	// Optimization decision vector size
	d_num_opt_vars = 2*d_Nyf+d_Nuf+d_Nsf;

	// NUMBER OF DECISION VARIABLES
		d_col_num=d_num_outputs*(d_N+1)+d_num_outputs*(d_N+1)+d_num_inputs*d_N+(d_N+1);

		d_uf_start_i=d_Nyf*2;

	s_Rampc_mutex.lock();
 	d_current_input=MatrixXf::Zero(d_num_inputs,1);
 	s_current_input=MatrixXf::Zero(d_num_inputs,1);
 	m_current_input=MatrixXf::Zero(d_num_inputs,1);
 	d_previous_input=MatrixXf::Zero(d_num_inputs,1);
 	s_previous_input=MatrixXf::Zero(d_num_inputs,1);
 	m_previous_input=MatrixXf::Zero(d_num_inputs,1);
 	s_Rampc_mutex.unlock();
}



void get_variable_lengths_full_state()
{

	d_T_s=0.1; //time step

	// number of inputs and outputs
	d_num_inputs=4;
	
	d_num_outputs=12;
	//d_num_theta=1;
	// Number of block rows for Hankel matrices
	//d_num_block_rows = d_Tini + d_N + 1;
	// Previous inputs vector (uini) length
	d_Nuini = d_num_inputs * d_Tini;
	// Previous outputs vector (yini) length
	d_Nyini = d_num_outputs * d_Tini;

	d_uini = MatrixXf::Zero(d_Nuini, 1);
    d_yini = MatrixXf::Zero(d_Nyini, 1);

    // Number of uncertain parameters
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

	
 	s_Rampc_mutex.lock();
 	d_current_input=MatrixXf::Zero(d_num_inputs,1);
 	s_current_input=MatrixXf::Zero(d_num_inputs,1);
 	m_current_input=MatrixXf::Zero(d_num_inputs,1);
 	d_previous_input=MatrixXf::Zero(d_num_inputs,1);
 	s_previous_input=MatrixXf::Zero(d_num_inputs,1);
 	m_previous_input=MatrixXf::Zero(d_num_inputs,1);
 	s_Rampc_mutex.unlock();
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


void get_cost_matrices_all_rotors()
{
	// Output cost and terminal output cost matrix
	d_Q = MatrixXf::Zero(d_num_outputs, d_num_outputs);
	d_P = MatrixXf::Zero(d_num_outputs, d_num_outputs);
	/*
	for (int i = 0; i < 2; i++)
	{
		d_Q(i,i) = d_Q_vec[i];
		//d_P(i,i) = d_P_vec[i];
	}
	*/
	d_Q<<	1.0, 0.0,
			0.0, 0.3;

	/*
	d_P(0,0)=d_P_vec[0];
	d_P(0,1)=d_P_vec[1];
	d_P(1,0)=d_P_vec[1];
	d_P(1,1)=d_P_vec[2];
	*/
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
	/*
	for (int i = 0; i < d_num_inputs; i++)
		d_R(i,i) = d_R_vec[i];
		*/
	d_R<< 0.4;

	d_P << 	76.2148, 12.5032,
			12.5032, 3.3180;
}



void get_cost_matrices_full_state()
{
	// Output cost and terminal output cost matrix
	d_Q = MatrixXf::Zero(d_num_outputs, d_num_outputs);
	d_P = MatrixXf::Zero(d_num_outputs, d_num_outputs);
	/*
	for (int i = 0; i < 2; i++)
	{
		d_Q(i,i) = d_Q_vec[i];
		//d_P(i,i) = d_P_vec[i];
	}
	*/
	d_Q(0,0)=0.9;
	d_Q(1,1)=0.9;
	d_Q(2,2)=1.0;
	d_Q(3,3)=0.3;
	d_Q(4,4)=0.3;
	d_Q(5,5)=0.3;
	d_Q(6,6)=0.05;
	d_Q(7,7)=0.05;
	d_Q(8,8)=0.1;
	d_Q(9,9)=0.02;
	d_Q(10,10)=0.02;
	d_Q(11,11)=0.02;
			

	/*
	d_P(0,0)=d_P_vec[0];
	d_P(0,1)=d_P_vec[1];
	d_P(1,0)=d_P_vec[1];
	d_P(1,1)=d_P_vec[2];
	*/
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
	/*
	for (int i = 0; i < d_num_inputs; i++)
		d_R(i,i) = d_R_vec[i]; 
		*/
	d_R(0,0)=0.4;
	d_R(1,1)=0.4;
	d_R(2,2)=0.4;
	d_R(3,3)=0.4;

	d_P << 	54229.2279187030,	    0.0992039942,	   -0.0000129899,	17159.2068949463,	    0.0239649948,	   -0.0000020037,	   -0.0229258231,	20437.1514332923,	   -0.0005057228,	   -0.0004538410,	  852.9362829798,	   -0.0000638293,	
    0.0992039942,	54232.1430739114,	    0.0000191476,	    0.0253572109,	17159.9226388950,	    0.0000028774,	-20437.8457202665,	    0.0264007607,	    0.0001517089,	 -852.9553803628,	    0.0008154779,	    0.0000175069,	
   -0.0000129899,	    0.0000191476,	  177.5427490627,	   -0.0000014998,	    0.0000029058,	   20.5585998629,	   -0.0000008279,	    0.0000004392,	   -0.0000001166,	    0.0000002101,	    0.0000002211,	   -0.0000000423,	
17159.2068949464,	    0.0253572109,	   -0.0000014998,	 5738.3524446442,	    0.0055302742,	   -0.0000002668,	   -0.0046151401,	 7079.3576625229,	    0.0000396724,	   -0.0000116423,	  311.7685346154,	    0.0000183803,	
    0.0239649948,	17159.9226388951,	    0.0000029058,	    0.0055302742,	 5738.5132569047,	    0.0000004849,	-7079.4946156913,	    0.0051329842,	   -0.0000283631,	 -311.7707478024,	    0.0001047917,	   -0.0000093300,	
   -0.0000020037,	    0.0000028774,	   20.5585998629,	   -0.0000002668,	    0.0000004849,	    3.6067060346,	   -0.0000001780,	    0.0000000361,	   -0.0000000384,	    0.0000000343,	    0.0000000375,	   -0.0000000108,	
   -0.0229258231,	-20437.8457202666,	   -0.0000008279,	   -0.0046151401,	-7079.4946156913,	   -0.0000001780,	 9029.4386820912,	   -0.0034546041,	    0.0001293783,	  414.9884992086,	    0.0000092942,	    0.0000305088,	
20437.1514332924,	    0.0264007607,	    0.0000004392,	 7079.3576625229,	    0.0051329842,	    0.0000000361,	   -0.0034546041,	 9029.3496230294,	    0.0002894366,	    0.0001015612,	  414.9896571827,	    0.0000707823,	
   -0.0005057228,	    0.0001517089,	   -0.0000001166,	    0.0000396724,	   -0.0000283631,	   -0.0000000384,	    0.0001293783,	    0.0002894366,	   46.1124071511,	    0.0000143645,	    0.0000351194,	    7.2050530475,	
   -0.0004538410,	 -852.9553803629,	    0.0000002101,	   -0.0000116423,	 -311.7707478024,	    0.0000000343,	  414.9884992086,	    0.0001015612,	    0.0000143645,	   20.6151070161,	    0.0000132845,	    0.0000033051,	
  852.9362829799,	    0.0008154779,	    0.0000002211,	  311.7685346154,	    0.0001047917,	    0.0000000375,	    0.0000092942,	  414.9896571827,	    0.0000351194,	    0.0000132845,	   20.6154294897,	    0.0000081312,	
   -0.0000638293,	    0.0000175069,	   -0.0000000423,	    0.0000183803,	   -0.0000093300,	   -0.0000000108,	    0.0000305088,	    0.0000707823,	    7.2050530475,	    0.0000033051,	    0.0000081312,	    1.5310726537;
}






// Get input/output constraint vectors
void get_input_output_constr()
{
	// State and input constraints
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
	
}


void get_input_output_constr_full_state()
{
	// state and input constraints
	d_F=MatrixXf::Zero(d_num_outputs*2+d_num_inputs*2,d_num_outputs);
	d_G=MatrixXf::Zero(d_num_outputs*2+d_num_inputs*2,d_num_inputs);
	d_z=MatrixXf::Zero(d_num_outputs*2+d_num_inputs*2,1);
	vector<float> d_F_v={0.7,-0.7,0.7,-0.7,0.7,-0.7,10.0,-10.0,10.0,-10.0,10.0,-10.0,M_PI*0.5,-M_PI*0.5,M_PI*0.5,-M_PI*0.5,M_PI*0.5,-M_PI*0.5,M_PI*0.5,-M_PI*0.5,M_PI*0.5,-M_PI*0.5,M_PI*0.5,-M_PI*0.5};
	for(int i=0;i<d_num_outputs;i++){
		d_F(2*i,i)=1.0/d_F_v[2*i];
		d_F(2*i+1,i)=1.0/d_F_v[2*i+1];
	}


	vector<float> d_G_v={0.1597,0.0,0.1597,0.0,0.1597,0.0,0.1597,0.0};
	for(int i=0;i<d_num_inputs;i++){
		d_G(2*i+2*d_num_outputs,i)=1.0/(d_G_v[2*i]-m_cf_weight_in_newtons/4.0);
		d_G(2*i+1+2*d_num_outputs,i)=1.0/(d_G_v[2*i+1]-m_cf_weight_in_newtons/4.0);
	}
	for(int i=0;i<d_num_outputs*2+d_num_inputs*2;i++){
		d_z(i,0)=1;
	}
	
}

void get_control_feedback(){
	// prestabilisation feedback
	d_K = MatrixXf::Zero(d_num_inputs,d_num_outputs);
	for(int i=0;i<d_K_vec.size();i++){
		d_K(0,i)=d_K_vec[i];
	}
}

void get_control_feedback_all_rotors(){
	// prestabilisation feedback
	d_K = MatrixXf::Zero(d_num_inputs,d_num_outputs);
	/*
	for(int i=0;i<d_K_vec.size();i++){
		d_K(0,i)=d_K_vec[i];
	}
	*/
	d_K << -1.3539, -0.4773;

}

void get_control_feedback_full_state(){
	// prestabilisation feedback
	d_K = MatrixXf::Zero(d_num_inputs,d_num_outputs);
	/*
	for(int i=0;i<d_K_vec.size();i++){
		d_K(0,i)=d_K_vec[i];
	}
	*/
	d_K << 0.1277136261,	   -0.0678270848,	   -0.4448753543,	    0.0594547148,	   -0.0315752270,	   -0.1225344482,	    0.0573221897,	    0.1079361573,	    0.0528144132,	    0.0049394592,	    0.0093009623,	    0.0165044733,	
   -0.1224348545,	   -0.0375613372,	   -0.4448753551,	   -0.0569973376,	   -0.0174847923,	   -0.1225344485,	    0.0317414273,	   -0.1034749930,	   -0.0523476520,	    0.0027350856,	   -0.0089165443,	   -0.0163586105,	
   -0.0652477551,	    0.0428397918,	   -0.4448753543,	   -0.0303726536,	    0.0199420619,	   -0.1225344482,	   -0.0362024459,	   -0.0551378941,	    0.0511842100,	   -0.0031194966,	   -0.0047511137,	    0.0159950356,	
    0.0599692824,	    0.0625482772,	   -0.4448753551,	    0.0279153790,	    0.0291178341,	   -0.1225344485,	   -0.0528610077,	    0.0506768642,	   -0.0516509757,	   -0.0045550400,	    0.0043667022,	   -0.0161408995;
}

// Get optimization quadratic cost matrix
// OSQP refers to this matrix as 'P'
MatrixXf get_quad_cost_matrix()
{	
	// Construct the cost matrix for the RAMPC problem
	MatrixXf quad_cost_mat_xf = d_Q+d_K.transpose()*d_R*d_K;
	MatrixXf quad_cost_mat_uxf = d_K.transpose()*d_R;
	MatrixXf quad_cost_mat_xuf = d_R*d_K;
	MatrixXf quad_cost_mat_uf = d_R;




	MatrixXf quad_cost_mat = MatrixXf::Zero(d_col_num, d_col_num);


	for(int i=0;i<d_N;i++){
		quad_cost_mat.block(i*d_num_outputs,i*d_num_outputs,d_num_outputs,d_num_outputs)=quad_cost_mat_xf;
		quad_cost_mat.block(i*d_num_outputs,2*(d_N+1)*d_num_outputs+d_num_inputs*i,d_num_outputs,d_num_inputs)=quad_cost_mat_uxf;
		quad_cost_mat.block(2*(d_N+1)*d_num_outputs+i*d_num_inputs,i*d_num_outputs,d_num_inputs,d_num_outputs)=quad_cost_mat_xuf;
		quad_cost_mat.block(2*(d_N+1)*d_num_outputs+i*d_num_inputs,2*(d_N+1)*d_num_outputs+i*d_num_inputs,d_num_inputs,d_num_inputs)=quad_cost_mat_uf;
	}

	quad_cost_mat.block(d_N*d_num_outputs,d_N*d_num_outputs,d_num_outputs,d_num_outputs)=d_P;	

	quad_cost_mat=2*quad_cost_mat;
	
	return quad_cost_mat;
}



MatrixXf get_quad_cost_matrix_full_state()
{
	// Construct the cost function for the RAMPC optimisation
	MatrixXf quad_cost_mat_xf = d_Q+d_K.transpose()*d_R*d_K;
	MatrixXf quad_cost_mat_uxf = d_K.transpose()*d_R;
	MatrixXf quad_cost_mat_xuf = d_R*d_K;
	MatrixXf quad_cost_mat_uf = d_R;




	MatrixXf quad_cost_mat = MatrixXf::Zero(d_col_num, d_col_num);


	for(int i=0;i<d_N;i++){
		quad_cost_mat.block(i*d_num_outputs,i*d_num_outputs,d_num_outputs,d_num_outputs)=quad_cost_mat_xf;
		quad_cost_mat.block(i*d_num_outputs,2*(d_N+1)*d_num_outputs+d_num_inputs*i,d_num_outputs,d_num_inputs)=quad_cost_mat_uxf;
		quad_cost_mat.block(2*(d_N+1)*d_num_outputs+i*d_num_inputs,i*d_num_outputs,d_num_inputs,d_num_outputs)=quad_cost_mat_xuf;
		quad_cost_mat.block(2*(d_N+1)*d_num_outputs+i*d_num_inputs,2*(d_N+1)*d_num_outputs+i*d_num_inputs,d_num_inputs,d_num_inputs)=quad_cost_mat_uf;
	}

	quad_cost_mat.block(d_N*d_num_outputs,d_N*d_num_outputs,d_num_outputs,d_num_outputs)=d_P;	

	quad_cost_mat=2*quad_cost_mat;
	
	return quad_cost_mat;
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

// Extended cleanup for the theta optimisation
void osqp_theta_extended_cleanup()
{
	osqp_cleanup(d_osqp_theta_max_work);
	osqp_cleanup(d_osqp_theta_min_work);
	c_free(d_osqp_theta_settings);
	c_free(d_osqp_theta_l_new);
	c_free(d_osqp_theta_u_new);
}


// Free Optimisation data
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


        case RAMPC_CONTROLLER_STATE_RAMPC:
            computeResponse_for_Rampc(request, response);
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
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1460");
	m_setupRampc_success = s_setupRampc_success;
	s_Rampc_mutex.unlock();


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



void computeResponse_for_Rampc(Controller::Request &request, Controller::Response &response)
{
	bool Rampc_first_pass = false;

	// Check if the state "just recently" changed
	if (m_current_state_changed)
	{
		// PERFORM "ONE-OFF" OPERATIONS HERE
		//for (int i = 0; i < 9; i++)
			//m_previous_stateErrorInertial[i] = 0.0;
		if (!m_write_data)
			Rampc_first_pass = true;
		m_Rampc_solving_first_opt = false;
		m_Rampc_cycles_since_solve = 0;
		// Set the change flag back to false
		m_current_state_changed = false;
		
		// Publish the change
		publishCurrentSetpointAndState();
		// Inform the user
		ROS_INFO_STREAM("[RAMPC CONTROLLER] State \"Rampc\" started");
	}

	// Check if Rampc is not setup and exit Rampc control mode
	// Rampc control is not allowed to start unless setup, but on exceptions setup success flag is reset
	// Rampc must be (re-)setup in this case to allow restart

	// Get relevant bools and quadrotor data
	s_Rampc_mutex.lock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1460");
	bool setupRampc_success = s_setupRampc_success;
	bool solveRampc = s_solveRampc;
	bool updateTheta = s_updateTheta;
	int experiment=s_experiment;
	m_u_f = s_u_f;
	m_y_f = s_y_f;
	m_solve_time = s_solve_time;
	m_Rampc_active_setpoint = s_Rampc_active_setpoint;
	s_Rampc_mutex.unlock();
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1460");

	bool use_LQR = false;
	control_output output;

	// Use LQR if setup not successful
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
		// count cycles since last solve 
		m_Rampc_cycles_since_solve++;
		if (m_Rampc_cycles_since_solve >= yaml_N)
			use_LQR = true;
	}
	else if (!Rampc_first_pass)
	{
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Rampc solving optimization took " << m_Rampc_cycles_since_solve + 1 << " cycles");
		m_Rampc_cycles_since_solve = 0;
	}

	// Set reference 
	m_setpoint_for_controller[0] = m_setpoint[0];
	m_setpoint_for_controller[1] = m_setpoint[1];
	m_setpoint_for_controller[2] = m_setpoint[2];
	m_setpoint_for_controller[3] = m_setpoint[3];
	
	

	if (use_LQR)
	{	 
		// Call the LQR control function
		calculateControlOutput_viaLQR(request, output);
	}
	else
	{	
		// Compute LQR for x, y and yaw control
		calculateControlOutput_viaLQR(request, output);
		switch (experiment){
		case RAMPC_CONTROLLER_EXPERIMENT_MASS:
		case RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS:
		
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] LQR thrust output: "<<output.thrust);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] index number: "<<input_number * m_num_inputs);
		input_number= m_Rampc_cycles_since_solve/20;
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] cycle: "<< m_Rampc_cycles_since_solve<< "; index number: "<<input_number * m_num_inputs);

		// use the relevant output from the RAMPC computation
		output.thrust = m_u_f(input_number * m_num_inputs);

		// Rotor Failure for the second experiment
		if (experiment==2){
			rotorFailureTime=ros::WallTime::now().toSec();
			rotorFailureDuration=rotorFailureTime-rotorFailureTimeStart;
			if(rotorFailureDuration>20.0){
				output.thrust=output.thrust*0.7;
			}
		}
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Actual output: "<<output.thrust);
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
		thrust_request_per_motor = output.thrust / 4.0;
		response.controlOutput.motorCmd1 = computeMotorPolyBackward(thrust_request_per_motor);
		response.controlOutput.motorCmd2 = computeMotorPolyBackward(thrust_request_per_motor);
		response.controlOutput.motorCmd3 = computeMotorPolyBackward(thrust_request_per_motor);
		response.controlOutput.motorCmd4 = computeMotorPolyBackward(thrust_request_per_motor);
		break;

	case RAMPC_CONTROLLER_EXPERIMENT_FULL_STATE:
		input_number= m_Rampc_cycles_since_solve/20;
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] cycle: "<< m_Rampc_cycles_since_solve<< "; index number: "<<input_number * m_num_inputs);
		//ROS_INFO_STREAM("[RAMPC CONTROLLER] Debug"<<m_num_inputs);
		output.thrust1 = m_u_f(input_number * m_num_inputs+0);
		output.thrust2 = m_u_f(input_number * m_num_inputs+1);
		output.thrust3 = m_u_f(input_number * m_num_inputs+2);
		output.thrust4 = m_u_f(input_number * m_num_inputs+3);
		ROS_INFO_STREAM("[RAMPC CONTROLLER] QUAD INPUT: ");
		ROS_INFO_STREAM(output.thrust1);
		ROS_INFO_STREAM(output.thrust2);
		ROS_INFO_STREAM(output.thrust3);
		ROS_INFO_STREAM(output.thrust4);

		// use direct thrust mode for the full state
		
		response.controlOutput.onboardControllerType = CF_COMMAND_TYPE_MOTORS;
		

		response.controlOutput.motorCmd1 = computeMotorPolyBackward(output.thrust1);
		response.controlOutput.motorCmd2 = computeMotorPolyBackward(output.thrust2);
		response.controlOutput.motorCmd3 = computeMotorPolyBackward(output.thrust3);
		response.controlOutput.motorCmd4 = computeMotorPolyBackward(output.thrust4);

		response.controlOutput.roll = 0.0;
		response.controlOutput.pitch = 0.0;
		response.controlOutput.yaw = 0.0;



	break;




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

	data_Time_prev=data_Time;
	data_Time=ros::WallTime::now().toSec();
	if((!updateTheta) && (data_Time-data_Time_prev<0.006)){
		s_Rampc_mutex.lock();
		s_updateTheta=true;
		s_Rampc_mutex.unlock();
	}

	if (Rampc_first_pass)
			m_Rampc_solving_first_opt = true;
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

	float angle_vel_Error[3];
	angle_vel_Error[0]=(stateErrorInertial[6] - m_previous_stateErrorInertial[6]) * yaml_control_frequency;
	angle_vel_Error[1]=(stateErrorInertial[7] - m_previous_stateErrorInertial[7]) * yaml_control_frequency;
	angle_vel_Error[2]=(stateErrorInertial[8] - m_previous_stateErrorInertial[8]) * yaml_control_frequency;
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
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] PREVIOUS LQR STATE: ");
	for(int i = 0; i < 9; ++i)
	{
		//ROS_INFO_STREAM(m_previous_stateErrorInertial[i]);
		stateErrorBody[i]=stateErrorInertial[i];
		m_previous_stateErrorInertial[i] = stateErrorInertial[i];
	}

	s_Rampc_mutex.lock();
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] LQR STATE: ");
	for(int i=0;i<9;i++){
	s_previous_stateErrorInertial[i]=m_previous_stateErrorInertial[i];
	s_current_stateErrorInertial[i]=stateErrorInertial[i];
	//ROS_INFO_STREAM(s_current_stateErrorInertial[i]);
	}
	for(int i=9;i<12;i++){
	//ROS_INFO_STREAM("[RAMPC CONTROLLER] DEBUG 123: "<<stateErrorInertial[i-3]<<" "<<m_previous_stateErrorInertial[i-3]);
	s_current_stateErrorInertial[i]=angle_vel_Error[i-9];
	//ROS_INFO_STREAM(s_current_stateErrorInertial[i]);
	}
	s_Rampc_mutex.unlock();
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
	//ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 1927");
	for (int i = 0; i < 4; i++)
	{
		s_setpoint(i) = m_setpoint[i];
	}
	s_setpoint_changed = true;
	s_Rampc_mutex.unlock();
	 //ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Unlock 1927");

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

			// > FOR CUSTOM BUTTON 2 - SETUP OSQP OPTIMIZATION
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
	/*
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
    */
}

// CUSTOM BUTTON 2 - SETUP RAMPC OPTIMIZATION
void processCustomButton2(float float_data, int int_data, bool* bool_data)
{
	// Inform the user
    ROS_INFO("[RAMPC CONTROLLER] Received request to setup Rampc optimization");

    s_Rampc_mutex.lock();
    // ROS_INFO("[RAMPC CONTROLLER] DEBUG Mutex Lock 2142");
    s_setupButtonPressed=true;
    s_experiment = int_data;
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
	int experiment=s_experiment;
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
            if(experiment==RAMPC_CONTROLLER_EXPERIMENT_ALL_ROTORS){
            	rotorFailureTimeStart=ros::WallTime::now().toSec();
            }
            break;

        case RAMPC_CONTROLLER_STATE_RAMPC:
            // Inform the user
            ROS_INFO("[RAMPC CONTROLLER] Received request to stop Rampc. Switch to state: LQR");
            // Update the state accordingly
            m_current_state = RAMPC_CONTROLLER_STATE_LQR;
            m_current_state_changed = true;
            break;

        
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
	/*
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
	*/
}

// CUSTOM BUTTON 5 - CHANGING REFERENCE
void processCustomButton5(float float_data, int int_data, bool* bool_data)
{	
	/*
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
    */
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

	yaml_theta_update_num=getParameterInt(nodeHandle_for_paramaters, "theta_update_num");
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
	s_yaml_theta_update_num=yaml_theta_update_num;
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
	ROS_INFO_STREAM("[RAMPC CONTROLLER] Advertised service: " << getCurrentSetpointService.getService());



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
