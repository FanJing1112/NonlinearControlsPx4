#include "QuadrotorLQRControl.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

//#include <eigen3/Eigen/Dense> // ++ Math operations with Matrices ++

/* -------------------- added part ---------------------*/
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <memory>
#include <sstream>

using namespace matrix;
using namespace std;

QuadrotorLQRControl::QuadrotorLQRControl()
{

    for (int i=0;i<12;i++)
    {
       _current_state(i,0) = 0.0f;
       _eq_point(i,0) = 0.0f;
    }

    _eq_point(1,0) =  0.0f;
    _eq_point(3,0) =  0.0f;
    _eq_point(5,0) = -2.0f;

    u_control(0,0) = 0.0f;
    u_control(1,0) = 0.0f;
    u_control(2,0) = 0.0f;
    u_control(3,0) = 0.0f;

    _K = readMatrixK("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/lqr_files/new_controller2.txt");
    _P = readMatrixP("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/lqr_files/new_pe.txt");

    ff_thrust = 7.848f; // [N]

    _auto_eq_point_flag = true;

    ofstream outfile1;
    outfile1.open("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/control_input.txt", std::ios::out);
    outfile1.close();

      
    ofstream outfile3;
    outfile3.open("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/state.txt", std::ios::out);
    outfile3.close();

   
    ofstream outfile5;
    outfile5.open("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt", std::ios::out);
    outfile5.close();

    ofstream outfile4;
    outfile4.open("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/ekf.txt", std::ios::out);
    outfile4.close();

   _past_time = hrt_absolute_time() * 1e-6;

   // ----- Changes for Feedback Linearization---

   _init_time = 0;
   eta = 0;
   zeta = 9.8*0.8;
   u_bar1 = 0;
   // -------------------------------------------



}



Matrix <double, 4, 14>  QuadrotorLQRControl::readMatrixK(const char *filename)
    {

    static Matrix <double, 4, 14> result;
    static int rows = 4;
    static int cols = 14;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file";
    return result;

 }

Matrix<float,4,1> QuadrotorLQRControl::LQRcontrol()
{  

     //static Matrix<float,4,1> u_control;
     static Matrix<float,4,1> u_control_norm;
     static Matrix<float,12,1> delta_x;
     static Matrix<float, 1,12> v_b;
     static Matrix<float,1,12> delta_x_T;
     static Matrix<float,1,1> _lyap_fun;     
     const hrt_abstime now = hrt_absolute_time();

    float _current_time = now *1e-6;
    float dt = _current_time-_past_time; // POSSIBLY CHECK IF THIS IS TOO BIG
    if(abs((double)_init_time) <  0.01){
      _init_time = _current_time;
     }
    float t = _current_time - _init_time;
     
    _past_time = _current_time;
    
    // HERE IS WHERE I WILL IMPLEMENT THE CONTROL

    // Update the control terms: // THIS INTEGRATION SCHEME COULD BE A PROBLEM
    zeta = zeta + eta*dt; //+ 0.5f*u_bar1*dt*dt;
    eta = eta + u_bar1*dt;

    //get the reference Z values and accels;
    double des_state[14];
    double des_accel[4];
    get_reference_z(t, 1.0, des_state,des_accel);

    Matrix<double,14,1> des_z;
    for(int i=0;i<14;i++){
      des_z(i,0) = des_state[i];
    }
    Matrix<double,4,1> forward_acc;
    for(int i=0;i<4;i++){
      forward_acc(i,0) = des_accel[i];
    }

    //get the current Z values
    double states[14] =  {_current_state(1,0),_current_state(3,0),_current_state(5,0), 
                          _current_state(11,0), _current_state(9,0), _current_state(7,0),
                          _current_state(0,0), _current_state(2,0), _current_state(4,0), 
                          zeta, eta, 
                          _current_state(10,0),_current_state(8,0), _current_state(6,0)};







    double error_ratio = 1.0;






    double params[5] = {0.05*error_ratio, 0.05*error_ratio, 0.09, 0.8, 9.8};
    double z[14];
    calc_z(states,params,z);
    Matrix<double,14,1> current_z;
    for(int i=0;i<14;i++){
      current_z(i,0) = z[i];
    }
    if(t > 25){
      cout << "END THE EXPERIMENT" << endl;
    }
    // cout << "Time: " << t << endl;
    // //get the linearized control law to apply
    // cout << "Current Z: " << endl;
    // for(int i=0; i < 14; i++){
    //   cout << current_z(i,0) << " ";
    // }
    // cout << "-------------" << endl;

    // cout << "Desired Z: " << endl;
    // for(int i=0; i < 14; i++){
    //   cout << des_z(i,0) << " ";
    // }
    // cout << "-------------" << endl;

    // cout << "Forward Acceleration: " << endl;
    // for(int i=0; i < 4; i++){
    //   cout << forward_acc(i,0) << " ";
    // }
    // cout << "-------------" << endl;

    Matrix<double,14,1> delta_z = des_z - current_z;
    Matrix<double,4,1> v = forward_acc + _K*(delta_z);
    //cout << "Error Matrix: " << delta_z(0,0) << ", " << delta_z(1,0) << ", " << delta_z(2,0) << ", " << delta_z(3,0) << ", " << endl;
    double v_arr[4];
    for(int i=0;i<4;i++){
      v_arr[i] = v(i,0);
      // cout << v_arr[i] << " ";
    }
    // cout << endl << "-----------" << endl;

    //get the non-linearized control to apply
    double u_bar[4];
    // cout << u_bar[0] << " " << u_bar[1] << " " << u_bar[2] << " " << u_bar[3] << endl;
    calc_feedback_linearization(states, v_arr, params, u_bar);

    // put the control into the control array:
    u_control(1,0) = (float)(u_bar[3]);
    u_control(2,0) = (float)(u_bar[2]);
    u_control(3,0) = (float)(u_bar[1]);
    u_control(0,0) = zeta;
    u_bar1 = u_bar[0];
    // cout << "u_bar: " << u_bar1 << endl;

    v_b = delta_x_T*_P;
    _lyap_fun = v_b*delta_x;
    //cout<< dt << "\t" << _P(0,0) << "\n";
   // !! IMPORTANT scale the control inputs.......
    float torque_scaling = 0.1650f*8.0f*4.0f;
    float force_scaling = 32.0f;


    u_control_norm(1,0) = fmin(fmax((u_control(1,0))/(torque_scaling), -1.0f), 1.0f);  
    u_control_norm(2,0) = fmin(fmax((u_control(2,0))/(torque_scaling),  -1.0f), 1.0f);
    u_control_norm(3,0) = fmin(fmax((u_control(3,0))/(0.1f*1.0f), -1.0f), 1.0f);
    u_control_norm(0,0) = fmin(fmax((u_control(0,0)+ff_thrust)/force_scaling, 0.0f), 1.0f);

   // not normalized control inputs
     u_control(0,0) = u_control_norm(0,0)*force_scaling;
     u_control(1,0) = u_control_norm(1,0)*torque_scaling;
     u_control(2,0) = u_control_norm(2,0)*torque_scaling;
     u_control(3,0) = u_control_norm(3,0)*0.05f;
     
    //"\t" <<  u_control(0,0)+ff_thrust << "\n";
         /* Save data*/
    writeStateOnFile("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/state.txt", _current_state, now);
    writeInputOnFile("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/control_input.txt", u_control, now); 
    writeLyapunovOnFile("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt", _lyap_fun(0,0), now); 
    writeStateOnFile("/home/joe/NonlinearControls/Firmware/src/modules/mc_att_control/output_files/ekf.txt", _current_state_ekf, now);
    
    return u_control_norm;    

}

Matrix<float,4,1> QuadrotorLQRControl::normalizationControlInputs(Matrix<float,4,1> _u)
{
   Matrix<float,4,1> _u_norm;
   _u_norm(0,0) = _u(0,0)*16.0f;
   _u_norm(1,0) = _u(1,0)*(0.1080f*4.0f);
   _u_norm(2,0) = _u(2,0)*(0.1080f*4.0f);
   _u_norm(3,0) = _u(3,0)*(0.1f*1.0f); 

return _u_norm;
}

Matrix<float,4,1> QuadrotorLQRControl::getLQRcontrols()
{

return u_control;

}

void QuadrotorLQRControl::setCurrentState(Matrix<float,12,1> _state_estimate)
{

      _current_state = _state_estimate;

}



void QuadrotorLQRControl::setCurrentState(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state(0,0) = _v_local_pos.vx;
      _current_state(1,0) = _v_local_pos.x;
      _current_state(2,0) = _v_local_pos.vy;
      _current_state(3,0) = _v_local_pos.y;
      _current_state(4,0) = _v_local_pos.vz;
      _current_state(5,0) = _v_local_pos.z;
    
      _current_state(6,0)  = _v_att.rollspeed;
      _current_state(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _current_state(8,0)  = _v_att.pitchspeed;
      _current_state(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _current_state(10,0) = _v_att.yawspeed;	
      _current_state(11,0) = Eulerf(Quatf(_v_att.q)).psi();

}

void QuadrotorLQRControl::setCurrentStateEkf(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state_ekf(0,0) = _v_local_pos.vx;
      _current_state_ekf(1,0) = _v_local_pos.x;
      _current_state_ekf(2,0) = _v_local_pos.vy;
      _current_state_ekf(3,0) = _v_local_pos.y;
      _current_state_ekf(4,0) = _v_local_pos.vz;
      _current_state_ekf(5,0) = _v_local_pos.z;
    
      _current_state_ekf(6,0)  = _v_att.rollspeed;
      _current_state_ekf(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _current_state_ekf(8,0)  = _v_att.pitchspeed;
      _current_state_ekf(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _current_state_ekf(10,0) = _v_att.yawspeed;	
      _current_state_ekf(11,0) = Eulerf(Quatf(_v_att.q)).psi();

}


void QuadrotorLQRControl::setAutoEqPoint(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _eq_point(0,0) = _v_local_pos.vx;
      _eq_point(1,0) = _v_local_pos.x;
      _eq_point(2,0) = _v_local_pos.vy;
      _eq_point(3,0) = _v_local_pos.y;
      _eq_point(4,0) = _v_local_pos.vz;
      _eq_point(5,0) = _v_local_pos.z;
    
      _eq_point(6,0)  = _v_att.rollspeed;
      _eq_point(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _eq_point(8,0)  = _v_att.pitchspeed;
      _eq_point(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _eq_point(10,0) = _v_att.yawspeed;	
      _eq_point(11,0) = Eulerf(Quatf(_v_att.q)).psi();


}

void QuadrotorLQRControl::setEquilibriumPoint(Matrix<float,12,1> eqPoint)
{

      _eq_point(0,0) = eqPoint(0,0);
      _eq_point(1,0) = eqPoint(1,0);
      _eq_point(2,0) = eqPoint(2,0);
      _eq_point(3,0) = eqPoint(3,0);
      _eq_point(4,0) = eqPoint(4,0);
      _eq_point(5,0) = eqPoint(5,0);
    
      _eq_point(6,0)  = eqPoint(6,0);
      _eq_point(7,0)  = eqPoint(7,0);
      _eq_point(8,0)  = eqPoint(8,0);
      _eq_point(9,0)  = eqPoint(9,0);
      _eq_point(10,0) = eqPoint(10,0);	
      _eq_point(11,0) = eqPoint(11,0);

    
      

}

void QuadrotorLQRControl::setAutoEqPointFlag(bool flag)
{

   _auto_eq_point_flag = flag;
}

bool QuadrotorLQRControl::getAutoEqPointFlag()
{

   return _auto_eq_point_flag;
}

/* Save data on files */

void QuadrotorLQRControl::writeStateOnFile(const char *filename, Matrix <float, 12, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
       
	for(int i=0;i<12;i++){
		if(i==11){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}


void QuadrotorLQRControl::writeInputOnFile(const char *filename, Matrix <float, 4, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
        
	for(int i=0;i<4;i++){
		if(i==3){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}

void QuadrotorLQRControl::writeLyapunovOnFile(const char *filename, float value, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t" << value << "\n";   
	outfile.close();
	return;
}

Matrix <float, 12, 12>  QuadrotorLQRControl::readMatrixP(const char *filename)
    {

    static Matrix <float, 12, 12> result;
    static int rows = 12;
    static int cols = 12;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file";
    return result;

 }

 void QuadrotorLQRControl::calc_z(const double in1[14], const double in2[5], double T_diffeomorphism[14])
{
  double t10;
  double t11;
  double t11_tmp;
  double t12;
  double t12_tmp;
  double t14;
  double t15;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  //     This function was generated by the Symbolic Math Toolbox version 8.6.
  //     02-Dec-2020 11:54:49
  t2 = std::cos(in1[5]);
  t3 = std::cos(in1[3]);
  t4 = std::cos(in1[4]);
  t5 = std::sin(in1[5]);
  t6 = std::sin(in1[3]);
  t7 = std::sin(in1[4]);
  t8 = 1.0 / in2[3];
  t9 = t3 * t5;
  t10 = t5 * t6;
  t11_tmp = t2 * t3;
  t11 = t11_tmp * t7;
  t12_tmp = t2 * t6;
  t12 = t12_tmp * t7;
  t14 = t10 + t11;
  t15 = t9 + -t12;
  T_diffeomorphism[0] = in1[0];
  T_diffeomorphism[1] = (t14 * in1[8] - in1[7] * (t12_tmp - t7 * t9)) + t3 * t4 *
    in1[6];
  T_diffeomorphism[2] = -t8 * t14 * in1[9];
  t12_tmp = in1[13] * t2;
  t14 = in1[13] * t7;
  T_diffeomorphism[3] = -t8 * ((((in1[10] * t10 + in1[10] * t11) + t12_tmp * t6 *
    in1[9]) - t14 * t9 * in1[9]) + in1[12] * t3 * t4 * in1[9]);
  T_diffeomorphism[4] = in1[1];
  T_diffeomorphism[5] = (-t15 * in1[8] + in1[7] * (t11_tmp + t7 * t10)) + t4 *
    t6 * in1[6];
  T_diffeomorphism[6] = t8 * t15 * in1[9];
  T_diffeomorphism[7] = t8 * ((((in1[10] * t9 + in1[10] * -t12) + t12_tmp * t3 *
    in1[9]) + t14 * t10 * in1[9]) - in1[12] * t4 * t6 * in1[9]);
  T_diffeomorphism[8] = in1[2];
  t12_tmp = t2 * t4;
  T_diffeomorphism[9] = (-t7 * in1[6] + t4 * t5 * in1[7]) + t12_tmp * in1[8];
  T_diffeomorphism[10] = t8 * (in2[4] * in2[3] - t12_tmp * in1[9]);
  T_diffeomorphism[11] = t8 * ((-in1[10] * t2 * t4 + in1[12] * t7 * in1[9]) +
    in1[13] * t4 * t5 * in1[9]);
  T_diffeomorphism[12] = in1[3];
  T_diffeomorphism[13] = (in1[12] * t5 + in1[11] * t2) / t4;
}


void QuadrotorLQRControl::calc_feedback_linearization(const double in1[14], const double in2[4], const double in3[5], double u_bar[4])
{
  double b_t152_tmp;
  double t10;
  double t11;
  double t12;
  double t13;
  double t137;
  double t139;
  double t14;
  double t140;
  double t140_tmp;
  double t141;
  double t141_tmp;
  double t142;
  double t142_tmp;
  double t143;
  double t145;
  double t148;
  double t149;
  double t15;
  double t151;
  double t152;
  double t152_tmp;
  double t154;
  double t156;
  double t158;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t20;
  double t21;
  double t22;
  double t24;
  double t25;
  double t26;
  double t28;
  double t3;
  double t32;
  double t33;
  double t35;
  double t36;
  double t4;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46_tmp;
  double t5;
  double t53;
  double t54;
  double t56;
  double t57;
  double t58;
  double t6;
  double t60;
  double t7;
  double t8;
  double t86;
  double t9;
  //     This function was generated by the Symbolic Math Toolbox version 8.6.
  //     02-Dec-2020 11:54:49
  t2 = std::cos(in1[5]);
  t3 = std::cos(in1[3]);
  t4 = std::cos(in1[4]);
  t5 = std::sin(in1[5]);
  t6 = std::sin(in1[3]);
  t7 = std::sin(in1[4]);
  t8 = std::tan(in1[4]);
  t9 = in3[0] * in3[0];
  t10 = in3[1] * in3[1];
  t11 = in1[13] * in1[13];
  t12 = in1[12] * in1[12];
  t20 = 1.0 / in3[0];
  t21 = 1.0 / in3[1];
  t13 = t2 * t2;
  t14 = t3 * t3;
  t15 = t4 * t4;
  t16 = t5 * t5;
  t17 = t6 * t6;
  t18 = t7 * t7;
  t19 = in1[12] * t2;
  t22 = in1[11] * t5;
  t24 = 1.0 / t4;
  t28 = t2 * t3 * t7;
  t32 = in3[0] * t3 * t5 * t7;
  t33 = in3[0] * t5 * t6 * t7;
  t25 = t15 * in1[9];
  t26 = t18 * in1[9];
  t42 = t13 * t15;
  t43 = t13 * t18;
  t44 = t15 * t16;
  t45 = t16 * t18;
  t46_tmp = -(t2 * t6 * t7);
  t35 = t2 * t25;
  t36 = t2 * t26;
  t53 = t13 * t25;
  t54 = t14 * t25;
  t56 = t13 * t26;
  t57 = t16 * t25;
  t58 = t14 * t26;
  t60 = t16 * t26;
  t86 = t19 + -t22;
  t140_tmp = t5 * t6;
  t140 = (t28 + t140_tmp * t15) + t140_tmp * t18;
  t141_tmp = t3 * t5;
  t141 = (t141_tmp * t15 + t141_tmp * t18) + t46_tmp;
  t156 = in3[0] * t2;
  t142_tmp = t156 * t3;
  t142 = (t33 + t142_tmp * t15) + t142_tmp * t18;
  t156 *= t6;
  t143 = (-t32 + t156 * t15) + t156 * t18;
  t137 = 1.0 / (t25 + t26);
  t145 = 1.0 / (((t42 + t43) + t44) + t45);
  t151 = in3[0] * in3[1];
  t140_tmp = t151 * in1[10];
  t156 = t151 * t2 * t4;
  t13 = in3[1] * in3[2];
  t149 = in1[13] * in1[11];
  t158 = t140_tmp * in1[12];
  t140_tmp *= in1[13];
  t139 = t151 * in1[13] * in1[11];
  t148 = in3[0] * in3[2] * in1[13] * in1[11];
  t152_tmp = t151 * in1[12];
  b_t152_tmp = t13 * in1[12];
  t152 = ((((((((t139 * t7 * in1[9] + t148 * t7 * in1[9]) + t158 * t7 * 2.0) +
               t140_tmp * t4 * t5 * 2.0) + in1[12] * t4 * t10 * t22 * in1[9]) +
             -(t149 * t7 * t9 * in1[9])) + t156 * t11 * in1[9]) + t156 * t12 *
           in1[9]) + t152_tmp * t4 * -t22 * in1[9]) + b_t152_tmp * t4 * -t22 *
    in1[9];
  t156 = t151 * t5 * t6;
  t18 = t151 * in1[11];
  t142_tmp = t13 * in1[11];
  t13 = t140_tmp * t2;
  t140_tmp = in3[1] * in1[10] * in1[13];
  t154 = t151 * t11;
  t141_tmp = t151 * t12;
  t28 = ((((((((((((((t18 * t6 * t19 * in1[9] + t142_tmp * t6 * t19 * in1[9]) +
                     t156 * t11 * in1[9]) + t156 * t12 * in1[9]) + t149 * t3 *
                   t4 * t9 * in1[9]) + -(t158 * t3 * t4 * 2.0)) + -(t13 * t6 *
    2.0)) + t140_tmp * t32 * 2.0) + -(t139 * t3 * t4 * in1[9])) + -(t148 * t3 *
    t4 * in1[9])) + in1[12] * t3 * t7 * t10 * t22 * in1[9]) + -(in1[11] * t6 *
             t10 * t19 * in1[9])) + t154 * t28 * in1[9]) + t141_tmp * t28 * in1
          [9]) + t152_tmp * t3 * t7 * -t22 * in1[9]) + b_t152_tmp * t3 * t7 *
    -t22 * in1[9];
  t156 = t151 * t3 * t5;
  t158 = ((((((((((((((t139 * t4 * t6 * in1[9] + t148 * t4 * t6 * in1[9]) + t158
                      * t4 * t6 * 2.0) + t18 * t3 * t19 * in1[9]) + t142_tmp *
                    t3 * t19 * in1[9]) + -(t13 * t3 * 2.0)) + t156 * t11 * in1[9])
                 + t156 * t12 * in1[9]) + t152_tmp * t6 * t7 * t22 * in1[9]) +
               b_t152_tmp * t6 * t7 * t22 * in1[9]) + -(in1[11] * t3 * t10 * t19
    * in1[9])) + -(t149 * t4 * t6 * t9 * in1[9])) + -(t140_tmp * t33 * 2.0)) +
           in1[12] * t6 * t7 * t10 * -t22 * in1[9]) + t154 * t46_tmp * in1[9]) +
    t141_tmp * t46_tmp * in1[9];
  t139 = 1.0 / (t35 + t36);
  t148 = 1.0 / (((t53 + t56) + t57) + t60);
  t149 = 1.0 / (((t54 + t58) + t17 * t25) + t17 * t26);
  t154 = 1.0 / (((((((t14 * t43 + t17 * t42) + t14 * t44) + t17 * t43) + t14 *
                   t45) + t17 * t44) + t17 * t45) + t14 * t42);
  t151 = 1.0 / (((t14 * t35 + t14 * t36) + t17 * t35) + t17 * t36);
  t156 = 1.0 / (((((((t14 * t53 + t14 * t56) + t17 * t53) + t16 * t54) + t17 *
                   t56) + t16 * t58) + t17 * t57) + t17 * t60);
  t18 = t20 * t21;
  u_bar[0] = ((((-in3[3] * t140 * t154 * in2[0] + in3[3] * t141 * t154 * in2[1])
                - in3[3] * t2 * t4 * t145 * in2[2]) + t18 * t140 * t154 * t28) +
              t18 * t141 * t154 * t158) + t2 * t4 * t20 * t21 * t145 * t152;
  t154 = in1[12] * t5;
  t141_tmp = in1[11] * t2;
  t142_tmp = in3[2] * t4;
  t13 = t142_tmp * (1.0 / t2);
  t140_tmp = in3[2] * in3[3];
  u_bar[1] = ((((((t13 * in2[3] - t13 * (((t24 * t86 * ((in1[13] + t154 * t8) +
    t141_tmp * t8) + t7 * t86 * (t154 + t141_tmp) / t15) + in1[13] * t21 * t24 *
    -t22 * (in3[0] - in3[2])) + in1[13] * t19 * t24 * (in3[0] - in3[1]) / in3[2]))
                  - t140_tmp * t5 * t7 * t139 * in2[2]) + t140_tmp * t3 * t4 *
                 t5 * t151 * in2[0]) + t140_tmp * t4 * t5 * t6 * t151 * in2[1])
               + in3[2] * t5 * t7 * t20 * t21 * t139 * t152) - in3[2] * t3 * t4 *
              t5 * t20 * t21 * t151 * t28) + t142_tmp * t5 * t6 * t20 * t21 *
    t151 * t158;
  t154 = in3[1] * in3[3];
  u_bar[2] = ((((-t7 * t20 * t137 * t152 + t154 * t7 * t137 * in2[2]) + t3 * t4 *
                t20 * t149 * t28) - t4 * t6 * t20 * t149 * t158) - t154 * t3 *
              t4 * t149 * in2[0]) - t154 * t4 * t6 * t149 * in2[1];
  u_bar[3] = ((((in3[3] * t142 * t156 * in2[1] - in3[3] * t143 * t156 * in2[0])
                - t4 * t5 * t21 * t148 * t152) + t18 * t142 * t156 * t158) + t18
              * t143 * t156 * t28) + in3[0] * in3[3] * t4 * t5 * t148 * in2[2];
}

double QuadrotorLQRControl::rt_powd_snf(double u0, double u1)
{
  double y;

  y = std::pow(u0, u1);


  return y;
}

void QuadrotorLQRControl::get_reference_z(double t, double radius, double des_state[14], double des_accel[4])
{
  double des_state_tmp;
  double des_x_tmp;
  double des_xdot_tmp;
  des_x_tmp = std::cos(t);
  des_xdot_tmp = std::sin(t);
  
  //  des_z = -sqrt(t+0.5); des_zdot = -1/(2*(t + 5/10)^(1/2)); des_zdot2 = 1/(4*(t + 5/10)^(3/2));  
  //  des_zdot3 = -3/(8*(t + 0.5)^(5/2));des_zdot4 = 15/(16*(t + 0.5)^(7/2));
  //  des_psi = sin(t+0.1)/(t+0.1); des_psidot = cos(t + 1/10)/(t + 1/10) - sin(t + 1/10)/(t + 1/10)^2; 
  //  des_psidot2 = (2*sin(t + 1/10))/(t + 1/10)^3 - sin(t + 1/10)/(t + 1/10) - (2*cos(t + 1/10))/(t + 1/10)^2;  
  //  des_state = [des_xdot;des_x;des_ydot;des_y;des_zdot;des_z;0;0;0;0;des_psidot;des_psi]; 
  // simplified state
  des_state[0] = radius * des_x_tmp;
  des_state[1] = -radius * des_xdot_tmp;
  des_state[2] = -radius * des_x_tmp;
  des_state[3] = radius * des_xdot_tmp;
  des_x_tmp = radius * std::sin(t);
  des_state[4] = des_x_tmp;
  des_xdot_tmp = radius * std::cos(t);
  des_state[5] = des_xdot_tmp;
  des_state[6] = -radius * std::sin(t);
  des_state[7] = -radius * std::cos(t);
  des_state_tmp = std::sqrt(t + 0.1);
  des_state[8] = -des_state_tmp;
  des_state[9] = -1.0 / (2.0 * des_state_tmp);
  des_state[10] = 1.0 / (4.0 * rt_powd_snf(t + 0.1, 1.5));
  des_state[11] = -3.0 / (8.0 * rt_powd_snf(t + 0.1, 2.5));
  des_state[12] = t;
  des_state[13] = 1.0;
  des_accel[0] = des_xdot_tmp;
  des_accel[1] = des_x_tmp;
  des_accel[2] = 15.0 / (16.0 * rt_powd_snf(t + 0.1, 3.5));
  des_accel[3] = 0.0;
}



