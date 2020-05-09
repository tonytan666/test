#pragma once
#include <iostream>
#include<math.h>
#include<thread>
#include<algorithm>
//#include<Eigen/Dense>
#include<aris.hpp>
#include <fstream>

using namespace aris::dynamic;
using namespace std;

// second order low pass filter
class SecOrderLPF
{
public:
	double Xout[6][3];
	double Xin[6][3];//Xin[i][0] is current value, Xin[i][1] is last value, Xin[i][2] is previous value
	void GetLPFedValue(double SensorData[6])
	{
		for (int i = 0;i < 6;i++)
		{
			Xin[i][0] = SensorData[i];
			Xout[i][0] = b[0] * Xin[i][0] + b[1] * Xin[i][1] + b[2] * Xin[i][2] - (a[1] * Xout[i][1] + a[2] * Xout[i][2]);
			Xin[i][2] = Xin[i][1];
			Xin[i][1] = Xin[i][0];
			Xout[i][2] = Xout[i][1];
			Xout[i][1] = Xout[i][0];
		}
	};
private:
	//const double a[3] = { 1,-1.9112,0.91498 };
	//const double b[3] = { 0.0009447,0.0018894,0.0009447 };
	//const double a[3] = { 1,-1.92894,0.93138 };
	//const double b[3] = { 0.00060985,0.00121971,0.00060985 };
	const double a[3] = { 1,-1.822695,0.83718 };
	const double b[3] = { 0.00362168,0.00724336,0.00362168 };
};

//admittance model

class AdmitModel
{
public:
	double dx[6];
	void GetCorrVelo( double FTd[6]);
//private:
	struct MBKParm
	{
		double M[6][6] = { {0,0,0,0,0,0},
							{0,0,0,0,0,0 },
							{0,0,0,0,0,0},
							{0,0,0,0,0,0},
							{0,0,0,0,0,0},
							{0,0,0,0,0,0} };
		double B[6][6] = { {10.0,0,0,0,0,0},
							{0,10,0,0,0,0 },
							{0,0,-0.6,0,0,0},
							{0,0,0,20.0,0,0},
							{0,0,0,0,20.0,0},
							{0,0,0,0,0,30000.0} };
		double K[6][6] = { {0,0,0,0,0,0},
							{0,0,0,0,0,0 },
							{0,0,0,0,0,0},
							{0,0,0,0,0,0},
							{0,0,0,0,0,0},
							{0,0,0,0,0,0} };
	}MBKParms;
};
void AdmitModel::GetCorrVelo( double FTd[6])
{
	for (int i = 0; i < 6; i++)
	{

		//ddx[i] = (FTd[i] - MBKParms.K[i][i] * x[i] - MBKParms.B[i][i] * dx[i]) / MBKParms.M[i][i];
		dx[i] = FTd[i] / MBKParms.B[i][i];
	}
}
//angle translation
class AngleTrans
{
public:
	void EulerVeloZYX2Quat(double W[3],double dt);
	double q[4];
private:
	double theta;
	double Norm;
};
void AngleTrans::EulerVeloZYX2Quat(double W[3],double dt)
{
	Norm = sqrt(pow(W[0], 2) + pow(W[1], 2) + pow(W[2], 2));
	if (std::abs(Norm) > 1e-10)
	{
		theta = Norm * dt;
		W[0] = W[0] / Norm;
		W[1] = W[1] / Norm;
		W[2] = W[2] / Norm;
		q[0] = cos(theta / 2.0);
		q[1] = W[0] * sin(theta / 2.0);
		q[2] = W[1] * sin(theta / 2.0);
		q[3] = W[2] * sin(theta / 2.0);
	}
}

//double sensor_data[6];//sensor data
//double FTData0[6];// sensor offset
double sensor_filted_data[6];//filted sensor data
double sensor_fext[6];//external force
double move_corr_pos[6];//modified postion
double sensor_offset[6]{ 0,0,0,0,0,0 };
double w_corr[3]; double p_corr[3];
double pm_now[16] = { 1.0,0,0,0,
				0,1.0,0,0,
				0,0,1.0,0,
				0,0,0,1.0 };
double pm[16];
double pm_target[16];
double pe_target[6];
double pm_fce2ee[16];
//double ee_dead_zone[6];
//aris::dynamic::s_pe2pm(pe_fce2ee, pm_fce2ee);
double ft_in_ee[6];// external force in end effector
double ft_in_ee_origin[6];

//double move_corr_vel[6];//modified velocity
double dt = 0.002;// interval
const double pi = 3.141592654;



//辨识结果：
const double tool_propertity[4]{15.7117,0,0,0.047};//1.G, 2.x, 3.y, 4.z 
//double sensor_offset[6]{1.532,-3.834,13.21,-0.2038,-0.0577,0.1120};//Fx0, Fy0, Fz0, Mx0, My0, Mz0

//const double U = -0.0649;// 机器人坐标系与世界坐标系角度。
//const double V = 0.0583;
const double U = -0.0212;// 机器人坐标系与世界坐标系角度。
const double V = 0.0129;
//euler angle of the sensor
//double sensor_eul_angle[3];//1.A, 2.B, 3.C
double gra_decomp[3]; //重力在sensor x，y，z坐标系下的分解
double sensor_eul_rotm_trans[9];//sensor euler angle rotate matrix transpose
double grav_calc_temp[3];
double gra_comp[6];// gravity and torque compensation
bool move_back_trg;
//create the filter and excecute the filting
SecOrderLPF sec_ord_filter;
// admittance control model
AdmitModel admit_model;
// angle translation
AngleTrans angle_trans;

//const double pe_fce2ee[6]{ 0,0,0.2567,0,0,0 };
//double pe_sensor2tool[6]{0,0,-0.17022,208.0943*pi/ 180.0,0,0};
double pe_sensor2tool[6]{ 0,0,-0.17022,20.68*pi / 180.0,0,0 };
double pm_sensor2tool[16];
double pe_base1[6]{1.485865,-0.013552,1.494171,51.363*pi/ 180.0,-12.693*pi/ 180.0,179.827*pi/ 180.0 };
double pm_base1[16];
//double pe_tool[6]{-0.1342,0.00049,0.20528,-28.0943*pi/ 180.0,-41.4179*pi/ 180.0,19.4506*pi/ 180.0 };
double pm_tool[16];
int inited = 0;
bool inited_flag;
double ft_err[6];
double ext_fce_deadzone[6]{ 10,10,2.5,10,10,10 };
double admit_dead_zone[6] = { 1, 1, 1, 0.3, 0.25, 8.0 };
double pos_corr_limit[6]{ 20, 20, 20, 15 * pi / 180.0, 15 * pi / 180.0, 15 * pi / 180.0 };
double vel_up_limit[6] = { 20.0,20.0,60.0,0.16*pi / 180.0,0.16*pi / 180.0,0.16*pi / 180.0 };
//struct rtn_value
//{
//	double corr_pos[6];
//	bool inited_flag;
//};
ofstream ocout("LIN.txt");
auto cpt_admit(float *fce, double *eul, double *ctr_fce, bool RSI_status)->std::array<double,6>
{
	// rtn_value rtn_val;
	 
	//LOG_INFO << "123" <<","<< "456" << std::endl;
	if (RSI_status)
	{
		admit_model.MBKParms.B[2][2] = -0.05;
		//aris::dynamic::s_inv_fs2fs(pm_fce2ee, admit_dead_zone, ee_dead_zone);
		//std::cout << "ee_dead_zone" << ee_dead_zone[0] << "," << ee_dead_zone[1] << "," << ee_dead_zone[2] << "," << ee_dead_zone[3] << "," << ee_dead_zone[4] << "," << ee_dead_zone[5] << std::endl;


		//std::chrono::high_resolution_clock clock;
		//auto begin_time = clock.now();
		//while (true)
		//{
		double force_data[6]{ fce[0],fce[1], fce[2], fce[3], fce[4], fce[5] };
		
		double pe_tool[6]{ eul[3],eul[4],eul[5], eul[0] / 180.0 * pi,eul[1] / 180.0 * pi,eul[2] / 180.0 * pi };//x y z a,b,c
		double ctrl_fce[6] = { ctr_fce[0],ctr_fce[1],ctr_fce[2],ctr_fce[3],ctr_fce[4],ctr_fce[5] };
		aris::dynamic::s_pe2pm(pe_base1, pm_base1, "321");
		aris::dynamic::s_pe2pm(pe_tool, pm_tool, "321");
		aris::dynamic::s_pe2pm(pe_sensor2tool, pm_sensor2tool, "321");
		double pm_sensor2base_temp[16];
		aris::dynamic::s_pm_dot_pm(pm_base1, pm_tool, pm_sensor2base_temp);
		double pm_sensor2base[16];
		aris::dynamic::s_pm_dot_pm(pm_sensor2base_temp, pm_sensor2tool, pm_sensor2base);
		double pe_sensor2base[6];
		aris::dynamic::s_pm2pe(pm_sensor2base, pe_sensor2base, "321");
		double sensor_eul_angle[6];
		for (int i = 0; i < 3; i++)
		{
			sensor_eul_angle[i] = pe_sensor2base[i+3];
		}
		sensor_eul_rotm_trans[0] = cos(sensor_eul_angle[0])*cos(sensor_eul_angle[1]);
		sensor_eul_rotm_trans[1] = cos(sensor_eul_angle[1])*sin(sensor_eul_angle[0]);
		sensor_eul_rotm_trans[2] = -sin(sensor_eul_angle[1]);
		sensor_eul_rotm_trans[3] = cos(sensor_eul_angle[0])*sin(sensor_eul_angle[1])*sin(sensor_eul_angle[2]) - cos(sensor_eul_angle[2])*sin(sensor_eul_angle[0]);
		sensor_eul_rotm_trans[4] = cos(sensor_eul_angle[0])*cos(sensor_eul_angle[2]) + sin(sensor_eul_angle[0])*sin(sensor_eul_angle[1])*sin(sensor_eul_angle[2]);
		sensor_eul_rotm_trans[5] = cos(sensor_eul_angle[1])*sin(sensor_eul_angle[2]);
		sensor_eul_rotm_trans[6] = sin(sensor_eul_angle[0])*sin(sensor_eul_angle[2]) + cos(sensor_eul_angle[0])*cos(sensor_eul_angle[2])*sin(sensor_eul_angle[1]);
		sensor_eul_rotm_trans[7] = cos(sensor_eul_angle[2])*sin(sensor_eul_angle[1])*sin(sensor_eul_angle[0]) - cos(sensor_eul_angle[0])*sin(sensor_eul_angle[2]);
		sensor_eul_rotm_trans[8] = cos(sensor_eul_angle[1])*cos(sensor_eul_angle[2]);
		grav_calc_temp[0] = tool_propertity[0] * cos(U) * sin(V);
		grav_calc_temp[1] = -tool_propertity[0] * sin(U);
		grav_calc_temp[2] = -tool_propertity[0] * cos(U)*cos(V);
		s_mm(3, 1, 3, sensor_eul_rotm_trans, grav_calc_temp, gra_decomp);
		gra_comp[0] = gra_decomp[0];
		gra_comp[1] = gra_decomp[1];
		gra_comp[2] = gra_decomp[2];
		gra_comp[3] = gra_decomp[2] * tool_propertity[2] - gra_decomp[1] * tool_propertity[3];
		gra_comp[4] = gra_decomp[0] * tool_propertity[3] - gra_decomp[2] * tool_propertity[1];
		gra_comp[5] = gra_decomp[1] * tool_propertity[1] - gra_decomp[0] * tool_propertity[2];

		sec_ord_filter.GetLPFedValue(force_data);
		for (int i = 0; i < 6; i++)
		{
			sensor_filted_data[i] = sec_ord_filter.Xout[i][0];
		}
		//static int inited = 0;
		if (inited == 0)
		{
			for (int i = 0; i < 3; i++)
			{
				w_corr[i]=0;  
				p_corr[i]=0;
			}
			for (int i = 0; i < 6; i++)
			{
				sensor_fext[i] = 0;
				move_corr_pos[i] = 0;
				sensor_offset[i] = 0;
				pe_target[i] = 0;
				ft_in_ee[i] = 0;
				ft_err[i] = 0;
			}
			for (int i = 0; i < 16; i++)
			{
				pm_now[i] = 0;
				pm[i] = 0;
				pm_target[i] = 0;
				pm_fce2ee[i] = 0;
					//ee_dead_zone
			}
			pm_now[0] = 1.0; pm_now[5] = 1.0; pm_now[10] = 1.0; pm_now[15] = 1.0;
			// ee_dead_zone[6];
			//aris::dynamic::s_pe2pm(pe_fce2ee, pm_fce2ee);
			 ;// external force in end effector
		}
		if (inited < 40)
		{
			inited++;

		}
		else if (inited < 50)
		{

			for (int i = 0; i < 6; i++)
			{
				sensor_offset[i] += (sensor_filted_data[i] - gra_comp[i]);
				//std::cout << "offset:" << sensor_offset[i] << std::endl;
				if (inited == 49)
				{
					sensor_offset[i] = sensor_offset[i] / 10.0;
					//std::cout << "offset:" << sensor_offset[i] << std::endl;
				}
			}
			inited++;
		}
		else
		{
			//inited_flag = true;
			// dead zone for force //
			for (int i = 0; i < 6; i++)
			{
				sensor_fext[i] = sensor_filted_data[i] - sensor_offset[i] - gra_comp[i];
			}
			// calculate the force in base1;
			//aris::dynamic::s_pe2pm(pe_base1, pm_base1);
	
			//aris::dynamic::s_inv_pm_dot_pm(pm_base1, pm_tool, pm_fce2ee);
			aris::dynamic::s_pm_dot_pm(pm_tool, pm_sensor2tool, pm_fce2ee);
			aris::dynamic::s_fs2fs(pm_fce2ee, sensor_fext, ft_in_ee);
			//double pm_fce_a[3];
			//pm_fce_a[0] = pm_fce2ee[0, 2];
			//pm_fce_a[1] = pm_fce2ee[1, 2];
			//pm_fce_a[2] = pm_fce2ee[2, 2];
			//double fce_f[3];
			//fce_f[0] = sensor_fext[0];
			//fce_f[1] = sensor_fext[1];
			//fce_f[2] = sensor_fext[2];
			//double f_z;
			//aris::dynamic::s_mm(1,1,3,pm_fce_a,fce_f,ft_in_ee[2]);
			//ft_in_ee[2] = pm_fce2ee[2] * sensor_fext[0] + pm_fce2ee[6] * sensor_fext[1] + pm_fce2ee[10] * sensor_fext[2];
			for (int i = 0; i < 6; i++)
			{
				//ft_in_ee_origin[i] = ft_in_ee[i];
				ft_err[i] = ctrl_fce[i] - ft_in_ee[i];
				if (std::abs(ft_err[i]) < admit_dead_zone[i])
				{
					ft_err[i] = 0;
				}
				else
				{
					ft_err[i] = ft_err[i] < 0 ? (ft_err[i] + admit_dead_zone[i]) : (ft_err[i] - admit_dead_zone[i]);
				}
				//ext_fce_deadzone
				if (std::abs(ft_err[i])> ext_fce_deadzone[i])
				{
					if (ft_err[i] > 0)
					{
						ft_err[i] = ext_fce_deadzone[i];
					}
					else
					{
						ft_err[i] = -ext_fce_deadzone[i];
					}
				}
			}


			// dead zone for force //
			admit_model.GetCorrVelo(ft_err);

			for (int i = 0; i < 3; i++)
			{
				p_corr[i] = std::min(std::max(-vel_up_limit[i], admit_model.dx[i]), vel_up_limit[i])*dt;
				move_corr_pos[i] += p_corr[i];
				//w_corr[i] = std::min(std::max(-vel_up_limit[i + 3], admit_model.dx[i + 3]), vel_up_limit[i + 3]);
			}
			//w_corr[2] = 0;
			//angle_trans.EulerVeloZYX2Quat(w_corr, dt);
			//double pq[7]{ p_corr[0], p_corr[1], p_corr[2], angle_trans.q[1], angle_trans.q[2], angle_trans.q[3], angle_trans.q[0] };
			//for (int i = 0; i < 7; i++)
			//{
			//	pq[i] = std::abs(pq[i]) > 1e-6 ? pq[i] : 0;
			//}
			////static int count=0;
			////if (++count % 100 == 0)
			////{
			////	std::cout << "pq:" << pq[0]<<","<<pq[1] << "," << pq[2] << "," << pq[3] << "," << pq[4] << "," << pq[5] << "," << pq[6] << std::endl;
			////}
			//s_pq2pm(pq, pm);
			//for (int i = 0; i < 16; i++)
			//{
			//	pm[i] = std::abs(pm[i]) > 1e-6 ? pm[i] : 0;
			//}
			////if (count % 100 == 0)
			////{
			////	std::cout << "pm:" << pm[0] << "," << pm[1] << "," << pm[2] << "," << pm[3] << "\n" 
			////		<< pm[4] << "," << pm[5] << "," << pm[6] << "," << pm[7] << "\n" 
			////		<< pm[8] << "," << pm[9] << "," << pm[10] << "," << pm[11] << "\n"
			////		<< pm[12] << "," << pm[13] << "," << pm[14] << "," << pm[15] <<std::endl;
			////}

			//s_pm_dot_pm(pm_now, pm, pm_target);
			//for (int i = 0; i < 16; i++)
			//{
			//	pm_target[i] = std::abs(pm_target[i]) > 1e-6 ? pm_target[i] : 0;
			//}
			////pm_target[11] = std::abs(pm_target[11]) > 1e-5 ? pm_target[11] : 0;
			////if (++count % 20 == 0)
			////{
			////	std::cout << "pmtarget:" << pm_target[0] << "," << pm_target[1] << "," << pm_target[2] << "," << pm_target[3] << "\n"
			////		<< pm_target[4] << "," << pm_target[5] << "," << pm_target[6] << "," << pm_target[7] << "\n"
			////		<< pm_target[8] << "," << pm_target[9] << "," << pm_target[10] << "," << pm_target[11] << "\n"
			////		<< pm_target[12] << "," << pm_target[13] << "," << pm_target[14] << "," << pm_target[15] << std::endl;
			////}
			//s_pm2pe(pm_target, pe_target, "321");
			//for (int i = 0; i < 6; i++)
			//{
			//	pe_target[i] = std::abs(pe_target[i]) > 1e-6 ? pe_target[i] : 0;
			//}
			////if (count % 100 == 0)
			////{
			////	std::cout << "pe:" << pe_target[0] << "," << pe_target[1] << "," << pe_target[2] << "," << pe_target[3] << "," << pe_target[4] << "," << pe_target[5] << endl;
			////}
			//s_vc(16, pm_target, pm_now);
			////if (count % 100 == 0)
			////{
			////	std::cout << "pm_now:" << pm_now[0] << "," << pm_now[1] << "," << pm_now[2] << "," << pm_now[3] << "\n"
			////		<< pm_now[4] << "," << pm_now[5] << "," << pm_now[6] << "," << pm_now[7] << "\n"
			////		<< pm_now[8] << "," << pm_now[9] << "," << pm_now[10] << "," << pm_now[11] << "\n"
			////		<< pm_now[12] << "," << pm_now[13] << "," << pm_now[14] << "," << pm_now[15] << std::endl;
			////}
			////correction limit
			////s_pq2pe(pq[7], pe_target);

			//for (int i = 3; i < 6; i++)
			//{
			//	while (pe_target[i] > pi)pe_target[i] -= 2 * pi;
			//	while (pe_target[i] < -pi)pe_target[i] += 2 * pi;
			//}
			////pe_target[0] = std::abs(pm_target[3]) > 1e-5 ? pm_target[3] : 0;
			////pe_target[1] = std::abs(pm_target[7]) > 1e-5 ? pm_target[7] : 0;
			////pe_target[2] = std::abs(pm_target[11]) > 1e-5 ? pm_target[11] : 0;
			//for (int i = 3; i < 6; i++)
			//{
			//	//MoveCorrPos[i] = admit_model_xyz.x[i];
			//	move_corr_pos[i] = std::min(std::max(-pos_corr_limit[i], pe_target[i]), pos_corr_limit[i]);
			//	move_corr_pos[i] = move_corr_pos[i] * 180 / pi;
			//}
			for (int i = 0; i < 3; i++)
			{
				move_corr_pos[i] = std::min(std::max(-pos_corr_limit[i], move_corr_pos[i]), pos_corr_limit[i]);
			}
			static int count1 = 0;
			if (++count1 % 50 == 0)
			{
				LOG_INFO << "Sensor External Force:" << sensor_fext[0] << "," << sensor_fext[1] << "," << sensor_fext[2] << "," << sensor_fext[3] << "," << sensor_fext[4] << "," << sensor_fext[5] << std::endl;
				LOG_INFO << "corr_vel:" << admit_model.dx[2] << std::endl;
				LOG_INFO << "corr_pos:" << move_corr_pos[2] << std::endl;
				LOG_INFO << "EE  Force:" << ft_in_ee[2] << std::endl;
				LOG_INFO << "EE External Force:" << ft_err[2] << std::endl;
				LOG_INFO << "tool_pos:" << pe_tool[0] << "," << pe_tool[1] << "," << pe_tool[2] << "," << pe_tool[3] << "," << pe_tool[4] << "," << pe_tool[5] << std::endl;//current angle and position
				LOG_INFO << "pm_fce2ee:" << pm_fce2ee[0] << "," << pm_fce2ee[1] << "," << pm_fce2ee[2] << "," << pm_fce2ee[3] << "," <<
					pm_fce2ee[4] << "," << pm_fce2ee[5] << "," << pm_fce2ee[6] << "," << pm_fce2ee[7] << "," <<
					pm_fce2ee[8] << "," << pm_fce2ee[9] << "," << pm_fce2ee[10] << "," << pm_fce2ee[11] << "," <<
					pm_fce2ee[12] << "," << pm_fce2ee[13] << "," << pm_fce2ee[14] << "," << pm_fce2ee[15] << std::endl;
			}
			move_corr_pos[0] = 0;
			move_corr_pos[1] = 0;
			//move_corr_pos[2] = 0;
			move_corr_pos[3] = 0;
			move_corr_pos[4] = 0;
			move_corr_pos[5] = 0;
			static int count2 = 0;
			if (++count2 % 200 == 0)
			{
				if (abs(ft_err[2]) < 0.2)
				{
					ocout << "LIN" << "{" << "X" << " " << pe_tool[0] << "," << "Y" << " " << pe_tool[1] << "," << "Z" << " " << pe_tool[2] << ","
						<< "A" << " " << pe_tool[3] / pi * 180 << "," << "B" << " " << pe_tool[4] / pi * 180 << "," << "C" << " " << pe_tool[5] / pi * 180 << "}" << " " << "C_DIS" << "\n";
				}
			}

			

		}

		/*if (abs(ft_in_ee[2]) >= admit_dead_zone[2])
		{
			move_back_trg = true;
		}
		if (move_back_trg)
		{
			move_corr_pos[2] += -0.0005;

		}
		if (move_corr_pos[2] > -4.5)
		{
			move_back_trg = false;
		}
		move_corr_pos[3] = 0;*/

		//LOG_INFO << force_data[0]<<"," << force_data[1] << "," << force_data[2] << "," << force_data[3] << "," << force_data[4] << "," << force_data[5] << std::endl;//original force sensor data
		//LOG_INFO << sensor_filted_data[0]<<","<< sensor_filted_data[1] << "," <<sensor_filted_data[2] << ","<< sensor_filted_data[3] << "," <<sensor_filted_data[4] << "," <<sensor_filted_data[5] << std::endl;// filted force sensor data
		//LOG_INFO << sensor_fext[0]<<"," << sensor_fext[1] << "," << sensor_fext[2] << "," << sensor_fext[3] << "," << sensor_fext[4] << "," << sensor_fext[5]  << std::endl;// calculated external force sensor data
		//LOG_INFO << ft_in_ee_origin[0] << "," << ft_in_ee_origin[1] << "," << ft_in_ee_origin[2] << "," << ft_in_ee_origin[3] << "," << ft_in_ee_origin[4] << "," << ft_in_ee_origin[5] << std::endl;//end effector force without deadzone
	 //   LOG_INFO << ft_in_ee[0] << "," << ft_in_ee[1] << "," << ft_in_ee[2] << "," << ft_in_ee[3] << "," << ft_in_ee[4] << "," << ft_in_ee[5] << std::endl;//end effector force with deadzone
		//LOG_INFO << move_corr_pos[0] << "," << move_corr_pos[1] << "," << move_corr_pos[2] << "," << move_corr_pos[3] << "," << move_corr_pos[4] << "," << move_corr_pos[5] << std::endl;//move correction position
		//LOG_INFO << sensor_eul_angle[0] << "," << sensor_eul_angle[1] << "," << sensor_eul_angle[2] << "," << sensor_eul_angle[3] << "," << sensor_eul_angle[4] << "," << sensor_eul_angle[5] << std::endl;//current angle and position

    }
	else
	{
		/*for (int i = 0; i < 6; i++)
		{*/
			//move_corr_pos[i] = 0;
			//inited_flag = false;
			inited = 0;
			move_corr_pos[2] = 0;
			ocout.close();
			//admit_model.MBKParms.B[2][2] = -100000;
		//}
	}
	/*for (int i = 0; i < 6; i++)
	{
		rtn_val.corr_pos[i] = move_corr_pos[i];
	}
	rtn_val.inited_flag = inited_flag;
	return rtn_val;*/
	static int icount = 0;
	if (++icount % 50 == 0)
	{
		//std::cout << "Origin Force:" << sensor_filted_data[0] << "," << sensor_filted_data[1] << "," << sensor_filted_data[2] << "," << sensor_filted_data[3] << "," << sensor_filted_data[4] << "," << sensor_filted_data[5] << std::endl;
		std::cout << "GCompsent:" << gra_comp[0] << "," << gra_comp[1] << "," << gra_comp[2] << "," << gra_comp[3] << "," << gra_comp[4] << "," << gra_comp[5] << std::endl;
		std::cout << "Sensor Offset:" << sensor_offset[0] << "," << sensor_offset[1] << "," << sensor_offset[2] << "," << sensor_offset[3] << "," << sensor_offset[4] << "," << sensor_offset[5] << std::endl;
		//std::cout << "sensorOffset:" << sensor_filted_data[0] - gra_comp[0] << "," << sensor_filted_data[1] - gra_comp[1] << "," << sensor_filted_data[2] - gra_comp[2] << ","
			//<< sensor_filted_data[3] - gra_comp[3] << "," << sensor_filted_data[4] - gra_comp[4] << "," << sensor_filted_data[5] - gra_comp[5] << std::endl;
		std::cout << "Sensor External Force:" << sensor_fext[0] << "," << sensor_fext[1] << "," << sensor_fext[2] << "," << sensor_fext[3] << "," << sensor_fext[4] << "," << sensor_fext[5] << std::endl;
		//std::cout << "EE External Force:" << ft_in_ee[0] << "," << ft_in_ee[1] << "," << ft_in_ee[2] << "," << ft_in_ee[3] << "," << ft_in_ee[4] << "," << ft_in_ee[5] << std::endl;
		std::cout << "corr_vel:" << admit_model.dx[2] << std::endl;
		std::cout << "corr_pos:" << move_corr_pos[2] << std::endl;
		//std::cout << "Sensor External Force:"  << sensor_fext[2] <<  std::endl;
		std::cout << "EE  Force:" << ft_in_ee[2] << std::endl;
		std::cout << "EE External Force:" << ft_err[2] << std::endl;
		//std::cout << "corr_pos:" << move_corr_pos[2] << std::endl;
		//std::cout << "B:" << admit_model.MBKParms.B[2][2] << std::endl;
		//std::cout << "RSI_status:" << RSI_status << std::endl;

	}
	
		return std::array<double, 6>{move_corr_pos[0], move_corr_pos[1], move_corr_pos[2], move_corr_pos[3], move_corr_pos[4], move_corr_pos[5]};
		//static int count = 0;
		//std::this_thread::sleep_until(begin_time + std::chrono::milliseconds((count++) * 4));
	//}

}


















// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单


