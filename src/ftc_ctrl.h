#pragma once
#ifndef _FTC_CTRL_
#define _FTC_CTRL_

#include <vector>
#include <iostream>
#include "aris.hpp"
using namespace std;
namespace cpt_ftc
{
	class LowPass
	{
	public:
		void get_filter_data(int n_order, int fc, double Ts, double x_in, double x_out);
	private:
		const double PI = 3.14159265;
		double Wc;
		double a[3]{};
		double b[3]{};
		double den = 1;
		double x_pre = 0;
		double x_last = 0;
		double y_pre = 0;
		double y_last = 0;
	
		//struct Imp;
		//aris::core::ImpPtr<Imp> imp_;

	};
	class Admit
	{
	public:
		LowPass low_pass;
		auto admit_init()->void;
		auto get_cor_pos(double B[6], double ft_origin[6], double ft_set[6], double G[6], double pm_fce2target[16], double dead_zone[6], double vel_limit[6], double cor_pos_limit[6], double dt, double cor_pos[6])->void;
	private:
		const double PI = 3.14159265;
		double cor_vel[6]{};
		double cor_pos_now[6]{};
		double pm_now[16] = { 1.0,0,0,0,
				0,1.0,0,0,
				0,0,1.0,0,
				0,0,0,1.0 };
		double init;
		double ft_offset[6];//���Խ��˴����������ĺ������á�
	};
}
#endif
