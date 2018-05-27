#ifndef __Ranging_Front_HPP
#define __Ranging_Front_HPP

#include <iostream>
#include <vector>

#include "config.h"
#include "dt.hpp"
//#include <rotation_mat.h>
#include <lidar_driver_sim.h>

using namespace std;

class __ranging {

public:
	__ranging() {

		x_front  = x_rear  = x_left  = x_right = 0.0f;
		xl_front = xl_rear = xl_left = xl_right = 0.0f;

		v_front = v_rear = v_left = v_right = 0.0f;
		dx = dy = 0;
	}

	void set_LidarData(vector<__scandot> in, bool is_clear_last) {
		int i;

		if (is_clear_last) {
			data_last.clear();
			for (i = 0; i < data.size(); i++) {
				__scandot temp;
				temp = data[i];
				data_last.push_back(temp);
			}
		}
		data.clear();
		for (i = 0; i < in.size(); i++) {
			__scandot temp;
			temp = in[i];
			data.push_back(temp);
		}
	}

	void run() {
		xl_front = x_front;
		xl_rear  = x_rear;
		xl_left  = x_left;
		xl_right = x_right;

		x_front = get_Range(0,   10);
		x_rear  = get_Range(180, 10);
		x_left  = get_Range(270, 10);
		x_right = get_Range(90,  10);
		double t = dt.get() / 1000.0f;
		
		v_front = v_rear = v_left = v_right = 0.0f;
		if (x_front != 0 && xl_front != 0)
			v_front = (x_front - xl_front) / t;
		if (x_rear  != 0 && xl_rear  != 0)
			v_rear  = (x_rear - xl_rear) / t;
		if (x_left  != 0 && xl_left  != 0)
			v_left  = (x_left - xl_left) / t;
		if (x_right != 0 && xl_right != 0)
			v_right = (x_right - xl_right) / t;

		// ǰ���ֵõ��ٶ�
		if (v_front == 0 && v_rear == 0)
			vx = 0;
		else if (v_front != 0 && v_rear == 0)
			vx = v_front;
		else if (v_front == 0 && v_rear != 0)
			vx = -v_rear;
		else
			vx = (v_front + -v_rear) / 2;

		// ���Ҳ�ֵõ��ٶ�
		if (v_right == 0 && v_left == 0)
			vy = 0;
		else if (v_right != 0 && v_left == 0)
			vy = v_right;
		else if (v_right == 0 && v_left != 0)
			vy = -v_left;
		else
			vy = (v_right + -v_left) / 2;

		dx = vx * t;
		dy = vy * t;
	}

	double get_Vx() { return vx; }
	double get_Vy() { return vy; }
	double get_dx() { return dx; }
	double get_dy() { return dy; }
	double get_XFront() { return x_front; }

	double get_Range(float angle, float sample_angle) {

		int i;
		double ret = 0;
		int pt_eff_num = 0;

		float lower_threshold = angle - sample_angle / 2;
		float upper_threshold = angle + sample_angle / 2;
		float mid_l = angle, mid_u = angle;
		if (lower_threshold < 0) {
			lower_threshold += 360.0f;
			if (mid_l <= 0)
				mid_l += 360.0f;
		}

		for (i = 0; i < data.size(); i++) {
			if (data[i].angle < lower_threshold || data[i].angle > mid_l) {
				if (data[i].angle < mid_u || data[i].angle > upper_threshold)
					continue;
			}
			ret += data[i].dst;
			pt_eff_num++;	
		}

		if (pt_eff_num == 0)
			return 0;
		ret /= pt_eff_num;
		return ret;

	}// double get_Range(float angle, float sample_angle)

private:

	vector<__scandot> data, data_last;
	__dt dt;

	double x_front,  x_rear,  x_left,  x_right;
	double xl_front, xl_rear, xl_left, xl_right;

	double v_front, v_rear, v_left, v_right;
	double dx, dy;
	double vx, vy;

};


#endif	// __Ranging_Front_HPP

