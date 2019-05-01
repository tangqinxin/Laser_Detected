#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
/*2019-4-30�°汾�����ݴ���������Ҫ��
1.��ԭ�е�ɨ�躯������Ӵ��룬������д����DATA.txt�ļ��С�
2.��main()����Ӻ������ã���ȡDATA.TXT�ļ������ݣ����ҽ���ɸѡ��ʹ��vector<laser_pos>�����ݽṹ��ɸѡ�ĺ����ο�ԭ�еĺ���
3.����2�����ݽṹ�����˱仯�����ԭ����ɸѡ������Ҫ������
*/


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms) {
	while (ms >= 1000) {
		usleep(1000 * 1000);
		ms -= 1000;
	};
	if (ms != 0)
		usleep(ms * 1000);
}
#endif
using namespace rp::standalone::rplidar;

#ifndef TMHEAD_H
#define TMHEAD_H

void sort_data(rplidar_response_measurement_node_t arr[], int count, bool sort_angle, bool sort_distance);
bool sort_float_bool(float data, float min, float max);


/*2019-4-30��ӣ�Ҫ�����һ�����ݽṹ�������ṩһ�����룬�ܹ���txt�ļ��ж�ȡ���ݣ����ҽ���ɸѡ�����*/
using std::vector;
struct laser_pos
{
	float theta;
	float dist;
};

bool Fill_laser_pos(laser_pos& lp);

void Show_laser_pos(const laser_pos& lp);

void Case_Choose(float& ang_min, float& ang_max, float& dist_min, float& dist_max);

bool Select_vec_lp(const laser_pos& lp, float ang_min, float ang_max, float dist_min, float dist_max);

#endif
