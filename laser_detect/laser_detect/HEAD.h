#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
/*2019-4-30新版本的数据处理流程与要求：
1.在原有的扫描函数中添加代码，将数据写出到DATA.txt文件中。
2.在main()中添加函数调用，读取DATA.TXT文件的数据，并且进行筛选，使用vector<laser_pos>的数据结构，筛选的函数参考原有的函数
3.由于2的数据结构发生了变化，因此原本的筛选函数需要调整。
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


/*2019-4-30添加：要求设计一种数据结构，并且提供一个代码，能够从txt文件中读取数据，并且进行筛选结果。*/
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
