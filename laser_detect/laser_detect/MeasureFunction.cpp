//Add some c++ funtion to know the state of the radiar  Tangming2019/4/11
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header


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

#include "HEAD.h"

/* new_display()函数主要是打印数据和输出数据到DATA.TXT文件*/
u_result new_display(RPlidarDriver * drv)
{
	//2019-4-21添加的代码
	std::ofstream OutFile("DATA.txt");

	u_result ans;

	rplidar_response_measurement_node_t nodes[8192];
	size_t   count = _countof(nodes);

	printf("waiting for data...\n");

	// fetech extactly one 0-360 degrees' scan
	ans = drv->grabScanData(nodes, count);
	if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
		drv->ascendScanData(nodes, count);
		printf("Do you want to see all the data? (y/n) ");
		int key = getchar();
		if (key == 'Y' || key == 'y') {
			for (int pos = 0; pos < (int)count; ++pos) {
				std::cout << "第" << pos + 1 << "个点:  " << std::endl;//添加了1行，说明每个点是第几个点
				printf("%s theta: %03.2f Dist: %08.2f \n",
					(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
					(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
					nodes[pos].distance_q2 / 4.0f);
				OutFile << (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f
					<< "\t" << nodes[pos].distance_q2 / 4.0f << std::endl;
			}
		}
	}
	else {
		printf("error code: %x\n", ans);
	}
	//关闭文档
	OutFile.close();
	return ans;
}

//2019-4-15修改，能够进行筛选角度的范围筛选和距离的范围筛选
void sort_data(rplidar_response_measurement_node_t arr[], int count, bool sort_angle, bool sort_distance) {
	float angle_min;
	float angle_max;
	float distance_min;
	float distance_max;
	if (sort_angle == true) {
		std::cout << "请输入筛选角度的最小值: ";
		std::cin >> angle_min;
		std::cout << "请输入筛选角度的最大值: ";
		std::cin >> angle_max;
		std::cin.get();
		std::cout << std::endl;
	}
	if (sort_distance == true) {
		std::cout << "请输入筛选距离的最小值: ";
		std::cin >> distance_min;
		std::cout << "请输入筛选距离的最大值: ";
		std::cin >> distance_max;
		std::cin.get();
		std::cout << std::endl;
	}
	std::cout << "现在将对数据进行筛选....." << std::endl;
	//以下为进行逻辑判断
	const int ARRAYLENTH = 3000;
	bool index[ARRAYLENTH] = {};
	bool ANGLE_CASE;
	bool DISTANCE_CASE;
	//以下为3种情况的判断，基本上只会运行一种情况
	//角度与距离都要判断
	if (sort_angle == true && sort_distance == true) {
		for (int i = 0; i < count; i++) {
			ANGLE_CASE = sort_float_bool((arr[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f, angle_min, angle_max);
			DISTANCE_CASE = sort_float_bool(arr[i].distance_q2 / 4.0f, distance_min, distance_max);
			index[i] = ANGLE_CASE&&DISTANCE_CASE;
		}
	}
	//只判断角度
	if (sort_angle == true && sort_distance == false) {
		for (int i = 0; i < count; i++) {
			ANGLE_CASE = sort_float_bool((arr[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f, angle_min, angle_max);
			index[i] = ANGLE_CASE;
		}
	}
	//只判断距离
	if (sort_angle == false && sort_distance == true) {
		for (int i = 0; i < count; i++) {
			DISTANCE_CASE = sort_float_bool(arr[i].distance_q2 / 4.0f, distance_min, distance_max);
			index[i] = ANGLE_CASE&&DISTANCE_CASE;
		}
	}
	//输出结果
	for (int i = 0; i < count; i++) {
		if (index[i] == true) {
			printf(" theta: %03.2f Dist: %08.2f \n",
				(arr[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
				arr[i].distance_q2 / 4.0f);
		}
	}
	//输出文件
	std::ofstream OutFile("SelectResult.txt");
	for (int i = 0; i < count; i++) {
		if (index[i] == true) {
			OutFile << "theta: " << (arr[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f
				<< "\t distance:" << arr[i].distance_q2 / 4.0f << std::endl;
		}
	}
	OutFile.close();
}

bool sort_float_bool(float data, float min, float max) {
	if (data >= min&& data <= max) {
		return true;
	}
	else
		return false;
}

/*2019-4-30 这一部分添加的是针对vec_lp向量的处理函数*/
bool Fill_laser_pos(laser_pos& lp) {
	std::cin >> lp.theta;
	std::cin >> lp.dist;
	return true;
}

void Show_laser_pos(const laser_pos& lp) {
	std::cout << lp.theta << "\t\t" << lp.dist << std::endl;
}



/*这一部分的作用是输入最小角度和最大角度*/
void Case_Choose(float& ang_min, float& ang_max, float& dist_min, float& dist_max) {
	std::cout << "输入最小角度\n"; 
	std::cin >> ang_min;
	std::cout << "输入最大角度\n";
	std::cin >> ang_max;
	std::cout << "输入最小距离\n";
	std::cin >> dist_min;
	std::cout << "输入最大距离\n";
	std::cin >> dist_max;
	std::cin.get();
}

bool Select_vec_lp(const laser_pos& lp, float ang_min ,float ang_max ,float dist_min, float dist_max) {
	if (lp.theta > ang_min&&lp.theta < ang_max&&lp.dist<dist_max&&lp.dist>dist_min)
		return true;
	else
		return false;
}