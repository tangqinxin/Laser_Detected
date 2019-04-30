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
				OutFile << "theta: " << (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f
					<< "\t distance:" << nodes[pos].distance_q2 / 4.0f << std::endl;
			}
			//
			std::cout << "the number of points of total data: " << (int)count << std::endl;//此处添加了1行，帮助说明一共有多少个点	
																						   //进行角度判断与距离判断
																						   //以下利用了switch来进行情况的选择
			std::cout << "输入a,b,c来进行case选择\n";
			std::cout << "a.筛选角度和距离\n";
			std::cout << "b.筛选距离\n";
			std::cout << "c.筛选角度\n";
			char ch;
			std::cin >> ch;
			switch (ch) {
			case'a':
				sort_data(nodes, (int)count, true, true);
				break;
			case'b':
				sort_data(nodes, (int)count, false, true);
				break;
			case'c':
				sort_data(nodes, (int)count, true, false);
				break;
			}

		}
	}
	else {
		printf("error code: %x\n", ans);
	}

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