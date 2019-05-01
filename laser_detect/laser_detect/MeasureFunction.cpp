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

/* new_display()������Ҫ�Ǵ�ӡ���ݺ�������ݵ�DATA.TXT�ļ�*/
u_result new_display(RPlidarDriver * drv)
{
	//2019-4-21��ӵĴ���
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
				std::cout << "��" << pos + 1 << "����:  " << std::endl;//�����1�У�˵��ÿ�����ǵڼ�����
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
	//�ر��ĵ�
	OutFile.close();
	return ans;
}

//2019-4-15�޸ģ��ܹ�����ɸѡ�Ƕȵķ�Χɸѡ�;���ķ�Χɸѡ
void sort_data(rplidar_response_measurement_node_t arr[], int count, bool sort_angle, bool sort_distance) {
	float angle_min;
	float angle_max;
	float distance_min;
	float distance_max;
	if (sort_angle == true) {
		std::cout << "������ɸѡ�Ƕȵ���Сֵ: ";
		std::cin >> angle_min;
		std::cout << "������ɸѡ�Ƕȵ����ֵ: ";
		std::cin >> angle_max;
		std::cin.get();
		std::cout << std::endl;
	}
	if (sort_distance == true) {
		std::cout << "������ɸѡ�������Сֵ: ";
		std::cin >> distance_min;
		std::cout << "������ɸѡ��������ֵ: ";
		std::cin >> distance_max;
		std::cin.get();
		std::cout << std::endl;
	}
	std::cout << "���ڽ������ݽ���ɸѡ....." << std::endl;
	//����Ϊ�����߼��ж�
	const int ARRAYLENTH = 3000;
	bool index[ARRAYLENTH] = {};
	bool ANGLE_CASE;
	bool DISTANCE_CASE;
	//����Ϊ3��������жϣ�������ֻ������һ�����
	//�Ƕ�����붼Ҫ�ж�
	if (sort_angle == true && sort_distance == true) {
		for (int i = 0; i < count; i++) {
			ANGLE_CASE = sort_float_bool((arr[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f, angle_min, angle_max);
			DISTANCE_CASE = sort_float_bool(arr[i].distance_q2 / 4.0f, distance_min, distance_max);
			index[i] = ANGLE_CASE&&DISTANCE_CASE;
		}
	}
	//ֻ�жϽǶ�
	if (sort_angle == true && sort_distance == false) {
		for (int i = 0; i < count; i++) {
			ANGLE_CASE = sort_float_bool((arr[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f, angle_min, angle_max);
			index[i] = ANGLE_CASE;
		}
	}
	//ֻ�жϾ���
	if (sort_angle == false && sort_distance == true) {
		for (int i = 0; i < count; i++) {
			DISTANCE_CASE = sort_float_bool(arr[i].distance_q2 / 4.0f, distance_min, distance_max);
			index[i] = ANGLE_CASE&&DISTANCE_CASE;
		}
	}
	//������
	for (int i = 0; i < count; i++) {
		if (index[i] == true) {
			printf(" theta: %03.2f Dist: %08.2f \n",
				(arr[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
				arr[i].distance_q2 / 4.0f);
		}
	}
	//����ļ�
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

/*2019-4-30 ��һ������ӵ������vec_lp�����Ĵ�����*/
bool Fill_laser_pos(laser_pos& lp) {
	std::cin >> lp.theta;
	std::cin >> lp.dist;
	return true;
}

void Show_laser_pos(const laser_pos& lp) {
	std::cout << lp.theta << "\t\t" << lp.dist << std::endl;
}



/*��һ���ֵ�������������С�ǶȺ����Ƕ�*/
void Case_Choose(float& ang_min, float& ang_max, float& dist_min, float& dist_max) {
	std::cout << "������С�Ƕ�\n"; 
	std::cin >> ang_min;
	std::cout << "�������Ƕ�\n";
	std::cin >> ang_max;
	std::cout << "������С����\n";
	std::cin >> dist_min;
	std::cout << "����������\n";
	std::cin >> dist_max;
	std::cin.get();
}

bool Select_vec_lp(const laser_pos& lp, float ang_min ,float ang_max ,float dist_min, float dist_max) {
	if (lp.theta > ang_min&&lp.theta < ang_max&&lp.dist<dist_max&&lp.dist>dist_min)
		return true;
	else
		return false;
}