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
				OutFile << "theta: " << (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f
					<< "\t distance:" << nodes[pos].distance_q2 / 4.0f << std::endl;
			}
			//
			std::cout << "the number of points of total data: " << (int)count << std::endl;//�˴������1�У�����˵��һ���ж��ٸ���	
																						   //���нǶ��ж�������ж�
																						   //����������switch�����������ѡ��
			std::cout << "����a,b,c������caseѡ��\n";
			std::cout << "a.ɸѡ�ǶȺ;���\n";
			std::cout << "b.ɸѡ����\n";
			std::cout << "c.ɸѡ�Ƕ�\n";
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