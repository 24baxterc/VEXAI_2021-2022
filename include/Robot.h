#ifndef _Robot
#define _Robot
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include "system/json.hpp"
#include "PD.h"
#include <atomic>
using namespace pros;
using namespace std;

class Robot {
	public:
		static Controller master;
		static Motor FR;
		static Motor FL;
		static Motor BR;
		static Motor BL;

		// static Motor roller;
		static Imu IMU;
		static ADIEncoder LE;
		static ADIEncoder RE;
		static ADIEncoder BE;
		static PD power_PD;
		static PD strafe_PD;
		static PD turn_PD;
		
		static std::atomic<double> x;
		static std::atomic<double> y;
		static std::atomic<double> turn_offset_x;
		static std::atomic<double> turn_offset_y;
		static std::atomic<double> new_x;
		static std::atomic<double> new_y;
		static std::atomic<double> heading;

		static double offset_back;
		static double offset_middle;
		static double wheel_circumference; 

		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

		static void reset_PD();
		static void mecanum(int power, int strafe, int turn, int max_power);
		static void drive(void *ptr);
		static void receive(nlohmann::json msg);
		static void print(nlohmann::json msg);

		static void start_task(std::string name, void (*func)(void *));
		static bool task_exists(std::string name);
		static void kill_task(std::string name);

		static void brake(std::string mode);
		static void fps(void *ptr);
		static void move_to(void *ptr);
};
#endif