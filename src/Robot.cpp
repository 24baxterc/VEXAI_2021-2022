#include "main.h"
#include "Robot.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "PD.h"
// #include "PD.cpp"
#include <map>
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
using namespace pros;
using namespace std;


std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

Controller Robot::master(E_CONTROLLER_MASTER);

Motor Robot::FR(17, true);
Motor Robot::FL(8);
Motor Robot::BR(3, true);
Motor Robot::BL(10);
Motor Robot::angler(15);
Motor Robot::conveyor(18);

Imu Robot::IMU(15);
ADIEncoder Robot::LE(5, 6);
ADIEncoder Robot::RE(3, 4);
ADIEncoder Robot::BE(7, 8);
ADIDigitalOut Robot::piston(2);
ADIAnalogIn Robot::potentiometer(1);
Distance Robot::dist(9);

PD Robot::power_PD(.32, 5, 0);
PD Robot::strafe_PD(.17, .3, 0);
PD Robot::turn_PD(2.4, 1, 0);

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::new_x = 0;
std::atomic<double> Robot::new_y = 0;
std::atomic<double> Robot::heading = 0;

double Robot::offset_back = 2.875;
double Robot::offset_middle = 5.0;
double pi = 3.141592653589793238;
double Robot::wheel_circumference = 2.75 * pi;

double angle_threshold = 5;
double depth_threshold1 = 200;
double depth_threshold2 = 50;
double depth_coefficient1 = .2;
double depth_coefficient2 = .02;
double inches_to_encoder = 41.669
double meters_to_inches = 39.3701



bool fff = true;
void Robot::receive(nlohmann::json msg) {
    if(fff){
        string msgS = msg.dump();
        std::size_t found = msgS.find(",");

        double lidar_depth = std::stod(msgS.substr(1, found - 1));
        double angle = std::stod(msgS.substr(found + 1, msgS.size() - found - 1));
        double phi = IMU.get_rotation() * pi / 180; //should this be IMU.get_rotation() or heading?

        heading = (IMU.get_rotation() - angle);
        new_y = ((y+lidar_depth*sin((angle/180)*pi))*meters_to_inches)*inches_to_encoder;
        new_x = ((x+lidar_depth*cos((angle/180)*pi))*meters_to_inches)*inches_to_encoder;
        fff = false;
        /* 
        * When within a certain distance of mogo, switch from lidar to other sensor
        * Var storing where depth is reading in from
        * Assert distance >12
        * update new_x and new_y for every new frame (i.e. constantly update angle+depth to ensure more accuracy)
        * threshold for turning angle
        */
    }    

}

void Robot::reset_PD() {
    power_PD.reset();
    strafe_PD.reset();
    turn_PD.reset();
}

void Robot::drive(void *ptr) {
    while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool angler_forward = master.get_digital(DIGITAL_L1);
        bool angler_backward = master.get_digital(DIGITAL_L2);

        bool piston_open = master.get_digital(DIGITAL_A);
        bool piston_close = master.get_digital(DIGITAL_B);

        bool conveyor_forward = master.get_digital(DIGITAL_R1);
        bool conveyor_backward = master.get_digital(DIGITAL_R2);

        if (angler_forward) angler = 50;
        else if (angler_backward) angler = -50;
        else angler = 0;

        if (piston_open) piston.set_value(true);
        else if (piston_close) piston.set_value(false);
        
        if (conveyor_forward) conveyor = 100;
        else if (conveyor_backward) conveyor = -100;
        else conveyor = 0;
        
        //mecanum(power, strafe, turn);
        delay(5);
    }
}

void Robot::mecanum(int power, int strafe, int turn) {
//take in a max power (?)
    int powers[] {
        power + strafe + turn,
        power - strafe - turn,
        power - strafe + turn, 
        power + strafe - turn
    };

    int max = *max_element(powers, powers + 4);
    int min = abs(*min_element(powers, powers + 4));

    double true_max = double(std::max(max, min));
    double scalar = (true_max > 127) ? 127 / true_max : 1;
    scalar = 1; //try removing this line
    

    FL = (power + strafe + turn) * scalar;
    FR = (power - strafe - turn) * scalar;
    BL = (power - strafe + turn) * scalar;
    BR = (power + strafe - turn) * scalar;
}


void Robot::start_task(std::string name, void (*func)(void *)) {
    if (!task_exists(name)) {
        tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
    }
}

bool Robot::task_exists(std::string name) {
    return tasks.find(name) != tasks.end();
}

void Robot::kill_task(std::string name) {
    if (task_exists(name)) {
        tasks.erase(name);
    }
}

void Robot::fps(void *ptr) {
    double last_x = 0;
    double last_y = 0;
    double last_phi = 0;
    double turn_offset_y = 0;
    double turn_offset_x = 0;

    while (true) {
        double cur_phi = IMU.get_rotation() * pi / 180;
        double dphi = cur_phi - last_phi;

        double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
        double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;

        turn_offset_x = turn_offset_x + cur_turn_offset_x;
        turn_offset_y = turn_offset_y + cur_turn_offset_y;

        double cur_y = ((LE.get_value() - turn_offset_y) - (RE.get_value() + turn_offset_y)) / -2;
        double cur_x = BE.get_value() - turn_offset_x;

        double dy = cur_y - last_y;
        double dx = cur_x - last_x;

        double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
        double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);

        y = (float)y + global_dy;
        x = (float)x + global_dx;

        lcd::print(1,"Y: %f - X: %f", (float)y, (float)x, IMU.get_rotation());

        last_y = cur_y;
        last_x = cur_x;
        last_phi = cur_phi;

        delay(5);
    }
}
void Robot::brake(std::string mode)
{

    if (mode.compare("coast") == 0)
    {
        FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
        FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
    }
    else if (mode.compare("hold") == 0)
    {
        FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    }
    else FL = FR = BL = BR = 0;
}
void Robot::move_to(void *ptr) 
{
    while (true)
    { 

        double imu_error = -(IMU.get_rotation() - heading);
        double y_error = new_y - y;
        double x_error = -(new_x - x);

        double phi = (IMU.get_rotation()) * pi / 180;
        double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
        double turn = turn_PD.get_value(imu_error);
        turn = (abs(turn) < 15) ? turn : abs(turn)/turn * 15;
   
        delay(5);
    }
}


