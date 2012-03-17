/*

C Wrapper for Pongsak Suvanpong's AR.Drone library.
Developed by Jeremy Rand of VECLabs / Team SNARC / University of Oklahoma,
under contract for KISS Institute for Practical Robotics.

*/

#include "ARDrone.h"
#include "ARDrone_c.h"

#include "compat.h"
#include <math.h>

using namespace ARDrone;
using namespace std;

Drone * myDrone;
bool drone_connected = false;

bool watchdog_enable = false;
int watchdog_pid;

int cached_battery = 0;
float x,y,z,yaw;
float vx, vy, vz;
float last_nav_receive;
float last_nav_calc;
int sensors_pid;

int requested_enable_move = 0;
float requested_x_tilt = 0.0;
float requested_y_tilt = 0.0;
float requested_yaw_vel = 0.0;
float requested_z_vel = 0.0;
int control_pid;

void watchdog();
void monitor_sensors();
void init_position_tracking();
void update_position_tracking();
void move_control_thread();
void send_control_parameters(int enable, float x_tilt, float y_tilt, float yaw_vel, float z_vel);

void watchdog()
{
	while(1)
	{
		if(watchdog_enable)
		{
			myDrone->controller().sendWatchDogReset();
		}

		msleep(50);
	}
}

void monitor_sensors()
{
	while(1)
	{
		drone_get_battery();
		msleep(50);
	}
}

// ToDo: return value
void drone_connect()
{
	if(!drone_connected)
	{
		myDrone = new Drone();
		myDrone->start();
	
		watchdog_pid = start_process(watchdog);
		watchdog_enable = true;

		cached_battery = 0;
		init_position_tracking();

		sensors_pid = start_process(monitor_sensors);

		control_pid = start_process(move_control_thread);
		
		drone_connected = true;
	}
}

void drone_disconnect()
{
	if(drone_connected)
	{
		kill_process(control_pid);
		kill_process(sensors_pid);
		
		watchdog_enable = false;
		kill_process(watchdog_pid);
		
		myDrone->stop();
		delete myDrone;

		drone_connected = false;
	}
}

void drone_takeoff()
{
	if(drone_connected)
	{
		myDrone->controller().takeOff();
	}
}

void drone_land()
{
	if(drone_connected)
	{
		myDrone->controller().land();
	}
}

void drone_emergency()
{
	if(drone_connected)
	{
		myDrone->controller().sendEmergencyShutdown();
	}
}

// The Drone doesn't always report a valid battery level (it instead reports 0), so we cache it.
int drone_get_battery()
{
	if(drone_connected)
	{
		NavigationData batteryNavData;
		myDrone->navigationDataReceiver().copyDataTo(batteryNavData);
	
		if(batteryNavData.batteryLevel)
		{
			cached_battery = batteryNavData.batteryLevel;
		}
	
		return cached_battery;
	}
	else
	{
		return 0;
	}
}

void init_position_tracking()
{
    x=0;
    y=0;
    z=0;
    yaw=0;
    vx=0;
    vy=0;
    vz=0;

    last_nav_receive=seconds();
    last_nav_calc=seconds();

	requested_enable_move = 0;
	requested_x_tilt = 0.0;
	requested_y_tilt = 0.0;
	requested_yaw_vel = 0.0;
	requested_z_vel = 0.0;
}

void update_position_tracking()
{
	NavigationData latest_data;

	myDrone->navigationDataReceiver().copyDataTo(latest_data);

	float current_receive_time = myDrone->navigationDataReceiver().navTimestamp / 1000.0;
	float current_time = seconds();


	bool zero_vx = false;
	bool zero_vy = false;
	bool zero_z = false;


	if(latest_data.speed.vx < 0.001 && latest_data.speed.vx > -0.001)
	{
		//printf("Zero vx\n");
		zero_vx = true;
	}
	if(latest_data.speed.vy < 0.001 && latest_data.speed.vy > -0.001)
	{
		//printf("Zero vy\n");
		zero_vy = true;
	}

	if(latest_data.altitude < 0.001 && latest_data.altitude > -0.001)
	{
		//printf("Zero z\n");
		zero_z = true;
	}

	if(! zero_vx || ! requested_enable_move) vx = latest_data.speed.vx;
	if(! zero_vy || ! requested_enable_move) vy = -latest_data.speed.vy;
	if(! zero_z)
	{
		if(fabs(latest_data.altitude * 1000.0 - z)>0.001)
		{
			vz = (latest_data.altitude * 1000.0 - z) * (current_receive_time - last_nav_receive) / 1000.0;
			z = latest_data.altitude * 1000.0; // this returns mm
		}
	}

	x += vx * (current_time-last_nav_calc);
	y += (vy * 7.3 / 1.256) * (current_time-last_nav_calc);

	yaw = latest_data.orientation.yaw; // this returns deg

	last_nav_calc = current_time;
	last_nav_receive = current_receive_time;
}

void move_control_thread()
{
	while(1)
	{
		send_control_parameters(requested_enable_move, requested_x_tilt, requested_y_tilt, requested_yaw_vel, requested_z_vel);
		msleep(5);
	}
}

void send_control_parameters(int enable, float x_tilt, float y_tilt, float yaw_vel, float z_vel)
{
	myDrone->controller().sendControlParameters(enable, -x_tilt, -y_tilt, yaw_vel, z_vel);
}

void drone_move(int enable, float x_tilt, float y_tilt, float yaw_vel, float z_vel)
{
	requested_enable_move = enable;
	requested_x_tilt = x_tilt;
	requested_y_tilt = y_tilt;
	requested_yaw_vel = yaw_vel;
	requested_z_vel = z_vel;
}

void drone_hover()
{
	drone_move(0, 0.0, 0.0, 0.0, 0.0);
}
