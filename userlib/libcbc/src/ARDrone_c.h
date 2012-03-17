/*

C Wrapper for Pongsak Suvanpong's AR.Drone library.
Developed by Jeremy Rand of VECLabs / Team SNARC / University of Oklahoma,
under contract for KISS Institute for Practical Robotics.

*/

#ifndef __DRONE_C_H__
#define __DRONE_C_H__

#ifdef __cplusplus
extern "C" {
#endif

void drone_connect();
void drone_disconnect();

void drone_takeoff();
void drone_land();
void drone_emergency();

int drone_get_battery();

float drone_get_x();
float drone_get_y();
float drone_get_z();
float drone_get_x_vel();
float drone_get_y_vel();
float drone_get_z_vel();
float drone_get_yaw();

void drone_move(int enable, float x_tilt, float y_tilt, float yaw_vel, float z_vel);
void drone_hover();

#ifdef __cplusplus
}
#endif

#endif
