#include <project1/pid.h>
#include <cmath>
#include <iostream>

#define BOUND(a) ((a > M_PI) ? (a - 2*M_PI) : ((a < -M_PI) ? (a + 2*M_PI) : a))

PID::PID() : error(0), error_sum(0), error_diff(0),
			 Kp(0.3f), Ki(0), Kd(0)
{

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */

}

float PID::get_control(point car_pose, point goal_pose)
{

    float ctrl;

    /* TO DO
     *
     * implement pid algorithm
     *
    */
	const float dt = 0.1f;

	float theta_g = atan2(goal_pose.y - car_pose.y, goal_pose.x - car_pose.x);

	std::cout << "target : " << theta_g * 0.7 + goal_pose.th * 0.3 << std::endl;

	float newerror = theta_g - car_pose.th;
	BOUND(newerror);

	const float pterm = Kp * newerror;

	error_sum += newerror * dt;
	const float iterm = Ki * error_sum;

	error_diff += (newerror - error) / dt;
	const float dterm = Kd * error_diff;

	ctrl = pterm + iterm + dterm;
	BOUND(ctrl);

	error = newerror;

    return ctrl;
}
