#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp_, double ki_, double kd_) {

	// Initialize PID controller terms and errors
	Kp = kp_;
	Ki = ki_;
	Kd = kd_;
	p_error = i_error = d_error = 0;
}

void PID::UpdateError(double cte) {

	//difference from previous cte (p_error) to the new cte
	d_error = cte - p_error;

	//set to the new cte
	p_error = cte;

	//sum of all ctes 
	i_error += cte;
}

double PID::SteeringAngle() {
	//return total error of each term multiplied by its respective error
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

