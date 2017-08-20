#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  prev_cte = 0.;
  squared_error = 0.;
  n = 0;
}

void PID::UpdateError(double cte) {
  i_error += cte;
  p_error = cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
  n++;
  squared_error += cte * cte;
}

double PID::TotalError() {
  double total_error = - p_error * Kp - d_error * Kd - i_error * Ki;
  return total_error;
}

double PID::MeanSquaredError() {
  return squared_error / n;
}

