//
//  Twiddle.cpp
//  PID
//
//  Created by Steven De Gryze on 8/19/17.
//
//

#include "Twiddle.h"
#include <iostream>

using namespace std;

void Twiddle::Init(double Kp, double Ki, double Kd) {
  coefs = {Kp, Ki, Kd};
  // we are initializing the step sizes to 1000th of the initial values. This assumes that the initial values are
  // already quite good
  steps = {Kp / 1000., Ki / 1000., Kd / 1000.};
  best_mse = -1.;
  n = 0;
  coef_idx = 0;
  squared_error = 0;
  
  // set to true to get extensive information about the Twiddle optimization
  verbose = true;
}

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

bool Twiddle::Step(double cte) {
  n++;
  
  // when we are no longer in the equilibration period, start accumulating the error
  if (n >= equilibration_period) squared_error += cte * cte;
  bool reset_needed = (n == (equilibration_period + error_period));
  if (reset_needed) {
    double mse = squared_error / n;
    if (verbose) {
      cout << "Twiddle: -------------------------" << endl;
      cout << "Twiddle: run with coefs = [" << coefs[0] << ", " << coefs[1] << ", " << coefs[2] << "] and steps = [" << steps[0] << ", " << steps[1] << ", " << steps[2] << "]" << endl;
      cout << "Twiddle: new MSE = " << mse << " best MSE = " << best_mse << endl;
    }

    
    bool next_coef = false;
    if (best_mse == -1.) {
      if (verbose) {
        cout << "Twiddle: Initializing best MSE" << endl;
        cout << "Twiddle: increasing coef " << coef_idx << endl;
      }
      best_mse = mse;
      coefs[coef_idx] += steps[coef_idx];
      try_increase = true;
      
    } else if (try_increase) {
      // first evaluate the increase of the coefficient
      if (mse < best_mse) {
        if (verbose) cout << "Twiddle: accepting increase for coef " << coef_idx << endl;
        next_coef = true;
      } else {
        // if increasing the coefficient does not improve the error, try decreasing
        if (verbose) cout << "Twiddle: rejecting increase for coef " << coef_idx << "; trying decrease" << endl;
        coefs[coef_idx] -= 2 * steps[coef_idx];
        try_increase = false;
      }
    } else {
      if (mse < best_mse) {
        if (verbose) cout << "Twiddle: accepting decrease for coef " << coef_idx << endl;
        next_coef = true;
      } else {
        // if decreasing the coefficient also does not improve the error, reduce the step size
        if (verbose) cout << "Twiddle: rejecting decrease for coef " << coef_idx << "; reducing step size" << endl;
        coefs[coef_idx] += steps[coef_idx];
        steps[coef_idx] *= 0.9;
      
        // move on to the next coefficient and try an increase
        coef_idx = (coef_idx + 1) % 3;
        coefs[coef_idx] += steps[coef_idx];
        try_increase = true;
        if (verbose) cout << "Twiddle: moving to next coef = " << coef_idx << endl;
      
      }
    }
  
    if (next_coef) {
      // move on to the next coefficient but increase the step size
      best_mse = mse;
      steps[coef_idx] *= 1.1;
      coef_idx = (coef_idx + 1) % 3;
      coefs[coef_idx] += steps[coef_idx];
      if (verbose) {
        cout << "Twiddle: moving to next coef = " << coef_idx << endl;
        cout << "Twiddle: increasing coef " << coef_idx << endl;
      }
    }
    
    n = 0;
    squared_error = 0;
  }
  return reset_needed;
}

vector<double> Twiddle::GetUpdatedCoefs() {
  return coefs;
}


