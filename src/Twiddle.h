//
//  Twiddle.h
//  PID
//
//  Created by Steven De Gryze on 8/19/17.
//
//

#ifndef Twiddle_h
#define Twiddle_h

#include <vector>

using namespace std;

class Twiddle {

public:
  /*
   * Vectors for 3 coefficients (Kp, Ki, and Kd) and the three corresponding step sizes
   */
  vector<double> coefs = vector<double>(3);
  vector<double> steps = vector<double>(3);
  
  double best_mse;
  
  // number of steps the car has driven for the current twiddle parameter attempt
  long n;
  
  // the index of the coefficient that is currently being twiddled (0 = Kp, 1 = Ki, 2 = Kd)
  int coef_idx;
  
  // flag to produce stdout feedback during the twiddle optimization
  bool verbose;
  
  // accumulator for the sum of squared errors during the current twiddle parameter attempt
  double squared_error;
  
  // flag to keep track of where we are in the twiddle algorithm. True = attempt to icnrease a coefficient.
  // false = attempt to decrease a coefficient
  bool try_increase;
  
  // number of steps that twiddle should wait before starting to measure the error
  const long equilibration_period = 500;
  
  // number of steps used in measuring the error
  const long error_period = 2000;
  
  /*
   * Constructor
   */
  Twiddle();
  
  /*
   * Destructor.
   */
  virtual ~Twiddle();
  
  /*
   * Initialize Twiddle with initial guesses of the three PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);
  
  /*
   * Provide feedback from 1 step/timeunit of the current car run. Returns a bool to indicate whether the
   * simulator should be reset and a new coefficient combination should be tested
   */
  bool Step(double cte);
  
  /*
   * Returns the current coefficient estimates;
   */
  vector<double> GetUpdatedCoefs();
  
};


#endif /* Twiddle_h */
