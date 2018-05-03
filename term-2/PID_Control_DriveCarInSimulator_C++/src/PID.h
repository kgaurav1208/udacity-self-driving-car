#ifndef PID_H
#define PID_H
#include <vector>
#include <math.h>
#include <cmath>
#include <numeric>
#include <limits>
#include <iostream>


using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double error;
  double sample_error;
  double best_error;
  
  double tolerance;

  double int_sum, int_ratio;
  int int_count;
  
  int sample_size;
  int count;
  int state;
  int index;
  int mod;
  bool is_initialized;
  bool integral_control;

  /*
  * Coefficients
  */ 
  vector <double> K;
  vector <double> dK;
  vector<double> int_q;
  

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  double IError(double cte);
  double altIError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();
  void Twiddlidum();
};

#endif /* PID_H */
