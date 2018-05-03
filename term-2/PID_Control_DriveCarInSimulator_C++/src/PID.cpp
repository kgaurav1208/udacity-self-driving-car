#include "PID.h"



/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	K = {Kp, Kd, Ki};
  
  integral_control = false;

  if (integral_control) {
    dK = {0.1, 1.0, 0.0001};
    mod = 3;
  }
	else {
    dK = {0.1, 1.0, 0.0};
    mod = 2;
  }

	i_error = p_error = d_error = error = sample_error = int_sum = 0.0;
	best_error = numeric_limits<double>::infinity();
  int_ratio = 0.9;
  
  count = 1;
  int_count = 1;
  index = 0;
  state = 0;
  sample_size = 100;
  is_initialized = false;

  tolerance = (dK[0]+dK[1]+dK[2])/10;

}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
  if(integral_control){
    //i_error = IError(cte);
    i_error = altIError(cte);
  }

	p_error = cte;
  cout << "proportional error: " << p_error << " differential error: " << d_error << " integral error: " << i_error << endl;

}

double PID::IError(double cte) {
  if (cte*p_error > 0){
    int_q.push_back(cte);
    return accumulate(int_q.begin(), int_q.end(), 0.0);
  }
  else {
    cout << "size of previous integral window: " << int_q.size() << " previous integral error: " << i_error << endl;
    int_q.clear();
    return 0.0;
  }
}

double PID::altIError(double cte) {
  int_sum = int_sum *int_ratio + cte;
  int_count = int_count*int_ratio + 1;
  return int_sum/int_count;

}

double PID::TotalError() {
	double total_error = K[0]*p_error + K[1]*d_error + K[2]*i_error;
	return total_error;
}

void PID::Twiddle() {
	switch(state) {
    case 0: {
      K[index] += dK[index];
      state = 1;
      break;
    }
    case 1: {
    if (error < best_error) {
      best_error = error;
      dK[index] *= 1.1;
      state = 3;
    }
    else {
      K[index] -= 2*dK[index];
      state = 2;
    }
    break;
    }
    case 2: {
      if (error < best_error) {
        best_error = error;
        dK[index] *= 1.1;
      } 
      else {
        K[index] += dK[index];
        dK[index] *= 0.9;
      }
      state = 3;
      break;
    }
    case 3: {
      index = (index + 1) % mod;
      state = 0;
      break;
    }
  }
}


void PID::Twiddlidum() {
  if(dK[0]+dK[1]+dK[2] > tolerance ) {
    if (count < sample_size) {
      sample_error += error;
      count += 1;
    }
    else {
      sample_error = sample_error/count;
      cout << "sample error: " << sample_error << " best error: " << best_error << endl;
      if(!is_initialized){
        best_error = sample_error;
        is_initialized = true;        
      }
      else{
        Twiddle();
        count = 1;
        sample_error = 0.0;            
      }
    }
  }
  else {
    cout << "optimized" << std::endl;
  }
}
