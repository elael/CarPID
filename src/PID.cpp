#include "PID.h"


double PID::input(double measurement) {
  measurements_[sample_index_] = measurement;
  
  for(u_int8_t i = 0; i < 3; ++i)
    output_ += coefs_[i] * measurements_[(sample_index_+i)%3];
  
  // move index to the one before: -1 mod 3 == 2 mod 3
  sample_index_ += 2;
  sample_index_ %= 3;
  return output_;
}

PID::PID(double Kp, double Ki, double Kd, double dt) {
  coefs_[0] = Kp + dt*Ki + Kd/dt;
  coefs_[1] = -(Kp + 2*Kd/dt);
  coefs_[2] = Kd/dt;
}
