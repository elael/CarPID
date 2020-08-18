#ifndef PID_H
#define PID_H

#include <array>

class PID {
  public:
  PID(double Kp, double Ki = 0, double Kd = 0, double dt = 1);

  double input(double measurement);

  double output() const {return output_;}

  private:
  std::array<double, 3> coefs_;
  std::array<double, 3> measurements_ = {0,0,0};
  size_t sample_index_ = 0;
  double output_ = 0;

};

#endif  // PID_H
