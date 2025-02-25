void PID_1 (int set) {
  error      = set - value_1;
  sum_error  += error;
  pwm_output = (kp * error) + (ki * sum_error) + (kd * (error_dydx));
  error_dydx = error - last_error;
  last_error = error;
  if (pwm_output > 255)  {
    pwm_output = 255;
  }
  if (pwm_output < -255) {
    pwm_output = -255;
  }
  if (pwm_output < 0) {                                         // จะล้มข้างหน้า มอเตอร์วิ่งไปหน้า
    digitalWrite (Motor_1A, 1);
    digitalWrite (Motor_1B, 0);
  }
  else {                                                        // จะล้มข้างหลัง มอเตอร์วิ่งกลับหลัง
    digitalWrite (Motor_1A, 0);
    digitalWrite (Motor_1B, 1);
  }
  if (pwm_output < 0) {
    pwm_output = pwm_output * (-1);
  }
  else {
    pwm_output = pwm_output;
  }
  analogWrite  (Motor_1_PWM, pwm_output);
} // END Void pid1
