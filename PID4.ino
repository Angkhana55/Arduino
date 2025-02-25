void PID_4 (int set){
  error_4      = set - value_4;
  sum_error_4  += error_4; 
  pwm_output_4 = (kp*error_4)+(ki*sum_error_4)+(kd*(error_dydx_4));   
  error_dydx_4 = error_4 - last_error_4;
  last_error_4 = error_4;
  if (pwm_output_4 > 255)  { pwm_output_4 = 255; }
  if (pwm_output_4 < -255) { pwm_output_4 = -255;}  
  if (pwm_output_4 < 0) {                                         // จะล้มข้างหน้า มอเตอร์วิ่งไปหน้า
    digitalWrite (Motor_4A, 1);
    digitalWrite (Motor_4B, 0);
  }
  else {                                                        // จะล้มข้างหลัง มอเตอร์วิ่งกลับหลัง
    digitalWrite (Motor_4A, 0);
    digitalWrite (Motor_4B, 1);
  }  
  if (pwm_output_4 < 0) {
    pwm_output_4 = pwm_output_4 * (-1);
  }
  else {
    pwm_output_4 = pwm_output_4;
  }  
  analogWrite  (Motor_4_PWM, pwm_output_4);  
} // END Void pid4
