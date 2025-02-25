void PID_3 (int set){
  error_3      = set - value_3;
  sum_error_3  += error_3; 
  pwm_output_3 = (kp*error_3)+(ki*sum_error_3)+(kd*(error_dydx_3));   
  error_dydx_3 = error_3 - last_error_3;
  last_error_3 = error_3;
  if (pwm_output_3 > 255)  { pwm_output_3 = 255; }
  if (pwm_output_3 < -255) { pwm_output_3 = -255;}  
  if (pwm_output_3 < 0) {                                         // จะล้มข้างหน้า มอเตอร์วิ่งไปหน้า
    digitalWrite (Motor_3A, 0);
    digitalWrite (Motor_3B, 1);
  }
  else {                                                        // จะล้มข้างหลัง มอเตอร์วิ่งกลับหลัง
    digitalWrite (Motor_3A, 1);
    digitalWrite (Motor_3B, 0);
  }  
  if (pwm_output_3 < 0) {
    pwm_output_3 = pwm_output_3 * (-1);
  }
  else {
    pwm_output_3 = pwm_output_3;
  }  
  analogWrite  (Motor_3_PWM, pwm_output_3);  
} // END Void pid3
