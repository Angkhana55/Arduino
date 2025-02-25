void PID_2 (int set){
  error_2      = set - value_2;
  sum_error_2  += error_2; 
  pwm_output_2 = (kp*error_2)+(ki*sum_error_2)+(kd*(error_dydx_2));   
  error_dydx_2 = error_2 - last_error_2;
  last_error_2 = error_2;
  if (pwm_output_2 > 255)  { pwm_output_2 = 255; }
  if (pwm_output_2 < -255) { pwm_output_2 = -255;}  
  if (pwm_output_2 < 0) {                                         // จะล้มข้างหน้า มอเตอร์วิ่งไปหน้า
    digitalWrite (Motor_2A, 0);
    digitalWrite (Motor_2B, 1);
  }
  else {                                                        // จะล้มข้างหลัง มอเตอร์วิ่งกลับหลัง
    digitalWrite (Motor_2A, 1);
    digitalWrite (Motor_2B, 0);
  }  
  if (pwm_output_2 < 0) {
    pwm_output_2 = pwm_output_2 * (-1);
  }
  else {
    pwm_output_2 = pwm_output_2;
  }  
  analogWrite  (Motor_2_PWM, pwm_output_2);  
} // END Void pid2
