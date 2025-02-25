void filter_all () {
  input_1 = analogRead(A9);
  map_1 = map(input_1, 0, 1023, 0, 1000);
  value_1 = (map_1 * (1 - filterVal)) + (value_1 * filterVal);
  input_2 = analogRead(A11);
  map_2 = map(input_2, 0, 1023, 0, 1000);
  value_2 = (map_2 * (1 - filterVal)) + (value_2 * filterVal);
  input_3 = analogRead(A10);
  map_3 = map(input_3, 0, 1023, 0, 1000);
  value_3 = (map_3 * (1 - filterVal)) + (value_3 * filterVal);
  input_4 = analogRead(A8);
  map_4 = map(input_4, 0, 1023, 0, 1000);
  value_4 = (map_4 * (1 - filterVal)) + (value_4 * filterVal);
}

void serial (int check) {
  if (check == 1) { Serial.println (String("==1==") + value_1 + String("||") + pwm_output);}
  if (check == 2) { Serial.println (String("==2==") + value_2 + String("||") + pwm_output_2);}
  if (check == 3) { Serial.println (String("==3==") + value_3 + String("||") + pwm_output_3);}
  if (check == 4) { Serial.println (String("==4==") + value_4 + String("||") + pwm_output_4);}
}

void plan () {
  if (digitalRead (sw_blue) == 0) {
    blue  = 1;
    oreng = 0;
    green = 0;
    red   = 0;
    digitalWrite (led_sw_blue,  HIGH);
    digitalWrite (led_sw_oreng, LOW);
    digitalWrite (led_sw_green, LOW);
    digitalWrite (led_sw_red,   LOW);
  }
  if (digitalRead (sw_oreng) == 0) {
    blue  = 0;
    oreng = 1;
    green = 0;
    red   = 0;
    digitalWrite (led_sw_blue,  LOW);
    digitalWrite (led_sw_oreng, HIGH);
    digitalWrite (led_sw_green, LOW);
    digitalWrite (led_sw_red,   LOW);
  }
  if (digitalRead (sw_green) == 0) {
    blue  = 0;
    oreng = 0;
    green = 1;
    red   = 0;
    digitalWrite (led_sw_blue,  LOW);
    digitalWrite (led_sw_oreng, LOW);
    digitalWrite (led_sw_green, HIGH);
    digitalWrite (led_sw_red,   LOW);
  }
  if (digitalRead (sw_red) == 0) {
    blue  = 0;
    oreng = 0;
    green = 0;
    red   = 1;
    digitalWrite (led_sw_blue,  LOW);
    digitalWrite (led_sw_oreng, LOW);
    digitalWrite (led_sw_green, LOW);
    digitalWrite (led_sw_red,   HIGH);
  }
}
