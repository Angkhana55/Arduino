#define Motor_1A      23      //A              
#define Motor_1B      25
#define Motor_1_PWM   3
#define Motor_2A      27      //B
#define Motor_2B      29
#define Motor_2_PWM   4
#define Motor_3A      31      //C
#define Motor_3B      33
#define Motor_3_PWM   5
#define Motor_4A      35      //D
#define Motor_4B      37
#define Motor_4_PWM   6
////////////////////////////////
#define solinoy_1 26
#define solinoy_2 28
#define solinoy_3 30
#define solinoy_4 32
#define solinoy_5 34
////////////////////////////////
#define sw_blue      45
#define led_sw_blue  43
#define sw_oreng     41
#define led_sw_oreng 39
#define sw_green     53
#define led_sw_green 51
#define sw_red       24
#define led_sw_red   22
#define sw_start     59
#define led_sw_start 47
bool blue, oreng, green, red;
//////////////////////
#define sen1 36
#define sen2 38
#define sen3 40
////////////////////////////////
float kp = 5.5;
float ki = 0.0008;
float kd = 8;
////////////////////////////////
float error, error_2, error_3, error_4;
float Max = 250, Max_2 = 250, Max_3 = 250, Max_4 = 250;
float error_dydx, error_dydx_2, error_dydx_3, error_dydx_4;
float sum_error, sum_error_2, sum_error_3, sum_error_4;
float last_error, last_error_2, last_error_3, last_error_4;
int pwm_output, pwm_output_2, pwm_output_3, pwm_output_4;
float filter_pid = 0;
float filter_pid_val = 0.9;
int set1 = 435;
int set2 = 352;
int set3 = 660;
int set4 = 516;
////////////////////////////////filter
float filterVal = 0.9;
int map_1, value_1 = 0, input_1 = 0;
int map_2, value_2 = 0, input_2 = 0;
int map_3, value_3 = 0, input_3 = 0;
int map_4, value_4 = 0, input_4 = 0;
////////////////////////////////
float couter = 0, couter_2 = 0;
bool state = 0, state_2 = 0;

void setup() {
  Serial.begin  (9600);
  pinMode (A8, INPUT);
  pinMode (A9, INPUT);
  pinMode (A10, INPUT);
  pinMode (A11, INPUT);
  pinMode (sen1, INPUT);
  pinMode (sen2, INPUT);
  pinMode (sen3, INPUT);
  pinMode (sw_blue, INPUT_PULLUP);  pinMode (led_sw_blue,  OUTPUT);
  pinMode (sw_oreng, INPUT_PULLUP); pinMode (led_sw_oreng, OUTPUT);
  pinMode (sw_green, INPUT_PULLUP); pinMode (led_sw_green, OUTPUT);
  pinMode (sw_red, INPUT_PULLUP);   pinMode (led_sw_red,   OUTPUT);
  pinMode (sw_start, INPUT_PULLUP);  pinMode (led_sw_start,   OUTPUT);
  digitalWrite (led_sw_blue,  1);
  digitalWrite (led_sw_oreng, 1);
  digitalWrite (led_sw_green, 1);
  digitalWrite (led_sw_red,   1);
  digitalWrite (led_sw_start, 1);
  digitalWrite(solinoy_5, LOW);
  delay (500);
  digitalWrite (led_sw_blue,  0);
  digitalWrite (led_sw_oreng, 0);
  digitalWrite (led_sw_green, 0);
  digitalWrite (led_sw_red,   0);
  digitalWrite (led_sw_start, 0);
  digitalWrite(solinoy_5, HIGH);
  delay (500);

  blue  = 0;
  oreng = 0;
  green = 0;
  red   = 0;

  for (int pin = 23 ; pin <= 37 ; pin += 2) {
    pinMode (pin, OUTPUT);
    Serial.println(pin);
  }
  for (int pin = 3 ; pin <= 6 ; pin++) {
    pinMode (pin, OUTPUT);
    Serial.println(pin);
  }
  for (int pin = 26 ; pin <= 34 ; pin += 2) {
    pinMode (pin, OUTPUT);
    Serial.println(pin);
  }
  for (int timer = 0 ; timer <= 1000 ; timer++) {
    filter_all ();
  }
  for (int timer = 0 ; timer <= 1000 ; timer++) {
    filter_all ();
    PID_1 (set1);
    PID_2 (set2);
    PID_3 (set3);
    PID_4 (set4);
  }

}

void loop() {
  ///////////////////////////////ยื่นรอรับลูก
  while (true) {
    plan ();
    filter_all ();
    set1 = 435;
    set2 = 352;
    set3 = 660;
    set4 = 516;
    PID_1 (set1);
    PID_2 (set2);
    PID_3 (set3);
    PID_4 (set4);
    if (digitalRead(sen1) == 0) {
      digitalWrite(solinoy_5, LOW);
      break;
    }
  }//end while
  //////////////////////////////////////วิ่ง
  while (true) {
    plan ();
    filter_all ();
    PID_1 (set1);
    PID_2 (set2);
    PID_3 (set3);
    PID_4 (set4);
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 425; //435-10
      set2 = 362; //352+10
      set3 = 680; //660+20
      set4 = 536; //516+20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 425; //435-10
      set2 = 362; //352+10
      set3 = 680; //660+20
      set4 = 536; //516+20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
      //      digitalWrite (solinoy_1, HIGH);
      //      digitalWrite (solinoy_2, LOW);
      //      digitalWrite (solinoy_3, LOW);
      //      digitalWrite (solinoy_4, HIGH);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 445; //435+10
      set2 = 342; //352-10
      set3 = 640; //660-20
      set4 = 496; //516-20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 445; //435+10
      set2 = 342; //352-10
      set3 = 640; //660-20
      set4 = 496; //516-20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
      //      digitalWrite (solinoy_1, LOW);
      //      digitalWrite (solinoy_2, HIGH);
      //      digitalWrite (solinoy_3, HIGH);
      //      digitalWrite (solinoy_4, LOW);
    } // End for
    if ( digitalRead(sen2) == 0 ) {
      break;
    }
  }//end while
  while (true) {
    for (int timer = 0 ; timer <= 2000 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    }// end for
    break;
  }// end while
  ////////////////////////////////////////////////////////////////////////ยกขาหน้าขึ้นคาน
  while (true) {
    plan ();
    filter_all ();
    for (int timer = 0 ; timer <= 2000 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      digitalWrite (solinoy_1, HIGH);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 352; //358
      set3 = 660; //643
      set4 = 463;//497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_1, LOW);
    delay (1000);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423
      set2 = 352; //358+47
      set3 = 660; //715
      set4 = 516; //497
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    digitalWrite (solinoy_3, HIGH);
    delay (200);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 352; //358
      set3 = 718; //643
      set4 = 516; //497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_3, LOW);
    delay(200);
    while (true) {
      for (int timer = 0 ; timer <= 1500 ; timer++) {
        plan ();
        filter_all ();
        set1 = 435;
        set2 = 352;
        set3 = 660;
        set4 = 516;
        PID_1 (set1);
        PID_2 (set2);
        PID_3 (set3);
        PID_4 (set4);
      } //end for
    }// end while
  }//end while big
  ////////////////////////////////////////////////////////เดินตรงเพื่อค่อมคาน
  while (true) {
    plan ();
    filter_all ();
    PID_1 (set1);
    PID_2 (set2);
    PID_3 (set3);
    PID_4 (set4);
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 425; //435-10
      set2 = 362; //352+10
      set3 = 680; //660+20
      set4 = 536; //516+20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 425; //435-10
      set2 = 362; //352+10
      set3 = 680; //660+20
      set4 = 536; //516+20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 445; //435+10
      set2 = 342; //352-10
      set3 = 640; //660-20
      set4 = 496; //516-20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 445; //435+10
      set2 = 342; //352-10
      set3 = 640; //660-20
      set4 = 496; //516-20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    if (digitalRead(sen3) == 0 ) {
      break;
    }
  } // End while
  while (true) {
    for (int timer = 0 ; timer <= 2000 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } //end for
    break;
  } // end while
  //////////////////////////////////////////////////////////ยกขาหลังขึ้นคาน
  while (true) {
    plan ();
    filter_all ();
    for (int timer = 0 ; timer <= 2000 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      digitalWrite (solinoy_2, HIGH);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 423; //423+39
      set2 = 352; //358
      set3 = 660; //643
      set4 = 516;//497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_2, LOW);
    delay (1000);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423
      set2 = 352; //358+47
      set3 = 660; //715
      set4 = 516; //497
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    digitalWrite (solinoy_4, HIGH);
    delay (200);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 338; //358
      set3 = 660; //643
      set4 = 516; //497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_4, LOW);
    delay(200);
    break;
  } // end while
  ///////////////////////////////////////////////////ยืนนิ่ง
  while (true) {
    plan ();
    filter_all ();
    set1 = 435;
    set2 = 352;
    set3 = 660;
    set4 = 516;
    PID_1 (set1);
    PID_2 (set2);
    PID_3 (set3);
    PID_4 (set4);
  }//end while

  ////////////////////////////////////////////// RED
  while (red == 1) {
    plan ();
    filter_all ();
    for (int timer = 0 ; timer <= 2000 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      digitalWrite (solinoy_1, HIGH);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 352; //358
      set3 = 660; //643
      set4 = 463;//497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_1, LOW);
    delay (1000);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423
      set2 = 352; //358+47
      set3 = 660; //715
      set4 = 516; //497
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    digitalWrite (solinoy_3, HIGH);
    delay (200);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 352; //358
      set3 = 718; //643
      set4 = 516; //497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_3, LOW);
    delay(200);
    // End for
    while (true)
    {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    }

  }

  plan ();
  /////////////////////////////////////////////////////// BLUE
  while (blue == 1) {
    plan ();
    filter_all ();
    digitalWrite (solinoy_1, LOW);
    digitalWrite (solinoy_2, LOW);
    digitalWrite (solinoy_3, LOW);
    digitalWrite (solinoy_4, LOW);
    set1 = 452;
    set2 = 365;
    set3 = 662;
    set4 = 522;
    PID_3 (set3); //หน้าขวา
    PID_1 (set1);//หลังขวา
    PID_4 (set4); //หน้าซ้าย
    PID_2 (set2);//หลังซ้าย
    serial (2);
    delay (1);
  } //End blue
  ////////////////////////////////////////////////// ORENG
  while (oreng == 1) {
    plan ();
    filter_all ();
    for (int timer = 0 ; timer <= 2000 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      digitalWrite (solinoy_2, HIGH);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 423; //423+39
      set2 = 352; //358
      set3 = 660; //643
      set4 = 516;//497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_2, LOW);
    delay (1000);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423
      set2 = 352; //358+47
      set3 = 660; //715
      set4 = 516; //497
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    digitalWrite (solinoy_4, HIGH);
    delay (200);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 338; //358
      set3 = 660; //643
      set4 = 516; //497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_4, LOW);
    delay(200);
    // End for
    while (true)
    {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    }

  } // End oreng
  ////////////////////////////////////////////// GREEN
  while (green == 1) {
    plan ();
    filter_all ();
    PID_1 (set1);
    PID_2 (set2);
    PID_3 (set3);
    PID_4 (set4);
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 425; //435-10
      set2 = 362; //352+10
      set3 = 680; //660+20
      set4 = 536; //516+20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 425; //435-10
      set2 = 362; //352+10
      set3 = 680; //660+20
      set4 = 536; //516+20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
      //      digitalWrite (solinoy_1, HIGH);
      //      digitalWrite (solinoy_2, LOW);
      //      digitalWrite (solinoy_3, LOW);
      //      digitalWrite (solinoy_4, HIGH);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 445; //435+10
      set2 = 342; //352-10
      set3 = 640; //660-20
      set4 = 496; //516-20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 100 ; timer++) {
      plan ();
      filter_all ();
      set1 = 445; //435+10
      set2 = 342; //352-10
      set3 = 640; //660-20
      set4 = 496; //516-20
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
      //      digitalWrite (solinoy_1, LOW);
      //      digitalWrite (solinoy_2, HIGH);
      //      digitalWrite (solinoy_3, HIGH);
      //      digitalWrite (solinoy_4, LOW);
    } // End for
  } // End green
  ////////////////////////////////////////////// RED
  while (red == 1) {
    plan ();
    filter_all ();
    for (int timer = 0 ; timer <= 2000 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      digitalWrite (solinoy_1, HIGH);
    } // End for
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 352; //358
      set3 = 660; //643
      set4 = 463;//497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_1, LOW);
    delay (1000);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423
      set2 = 352; //358+47
      set3 = 660; //715
      set4 = 516; //497
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    } // End for
    digitalWrite (solinoy_3, HIGH);
    delay (200);
    for (int timer = 0 ; timer <= 1500 ; timer++) {
      plan ();
      filter_all ();
      set1 = 435; //423+39
      set2 = 352; //358
      set3 = 718; //643
      set4 = 516; //497
      PID_3 (set3); //หน้าขวา
      PID_1 (set1);//หลังขวา
      PID_4 (set4); //หน้าซ้าย
      PID_2 (set2);//หลังซ้าย
    } // End for
    digitalWrite (solinoy_3, LOW);
    delay(200);
    // End for
    while (true) {
      plan ();
      filter_all ();
      set1 = 435;
      set2 = 352;
      set3 = 660;
      set4 = 516;
      PID_1 (set1);
      PID_2 (set2);
      PID_3 (set3);
      PID_4 (set4);
    }
  } // End while
  //////////////////////////////////////////////

}
