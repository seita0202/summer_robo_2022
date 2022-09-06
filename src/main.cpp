#include<Arduino.h>
#include"Encoder.h"
#include"motorrun.h"
#include<Servo.h>
//#include"motordigi.h"


#define mo0_1 PB9 //0
#define mo0_2 PA11
#define mo1_1 PC8 //1
#define mo1_2 PC9
#define mo2_1 PB8 //2
#define mo2_2 PC6
#define startpin PC0
#define LEDpin PC1
#define liftA PB1
#define liftB PB10
#define servoBottom_Rpin PA8
#define servoBottom_Lpin PB5
#define strategypin PA3


Encoder encoder;
Servo servoBottom_R;
Servo servoBottom_L;

int e0c, e1c, e2c;
double rotate_0, rotate_1, rotate_2;
double dis_0, dis_1, dis_2;
float now_x = 0, now_y = 0, now_z = 0; 
// double lf = 20;
// double lf = 10;
double lf =15 ; // 17;
double kimu = 30;
int i = 0;
double target_distance = 0, target_direction = 0; 
const double D = 60;   //直径
const double enc = 880;     // 一周当たりのカウント
const double circum = PI * D;
const int base_power = 130;
float alpha = 0;
int Sp=150;


int num = 1;

void stop(){
  motorrun(mo0_1, mo0_2, 0);
  motorrun(mo1_1, mo1_2, 0);
  motorrun(mo2_1, mo2_2, 0);
}


double getAlpha(double target_x[], double target_y[], int index_){
  e0c = encoder.M0encvalue;
  e1c = encoder.M1encvalue;
  e2c = -1*encoder.M2encvalue;

  rotate_0 = e0c / enc;
  rotate_1 = e1c / enc;
  rotate_2 = e2c / enc;

  dis_0 = rotate_0 * circum;   //回転数から各タイヤの距離
  dis_1 = rotate_1 * circum;
  dis_2 = rotate_2 * circum;

  /*---------自己位置推定-------------*/
  now_x= 2 * dis_0 / 3 - dis_1 / 3 - dis_2 / 3;
  now_y= dis_1 * sqrt(3) / 3 - dis_2 * sqrt(3) / 3;
  now_z= dis_0 + dis_1 + dis_2;
  /*---------------------------------*/

  target_distance = sqrt((target_x[i] - now_x) * (target_x[i] - now_x) + (target_y[i] - now_y) * (target_y[i] - now_y));
  target_direction = atan2((target_y[i] - now_y), (target_x[i] - now_x));
  target_direction = target_direction - 0.0436; 
  if (target_distance < lf) {
    if (i+1 < index_){
      i++;    // return std::forward_as_tuple(target_distance, target_direction);
    }
    else{
      i=index_;
    }
  }
  else{}
    return target_direction;
}

int lift_Max =165;
int lift_Middle = 60;
int lift_Middle2 = 100;
int lift_pos = 0;




void SetBottom(){
  servoBottom_R.write(100);
  servoBottom_L.write(0);
}

void roundR(){
  motorrun(mo0_1, mo0_2, base_power);
  motorrun(mo1_1, mo1_2, base_power);
  motorrun(mo2_1, mo2_2,base_power);
}

void resetpos(){
  now_x=0;
  now_y=0;
  now_z=0;
  alpha=0;
  return;
}

void setup(){
  Serial.begin(9600);
  pinMode(mo0_1,OUTPUT);
  pinMode(mo0_2,OUTPUT);
  pinMode(mo1_1,OUTPUT);
  pinMode(mo1_2,OUTPUT);
  pinMode(mo2_1,OUTPUT);
  pinMode(mo2_2,OUTPUT);
  pinMode(startpin,INPUT);
  pinMode(liftA, OUTPUT);
  pinMode(liftB, OUTPUT);
  pinMode(LEDpin,OUTPUT);
  servoBottom_R.attach(servoBottom_Rpin);
  servoBottom_L.attach(servoBottom_Lpin);
  pinMode(strategypin,INPUT);
  encoder.set();
}

void loop(){
    switch(num){
      case 1:{
        if(digitalRead(startpin) == LOW){
          num++;                              // START!!
          break;
        }
        else{                             // STOP & Blink LED & RESET positions
          num = 1;
          stop();
          encoder.set();
          resetpos();

          SetBottom();
          if(lift_pos == 0){
            //motordigi(liftA,liftB,0);
            motorrun(liftA,liftB, 0);
            lift_pos = 0;
          }
          else if (lift_pos == 50){
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(635);
            motorrun(liftA,liftB, 0);


            lift_pos = 0;

          }
          else if (lift_pos == 100){
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(940);
            motorrun(liftA,liftB, 0);
            lift_pos = 0;

          }
          else if (lift_pos == 160){
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(1440);
            motorrun(liftA,liftB, 0);
            lift_pos = 0;

          }
          else{
            motorrun(liftA,liftB,0);
            lift_pos = 0;

          }
            motorrun(liftA,liftB,0);
            digitalWrite(LEDpin, HIGH);
            delay(20);
            digitalWrite(LEDpin, LOW);
            delay(50);
           // Serial.println("STAY");
            break;
        }
      }
      case 2:{                                 // LINE_0
                  Serial.println(num);

        // i = 0;
        // int inc_case2 = 0;

        // const int index_num2 = 1; 
        // double t2_x[index_num2] = {0};
        // double t2_y[index_num2] = {0}; 

        // while(inc_case2 < index_num2){
        //   if(digitalRead(startpin) == HIGH){
        //     num = 0;
        //     break;
        //   }
        //   else{
              

        //  alpha = getAlpha(t2_x, t2_y, index_num2);




        //     /*------逆運動学-------*/
        //     float v_x = base_power * sin(alpha);
        //     float v_y = base_power * cos(alpha);
        //     float v_0 = v_x;
        //     float v_1 = -1 * v_x/2 + v_y * sqrt(3)/2;
        //     float v_2 = -1 * v_x/2 - v_y * sqrt(3)/2;
        //     /*--------------------*/

        //     int V_0 = (int)v_0;
        //     int V_1 = (int)v_1;
        //     int V_2 = (int)v_2;

        //     Serial.print("now_x : ");
        //     Serial.print(now_x);
        //     Serial.print("  ");
        //     Serial.print("now_y : ");
        //     Serial.print(now_y);
        //     Serial.print("  ");
        //     Serial.print(target_distance);
        //     Serial.print("  ");
        //     Serial.print(alpha);


        //     if (i > inc_case2){
        //       inc_case2++;
        //     }
        //     else {
        //       if (target_distance < lf + kimu){    // Speed Down
        //         V_0 = V_0 / 2;
        //         V_1 = V_1 / 2;
        //         V_2 = V_2 / 2;
        //         motorrun(mo0_1, mo0_2, V_0);
        //         motorrun(mo1_1, mo1_2, V_1);
        //         motorrun(mo2_1, mo2_2, V_2);
        //       }
        //       else{
        //         motorrun(mo0_1, mo0_2, V_0);
        //         motorrun(mo1_1, mo1_2, V_1);
        //         motorrun(mo2_1, mo2_2, V_2);
        //       }
        //     }
        //     Serial.print("  ");
        //     Serial.print(i);

        //     Serial.print("  ");
        //     Serial.println(inc_case2);

        //   }
        // }
        // stop();
        if(digitalRead(strategypin)==HIGH){  
          num++;
          break;
        }
        else {
          num+=2;
          break;
        }
      }
      case 3:{                                 // TASK_0  Monster1つ目 
                  Serial.println(num);

       // delay(3000);
        stop();
        servoBottom_R.write(10);    
        servoBottom_L.write(90);
        delay(1000);


        // servoTop_R.write(40);
        // servoTop_L.write(60);
        // delay(3000);

        if(digitalRead(startpin) == HIGH){
          num = 1;
          break;
        }

            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,Sp);
            delay(1400);
            motorrun(liftA,liftB, 0);
        lift_pos = 160;

        // Serial.println(lift_pos);

        // servoTop_R.write(5);    // hold
        // servoTop_L.write(95);
        // delay(1500);

        // servoBottom_R.write(60);   // open a little
        // servoBottom_L.write(40);
        // delay(1500);



        // motordigi(liftA, liftB, -100);
        // delay((lift_Max - 0)/55*500-100);
        // lift_pos = 0;
        // motordigi(liftA,liftB,0);
        // Serial.println(lift_pos);

        // servoBottom_R.write(50);   //open
        // servoBottom_L.write(50);
        // delay(1000);

        num++;

        break;
      }
      case 4:{                                 // LINE_1
                  Serial.println(num);

        i = 0;
        int inc_case4 = 0;

        const int index_num4 = 7; 
        double t4_x[index_num4] = {0,-50,-100,-150,-200,-230,-225};
        double t4_y[index_num4] = {-5,-5,-5,-5,-5,-5,0}; 
        // double t4_x[index_num4] = {0, 0, 0};
        // double t4_y[index_num4] = {200, 0, 200}; 

        while(i < index_num4){
          if(digitalRead(startpin) == HIGH){
            num = 0;
            break;
          }
          else{

        alpha = getAlpha(t4_x, t4_y, index_num4);

            /*------逆運動学-------*/
            float v_x = base_power * cos(alpha);
            float v_y = base_power * sin(alpha);
            //float v_x = 0;
            //float v_y = -200;
            int v_0 = v_x;
            int v_1 = -1 * v_x/2 + v_y * sqrt(3)/2;
            int v_2 = -1 * v_x/2 - v_y * sqrt(3)/2;
            /*--------------------*/

            int V_0 = (int)v_0;
            int V_1 = (int)v_1;
            int V_2 = (int)v_2;

            //Serial.print(e0c);
            //Serial.print("  ");
            // Serial.print("e1c: ");
            // Serial.print(e1c);
            // Serial.print("  ");
            // Serial.print("e2c: ");
            // Serial.print(e2c);
            // Serial.print("  ");
            
            Serial.print("case_4 ");
            Serial.print("  ");
            Serial.print("now_x: ");
            Serial.print(now_x);
            Serial.print("  ");
            Serial.print("now_y: ");
            Serial.print(now_y);
            Serial.print("  ");

            Serial.print("alpha: ");
            Serial.print(alpha);
            Serial.print("  ");
            Serial.print("target_distance: ");
            Serial.print(target_distance);

              /* if (i == index_num4){
                stop();
                break;
              }*/
              
              //else 
              if (target_distance < lf + kimu){
               V_0 = V_0 / 2;
               V_1 = V_1 / 2;
               V_2 = V_2 / 2;

                Serial.print("  ");
                Serial.print("v_1: ");
                Serial.print(v_1);
                Serial.print("  ");
                Serial.print("v_2: ");
                Serial.print(v_2);
                motorrun(mo0_1, mo0_2, v_0);
                motorrun(mo1_1, mo1_2, v_1);
                motorrun(mo2_1, mo2_2, v_2);
              }
              else{
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
            }
            Serial.print("  ");
            Serial.print("target_index: ");
            Serial.println(i);
          }
        stop();
        
        if(digitalRead(strategypin)==HIGH){  
          num+=2;
          break;
        }
        else {
          num++;
          break;
        }
      }
      case 5:{                                 // TASK_1  Monster2つ目

        delay(3000);

        servoBottom_R.write(10);
        servoBottom_L.write(90);
        delay(1000);
        

            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,Sp);
            delay(1800);
            motorrun(liftA,liftB, 0);
        lift_pos = 160;

        if(digitalRead(startpin) == HIGH){
          num = 1;
          break;
        }

        delay(1500);

        num++;
        break;
      }
      case 6:{                                 // LINE_2
        encoder.set();
        resetpos();

        i = 0;
        int inc_case6 = 0;

        const int index_num6 = 6; 
        double t6_x[index_num6] = {0,-50,-100,-150,-200,-230};
        double t6_y[index_num6] = {-15,-10,-10,-10,-10,-10}; 

        while(i < index_num6){
          if(digitalRead(startpin) == HIGH){
            num = 0;
            break;
          }
          else{
            alpha = getAlpha(t6_x, t6_y, index_num6);

            /*------逆運動学-------*/
            float v_x = base_power * cos(alpha);
            float v_y = base_power * sin(alpha);
            float v_0 = v_x;
            float v_1 = -1 * v_x/2 + v_y * sqrt(3)/2;
            float v_2 = -1 * v_x/2 - v_y * sqrt(3)/2;
            /*--------------------*/

            int V_0 = (int)v_0;
            int V_1 = (int)v_1;
            int V_2 = (int)v_2;

            // Serial.print("now_x : ");
            // Serial.print(now_x);
            // Serial.print("  ");
            // Serial.print("now_y : ");
            // Serial.print(now_y);
            // Serial.print("  ");
            // Serial.print(target_distance);
            // Serial.print("  ");
            // Serial.print(alpha);
            Serial.print("case_6 ");
            Serial.print("  ");
            Serial.print("now_x: ");
            Serial.print(now_x);
            Serial.print("  ");
            //Serial.print(target_distance);
            // Serial.print("  ");
            Serial.print("now_y: ");
            Serial.print(now_y);
            Serial.print("  ");
            Serial.print("alpha: ");
            Serial.print(alpha);
            Serial.print("  ");
            Serial.print("target_distance: ");
            Serial.print(target_distance);
            Serial.print("  ");
            Serial.print("v_0: ");
            Serial.print(v_0);
            Serial.print("  ");
            Serial.print("v_1: ");
            Serial.print(v_1);
            Serial.print("  ");
            Serial.print("v_2: ");
            Serial.print(v_2);


              /*
              if(i == index_num6){
                stop();
                break;
              }
              */
              //else 
              if (target_distance < lf + kimu){
                
                V_0 = V_0 / 2;
                V_1 = V_1 / 2;
                V_2 = V_2 / 2;
                
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
              else{
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
            }
            Serial.print("  ");
            Serial.print("target_index: ");
            Serial.println(i);
          
          }
      
        stop();
        num++;
        break;
      }
      case 7:{                                 // TASK_2  回転
        delay(3000);
        roundR();
        delay(1500);
        stop();
        delay(1500);

        encoder.set();
        resetpos();
        
        num++;
        break;
      }
      case 8:{                                 // LINE_3
        encoder.set();
        resetpos();
        i = 0;
        int inc_case8 = 0;

        const int index_num8 = 5; 
        double t8_x[index_num8] = {50,100,110,110,110};
        double t8_y[index_num8] = {0, 10,10,40,60}; 

        while(i < index_num8){
          if(digitalRead(startpin) == HIGH){
            num = 0;
            break;
          }
          else{

          alpha = getAlpha(t8_x, t8_y, index_num8);

            /*------逆運動学-------*/
            float v_x = base_power * cos(alpha);
            float v_y = base_power * sin(alpha);
            float v_0 = v_x;
            float v_1 = -1 * v_x/2 + v_y * sqrt(3)/2;
            float v_2 = -1 * v_x/2 - v_y * sqrt(3)/2;
            /*--------------------*/

            int V_0 = (int)v_0;
            int V_1 = (int)v_1;
            int V_2 = (int)v_2;

            
            Serial.print("case_8 ");
            Serial.print("  ");
            Serial.print("now_x: ");
            Serial.print(now_x);
            Serial.print("  ");
            Serial.print("now_y: ");
            Serial.print(now_y);
            Serial.print("  ");
            //Serial.print(target_distance);
            // Serial.print("  ");
            Serial.print("alpha: ");
            Serial.print(alpha);
            Serial.print("  ");
            Serial.print("target_distance: ");
            Serial.print(target_distance);
            Serial.print("  ");
            Serial.print("v_0: ");
            Serial.print(v_0);
            Serial.print("  ");
            Serial.print("v_1: ");
            Serial.print(v_1);
            Serial.print("  ");
            Serial.print("v_2: ");
            Serial.print(v_2);
            
            //delay(10);
              /*
              if(i == index_num8){
                stop();
                break;
              }
              */
              //else 
              if (target_distance < lf + kimu){
                
                V_0 = V_0 / 2;
                V_1 = V_1 / 2;
                V_2 = V_2 / 2;
                
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
              else{
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
            }
            Serial.print("  ");
            Serial.print("target_index: ");
            Serial.println(i);

          }

        stop();

        if(digitalRead(strategypin)==HIGH){  
          num++;
          break;
        }
        else {
          num+=2;
          break;
        }
      }
      case 9:{                                 // TASK_3  1つ目おろす
        delay(3000);

            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(730);
            motorrun(liftA,liftB, 0);
        delay(1000);
        lift_pos = 50;

        servoBottom_R.write(30);
        servoBottom_L.write(70);
        delay(3000);

        servoBottom_R.write(10);
        servoBottom_L.write(90);
        delay(1500);
       
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,Sp);
            delay(900);
            motorrun(liftA,liftB, 0);
        lift_pos = 160;


        num++;
        break;
      }
      case 10:{                                // LINE_4
        encoder.set();
        resetpos();
        i = 0;
        int inc_case10 = 0;

        const int index_num10 = 10; 
        // double t10_x[index_num10] = {0,500,500};
        // double t10_y[index_num10] = {80,80,80}; 
        double t10_x[index_num10] = {0,0, 100, 200, 300, 400, 500,570,570,570};
        double t10_y[index_num10] = {-30,-80, -100, -150, -150, -150, -150,-150,-90,-90}; 

        while(i < index_num10){
          if(digitalRead(startpin) == HIGH){
            num = 0;
            break;
          }
          else{

          alpha = getAlpha(t10_x, t10_y, index_num10);

            /*------逆運動学-------*/
            float v_x = base_power * cos(alpha);
            float v_y = base_power * sin(alpha);
            float v_0 = v_x;
            float v_1 = -1 * v_x/2 + v_y * sqrt(3)/2;
            float v_2 = -1 * v_x/2 - v_y * sqrt(3)/2;
            /*--------------------*/

            int V_0 = (int)v_0;
            int V_1 = (int)v_1;
            int V_2 = (int)v_2;

            Serial.print("case_10 ");
            Serial.print("  ");
            Serial.print("now_x: ");
            Serial.print(now_x);
            Serial.print("  ");
            Serial.print("now_y: ");
            Serial.print(now_y);
            Serial.print("  ");
            // Serial.print(target_distance);
            // Serial.print("  ");
            Serial.print("alpha: ");
            Serial.print(alpha);
            Serial.print("  ");
            Serial.print("target_distance: ");
            Serial.print(target_distance);
            Serial.print("  ");
            Serial.print("v_0: ");
            Serial.print(v_0);
            Serial.print("  ");
            Serial.print("v_1: ");
            Serial.print(v_1);
            Serial.print("  ");
            Serial.print("v_2: ");
            Serial.print(v_2);
/*
              if(i == index_num10){
                stop();
                break;
              }
              */
              //else 
              if (target_distance < lf + kimu){
                
                V_0 = V_0 / 2;
                V_1 = V_1 / 2;
                V_2 = V_2 / 2;
                
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
              else{
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
            }
            Serial.print("  ");
            Serial.print("target_index: ");
            Serial.println(i);

          }

        stop();
        num++;
        break;

      }
      case 11:{                                // TASK_4  2つ目おろす
        delay(3000);

            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(350);
            motorrun(liftA,liftB, 0);
        lift_pos = 100;

        servoBottom_R.write(30);
        servoBottom_L.write(70);
        delay(3000);

        servoBottom_R.write(10);
        servoBottom_L.write(90);
        delay(1500);
       
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,Sp);
            delay(200);
            motorrun(liftA,liftB, 0);
        lift_pos = 160;


        num++;
        break;
      }
      case 12:{                                // LINE_5
        encoder.set();
        resetpos();
        i = 0;
        int inc_case12 = 0;

        const int index_num12 = 2; 
        double t12_x[index_num12] = {0,0};
        double t12_y[index_num12] = {-50,-100}; 

        while(i < index_num12){
          if(digitalRead(startpin) == HIGH){
            num = 0;
            break;
          }
          else{

         alpha = getAlpha(t12_x, t12_y, index_num12);

            /*------逆運動学-------*/
            float v_x = base_power * cos(alpha);
            float v_y = base_power * sin(alpha);
            float v_0 = v_x;
            float v_1 = -1 * v_x/2 + v_y * sqrt(3)/2;
            float v_2 = -1 * v_x/2 - v_y * sqrt(3)/2;
            /*--------------------*/

            int V_0 = (int)v_0;
            int V_1 = (int)v_1;
            int V_2 = (int)v_2;

            // Serial.print("now_x : ");
            // Serial.print(now_x);
            // Serial.print("  ");
            // Serial.print("now_y : ");
            // Serial.print(now_y);
            // Serial.print("  ");
            // Serial.print(target_distance);
            // Serial.print("  ");
            // Serial.print(alpha);
            Serial.print("case_12 ");
            Serial.print("  ");
            Serial.print("now_x: ");
            Serial.print(now_x);
            Serial.print("  ");
            Serial.print("now_y: ");
            Serial.print(now_y);
            Serial.print("  ");
            // Serial.print(target_distance);
            // Serial.print("  ");
            Serial.print("alpha: ");
            Serial.print(alpha);
            Serial.print("  ");
            Serial.print("target_distance: ");
            Serial.print(target_distance);
            Serial.print("  ");
            Serial.print("v_0: ");
            Serial.print(v_0);
            Serial.print("  ");
            Serial.print("v_1: ");
            Serial.print(v_1);
            Serial.print("  ");
            Serial.print("v_2: ");
            Serial.print(v_2);

            /*
              if(i == index_num12){
                stop();
                break;
              }
              */
              //else 
              if (target_distance < lf + kimu){
                V_0 = V_0 / 2;
                V_1 = V_1 / 2;
                V_2 = V_2 / 2;
                
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
              else{
                motorrun(mo0_1, mo0_2, V_0);
                motorrun(mo1_1, mo1_2, V_1);
                motorrun(mo2_1, mo2_2, V_2);
              }
              
            }
            Serial.print("  ");
            Serial.print("target_index: ");
            Serial.println(i);

          }
        stop();
        num++;
        break;
      }
      default:{                                     // FINISH & STAY until reset
        if (digitalRead(startpin) == HIGH){
          num = 1;
          break;
        }
        else{
          motorrun(mo0_1, mo0_2, 0);
          motorrun(mo1_1, mo1_2, 0);
          motorrun(mo2_1, mo2_2, 0);

          if(lift_pos == 0){
            //motordigi(liftA,liftB,0);
            motorrun(liftA,liftB, 0);
            lift_pos = 0;
          }
          else if (lift_pos == 50){
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(540);
            motorrun(liftA,liftB, 0);


            lift_pos = 0;

          }
          else if (lift_pos == 100){
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(990);
            motorrun(liftA,liftB, 0);
            lift_pos = 0;

          }
          else if (lift_pos == 160){
            // motordigi(liftA,liftB,-100);
            // delay(540);
            // motordigi(liftA,liftB,0);
            motorrun(liftA,liftB,-1*Sp);
            delay(1490);
            motorrun(liftA,liftB, 0);
            lift_pos = 0;

          }
          else{
            motorrun(liftA,liftB,0);
            lift_pos = 0;

          }
            motorrun(liftA,liftB,0);
          SetBottom();
          // SetTop();          
          digitalWrite(LEDpin, HIGH);
          delay(2000);
          digitalWrite(LEDpin, LOW);
          delay(50);

          // Serial.println("FINISH");
          
          break;        }
      }
    }
}