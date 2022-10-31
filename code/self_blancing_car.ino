#include <SPI.h>
#include <RF24.h>
#include <MPU6050_tockn.h>


int V_feedback;
volatile long counter_value_L, counter_value_R;
int V_count, T_count;
float e_AngleY, e_Velocity, sum_e_Velocity, e_AngleZ, sum_e_AngleY;
float B_Pwm, V_Pwm, T_Pwm, Pluse_L, Pluse_R, Angle_Turn_Setting, V_Setting;
float angleY, angleZ, gyroY, gyroZ;
long loop_timer;

MPU6050 mpu6050(Wire);

RF24 myRadio (39, 41);    //NRF24L01 Pin(CE,CSN)
struct package
{
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};

byte addresses[][6] = {"0"};
typedef struct package Package;
Package data;


int pinA1 = 5;           //TB6612FNG
int pinA2 = 4;
int pwmA  = 6;

int pinB1 = 25;
int pinB2 = 24;
int pwmB  = 9;


void resetData()
{
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.pot1 = 0;
  data.pot2 = 0;
  data.tSwitch1 = 0;
  data.tSwitch2 = 0;
  data.button1 = 0;
  data.button2 = 0;
  data.button3 = 0;
  data.button4 = 0;
}


void setup()
{
  cli();
  TCCR4A = 0;                                         //timer4 setting
  TCCR4B = 0;
  TCCR4B |= (1 << CS40);                              //no prescaling
  TIMSK4 |= (1 << TOIE4);

  TCCR5A = 0;                                         //timer5 setting
  TCCR5B = 0;
  TCCR5B |= (1 << CS50);                              //no prescaling
  TIMSK5 |= (1 << TOIE5);
  sei();


  Serial.begin(115200);

  resetData();
  reset();

  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pwmA, OUTPUT);

  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  pinMode(pwmB, OUTPUT);

  pinMode(2,  INPUT_PULLUP);
  pinMode(3,  INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  attachInterrupt(0, counter0, FALLING);           //interrupt pin2;
  attachInterrupt(1, counter1, FALLING);           //interrupt pin3;

  attachInterrupt(5, counter2, FALLING);           //interrupt pin18;
  attachInterrupt(4, counter3, FALLING);           //interrupt pin19;


  myRadio.begin();
  myRadio.setChannel(115);
  myRadio.setPALevel(RF24_PA_LOW);
  myRadio.setDataRate(RF24_250KBPS) ;
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  delay(1000);
}


void loop()
{
  mpu6050.update();
  angleY = mpu6050.getAngleY();                                            //get angle and gyro value
  angleZ = mpu6050.getAngleZ();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();

  //  Serial.println(angleY);


  while (myRadio.available())
  {
    myRadio.read( &data, sizeof(data) );
  }


  if (data.j2PotY > 180)                                                          //velovity setting(RPM)
  {
    V_Setting = map(data.pot2, 0, 255, 30, 200);
  }
  else if (data.j2PotY < 80)
  {
    V_Setting = map(data.pot2, 0, 255, -30, -200);
  }
  else                                                                            //apply the brake
  {    
    V_Setting =0;
    if(sum_e_Velocity-200>0) 
    {
    sum_e_Velocity-=200;
    }
    else if(sum_e_Velocity+200<0)
    {
    sum_e_Velocity+=200;
    }
    else
    {
     sum_e_Velocity=0; 
    }
  }


  Angle_Turn_Setting = map(data.pot1, 0, 255, 180, -180);                         //angle of Z axis setting


  Blance_PD(angleY, gyroY);                                                       //blance pid runs each 5ms


  V_count++;
  if (V_count >= 2)
  {
    V_feedback = 60 * 100 * (counter_value_L - counter_value_R) / 1320;   //RPM；

    Velocity_PI( V_Setting, V_feedback);                                          //velocity pid runs each 10ms

    counter_value_L = 0;
    counter_value_R = 0;
    V_count = 0;
  }


  T_count++;
  if (T_count >= 4)
  {
    Turn_PID(gyroZ, angleZ, Angle_Turn_Setting);                                 //angle of Z axis pid runs each 20ms
    T_count = 0;
  }


  Pluse_L = Blance_PD(angleY, gyroY) - Velocity_PI(V_Setting, V_feedback) + Turn_PID(gyroZ, angleZ, Angle_Turn_Setting);       //Pluse of left Motor caculate
  Pluse_R = Blance_PD(angleY, gyroY) - Velocity_PI(V_Setting, V_feedback) - Turn_PID(gyroZ, angleZ, Angle_Turn_Setting);       //Pluse of left Motor caculate



  if (abs(angleY) < 60)                //if angleY is between -60 and 60 ,run it
  {
    if ((Pluse_L) < 0)
    {
      moveForwardL();
    }
    else
    {
      moveBackwardL();
    }

    if ((Pluse_R) < 0)
    {
      moveBackwardR();
    }
    else
    {
      moveForwardR();
    }
  }
  else
  {
    reset();
    Stop() ;
  }


  while (micros() - loop_timer < 5000);                                     //We wait until 5000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.
}


//---------------------------------------------------------------------------------------------------------
void timer1(int value)
{
  TCNT1 = 65535 - value * 16 ;
}
void timer3(int value)
{
  TCNT3 = 65535 - value * 16 ;
}



ISR(TIMER4_OVF_vect)
{
  static int i = 0;
  if (i == 0)
  {
    digitalWrite(pwmA, HIGH);
    timer4(abs(Pluse_L));
    i++;
  }
  if (i == 1)
  {
    digitalWrite(pwmA, LOW);
    timer4(5000 - abs(Pluse_L));
    i = 0;
  }
}


ISR(TIMER5_OVF_vect)
{
  static int i = 0;
  if (i == 0)
  {
    digitalWrite(pwmB, HIGH);
    timer5(abs(Pluse_R));
    i++;
  }
  if (i == 1)
  {
    digitalWrite(pwmB, LOW);
    timer5(5000 - abs(Pluse_R));
    i = 0;
  }
}

//-----------------------------------------------------------------------------------------------------------
float Blance_PD(float angleY, float gyroY)                                        //blance pid
{
  static float Kp = -26, Kd = -1;

  e_AngleY = angleY - 0;

  B_Pwm = Kp * e_AngleY + Kd * gyroY;

  B_Pwm = constrain(B_Pwm, -4000, 4000);

  return B_Pwm;
}


float Velocity_PI(float SpeedSet, float Speed)                                    //velocity pid
{
  static float e_low_out, e_low_last;
  static float Kp = -6, Ki = Kp / 200;

  e_Velocity = Speed - SpeedSet;
  e_low_out = 0.7 * e_low_last + 0.3 * e_Velocity;                                //Low-pass filter
  e_low_last = e_low_out;

  sum_e_Velocity += e_low_out;

  sum_e_Velocity = constrain(sum_e_Velocity, -4000, 4000);
  V_Pwm = Kp * e_low_out + Ki * sum_e_Velocity;

  V_Pwm = constrain(V_Pwm, -4000, 4000);

  return V_Pwm;
}


float Turn_PID(float gyroZ, float angleZ, float Angle_Turn_Setting)                 //angle of Z axis pid
{
  static float Kp = -1-, Ki = -1, Kd = -5;

  e_AngleZ = angleZ - Angle_Turn_Setting;
  sum_e_AngleY += e_AngleZ;
  T_Pwm = Kp * e_AngleZ + Ki * sum_e_AngleY + Kd * gyroZ;

  T_Pwm = constrain(T_Pwm, -4000, 4000);

  return T_Pwm;
}


void counter0()                                           //left Encoder caculate
{
  if (digitalRead(3) == HIGH)
  {
    counter_value_L--;
  }
  if (digitalRead(3) == LOW)
  {
    counter_value_L++;
  }
}

void counter1()
{
  if (digitalRead(2) == HIGH)
  {
    counter_value_L++;
  }
  if (digitalRead(2) == LOW)
  {
    counter_value_L--;
  }
}



void counter2()                                         //right Encoder caculate
{
  if (digitalRead(19) == HIGH)
  {
    counter_value_R++;
  }
  if (digitalRead(19) == LOW)
  {
    counter_value_R--;
  }
}
void counter3()
{
  if (digitalRead(18) == HIGH)
  {
    counter_value_R--;
  }
  if (digitalRead(18) == LOW)
  {
    counter_value_R++;
  }
}



void reset()
{
  sum_e_Velocity = 0;
  sum_e_AngleY = 0;
  B_Pwm = 0;
  V_Pwm = 0;
  T_Pwm = 0;
  T_count = 0;
  V_count = 0;
  Angle_Turn_Setting = 0;
  counter_value_L = 0;
  counter_value_R = 0;
}


void moveForwardL()                                                          //left motor Forward；
{
  digitalWrite(pinA1, LOW);
  digitalWrite(pinA2, HIGH);
}
void moveBackwardL()                                                         //left motor Backward；
{
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, LOW);
}




void moveForwardR()                                                          //right motor Forward；
{
  digitalWrite(pinB1, HIGH);
  digitalWrite(pinB2, LOW);
}
void moveBackwardR()                                                         //Right motor Backward；
{
  digitalWrite(pinB1, LOW);
  digitalWrite(pinB2, HIGH);
}


void Stop()                                                                  //stop all；
{
  digitalWrite(pinA1, LOW);
  digitalWrite(pinA2, LOW);
  digitalWrite(pinB1, LOW);
  digitalWrite(pinB2, LOW);
}
