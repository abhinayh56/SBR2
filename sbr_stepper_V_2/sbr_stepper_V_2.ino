/*  This loop only controls the tilt angle of vehicle and keep the vehicle in upright position
 *  For this program to work the reference angle of vehicle in balanced position should be calculated precisely so 
 *  that center of mass remains on top of wheel center point on horizontal flat surface
 *  Also ensure that gyroscope is calliberated
 *  This program only keep the vehicle upright, but the robot will keep on drifting due to no velocity feedback
 *  This drift can be compensated by using another controll loop which takes velocity of robot as feedback
*/

#include<Wire.h>
int MPU_addr=0x68; 

float AcX, AcY, AcZ;
float acc_sensitivity=4096;
float amx, amy, amz;
float phi_acc, th_acc;
float k=0.15, k2=0.002;
float AcX_P,AcY_P,AcZ_P;

float GyX, GyY, GyZ;
float GyXOff, GyYOff, GyZOff;
float gyro_sensitivity=65.5;
float p, q, r;
float phi_d_gyro, th_d_gyro;
float phi_gyro, th_gyro;
float sin_phi_gyro, cos_phi_gyro;

int n_L, n_0_L, n0_memry_L=10000, W_L = 1;
int n_R, n_0_R, n0_memry_R=10000, W_R = 1;

float setpoint_th, error_th, p_error_th, I_th, PID_th, kp_th, ki_th, kd_th;

float SampleFrequency = 250, dt = 1/SampleFrequency, loop_timer = 1000000*dt;
unsigned long t;
unsigned long start;

void setup(){
  Serial.begin(57600);
  Wire.begin();
  TWBR = 12;

  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2B register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 = 20us/(1s/(16.000.000MHz/8))-1
  TCCR2A |= (1 << WGM21);                                                   //Set ner 2 to CTC (clear timer on compare) mode

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(MPU_addr);                                         //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(MPU_addr);                                         //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(MPU_addr);                                         //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(MPU_addr);                                         //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission(); 

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(6,1);
  
  /*for(int i = 0; i < 500; i++){
    if(i % 15 == 0)digitalWrite(13, !digitalRead(13));
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU_addr, 6);
    GyX = Wire.read()<<8|Wire.read();
    GyY = Wire.read()<<8|Wire.read();
    GyZ = Wire.read()<<8|Wire.read();
    GyXOff += GyX;
    GyYOff += GyY;
    GyZOff += GyZ;
    delayMicroseconds(3700);                                                   //Wait for 3700 microseconds to simulate the main program loop time
  }
  GyXOff /= 500;
  GyYOff /= 500;
  GyZOff /= 500;  
  Serial.print("GyXOff : "); Serial.println(GyXOff);
  Serial.print("GyYOff : "); Serial.println(GyYOff);
  Serial.print("GyZOff : "); Serial.println(GyZOff);*/
  GyXOff = -171.79;
  GyYOff = 58.25;
  GyZOff = -62.60;

  digitalWrite(6,0);
  
  t = micros()+loop_timer;
}

void loop(){
  // angle calculation starts
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 6);
  AcX = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();

  AcX = AcX*k + (1-k)*AcX_P;
  AcY = AcY*k + (1-k)*AcY_P;
  AcZ = AcZ*k + (1-k)*AcZ_P;
  AcX_P = AcX;
  AcY_P = AcY;
  AcZ_P = AcZ;
  
  amx = AcX/acc_sensitivity;
  amy = AcY/acc_sensitivity;
  amz = AcZ/acc_sensitivity;

  if(amx>1)amx=1;
  if(amy>1)amy=1;
  if(amz>1)amz=1;
  
  phi_acc = atan2(amy,amz);
  th_acc = -asin(amx/sqrt(amx*amx + amy*amy + amz*amz));

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 6);
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
  p = (GyX-GyXOff)*0.00026646248122050833235476194938757; // PI = 3.1415926535897932384626433832795
  q = (GyY-GyYOff)*0.00026646248122050833235476194938757; // PI/(gyro_sensitivity*180) = 0.00026646248122050833235476194938757
  r = (GyZ-GyZOff)*0.00026646248122050833235476194938757;

  sin_phi_gyro = sin(phi_gyro);
  cos_phi_gyro = cos(phi_gyro);
  phi_gyro = phi_gyro + (p + (q*sin_phi_gyro + r*cos_phi_gyro)*tan(th_gyro))*dt;
  th_gyro  = th_gyro + (q*cos_phi_gyro - r*sin_phi_gyro)*dt;
  
  if(start==0){
      phi_gyro = phi_acc;
      th_gyro = th_acc;
      start = 1;
  }

  phi_gyro = phi_gyro + k2*(phi_acc - phi_gyro);
  th_gyro = th_gyro + k2*(th_acc - th_gyro);
  // angle calculation ends

  // controller starts
  // Inner loop controller i.e. inner loop controller
  setpoint_th = 0-(2.5); // it is the reference angle at upright position when the robot is balanced
  
  kp_th = 60; // 25, 35, 60, 60, 60, 60
  ki_th = 0.8; // 1.25, 1.25, 1.25, 1.25, 0.8, 0.75
  kd_th = 70;// 15, 15, 15, 70, 70, 100
  error_th = setpoint_th-th_gyro*57.295;
  I_th = I_th + ki_th*error_th;
  if(I_th>150)I_th=150;
  else if(I_th<-150)I_th=-150;
  PID_th = kp_th*error_th + I_th + kd_th*(error_th-p_error_th);
  if(PID_th>250)PID_th=250;
  else if(PID_th<-250)PID_th=-250;
  p_error_th = error_th;  
  // controller ends
  
  // motor pulse calculation starts
  W_L = -PID_th;
  W_R = -PID_th;
  n0_memry_L = 3750/W_L;
  n0_memry_R = 3750/W_R;
  // motor pulse calculation ends

  Serial.println(PID_th);
  //Serial.println(t-micros()); // don't print this line because the refresh rate of void loop is very high
  while(micros()<t);
  t += loop_timer;
}

ISR(TIMER2_COMPA_vect){
  // Left motor
  if(n_L==0 || n_L>=n_0_L){                                    //If the number of loops is larger then the n_memory variable
    n_L = 0;                                               //Reset the n variable
    n_0_L = n0_memry_L;
    if(n_0_L<0){
      PORTD &= 0b11011111;                                //Set output 5 low to reverse the direction of the stepper controller
      n_0_L = -n_0_L;
    }
    else PORTD |= 0b00100000;                               //Set output 5 high for a forward direction of the stepper motor
    PORTD |= 0b00010000;                                 //Set output 4 high to create a pulse for the stepper controller
  }
  else if(n_L==1)PORTD &= 0b11101111;                      //Set output 4 low because the pulse only has to last for 20us 
  n_L ++;

  // Right Motor
  if(n_R==0 || n_R>=n_0_R){                                    //If the number of loops is larger then the n_memory variable
    n_R = 0;                                               //Reset the n variable
    n_0_R = n0_memry_R;
    if(n_0_R<0){
      PORTD |= 0b00001000;                               //Set output 3 low to reverse the direction of the stepper controller
      n_0_R = -n_0_R;
    }
    else PORTD &= 0b11110111;
    PORTD |= 0b00000100;                                 //Set output 2 high to create a pulse for the stepper controller
  }
  else if(n_R==1)PORTD &= 0b11111011;                      //Set output 2 low because the pulse only has to last for 20us 
  n_R ++;
}
