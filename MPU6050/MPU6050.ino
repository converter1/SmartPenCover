/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <core.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#include <linux/input.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define PRESS 1
#define RELEASE 0
#define CORR_LENGTH 10
#define ARRAY_LENGTH 30
#define AMP 20
#define INPUT_TIME 300
#define MOVE_SHORT_THRESHOLD -200
#define MOVE_LONG_THRESHOLD 500

#define SensorAddressWrite 0x29 //
#define SensorAddressRead 0x29 // 
#define EnableAddress 0xa0 // register address + command bits
#define ATimeAddress 0xa1 // register address + command bits
#define WTimeAddress 0xa3 // register address + command bits
#define ConfigAddress 0xad // register address + command bits
#define ControlAddress 0xaf // register address + command bits
#define IDAddress 0xb2 // register address + command bits
#define ColorAddress 0xb4 // register address + command bits
#define COLORREGULATOR  80
#define EV_RELEASED 0
#define EV_PRESSED 1  

void button_control(int);
void move_cursor(int, int);
double corr(double *, double *);

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

double pre_kalAngleX = 0, pre_kalAngleY = 0;
double kal_accx, kal_accy;
double pre_kal_accx = 0, pre_kal_accy = 0;
double pre_accx = 0, pre_accy = 0, pre_accz = 0;
double cal_accz;
double pre_cal_accz = 0;

double c_accx[CORR_LENGTH] = {0}, c_accy[CORR_LENGTH] = {0};
double c_cal_accx[CORR_LENGTH] = {0}, c_cal_accy[CORR_LENGTH] = {0};
double c_result_x, c_result_y;
double pre_c_result_x = 0, pre_c_result_y = 0;
int i;

int move_x, move_y;

int move_array[2][ARRAY_LENGTH] = {0};
int array_count = 0;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// set pin numbers:
const int press_ledPin = 1;      // the number of the LED pin
const int recog_ledPin = 2;
const int error_ledPin = 3;
const int save_btnPin = 7;
const int reset_btnPin = 6;
const int rgb_btnPin = 5;

// Variables will change:
int press_ledState = LOW;             // ledState used to set the LED
int recog_ledState = LOW;
int error_ledState = LOW;

int reset_btnState = LOW;
int rgb_btnState = LOW;
int pre_rgb_btnState = LOW;
int save_btnState = LOW;
int pre_save_btnState = LOW;

int threshold_flag = 0;
int input_flag = 0;

int sum_move_x = 0;
int sum_move_y = 0;

struct input_event event_x, event_y, event_end, event_btn;

int fd_mouse = open("/dev/input/event1", O_RDWR);

byte i2cWriteBuffer[10];
byte i2cReadBuffer[10];

int fd_keyboard = open("/dev/input/event2", O_RDWR);

unsigned long red_data=0;
unsigned long green_data=0;
unsigned long blue_data=0;
unsigned long select_red[50];
unsigned long select_green[50];
unsigned long select_blue[50];
unsigned long red_sum;
unsigned long green_sum;
unsigned long blue_sum;
unsigned long rgb_data;

struct input_event event_key_start, event_key_end;

void select_key(unsigned int);
void key_pressed(unsigned long);
void keyboard(void);
void Writei2cRegisters(byte , byte);
byte Readi2cRegisters(int , byte);
void init_TCS34725(void);
void get_TCS34725ID(void);
void get_Colors(void);

void setup() {
  Wire.begin();

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    printf("Error reading sensor");
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (short)((i2cData[0] << 8) | i2cData[1]);
  accY = (short)((i2cData[2] << 8) | i2cData[3]);
  accZ = (short)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  timer = micros();

  if(!fd_mouse){
    printf("Errro open mouse:\n");
  }
  memset(&event_x, 0, sizeof(event_x));
  memset(&event_y, 0, sizeof(event_y));
  memset(&event_end, 0, sizeof(event_end));
  memset(&event_btn, 0, sizeof(event_btn));

  gettimeofday(&event_x.time, NULL);
  gettimeofday(&event_y.time, NULL);
  gettimeofday(&event_end.time, NULL);
  gettimeofday(&event_btn.time, NULL);

  pre_kalAngleX = kalAngleX;
  pre_kalAngleY = kalAngleY;

  event_x.type = EV_REL;
  event_y.type = EV_REL;
  event_btn.type = 1;
  event_end.type = EV_SYN;
  event_end.code = SYN_REPORT;
  event_end.value = 0;

  // set the digital pin as output:
  pinMode(press_ledPin, OUTPUT);
  pinMode(recog_ledPin, OUTPUT);
  pinMode(error_ledPin, OUTPUT);
  digitalWrite(press_ledPin, press_ledState);
  digitalWrite(recog_ledPin, recog_ledState);
  digitalWrite(error_ledPin, error_ledState);
  pinMode(reset_btnPin, INPUT);
  pinMode(rgb_btnPin, INPUT);
  pinMode(save_btnPin, INPUT);

  init_TCS34725();
  get_TCS34725ID();     // get the device ID, this is just a test to see if we're connected

  key_pressed(KEY_F12);
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (short)((i2cData[0] << 8) | i2cData[1]);
  accY = (short)((i2cData[2] << 8) | i2cData[3]);
  accZ = (short)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (short)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (short)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (short)((i2cData[12] << 8) | i2cData[13]);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } 
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  } 
  else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  reset_btnState = digitalRead(reset_btnPin); //get button status

  if(reset_btnState == HIGH){
    press_ledState = LOW;
    digitalWrite(press_ledPin, press_ledState);
    recog_ledState = LOW;
    digitalWrite(recog_ledPin, recog_ledState);
    delay(500);
    button_control(RELEASE);
    while(reset_btnState == HIGH){
      reset_btnState = digitalRead(reset_btnPin);
    }
    sum_move_x = 0;
    sum_move_y = 0;
    memset(move_array, 0, sizeof(move_array));
  }

  accX /= 256;
  accY /= 256;
  accZ /= 256;
  cal_accz = sqrt(accX * accX + accY * accY + accZ * accZ) - 64;
  cal_accz = 0.8 * pre_cal_accz + 0.2 * cal_accz;

  accX += 64;
  kal_accx = -64 * sin(kalAngleY * DEG_TO_RAD) + 64;
  kal_accx = 0.8 * pre_kal_accx + 0.2 * kal_accx;

  for(i = 0; i < CORR_LENGTH - 1; i++){
    c_cal_accx[i] = c_cal_accx[i + 1];
    c_accx[i] = c_accx[i + 1];
  }
  c_cal_accx[i] = kal_accx;
  c_accx[i] = accX;
  c_result_x = corr(c_cal_accx, c_accx);
  c_result_x = 0.8 * pre_c_result_x + 0.2 * c_result_x;

  if(c_result_x < -2){
    move_x = AMP * (pre_kalAngleY - kalAngleY);
  }
  else{
    move_x = 0;
  }

  accY += 64;
  kal_accy = 64 * sin(kalAngleX * DEG_TO_RAD) + 64;
  kal_accy = 0.8 * pre_kal_accy + 0.2 * kal_accy;

  for(i = 0; i < CORR_LENGTH - 1; i++){
    c_cal_accy[i] = c_cal_accy[i + 1];
    c_accy[i] = c_accy[i + 1];
  }
  c_cal_accy[i] = kal_accy;
  c_accy[i] = accY;
  c_result_y = corr(c_cal_accy, c_accy);
  c_result_y = 0.8 * pre_c_result_y + 0.2 * c_result_y;

  if(c_result_y < -1){
    move_y = AMP * (pre_kalAngleX - kalAngleX);
  }
  else{
    move_y = 0;
  }

  move_array[0][array_count] = move_x;
  move_array[1][array_count] = move_y;

  array_count++;
  if(array_count == ARRAY_LENGTH){
    array_count = 0;
  }

  if(cal_accz > 8 && threshold_flag == 0){
    threshold_flag = 1;
    if(press_ledState == HIGH){
      press_ledState = LOW;
      digitalWrite(press_ledPin, press_ledState);
      button_control(RELEASE);
      move_cursor(20, 0);
    }
    else if(press_ledState == LOW){
      press_ledState = HIGH;
    }
  }
  else if(cal_accz < 1 && threshold_flag == 1){
    threshold_flag = 0;
    if(press_ledState == HIGH){
      memset(move_array, 0, sizeof(move_array));

      if(input_flag == 0){
        move_cursor(-1000, -1000);
        move_cursor(210, 210);
        recog_ledState = HIGH;
        digitalWrite(recog_ledPin, recog_ledState);
      }
      input_flag = 1;
      digitalWrite(press_ledPin, press_ledState);
      button_control(PRESS);
    }
    else if(press_ledState == LOW){
    }
  }


  if(press_ledState == HIGH){
    move_cursor(move_array[0][array_count], move_array[1][array_count]);
    sum_move_x += move_x;
    sum_move_y += move_y;
  }
  else if(press_ledState == LOW){
    if(input_flag != 0){
      input_flag++;
    }
    if(input_flag > INPUT_TIME){
      input_flag = 0;
      press_ledState = LOW;
      digitalWrite(press_ledPin, press_ledState);
      recog_ledState = LOW;
      digitalWrite(recog_ledPin, recog_ledState);
      error_ledState = LOW;
      digitalWrite(error_ledPin, error_ledState);
      delay(1000);
      sum_move_x = 0;
      sum_move_y = 0;
    }
  }

  if(sum_move_x > MOVE_LONG_THRESHOLD || sum_move_y > MOVE_LONG_THRESHOLD || sum_move_x < MOVE_SHORT_THRESHOLD || sum_move_y < MOVE_SHORT_THRESHOLD){
    error_ledState = HIGH;
    digitalWrite(error_ledPin, error_ledState);
    press_ledState = LOW;
    digitalWrite(press_ledPin, press_ledState);
    button_control(RELEASE);
    sum_move_x = 0;
    sum_move_y = 0;
  }

  pre_kal_accx = kal_accx;
  pre_accx = accX;
  pre_kalAngleY = kalAngleY;

  pre_kal_accy = kal_accy;
  pre_accy = accY;
  pre_kalAngleX = kalAngleX;

  pre_accz = accZ;

  pre_cal_accz = cal_accz;

  pre_c_result_x = c_result_x;
  pre_c_result_y = c_result_y;

  delay(2);
  
  while(digitalRead(rgb_btnPin) == HIGH){
    rgb_btnState = HIGH;
    if(pre_rgb_btnState == LOW){
      delay(1000);
      key_pressed(KEY_F12);
      get_Colors();
      key_pressed(KEY_F12);
    }
    pre_rgb_btnState = rgb_btnState;
  }
  pre_rgb_btnState = LOW;
  
  while(digitalRead(save_btnPin) == HIGH){
    save_btnState = HIGH;
    if(pre_save_btnState == LOW){
      key_pressed(KEY_F1);
    }
    pre_save_btnState = save_btnState;
  }
}

void button_control(int control){
  event_btn.code = BTN_LEFT;
  event_btn.value = control;
  write(fd_mouse, &event_btn, sizeof(event_btn));
  write(fd_mouse, &event_end, sizeof(event_end));
}

void move_cursor(int x, int y){
  event_x.code = REL_X;
  event_x.value = x;
  write(fd_mouse, &event_x, sizeof(event_x));// Move the mouse
  write(fd_mouse, &event_end, sizeof(event_end));// Show moveevent.type = EV_REL;
  event_y.code = REL_Y;
  event_y.value = y;
  write(fd_mouse, &event_y, sizeof(event_y));// Move the mouse
  write(fd_mouse, &event_end, sizeof(event_end));// Show move
}

double corr(double *x, double *y){
  double result = 0, x_result = 0, y_result = 0;
  for(i = 0; i < CORR_LENGTH; i++){
    result += *(x + i) * (*(y + i));
    x_result += *(x + i) * (*(x + i));
    y_result += *(y + i) * (*(y + i));
  }
  x_result = sqrt(x_result);
  y_result = sqrt(y_result);
  result = 100000 * (result / x_result / y_result) - 100000;
  return result;
}

void select_key(unsigned int my_key){
  if(my_key ==0x00)
    event_key_start.code = KEY_0;
  else if(my_key == 0x01)
    event_key_start.code = KEY_1;
  else if(my_key ==0x02)
    event_key_start.code = KEY_2;
  else if(my_key == 0x03)
    event_key_start.code = KEY_3;
  else if(my_key == 0x04)
    event_key_start.code = KEY_4;
  else if(my_key == 0x05)
    event_key_start.code = KEY_5;
  else if(my_key == 0x06)
    event_key_start.code = KEY_6;
  else if(my_key == 0x07)
    event_key_start.code = KEY_7;
  else if(my_key == 0x08)
    event_key_start.code = KEY_8;
  else if(my_key == 0x09)
    event_key_start.code = KEY_9;
  else if(my_key == 0x0a)
    event_key_start.code = KEY_A;
  else if(my_key == 0x0b)
    event_key_start.code = KEY_B;
  else if(my_key == 0x0c)
    event_key_start.code = KEY_C;
  else if(my_key == 0x0d)
    event_key_start.code = KEY_D;
  else if(my_key == 0x0e)
    event_key_start.code = KEY_E;
  else if(my_key == 0x0f)
    event_key_start.code = KEY_F;
  else if(my_key == KEY_ENTER)
    event_key_start.code = KEY_ENTER;
  else if(my_key == KEY_SLASH)
    event_key_start.code = KEY_SLASH;
  else if(my_key == KEY_F12)
    event_key_start.code = KEY_F12;
  else if(my_key == KEY_F1)
    event_key_start.code = KEY_F1;
}

void key_pressed(unsigned long key)
{
  event_key_start.type = EV_KEY;
  event_key_start.value = EV_PRESSED;
  select_key(key);
  write(fd_keyboard, &event_key_start, sizeof(event_key_start));
  event_key_start.value = EV_RELEASED;
  write(fd_keyboard, &event_key_start, sizeof(event_key_start));
  write(fd_keyboard, &event_key_end, sizeof(event_key_end));
}

void keyboard(){
  unsigned int buf = 0;

  key_pressed(KEY_ENTER);
  key_pressed(KEY_SLASH);
  for(i = 20; i > -1; i = i-4){
    buf = rgb_data >> i & 0x0f;
    key_pressed(buf);
  }
  key_pressed(KEY_SLASH);
  key_pressed(KEY_ENTER);
}
                     
void Writei2cRegisters(byte numberbytes, byte command)
{
  byte i = 0;

  Wire.beginTransmission(SensorAddressWrite);   // Send address with Write bit set
  Wire.write(command);                          // Send command, normally the register address 
  for (i=0;i<numberbytes;i++)                       // Send data 
    Wire.write(i2cWriteBuffer[i]);
  Wire.endTransmission();

  delayMicroseconds(100);      // allow some time for bus to settle      
}

/*  
    Send register address to this function and it returns byte value
    for the magnetometer register's contents 
 */
byte Readi2cRegisters(int numberbytes, byte command)
{
  byte i = 0;

  Wire.beginTransmission(SensorAddressWrite);   // Write address of read to sensor
  Wire.write(command);
  Wire.endTransmission();

  delayMicroseconds(100);      // allow some time for bus to settle      

  Wire.requestFrom(SensorAddressRead,numberbytes);   // read data
  for(i=0;i<numberbytes;i++)
    i2cReadBuffer[i] = Wire.read();
  Wire.endTransmission();   

  delayMicroseconds(100);      // allow some time for bus to settle      
}  

void init_TCS34725(void)
{
  i2cWriteBuffer[0] = 0x10;
  Writei2cRegisters(1,ATimeAddress);    // RGBC timing is 256 - contents x 2.4mS =  
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1,ConfigAddress);   // Can be used to change the wait time
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1,ControlAddress);  // RGBC gain control
  i2cWriteBuffer[0] = 0x03;
  Writei2cRegisters(1,EnableAddress);    // enable ADs and oscillator for sensor  
}

void get_TCS34725ID(void)
{
  Readi2cRegisters(1,IDAddress);
  if (i2cReadBuffer[0] = 0x44)
    printf("TCS34725 is present");    
  else
    printf("TCS34725 not responding");    
}

/*
   Reads the register values for clear, red, green, and blue.
 */
void get_Colors(void){
  unsigned long clear_color = 0;
  unsigned long red_color = 0;
  unsigned long green_color = 0;
  unsigned long blue_color = 0;

  for(i = 0; i < 10; i++){
    Readi2cRegisters(8,ColorAddress);
    clear_color = (unsigned long)(i2cReadBuffer[1]<<8) + (unsigned long)i2cReadBuffer[0];
    red_color = ((unsigned long)(i2cReadBuffer[3]<<8) + (unsigned long)i2cReadBuffer[2])/COLORREGULATOR;
    green_color = ((unsigned long)(i2cReadBuffer[5]<<8) + (unsigned long)i2cReadBuffer[4])/COLORREGULATOR;
    blue_color = ((unsigned long)(i2cReadBuffer[7]<<8) + (unsigned long)i2cReadBuffer[6])/COLORREGULATOR;
    // send register values to the serial monitor 
    if(red_color>255){
      red_color=255;
    }
    if(green_color>255){
      green_color=255;
    }
    if(blue_color>255){
      blue_color=255;
    }
    red_sum+=red_color;
    green_sum+=green_color;
    blue_sum+=blue_color;
  }
  
  red_data = red_sum / 10;
  red_sum=0;
  green_data = green_sum / 10;
  green_sum=0;
  blue_data = blue_sum / 10;
  blue_sum = 0;
  rgb_data=(red_data<<16)|(green_data<<8)|(blue_data);
  memset(&event_key_start, 0, sizeof(event_key_start));
  memset(&event_key_end, 0, sizeof(event_key_end));
  event_key_end.type = EV_SYN;
  event_key_end.code = SYN_REPORT;
  event_key_end.value = 0;
  keyboard();
}
