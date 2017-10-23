#include <TinyGPS++.h>
#include <PinChangeInt.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <PID_v1.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include <BMP085.h>
#include <ADXL345.h>
#include <Servo.h>
#include "math.h"

#define SOL_ON_MOTOR_PINI 12
#define SAG_ON_MOTOR_PINI 6
#define SOL_ARKA_MOTOR_PINI 9
#define SAG_ARKA_MOTOR_PINI 3
#define SERVO1 4
#define SERVO2 11

#define Aileron_Roll_PIN A15 //ch1
#define Elevator_Pitch_PIN A14 //ch2
#define Throttle_PIN A13 //ch3
#define Rudder_Yaw_PIN A12 //ch4  
#define AUXCH1 A11
#define AUXCH2 A10 
#define AUXCH3 A9 // ch7
#define AUXCH4 A8 // ch8

#define ARM 950 //calibration neyle yaptıysan dikkat
#define CH1_MIN 960
#define CH1_MAX 1920
#define CH2_MIN 960
#define CH2_MAX 1920
#define CH3_MIN 960
#define CH3_MAX 1920
#define CH4_MIN 960
#define CH4_MAX 1920
#define CH5_MIN 1028
#define CH5_MAX 1844
#define CH6_MIN 1028
#define CH6_MAX 1844
#define CH7_MIN 1012
#define CH7_MAX 1844
#define CH8_MIN 1012
#define CH8_MAX 1844
#define SMIN 1000 //servo writeMicroseconds minimum
#define SMAX 2000 //maksimum
#define ARM_DELAY 10000 //wait after arm 10 SECONDS
#define accZOffset -204 

float PITCH_P_VAL=0;
float PITCH_I_VAL=0;
float PITCH_D_VAL=0;

float ROLL_P_VAL=0;
float ROLL_I_VAL=0;
float ROLL_D_VAL=0;

float YAW_P_VAL=0;
float YAW_I_VAL=0;
float YAW_D_VAL=0;

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -60
#define YAW_MAX 60
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20

#define SENS   0.0078125 // =8/1024 //#define TODEG 57.2957795 there already exist RAD_TO_DEG no need for this

ADXL345 acc;  int16_t ax, ay, az;
BMP085 baro;  float temperature, pressure, altitude;
HMC5883L mag; int16_t mx, my, mz;
ITG3200 gyro; int16_t gx, gy, gz; 

TinyGPSPlus    gps;
Servo          a,b,c,d, s1, s2;

float          velocity, velocityLast;
float          bal_roll = 0.0f, bal_pitch = 0.0f, bal_axes = 0.0f;
int            va, vb, vc, vd; // motor hızları //1000-2000
float          ctrl; //rc dğer kontrol <MIN >MAX
boolean        interruptLock = false;
float          ch1, ch1Last, ch2, ch2Last, ch3, ch3Last, ch4, ch4Last, ch5, ch6, ch7, ch8;
float          ypr[3]     = {0.0f, 0.0f, 0.0f}; //Acc
float          yprLast[3] = {0.0f, 0.0f, 0.0f};
float          c_ypr[3]   = {0.0f, 0.0f, 0.0f}; //Complimentary with 0.98
float	       c_yprLast[3] = {0.0f, 0.0f, 0.0f};
float          Ax, Ay, Az, Xh, Yh, t_roll, t_pitch, deltaT, gyroX, gyroY, gyroZ;
float          yawOffset = 0, tch1, tch2, tch3, tch4, tch7, tch7Last, tch8, tch8Last;
int 		   servoTimer;

PID yawReg  (&c_ypr[0], &bal_axes,  &tch4,   YAW_P_VAL,   YAW_I_VAL,   YAW_D_VAL,   DIRECT );
PID pitchReg(&c_ypr[1], &bal_pitch, &tch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL,   REVERSE);
PID rollReg (&c_ypr[2], &bal_roll,  &tch1,  ROLL_P_VAL,  ROLL_I_VAL,  ROLL_D_VAL,   REVERSE);

unsigned long  rcLastChange1 = micros();
unsigned long  rcLastChange2 = micros();
unsigned long  rcLastChange3 = micros();
unsigned long  rcLastChange4 = micros(); 
unsigned long  rcLastChange5 = micros(); 
unsigned long  rcLastChange6 = micros(); 
unsigned long  rcLastChange7 = micros();
unsigned long  rcLastChange8 = micros();

void setup(){

	Wire.begin();
	Serial1.begin(38400);
	initServos();
	initRC();
	initDOF();
	initMotors();
	initReg();
	initFilter();
 }
void loop(){

	getYPR();
	computePID();
	calcVel();
	updateMotors();

 }
void getYPR(){

	acc.getAcceleration(&ax,&ay,&az); az+=accZOffset; //software calibration for z axis
	gyro.getRotation(&gx, &gy, &gz);  gx+=61; gy+=11; gz-=5;
	mag.getHeading(&mx, &my, &mz);

	deltaT = gyro.deltaTime /1000.0; //delta time; every call frequence for gyro

	Ax = SENS*ax; Ay = SENS*ay; Az = SENS*az;

	t_roll = sqrt((Ax*Ax) + (Az*Az));
	t_pitch = sqrt((Ay*Ay)+(Az*Az));
	ypr[1] = atan2(Ay,t_roll); // pitch
	ypr[2] = atan2(Ax,t_pitch); //roll

	//Yaw calculations from magnetometer heading
	Xh= mx*cos(ypr[1]) + mz*sin(ypr[1]);
	Yh= mx*sin(ypr[2])*sin(ypr[1]) + my*cos(ypr[2]) - mz*sin(ypr[2])*cos(ypr[1]);
	ypr[0] = atan(Yh/Xh);

	//Convert Radion YPR values to Degree
	ypr[0] = ypr[0] * RAD_TO_DEG - yawOffset;
	ypr[1] = ypr[1] * RAD_TO_DEG;
	ypr[2] = ypr[2] * RAD_TO_DEG;

	gyroY = (float)(gy / 14.375) * deltaT;
	gyroX = (float)(gx / 14.375) * deltaT;
	gyroZ = (float)(gz / 14.375) * deltaT;

	c_ypr[0] = 0.98*(c_ypr[0]+gyroZ) + 0.02*ypr[0]; 
	c_ypr[1] = 0.98*(c_ypr[1]+gyroY) + 0.02*ypr[1];
	c_ypr[2] = 0.98*(c_ypr[2]+gyroX) + 0.02*ypr[2];

 }
void computePID(){
	
	kitle();	tch1 = ch1; serbest_birak();
	if(tch1 < CH1_MIN || tch1 > CH1_MAX){
		tch1 = ch1Last;
	}
	else{
		tch1 = fmap(tch1, CH1_MIN, CH1_MAX, ROLL_MIN, ROLL_MAX);
		ch1Last = tch1;
	}

	kitle(); tch2 = ch2; serbest_birak();
	if(tch2 < CH2_MIN || tch2 > CH2_MAX){
		tch2 = ch2Last;
	}
	else{
		tch2 = fmap(tch2, CH2_MIN, CH2_MAX, PITCH_MIN, PITCH_MAX);
		ch2Last = tch2;
	}

	kitle(); tch4 = ch4; serbest_birak();
	if (tch4 < CH4_MIN || tch4 > CH4_MAX){
		tch4 = ch4Last;
	}
	else{
		tch4 = fmap(tch4, CH4_MIN, CH4_MAX, YAW_MIN, YAW_MAX);
		ch4Last = tch4;
	}

	yprLast[0] = ypr[0];
	yprLast[1] = ypr[1];
	yprLast[2] = ypr[2];

	c_yprLast[0] = c_ypr[0];
	c_yprLast[1] = c_ypr[1];
	c_yprLast[2] = c_ypr[2];
  
	PITCH_P_VAL=ROLL_P_VAL = fmap(ch5,CH5_MIN, CH5_MAX,0, 0.4 );
	PITCH_I_VAL=ROLL_I_VAL = fmap(ch6,CH6_MIN, CH6_MAX,0, 0.4 );
	PITCH_D_VAL=ROLL_D_VAL = fmap(ch7,CH7_MIN, CH7_MAX,0, 0.4 );

	rollReg.SetTunings(ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL); //input for PID from rc
	pitchReg.SetTunings(PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL);

	pitchReg.Compute();
	rollReg.Compute();
	yawReg.Compute();

 }
void calcVel(){

	kitle(); tch3 = ch3; serbest_birak();
	if (tch3 < CH3_MIN || tch3 > CH3_MAX){
		velocity = velocityLast;
	}
	else{
		velocity = fmap(tch3, CH3_MIN, CH3_MAX, SMIN, SMAX);
		velocityLast = velocity;
	}

	va = velocity + bal_pitch - bal_roll - bal_axes;
	vb = velocity + bal_pitch + bal_roll + bal_axes;

	vc = velocity - bal_pitch - bal_roll + bal_axes;
	vd = velocity - bal_pitch + bal_roll - bal_axes;

 }

void initServos(){
	s1.attach(SERVO1);
	s2.attach(SERVO2);
	s1.write(90);
	s2.write(70);
	delay(15);
	s1.detach();
	s2.detach();
 }
void initReg(){

	pitchReg.SetMode(AUTOMATIC);
	pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);

	rollReg.SetMode(AUTOMATIC);
	rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);

	yawReg.SetMode(AUTOMATIC);
	yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

 }
void initDOF(){

	delay(100);
	acc.initialize();  boolean a = acc.testConnection();  acc.setRange(1);  acc.setOffset(33,47,-127);
	baro.initialize(); boolean b = baro.testConnection();
	mag.initialize();  boolean m = mag.testConnection();
	gyro.initialize(); boolean g = gyro.testConnection();
	if(!(a & b & m & g))
		initDOF();

 }
void initMotors(){

	a.attach(SOL_ON_MOTOR_PINI);   b.attach(SAG_ON_MOTOR_PINI);
	c.attach(SOL_ARKA_MOTOR_PINI); d.attach(SAG_ARKA_MOTOR_PINI);
	delay(100);

	a.writeMicroseconds(ARM); b.writeMicroseconds(ARM);
	c.writeMicroseconds(ARM); d.writeMicroseconds(ARM);
	delay(ARM_DELAY);

 }
void updateMotors(){

	a.writeMicroseconds(va);  b.writeMicroseconds(vb);
	c.writeMicroseconds(vc);  d.writeMicroseconds(vd);

 }
void initRC(){

	PCintPort::attachInterrupt(Aileron_Roll_PIN,   rcInterrupt1,   CHANGE);
	PCintPort::attachInterrupt(Elevator_Pitch_PIN, rcInterrupt2,   CHANGE);
	PCintPort::attachInterrupt(Throttle_PIN,       rcInterrupt3,   CHANGE);
	PCintPort::attachInterrupt(Rudder_Yaw_PIN,     rcInterrupt4,   CHANGE); 
	PCintPort::attachInterrupt(AUXCH1,             rcInterrupt5,   CHANGE); 
	PCintPort::attachInterrupt(AUXCH2,             rcInterrupt6,   CHANGE); 
	PCintPort::attachInterrupt(AUXCH3,             rcInterrupt7,   CHANGE); 
	PCintPort::attachInterrupt(AUXCH4,             rcInterrupt8,   CHANGE);

 }
void initFilter(){

	getYPR();
	yawOffset = ypr[0];
	c_ypr[0] = ypr[0];
	c_ypr[1] = ypr[1];
	c_ypr[2] = ypr[2];

 }
void kitle(){ interruptLock = true;}
void serbest_birak(){ interruptLock = false;}
/*****Interrupt Service Routines ******/
void rcInterrupt1(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange1;
		if(ctrl<=CH1_MAX && ctrl>=CH1_MIN) ch1 = ctrl;
	}
	rcLastChange1 = micros();
 }
void rcInterrupt2(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange2;
		if(ctrl<=CH2_MAX && ctrl>=CH2_MIN) ch2 = ctrl;
	}
	rcLastChange2 = micros();
 }
void rcInterrupt3(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange3;
		if(ctrl<=CH3_MAX && ctrl>=CH3_MIN) ch3 = ctrl;
	}
	rcLastChange3 = micros();
 }
void rcInterrupt4(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange4;
		if(ctrl<=CH4_MAX && ctrl>=CH4_MIN) ch4 = ctrl;
	}
	rcLastChange4 = micros();
 }
void rcInterrupt5(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange5;
		if(ctrl<=CH5_MAX && ctrl>=CH5_MIN) ch5 = ctrl;
	}
	rcLastChange5 = micros();
 }
void rcInterrupt6(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange6;
		if(ctrl<=CH6_MAX && ctrl>=CH6_MIN) ch6 = ctrl;
	}
	rcLastChange6 = micros();
 }
void rcInterrupt7(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange7;
		if(ctrl<=CH7_MAX && ctrl>=CH7_MIN) ch7 = ctrl;
	}
	rcLastChange7 = micros();
 }
void rcInterrupt8(){
	if (!interruptLock)
	{
		ctrl = micros() - rcLastChange8;
		if(ctrl<=CH8_MAX && ctrl>=CH8_MIN) ch8 = ctrl;
	}
	rcLastChange8 = micros();
 }
/***********custom functions******************/
float fmap(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 }
