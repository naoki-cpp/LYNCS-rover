#include <math.h>
#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "local_libs/Matrix.h"

#define bv  0.05745024
#define cv 0.009521774
#define A_M 0.0017
#define A_m 0.0013
#define B_M 0.53
#define B_m 0.5996
#define u 0
#define PI 3.1415
#define OutR1 5
#define OutR2 6
#define OutL1 9
#define OutL2 10
#define maxi 2000
#define mini 1000
#define hhigh 1900
#define llow 1100
#define echoPin 13 // Echo Pin
#define trigPin 7 // Trigger Pin
#define VB 300
#define anga 1
#define angb 1
#define MaxP 1
#define MaxC 1 // per sec
#define MaxA 1
#define chigh 400
#define clow 0

double ttt;
double ddd = 0;
double x_k;
double P_k;
double ovh;
double nvh;
double vh;
double gk;
double xk;
double Pk;
double oldr;
double realaccel;

double oldaax = 0;
double oldaay = 0;
double oldaaz = 0;
double oldypr[3];

double aax;
double aay;
double aaz;
long int intaax;
long int intaay;
long int intaaz;
long int intypr[3];
double aaxT;
double aayT;
double aazT;
double qx1 = -0.001;
double qx2 = 0.93;
double qx3 = 0.003;
double qx4 = -0.0007;
double qx5 = 0.003;
double qx6 = -0.004;
double qx7 = 0.01;
double qx8 = 0.004;
double qx9 = 0.003;
double qx10 = 0.01;
double qy1 = 0.009;
double qy2 = -0.02;
double qy3 = 0.94;
double qy4 = -0.00034;
double qy5 = -0.04;
double qy6 = -0.03;
double qy7 = 0.03;
double qy8 = 0.04;
double qy9 = -0.03;
double qy10 = 0.007;
double qz1 = -0.004;
double qz2 = 0.015;
double qz3 = -0.017;
double qz4 = 0.005;
double qz5 = 0.009;
double qz6 = -0.01;
double qz7 = -0.03;
double qz8 = -0.03;
double qz9 = 0.009;
double qz10 = 0.99;

double q1[3];
double q2[3];
double q3[3];
double q4[3];
double q5[3];
double q6[3];
double q7[3];
double q8[3];
double q9[3];
double q10[3];

double vx;
double vy;
double vz;
double vn;
double vn1 = 0;
double vn2 = 0;
double rvn;
double rvn1 = 0;
double rvn2 = 0;
double we = 1600;
double wh = 2;

MPU6050 mpu;

double gzzz;
double gzz0;
double gztank = 0;
double kx_m = 0;
double countx;
double ky_m = 0;
double kz_m = 0;
double vkx;
double power;
double power_a[3];
double vky;
double vkz;
double l;
double q_m;
double kx_a[3];
double kxa_a[3];
double ky_a[3];
double kya_a[3];
double kz_a[3];
double kza_a[3];
double kv_a[3];
double l_a[3];
double gy[3];
double hv[2];
double gyv[3];
double zn[3][3];
double buffer1 [3][3];
double ppp = 0;
double ipp = 250;
double dpp = 0;//ppp=1000で１０度22ぐらむ
int ppp1;
int land = 0;
double h_a[3];
double h_m = 250;
int Alltimer1 = 0;
int Alltimer2;
long TIMET1 = 0;
long TIMET2 = 0;

double v00;
double vp = 0;
double center = 0;
double centerold = 0;
double ptx = 0;
double ptxold = 0;
double pty = 0;
double ptyold = 0;
double ptz = 0;
double ptzold = 0;
double BPP = 520;
long ford1 = 0;
long ford2 = 0;
long fordd = 0;

double oldReal = 0;
double kxa_m;
double kya_m;
int knt;
int fpga = 0;
int asd = 100;
int asg = 100;
char buf [100];
int country = 0;
int coucou = 0;
int w = 0;
int e;
int spi1;
int spi2;
int spi3;
int spi4;
int spi5;
int spi6;
int spi7;
int spi8;
unsigned char cspi1;
unsigned char cspi2;
unsigned char c;
volatile byte pos;
volatile boolean process_it;
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus;   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorInt16 aa;         // [x, y, z]            加速度センサの測定値
VectorInt16 aaReal;     // [x, y, z]            重力を除いた加速度センサの測定値
VectorInt16 aaWorld;
VectorFloat gravity;  // [x, y, z]      gravity vector
VectorInt16 gyro;// [x, y, z]      gravity vector
float ypr[3];
lyncs::Matrix<double,3,3> rotation_matrix = {{{0,0,0},{0,0,0},{0,0,0}}};
double v;
double vv;
double y0;
double y1;
double y2;
volatile bool mpuInterrupt = false;   // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
void cleenarray3(double array[], double newdata);
void pidx(double array[], double a_m, double PB, double DT, double Td, double T);
void pidy(double array[], double a_m, double PB, double DT, double Td, double T);
void pidz(double array[], double a_m, double PB, double DT, double Td, double T);
void pidy_a(double , double , double , double , double , double );
void change(double, double, double, double);
void pidh(double array[], double a_m, double PB, double DT, double Td, double T);
double TimeUpdate(); //前回この関数が呼ばれてからの時間 us単位
void flypower(double outr, double outl);
void getrp(double, double);
void cmpid(double array[], double a_m, double PB, double DT, double Td, double T);
void gppid(double array[], double a_m, double PB, double DT, double Td, double T);
long curMicros;
char jo;
//MS5xxx sensor(&Wire);
void setup()
{
  double x;
  double y;
  double z;
  oldypr[0] = 0;
  oldypr[1] = 0;
  oldypr[2] = 0;
  countx = 0;
  jo = 1;
  h_a[0] = 0;
  h_a[1] = h_m;
  h_a[2] = h_m;
  gy[0] = 0;
  gy[1] = 0;
  gy[2] = 0;
  gyv[0] = 0;
  gyv[1] = 0;
  gyv[2] = 0;
  kxa_a[0] = 0;
  kxa_a[1] = 0;
  kxa_a[2] = 0;
  kya_a[0] = 0;
  kya_a[1] = 0;
  kya_a[2] = 0;
  kv_a[0] = 0;
  kv_a[1] = 0;
  kv_a[2] = 0;

  TCCR1B &= B11111000;
  TCCR1B |= B00000001;
  pinMode(OutR1, OUTPUT);
  pinMode(OutR2, OUTPUT);
  pinMode(OutL1, OUTPUT);
  pinMode(OutL2, OUTPUT);
  pinMode( echoPin, INPUT );
  pinMode( trigPin, OUTPUT );
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(209);
  mpu.setYGyroOffset(47);
  mpu.setZGyroOffset(-21);
  mpu.setZAccelOffset(1957); // 1688 factory default for my test chip
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(2, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  // 加速度/ジャイロセンサーの初期化。
  for (int i_r = 0; i_r < 3; i_r++)
  { // 重力加速度から角度を求める。
    x = 0.0000000001;
    y = 0.0000000001;
    z = 0.0000000001;
    cleenarray3(kx_a, x);
    cleenarray3(kxa_a, x);
    cleenarray3(ky_a, y);
    cleenarray3(kz_a, z);
  }
  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  // get ready for an interrupt
  pos = 0;   // buffer empty
  process_it = false;
  // now turn on interrupts
  SPI.attachInterrupt();
  delay(5000);


}

ISR (SPI_STC_vect)
{
  byte c = SPDR;  // grab byte from SPI Data Register
  // add to buffer if room
  if (pos < sizeof buf)
  {
    buf [pos++] = c;
    // example: newline means time to process buffer
    if (c == '\n')
      process_it = true;
  }  // end of room available
}

void loop()
{
  if (process_it)
  {
    buf [pos] = 0;
    spi1 = *(int*)(&buf[0]);
    spi2 = *(int*)(&buf[4]);
    spi3 = *(int*)(&buf[8]);
    spi4 = *(int*)(&buf[12]);
    spi5 = *(int*)(&buf[16]);
    spi6 = *(int*)(&buf[20]);
    spi7 = *(int*)(&buf[24]);
    spi8 = *(int*)(&buf[28]);
    cspi1 = buf[32];
    cspi2 = buf[33];
    pos = 0;
    process_it = false;
  }

  double k_m = 0;
  double h_m = 250;
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) { }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    /* Serial.print("ypr\t");
      Serial.print(gyv[0]);
      Serial.print("\t");
      Serial.print(gyv[1]);
      Serial.print("\t");
      Serial.println(gyv[2]);*/
    gy[0] = (double)ypr[0];
    gy[1] = (double)ypr[1];
    gy[2] = (double)ypr[2];
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    gyv[0] = (double)gyro.x;
    gyv[1] = (double)gyro.y;
    gyv[2] = (double)gyro.z;
    y0 = (-1) * ypr[0];
    y1 = (-1) * ypr[1];
    y2 = ypr[2];

	GetRotationMatrix(rotation_matrix, (double)y0, (double)y1, (double)y2);
    aax = (double)aa.x / 7600;
    aay = (double)aa.y / 8000;
    aaz = (double)aa.z / 10200;

    intaax = (long int)(aax * 1000);
    intaay = (long int)(aay * 1000);
    intaaz = (long int)(aaz * 1000);

    intypr[0] = (long int)(ypr[0] * 1000);
    intypr[1] = (long int)(ypr[1] * 1000);
    intypr[2] = (long int)(ypr[2] * 1000);
    //aaxT = qx1*ypr[0] + qx2*ypr[1] + qx3*ypr[2] + qx4*ypr[0]*ypr[0] + qx5*ypr[1]*ypr[1] + qx6*ypr[2]*ypr[2] +qx7*ypr[0]*ypr[1] + qx8*ypr[1]*ypr[2] + qx9*ypr[0]*ypr[2] + qx10*aax;
    //aayT = qy1*ypr[0] + qy2*ypr[1] + qy3*ypr[2] + qy4*ypr[0]*ypr[0] + qy5*ypr[1]*ypr[1] + qy6*ypr[2]*ypr[2] +qy7*ypr[0]*ypr[1] + qy8*ypr[1]*ypr[2] + qy9*ypr[0]*ypr[2] + qy10*aay;
    //aazT = qz1*ypr[0] + qz2*ypr[1] + qz3*ypr[2] + qz4*ypr[0]*ypr[0] + qz5*ypr[1]*ypr[1] + qz6*ypr[2]*ypr[2] +qz7*ypr[0]*ypr[1] + qz8*ypr[1]*ypr[2] + qz9*ypr[0]*ypr[2] + qz10*aaz;
    //aaxT = (-0.001)*ypr[0] + 0.93*ypr[1] + 0.003*ypr[2] +  0.003*ypr[1]*ypr[1] + (-0.004)*ypr[2]*ypr[2] +0.01*ypr[0]*ypr[1] + 0.00361*ypr[1]*ypr[2] + 0.003*ypr[0]*ypr[2] + 0.01*aax;
    //aayT = 0.009*ypr[0] + (-0.02)*ypr[1] + 0.94*ypr[2] + (-0.04)*ypr[1]*ypr[1] + (-0.03)*ypr[2]*ypr[2] +0.03*ypr[0]*ypr[1] + 0.04*ypr[1]*ypr[2] + (-0.03)*ypr[0]*ypr[2] + 0.007*aay;
    //aazT = (-0.004)*ypr[0] + 0.01*ypr[1] + (-0.02)*ypr[2] + 0.005*ypr[0]*ypr[0] + 0.0094*ypr[1]*ypr[1] + (-0.01)*ypr[2]*ypr[2] + (-0.03)*ypr[0]*ypr[1] + (-0.03)*ypr[1]*ypr[2] +  0.009*ypr[0]*ypr[2] + 0.99*aaz;
    aaxT = (double)((-1) * intypr[0] * 1000 + 930 * intypr[1] * 1000 + 3 * intypr[2] * 1000 +  3 * intypr[1] * intypr[1] + (-4) * intypr[2] * intypr[2] + 10 * intypr[0] * intypr[1] + 4 * intypr[1] * intypr[2] + 3 * intypr[0] * intypr[2] + 10 * intaax * 1000);
    aayT = (double)(9 * intypr[0] * 1000 + (-20) * intypr[1] * 1000 + 940 * intypr[2] * 1000 + (-40) * intypr[1] * intypr[1] + (-30) * intypr[2] * intypr[2] + 30 * intypr[0] * intypr[1] + 40 * intypr[1] * intypr[2] + (-30) * intypr[0] * intypr[2] + 7 * intaay * 1000);
    aazT = (double)((-4) * intypr[0] * 1000 + 10 * intypr[1] * 1000 + (-20) * intypr[2] * 1000 + 5 * intypr[0] * intypr[0] + 9 * intypr[1] * intypr[1] + (-10) * intypr[2] * intypr[2] + (-30) * intypr[0] * intypr[1] + (-30) * intypr[1] * intypr[2] +  9 * intypr[0] * intypr[2] + 990 * intaaz * 1000);
    aaxT *= 0.000000001;
    aayT *= 0.000000001;
    aazT *= 0.000000001;
    vx = rotation_matrix.GetElement(0,0) * aaxT +  rotation_matrix.GetElement(0,1)* aayT + rotation_matrix.GetElement(0,2) * aazT;
    vy = rotation_matrix.GetElement(1,0) * aaxT + rotation_matrix.GetElement(1,1) * aayT + rotation_matrix.GetElement(1,2) * aazT;
    vz = rotation_matrix.GetElement(2,0) * aaxT + rotation_matrix.GetElement(2,1) * aayT + rotation_matrix.GetElement(2,2) * aazT;
    
	double delta_time = TimeUpdate()/1000000;
    rvn = rvn1 + (vz - 1) * 9.8 * delta_time;
    //vn = rvn;
    vn = (rvn * we - rvn2 * we - (delta_time / 2 * wh - 1) * (we - 2 / delta_time) * vn2 - ((delta_time / 2 * wh - 1) * (2 / delta_time + we) + (we - 2 / delta_time) * (1 + delta_time / 2 * wh)) * vn1 ) / (1 + delta_time / 2 * wh) / (2 / delta_time + we);
    vn2 = vn1;
    vn1 = vn;
    rvn2 = rvn1;
    rvn1 = rvn;
    double Real;
    Real = realaccel * 5;

    if (spi7 == spi8) {
      vh = (double)spi8 / 1000;
    }
    if ((spi7 - spi8) == 256) {
      vh = (double)spi8 / 1000;
    }

    vh = 0.1 * vh + 0.9 * oldr;
    oldr = vh;

    // calman(vn, vh, TIME / 1000000);

  }



  gzz0 = gy[0];
  if ((gzzz - gy[0]) > PI) {
    gztank += 2 * PI;
  }
  if ((gy[0] - gzzz) > PI) {
    gztank += (-2) * PI;
  }
  gy[0] = gztank + gy[0];
  gzzz = gzz0;



  //通信系
  if (cspi1 == cspi2) {
    while (cspi1 == 1) {
      digitalWrite(OutR1, LOW);
      analogWrite(OutR2, 0);
      digitalWrite(OutL1, LOW);
      analogWrite(OutL2, 0);
      Serial.println("END");
      delay(100000);
    }
  }
  if (spi1 == spi2) {
    center = (double)spi1 / 1000 * MaxC;
  }
  center = center * 0.2 + centerold * 0.8;
  centerold = center;


  /*if (spi5 == spi6) {
    ptx = (double)spi5 * MaxA / 1000 / 180 * 3.14;
    ptx = ptx * 0.05 + ptxold * 0.95;
    ptxold = ptx;
  }*/
  if (spi5 == spi6) {
    ptx = (double)spi5*0.001;

  }
  if (spi3 == spi4) {
    pty = (double)spi3 * MaxA / 1000 / 180 * 3.14;
    pty = pty * 0.05 + ptyold * 0.95;
    ptyold = pty;
  }
  v = 0.1;


  cleenarray3(kx_a, gyv[0]);
  cleenarray3(ky_a, gyv[1]);
  cleenarray3(kz_a, gyv[2]);
  cleenarray3(kv_a, vn - v00);


  /*if (countx == 9) {
    cleenarray3(kxa_a, gy[2]);
    cleenarray3(kya_a, -gy[1]);
    cleenarray3(kza_a, -gy[0]);

    //  getrp(0,0);

    pidx_a(kxa_a, ptx, 180, 0, 0, 0.1);
    pidy_a(kya_a, pty, 180, 0, 0, 0.1);
    pidz_a(kza_a, ptz, 180, 0, 0, 0.1);
    countx = 0;
    }*/



  // pidx(kx_a, kx_m, 0.732, 5.2286, 0.02562, 0.01);
  //pidy(ky_a, ky_m, 0.732, 5.63, 0.024, 0.01);
  pidz(kz_a, 0, ptx, 0, 0, 0.01);
  // pidh(kv_a, center, 80, 20, 20, 0.01);

  //flypower(v, vkz);
  flypower(0.5, 0);
    Serial.println(vkz);
  //  Serial.println(gyv[2]);
  countx = countx + 1;

}

void cleenarray3(double array[], double newdata)
{
  array[0] = array[1];
  array[1] = array[2];
  array[2] = newdata;
}

void pidx(double array[], double a_m, double PB, double DT, double Td, double T)
{
  vkx = vkx + PB * (array[1] - array[2]) + T * DT * (a_m - array[2]) - Td / T * (array[2] - 2 * array[1] + array[0]);
}
void pidy(double array[], double a_m, double PB, double DT, double Td, double T)
{
  vky = vky + PB * (array[1] - array[2]) + T * DT * (a_m - array[2]) - Td / T * (array[2] - 2 * array[1] + array[0]);
}
void pidz(double array[], double a_m, double PB, double DT, double Td, double T)
{
  vkz = vkz + PB * (array[1] - array[2]) + T * DT * (a_m - array[2]) - Td / T * (array[2] - 2 * array[1] + array[0]);
}
void pidx_a(double array[], double a_m, double PB, double DT, double Td, double T)
{
  kx_m = PB * (a_m - array[2]);
}
void pidy_a(double array[], double a_m, double PB, double DT, double Td, double T)
{
  ky_m = PB * (a_m - array[2]);
}
void pidz_a(double array[], double a_m, double PB, double DT, double Td, double T)
{
  kz_m = PB * (a_m - array[2]);
}


void pidh(double array[], double a_m, double PB, double DT, double Td, double T)
{
  vv = vv + T * DT * (a_m - array[2]) - Td / T * (array[2] - 2 * array[1] + array[0]);
  v = PB * (a_m - array[2]) + vv;
}
/*void pidh(double array[], double a_m, double PB, double DT, double Td, double T)
  {
  v =  v +PB*(array[1]-array[2]) + T*DT*(a_m-array[2])-Td/T*(array[2]-2*array[1]+array[0]);
  }*/


//-1<out<1
void flypower(double outV, double outT ) {
  //上限下限
  if (0.5 < outV) {
    outV = 0.5;
  }
  if (-0.5 > outV) {
    outV = -0.5;
  }
  if (0.5 < outT) {
    outT = 0.5;
  }
  if (-0.5 > outT) {
    outT = -0.5;
  }

  int outR;
  int outL;
  if (outT >= 0) {
    outR = (outT+outV) * 255;
    digitalWrite(OutR1, LOW);
    analogWrite(OutR2, outR);

    outL = outV * 255;
    digitalWrite(OutL1, LOW);
    analogWrite(OutL2, outL);
  }
  if (outT < 0) {
    outR = outV * 255;
    digitalWrite(OutR1, LOW);
    analogWrite(OutR2, outR);

    outL = (outV-outT) * 255;
    digitalWrite(OutL1, LOW);
    analogWrite(OutL2, outL);
  }

}

void getrp(double a, double b) {
  double A;
  double B;
  double ro;
  double pic;
  double judge;
  A = tan(a);
  B = tan(b);
  pic = atan((A * cos(-gy[0]) + sin(-gy[0])) / B / sqrt(A * A + 1));
  ro = atan((A * sin(-gy[0]) - cos(-gy[0])) / (A * cos(-gy[0]) + sin(-gy[0])) * sin(pic));
  judge = 1 / (A * sin(-gy[0]) - cos(-gy[0])) * sin(ro);
  if (judge > 0) {
    pic = (-1) * pic;
    ro = (-1) * ro;
  }
  kxa_m = ro;
  kya_m = pic;
}
double TimeUpdate()
{
	static double previous_time = micros(); //前回この関数が呼ばれた時間
	double temp_time = micros();
	double return_time = temp_time - previous_time;
	previous_time = temp_time;
	return return_time;
}

void GetRotationMatrix(lyncs::Matrix<double, 3, 3> &rotation_matrix, const double psi, const double phi, const double theta)
{
	double buffer1[3][3];
	lyncs::Matrix<double, 3, 3> R_roll_theta = {{{1, 0, 0}, {0, cos(theta), -1 * sin(theta)}, {0, sin(theta), cos(theta)}}};
	lyncs::Matrix<double, 3, 3> R_pitch_phi = {{{cos(phi), 0, sin(phi)}, {0, 1, 0}, {-sin(phi), 0, cos(phi)}}};
	lyncs::Matrix<double, 3, 3> R_yaw_psi = {{{cos(psi), -1 * sin(psi), 0}, {sin(psi), cos(psi), 0}, {0, 0, 1}}};
	rotation_matrix = (R_yaw_psi * R_pitch_phi) * R_roll_theta;
}