#include <iostream>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <signal.h>

using namespace std;
//MS5607-02BA03 Barometric Pressure Sensor
class MS5607
{
public:
  void Init();
  void Get();
  double Read( unsigned char data_type);
private:
  static const unsigned char cmd_reset    = 0x1e;
  static const unsigned char cmd_adc_read = 0x00;
  static const unsigned char cmd_adc_conv = 0x40;
  static const unsigned char cmd_adc_d1   = 0x00;
  static const unsigned char cmd_adc_d2   = 0x10;
  static const unsigned char cmd_adc_256  = 0x00;
  static const unsigned char cmd_adc_512  = 0x02;
  static const unsigned char cmd_adc_1024 = 0x04;
  static const unsigned char cmd_adc_2048 = 0x06;
  static const unsigned char cmd_adc_4096 = 0x08;
  static const unsigned char cmd_prom_rd  = 0xa0;
  int fd;
  unsigned char DevAddr; // 0x77

  unsigned int c0,c1,c2,c3,c4,c5,c6,c7;//16bit
  unsigned int d1,d2;//24bit
  int dt,temp;
  int64_t off,sens;
  int p;

  unsigned int swap16(unsigned int reg);
  void debug();
  void debug1();
};
unsigned int MS5607::swap16(unsigned int reg)
{
  reg=reg&0x0000ffff;
  reg=(((reg<<8)|(reg>>8))&0x0000ffff);
  return(reg);
}
void MS5607::debug1()
{
//debug data;
  c1=46372;
  c2=43981;
  c3=29059;
  c4=27842;
  c5=31553;
  c6=28165;
  d1=6465444;
  d2=8077636;
}

void MS5607::debug()
{
  printf("c1   c2    c3    c4    c5    c6\n");
  printf("%05d %05d %05d %05d %05d %05d\n",
    c1,c2,c3,c4,c5,c6);

  printf("%04x %016x\n",0,c0);
  printf("%04x %016x\n",1,c1);
  printf("%04x %016x\n",2,c2);
  printf("%04x %016x\n",3,c3);
  printf("%04x %016x\n",4,c4);
  printf("%04x %016x\n",5,c5);
  printf("%04x %016x\n",6,c6);
  printf("%04x %016x\n",7,c7);

  printf("d1        d2\n");
  printf("%08d  %08d\n",d1,d2);

  printf("dt        temp\n");
  printf("%08d  %08d\n",dt,temp);

  printf("off       sens    p\n");
  printf("%10lld  %10lld  %08d\n",off,sens,p);

}

void MS5607::Init()
{
  DevAddr=0x77;
  fd= wiringPiI2CSetup(DevAddr);
  if(fd<0)
    {
      std::cerr <<"センサー設定エラー"<<std::endl;
    }

  wiringPiI2CWrite(fd, cmd_reset); //Reset
  usleep(1000000);

  unsigned int reg;
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+0);
  c0=swap16(reg);
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+2);
  c1=swap16(reg);
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+4);
  c2=swap16(reg);
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+6);
  c3=swap16(reg);
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+8);
  c4=swap16(reg);
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+10);
  c5=swap16(reg);
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+12);
  c6=swap16(reg);
  reg=wiringPiI2CReadReg16(fd,cmd_prom_rd+14);
  c7=swap16(reg);
}
void MS5607::Get()
{
  unsigned int reg;
  unsigned char buff[4];

  wiringPiI2CWrite(fd, cmd_adc_conv|cmd_adc_d1|cmd_adc_4096); //Convert
  usleep(100000);
  wiringPiI2CWrite(fd, cmd_adc_read); //
  read(fd,buff,3);
  reg=0;
  reg=reg |(buff[0]<<16);
  reg=reg |(buff[1]<<8);
  reg=reg |(buff[2]<<0);
  d1=reg;

  wiringPiI2CWrite(fd, cmd_adc_conv|cmd_adc_d2|cmd_adc_4096); //Convert
  usleep(100000);

  wiringPiI2CWrite(fd, cmd_adc_read); //
  read(fd,buff,3);
  reg=0;
  reg=reg |(buff[0]<<16);
  reg=reg |(buff[1]<<8);
  reg=reg |(buff[2]<<0);
  d2=reg;

  //  debug1();
  dt=(int)(d2-(c5<<8));
  temp=2000+((dt*c6)>>23);
  int64_t x1,x2,x3;
  x1=(int64_t)c2<<17;
  x2=(c4*dt);
  x3=x2>>6;
  off=x1+x3;
  //  printf("x1=%lld x2=%lld x3=%lld off=%lld\n",x1,x2,x3,off);
  //off=(c2<<17)+((c4*dt)>>6);
  sens=(c1<<16)+((c3*dt)>>7);
  x1=d1*sens>>21;
  p=(int)((static_cast<int64_t>(x1)-static_cast<int64_t>(off))>>15);
  //  debug();
}
double MS5607::Read(unsigned char data_type)
{
  if(data_type==0) return ((double)temp/100);
  if(data_type==1) return ((double)p/100);
  return((double)p/100);
}

//MPL115A2 Digital Barometer
class MPL115A2{
public:
  void Init();
  void Get();
  double Read( unsigned char data_type);
  void conv_test_exec();
private:
  double conv_double( unsigned int reg, unsigned int n);
  void conv_test(unsigned int test,unsigned int n);
  int fd;
  unsigned char DevAddr; // 0x60
  double padc;
  double tadc;
  double a0,b1,b2,c12,p_comp,p_hpa,temp;
};

double MPL115A2::conv_double( unsigned int reg, unsigned int n)
{
  union
  {
    double d;
    uint64_t i;
    struct
    {
      uint64_t frac :52;
      unsigned int exp :11;
      unsigned int sign :1;
    } b;
  }u;
  unsigned int s,m;
  u.i=0;
  reg=(((reg<<8)|(reg>>8))&0x0000ffff);
  if(reg==0)
    {
      u.d=0;
    }
  else
    {
      if(((reg>>15)&0x01)==0)
        {
          u.b.sign=0;
        }
      else
        {
          u.b.sign=1;
          reg=((reg^(-1))+1)&0x0000ffff;//2の補数を戻す。
        }
      m=0x8000;
      for(s=0;s<15;s++)//最上位の1が何ビットめか調べる。
    {
      if(((m>>s)&reg) != 0) break;
    }
      u.b.frac = (uint64_t)((reg<<s)&0x7fff)<<(52-15);
      u.b.exp  = 0x03ff+n-s;
    }
  return(u.d);
}
void MPL115A2::conv_test(unsigned int reg,unsigned int n)
{
  printf("--------------\n");
  printf("reg=%08x n=%d\n",reg,n);
  reg=(((reg<<8)|(reg>>8))&0x0000ffff);
  printf("reg=%08x n=%d\n",reg,n);
  double d;
  d=conv_double(reg,n);
  printf("dob=%g \n",d);
  printf("dob=%15.10f \n",d);
  printf("dob=%15.10e \n",d);
}
void MPL115A2::conv_test_exec()
{
  conv_test(0x0000,12);
  conv_test(0x8000,12);
  conv_test(0xffff,12);

  conv_test(0x3ece,12);
  conv_test(0xb3f9,2);
  conv_test(0xc517,1);
  conv_test(0x33c8,-9);

  conv_test(0x6680>>1,10);//unsigned なので>>1
  conv_test(0x7ec0>>1,10);
}
void MPL115A2::Init()
{
  DevAddr=0x60;
  fd= wiringPiI2CSetup(DevAddr);
  if(fd<0)
    {
      std::cerr <<"センサー設定エラー"<<std::endl;
    }

  unsigned int reg;
  reg=wiringPiI2CReadReg16(fd,0x04);//a0
  a0=conv_double(reg,12);

  reg=wiringPiI2CReadReg16(fd,0x06);//b1
  b1=conv_double(reg,2);

  reg=wiringPiI2CReadReg16(fd,0x08);//b2
  b2=conv_double(reg,1);

  reg=wiringPiI2CReadReg16(fd,0x0a);//c12
  c12=conv_double(reg,-9);
}
void MPL115A2::Get()
{

  /*
  //  printf("MPL115A2:StartConv1\n");
  wiringPiI2CWrite(fd, 0x12); //Start Conversions
  wiringPiI2CWrite(fd, 0x00); //
  usleep(100000);
  //  printf("MPL115A2:StartConv2\n");
  wiringPiI2CWrite(fd, 0x12);
  wiringPiI2CWrite(fd, 0x01);
  usleep(100000);
  */


  wiringPiI2CWriteReg8(fd,0x12,0);//Start Conversions
  usleep(100000);

  unsigned int reg;
  reg=wiringPiI2CReadReg16(fd,0x00);
  //  printf("MPL115A2:padc %x\n",reg);
  padc=conv_double(reg>>1,10);//unsigned なので >>1
  reg=wiringPiI2CReadReg16(fd,0x02);
  //  printf("MPL115A2:tadc %x\n",reg);
  tadc=conv_double(reg>>1,10);
  //  printf("MPL115A2:padc %.4f tadc %.4f\n",padc,tadc);

  p_comp = a0 + (b1 + c12 * tadc) * padc + b2 * tadc;
  p_hpa = ((65.0/1023.0) * p_comp + 50) * 10;
  temp=(tadc-498.0)/(-5.35)+25.0;
  //  printf("MPL115A2:temp %.4f p_hpa %.4f\n",temp,p_hpa);

}
double MPL115A2::Read(unsigned char data_type)
{
  if(data_type==0) return (temp);
  if(data_type==1) return (p_hpa);
  return(p_hpa);
}

//HDC1000 Humidity and Temperature Digital Sensor Class
class HDC1000{
public:
  void Init(); //
  void Get();
  double Read( unsigned char data_type);
private:
  int fd;
// デバイスアドレス
// i2cdetect -y 1 で調べられる
  unsigned char DevAddr;
// 温度取得ポインタとConfigポインタのアドレス指定。
// 基本的にこのままでOK。今回は温度と湿度を同時取得するので
// 温度側だけでOK
  unsigned char tempp;
  unsigned char confp;
// デバイスの設定
// 2バイトの設定データを1バイトに分けて設定。
// 1＝上位ビット側 2＝下位ビット側
// 0x1234という設定データなら 1＝0x12 2＝0x34と設定
  unsigned char devconf1;
  unsigned char devconf2;
  //
  double temp;
  double humi;

};
void HDC1000::Init()
{
// デバイスアドレス
// i2cdetect -y 1 で調べられる
  DevAddr=0x40;
// 温度取得ポインタとConfigポインタのアドレス指定。
// 基本的にこのままでOK。今回は温度と湿度を同時取得するので
// 温度側だけでOK
  tempp=0x00;
  confp=0x02;
// デバイスの設定
// 2バイトの設定データを1バイトに分けて設定。
// 1＝上位ビット側 2＝下位ビット側
// 0x1234という設定データなら 1＝0x12 2＝0x34と設定
  devconf1=0x10;
  devconf2=0x00;

  unsigned char ConfData[3];
  ConfData[0]= confp;
  ConfData[1]= devconf1;
  ConfData[2]= devconf2;
  fd=wiringPiI2CSetup(DevAddr);
  /*
  wiringPiSetupGpio();
  pinMode(RDPin,INPUT);
  */
  if(write(fd,ConfData,sizeof(ConfData))<0)
    {
      std::cerr <<"センサー設定エラー"<<std::endl;
    }
  //  return fd;
}
void HDC1000::Get()
{
  unsigned char GetData[1];
  unsigned char ReData[4];

  GetData[0]=tempp;
  // センサーに温度測定要求
  if(write(fd,GetData,1)<0)
    {
      printf("データ要求エラー\n");
    }
  // センサーが測定→変換を完了させるのを待つ
  //    while((digitalRead(RDPin))==1);
  usleep(500000);
  // データ取得
  if(read(fd,ReData,4)<0)
    {
      printf("データ受信エラー\n");
    }
  int conv;
  conv=(ReData[0]<<8)|(ReData[1]);
  // 戻ってきたデータをくっつけて返す
  temp= ( (double)(conv)/65536.000) * 165.000 - 40.000;
  conv=(ReData[2]<<8)|(ReData[3]);
  humi= ( (double)(conv)/65536.000) * 100.000;
}
double HDC1000::Read(unsigned char data_type)
{
  if(data_type==0) return (temp);
  if(data_type==1) return (humi);
  return(temp);
}

void governor() {
  sleep(1);
}

static int sig =0;
void sig_handler(int signo)
{
  sig=1;
}

int main()
{
  HDC1000 hdc1000_1;
  MPL115A2 mpl115a2_1;
  MS5607 ms5607_1;
  struct tm* date;
  time_t timer;

  hdc1000_1.Init();
  mpl115a2_1.Init();
  ms5607_1.Init();

  std::cout << "Hello\nWorld!\n";

  if( signal(SIGINT, sig_handler) == SIG_ERR) {
    printf("sig_handler fail\n");
  }

  while(1) {
    std::thread th0(governor);
    //

    std::thread th3([&]{ms5607_1.Get();});
    //現在時刻の取得


    th3.join();
    // 表示
    printf("%.2fmbar %.2fdeg\n",

       ms5607_1.Read(1),//
       ms5607_1.Read(0) //Temp
     );
    //  Wait
    th0.join();
    //  Terminate
    if(sig) {
        printf("Recieve SIGINT\n");
        break;
    }
  }
}
