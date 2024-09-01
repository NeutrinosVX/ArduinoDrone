#include <Kalman.h>
#include <Wire.h>
#include <Math.h>
#include <TFMPI2C.h>
#include <NRFLite.h>
#include <SPI.h>
#include <PID_v1.h>
//the project started at 2023/11/9
//later logs will be written as improvements are made
float fRad2Deg = 57.295779513; //Rads to Degrees
const int MPU = 0x68; //MPU-6050's I2C address, the Main Wire address, not regional data
const int nValCnt = 7; //the length of the array storing data, with seven sub addresses,0x3B,0x3D,0x3F,0x41,0x43,0x45,0x47,corresponding to three axis linear, temperature, and three axis angular.
//andresses：0x1C:Linear acceleration multiplier(n*g),0x1B:Angular acceleration multiplier(n*α).
const int nCalibTimes = 1500; //as it says, calib times
int calibData[nValCnt];
unsigned long nLastTime = 0; //Time ti
float fLastRoll = 0.0f; //Roll angle filtered
float fLastPitch = 0.0f; //Pitch angle filtered
uint8_t f=0;//multiplier of readings
uint8_t dist=0;//Radar readings
uint8_t Strength=0;//Radar signal Strength
const int altiTrig=8;
const int altiecho=9;
unsigned  int relalti=0;
Kalman kalmanRoll; //Roll filter
Kalman kalmanPitch; //Pitch filter
Kalman Kalmandist; //distance filter
Kalman kalmanalt; 
uint8_t frontleft=0;//PWMpins for motors
uint8_t frontright=A1;
uint8_t hindleft=A2;
uint8_t hindright=A3;
/*void getAltidata()
{
  unsigned int duration=0;
  digitalWrite(altiTrig, LOW);  
	delayMicroseconds(2);  
	digitalWrite(altiTrig, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(altiTrig, LOW);
  duration=pulseIn(altiecho,HIGH);
  relalti=duration*0.01791;//experimental data, only good for atmospheres that is at 298~300K, moisture 28%, pressure 0.99876635atm.
  //recalibration systems and program will be developed with further versions
  Serial.print(relalti);
  Serial.print("cm");
}*/
void getAltidata()//the multisampling version, works stable. adding Kalman filtering method in future version
{
  const int samp=4;// how many samples 
  int duration[samp];// samples
  int i;
  int sum=0;
  for(i=0;i<samp;i++)
  {
    duration[i]=0;
  }
  for(i=0;i<samp;i++)
  {
    digitalWrite(altiTrig, LOW);  
	  delayMicroseconds(2);  
	  digitalWrite(altiTrig, HIGH);  
	  delayMicroseconds(10);  
	  digitalWrite(altiTrig, LOW);
    duration[i]=pulseIn(altiecho,HIGH);
  }
  for(i=0;i<samp;i++)
  {
    sum+=duration[i];
  }
  relalti=sum*0.01791/4;//experimental data, only good for atmospheres that is at 298~300K, moisture 28%, pressure 0.99876635atm.
  //recalibration systems and program will be developed with further versions
  Serial.print(relalti);
  Serial.print("cm");//p
}
//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) 
{
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) 
{
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int *pVals) 
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for(int i=0;i<nValCnt;++i)
  {
    pVals[i] = Wire.read() << 8 | Wire.read();
  
  }
}

//对大量读数进行统计，校准平均偏移量
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) 
  {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) 
    {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) 
  {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。(到时候看看这个咋回事)
}

//算得Roll角。算法见文档。
float GetRoll(float *pRealVals, float fNorm) 
{
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//算得Pitch角。算法见文档。
float GetPitch(float *pRealVals, float fNorm) 
{
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals)
{
  for (int i = 0; i < 3; ++i) 
  {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) 
  {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}

void Get_LidarDatafromIIC(unsigned char address)
{
  char i = 0; 
  unsigned char rx_buf[9] = {0}; 
  unsigned char check_sum = 0;
  Wire.beginTransmission(address); // Begin a transmission to the I2C Slave device with the given address. 
  Wire.write(0x5A); // see product mannual table 11:Obtain Data Frame
  Wire.write(0x05); // 
  Wire.write(0x00); // 
  Wire.write(0x01); // 
  Wire.write(0x60); // 
  Wire.endTransmission(1);  // Send a STOP Sign
  Wire.endTransmission(0);  // Send a START Sign
  Wire.requestFrom(address, 9 , 1); // request 9 bytes from slave device address

  //Read IIC data and save to rx_buf[]
  Serial.print("Address=0x");
  Serial.print(address,HEX);
  Serial.print(":   ");
   while (Wire.available()) 
  { 
    rx_buf[i] = Wire.read(); // received one byte 
    Serial.print("0x");
    Serial.print(rx_buf[i],HEX);
    Serial.print(";");
    i++; 
  }
  //Judge and print result via serial
    for(i=0;i<8;i++)
    {
      check_sum += rx_buf[i];
    }
   if(rx_buf[0] == 0x59 && rx_buf[1] == 0x59 &&  rx_buf[8] == check_sum)
   {
      Serial.print("---------->");
      Serial.print("Distance=");
      dist=rx_buf[3]*256+rx_buf[2];
      Serial.print(dist);//distance in int
      Serial.print(";");
      Strength=rx_buf[5]*256+rx_buf[4];
      Serial.print("Strength=");
      Serial.print(Strength);//strength in int
   }
   else
   {
      Serial.print("Maybe something wrong to Get Lidar`s data.");   
   }
  Serial.print("\r\n"); 
}
//helper method to get the hardware version

void GetLidarVersionFromIIC(unsigned char address)
{
  char i = 0; 
  unsigned char rx_buf[7] = {0}; 
  unsigned char check_sum = 0;
  Wire.beginTransmission(address); // Begin a transmission to the I2C Slave device with the given address. 
  Wire.write(0x5A); // see product mannual table 11:Obtain Data Frame
  Wire.write(0x04); // 
  Wire.write(0x01); // 
  Wire.write(0x5f); // 
  Wire.endTransmission(1);  // Send a STOP Sign 
  delay(100);
  Wire.endTransmission(0);  // Send a START Sign 
  Wire.requestFrom(address, 7 , 1); // request 9 bytes from slave device address
  //Read IIC data and save to rx_buf[]
   while ( Wire.available()) 
  { 
    rx_buf[i] = Wire.read(); // received one byte 
    Serial.print("0x");
    Serial.print(rx_buf[i],HEX);
    Serial.print(";");
    i++;
   }
   //Judge and print result via serial
   for(i=0;i<6;i++)
   {
      check_sum += rx_buf[i];
   }

   if(rx_buf[0] == 0x5A && rx_buf[1] == 0x07 && rx_buf[2] == 0x01 && rx_buf[6] == check_sum)
   {
      Serial.print("    The Lidar firmware version is v");
      Serial.print(rx_buf[5],HEX);
      Serial.print(".");
      Serial.print(rx_buf[4],HEX);
      Serial.print(".");
      Serial.print(rx_buf[3],HEX);    
   }
   else
   {
      Serial.print("Check version error!");
   }

    
  Serial.print("\r\n"); 
}

void SetLidarFrequenceFromIIC(unsigned char address,byte frequence)
{
  unsigned char i = 0; 
  unsigned char rx_buf[6] = {0}; 
  unsigned char check_sum = 0;
  unsigned char fre_L , fre_H;
  fre_L = frequence;
  fre_H = frequence>>8;
  Wire.beginTransmission(address); // Begin a transmission to the I2C Slave device with the given address. 
  Wire.write(0x5A); // see product mannual table 11:Obtain Data Frame
  Wire.write(0x06); // 
  Wire.write(0x03); // 
  Wire.write(fre_L); // 
  Wire.write(fre_H); // 
  Wire.write(0x5A+0x06+0x03+fre_L+fre_H); // 
  Wire.endTransmission(1);  // Send a STOP Sign 
  delay(100);
  Wire.endTransmission(0);  // Send a START Sign 
  Wire.requestFrom(address, 6 , 1); // request 9 bytes from slave device address
  //Read IIC data and save to rx_buf[]
   while ( Wire.available()) 
  { 
    rx_buf[i] = Wire.read(); // received one byte 
    //Print the rx_buf data via serial
    Serial.print("0x");
    Serial.print(rx_buf[i],HEX);
    Serial.print(";");
    i++; 
   }
   //Judge and print result via serial
   for(i=0;i<5;i++)
   {
      check_sum += rx_buf[i];
   }

   if(rx_buf[0] == 0x5A && rx_buf[1] == 0x06 && rx_buf[2] == 0x03 && rx_buf[5] == check_sum)
   {
      Serial.print("    Lidar's frame rate has been set to ");
      Serial.print(rx_buf[3] + rx_buf[4]*256,DEC);
      Serial.print("Hz");
   }
   else
   {
      Serial.print("Set frame rate error!");
   }
  Serial.println();
  Serial.print("\r\n"); 

}
//void A*()
//{
 //distance weighted grid search using bfs/dfs depending on path
 //the distance d cells are the "undiscovered" neighbors of the distance d-1 cells
 // single source shortest path algorithm bfs
 // the "frontiers of the grid is of the current distance from the source"
 // base case: when the frontier is empty, that is there is nothing left to search
//}
void SaveLidarSetFromIIC(unsigned char address)
{
  char i = 0; 
  unsigned char rx_buf[5] = {0}; 
  Wire.beginTransmission(address); // Begin a transmission to the I2C Slave device with the given address. 
  Wire.write(0x5A); // see product mannual table 11:Obtain Data Frame
  Wire.write(0x04); // 
  Wire.write(0x11); // 
  Wire.write(0x6f); // 
  Wire.endTransmission(1);  // Send a STOP Sign 
  delay(10);
  Wire.endTransmission(0);  // Send a START Sign 
  Wire.requestFrom(address, 5 , 1); // request 9 bytes from slave device address
  //Read IIC data and save to rx_buf[]
   while ( Wire.available()) 
  { 
    rx_buf[i] = Wire.read(); // received one byte 
    Serial.print("0x");
    Serial.print(rx_buf[i],HEX);
    Serial.print(";");
    i++; 
   }
   //Judge and print result via serial
   if(rx_buf[0] == 0x5A && rx_buf[1] == 0x05 && rx_buf[2] == 0x11 && rx_buf[3] == 0x00 && rx_buf[4] == 0x70)
   {
       Serial.print("Lidar's set has been saved");
   }
   else
   {
      Serial.print("Save error!");
   }

  Serial.print("\r\n"); 
}
void setup() 
{

  pinMode(LED_BUILTIN, OUTPUT);//LED, system starts
  pinMode(altiTrig,OUTPUT);
  pinMode(altiecho,INPUT);
  pinMode(13,OUTPUT);
  Serial.begin(115200); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050设备,6B是启动电平发送
  GetLidarVersionFromIIC(0x10);
  SaveLidarSetFromIIC(0x10);//Save #10 Lidar`s set.
  Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间

}

void setamplfiers()//made for higher speed or rotational motion environments
{
  /*Wire.write(0x1C);
  Wire.requestFrom(0x68,1,true);
  unsigned char amp=Wire.read();
  amp=(amp)
  */
  Wire.beginTransmission(0x68); //开启MPU-6050的传输
  Wire.write(0x1C); //加速度倍率寄存器的地址
  Wire.requestFrom(0x68, 1, true); //先读出原配置
  unsigned char acc_conf = Wire.read();
  acc_conf = ((acc_conf & 0xE7) | (f << 3));
  Wire.write(acc_conf);
  Wire.endTransmission(true); //结束传输，true表示释放总线
}
void loop() 
{
  Get_LidarDatafromIIC(0x10);//Get #10 Lidar measurment result from IIC;
  int readouts[nValCnt];
  ReadAccGyr(readouts); //读出测量值
  getAltidata();
  float realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正
  //计算加速度向量的模长，均以g为单位
  analogWrite(13,4);
  
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
//  float fNewdist = Kalmandist;
  //跟据滤波值计算角度速
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
  //float f=kalmanRoll;
  
 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  //更新本次测的时间
  nLastTime = nCurTime;
  //向串口打印输出Roll角和Pitch角，运行时在Arduino的串口监视器中查看
  Serial.print("Roll:");
  Serial.print(fNewRoll); Serial.print('(');
  Serial.print(fRollRate); Serial.print("),\tPitch:");
  Serial.print(fNewPitch); Serial.print('(');
  Serial.print(fPitchRate); Serial.print(")");
  Serial.print("temp:");Serial.println(readouts[6]);
  delay(10);
}

