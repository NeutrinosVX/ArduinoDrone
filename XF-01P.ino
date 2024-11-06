#define RC_TYPE  1 //设备类型：遥控发射器=1，接收机=0
#define NRF24L01_CE_PIN   8
#define NRF24L01_CSN_PIN   10
#define NRF24L01_MOSI_PIN   11
#define NRF24L01_MISO_PIN   12
#define NRF24L01_SCK_PIN   13
#include <Kalman.h>
#include <Wire.h>
#include <Math.h>
#include <TFMPI2C.h>
#include <printf.h>
#include <RF24.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Servo.h>
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;
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
uint8_t Mission=1;
Kalman kalmanRoll; //Roll filter
Kalman kalmanPitch; //Pitch filter
RF24 radio(NRF24L01_CE_PIN, NRF24L01_CSN_PIN);
double Kp=2, Ki=5, Kd=1;
double target, Input, Output;
PID myPID(&Input, &Output, &target, Kp, Ki, Kd, DIRECT);
/*void getAltidata()
{
  #Single Sampling 
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

//要在一对无线电上使用不同的地址，让这些地址用于配对
uint8_t address[][6] = { "1Node", "2Node" };// 0 uses address[0] to transmit, 1 uses address[1] to transmit

//发送缓冲区
uint8_t payload[sizeof(float)*8+4];// // only using 4 bytes for TX payloads

//接收缓冲区
uint8_t received[4] = {0,0,0,0};// // only using 4 bytes for ACK payloads

uint16_t NRF24L01sender() {
  // This device is a TX node

  uint16_t ret = 0xFEEF;

  //payload[0] = 0xFE;
  //payload[1] = (byte)((value & 0xFF00)>>8);
  //payload[2] = (byte)(value & 0xFF);
  //payload[3] = 0xEF;

  unsigned long start_timer = micros();                  // start the timer
  bool report = radio.write(&payload, sizeof(payload));  // transmit & save the report
  unsigned long end_timer = micros();                    // end the timer
  if (report) {
    Serial.print(F("Transmission successful! "));  // payload was delivered
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);  // print the timer result
    Serial.print(F(" us. Sent: "));
    for(int i=0;i<sizeof(payload);i++){
  	Serial.print(payload[i],HEX);  // print incoming message
  	Serial.print(' ');
    }
    Serial.println("");
    uint8_t pipe;
    if (radio.available(&pipe)) {  // 有ACK（确认收到）有效载荷吗？获取收到它的信道号

  	radio.read(&received, sizeof(received));  // get incoming ACK payload
  	Serial.print(F(" Recieved "));
  	Serial.print(radio.getDynamicPayloadSize());  // print incoming payload size
  	Serial.print(F(" bytes on pipe "));
  	Serial.print(pipe);  // print pipe number that received the ACK
  	Serial.print(F(": "));
      for(int i=0;i<radio.getDynamicPayloadSize();i++){
  	  Serial.print(received[i],HEX);  // print incoming message
  	  Serial.print(' ');
      }
  	Serial.println("");

    }
     else {
  	Serial.println(F(" Recieved: an empty ACK packet"));  // empty ACK packet received
    }

    ret = 0;

  } 
  else {
    Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
  }

  return ret;
}
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
int Get_LidarDatafromIIC(unsigned char address)
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
      if(Strength>=30)
      {
        return dist;
      }
      else
      {
        return 0;
      }
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
  delay(500);
  //初始化无线收发模块
    // initialize the transceiver on the SPI bus
    if (!radio.begin()) {
      Serial.println(F("radio hardware is not responding!!"));
      while (1) {}  // hold in infinite loop
    }
    
    //当发射与接收节点非常接近的情况下运行时，将PA级别（耗电）设置为较低，以防止产生供电相关的问题
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

    // 要使用ACK有效载荷，我们需要启用动态有效载荷长度（针对所有节点）
    radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

    //默认情况下，确认数据包没有有效载荷。我们需要启用
    //此功能用于所有节点（TX和RX）使用ACK有效载荷。
    radio.enableAckPayload();

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[RC_TYPE]);  // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!RC_TYPE]);  // using pipe 1

    // additional setup specific to the node's role
    if (RC_TYPE) {
      // setup the TX payload
      radio.stopListening();                 // put radio in TX mode
    } 
    else {
      // 设置ACK有效载荷并将第一个响应加载到FIFO

      //memcpy(payload.message, "World ", 6);  // set the payload message
      // load the payload for the first received transmission on pipe 0
      radio.writeAckPayload(1, &payload, sizeof(payload));
      radio.startListening();  // put radio in RX mode
    }

  //初始化MPU6050
  WriteMPUReg(0x6B, 0);
  Calibration();
  nLastTime = micros();

}
void motor1(int p)
{
  while (Serial.available() == 0);
  {
    if(p < 1100 || p > 1900)
    {
      Serial.println("not valid");
    }
    else
    {
      Motor1.writeMicroseconds(p); // Send signal to ESC.
    }
  } 
}
void motor2()
{
  while (Serial.available() == 0);
  {
    int val = Serial.parseInt(); 
    if(val < 1100 || val > 1900)
    {
      Serial.println("not valid");
    }
    else
    {
      Motor2.writeMicroseconds(val); // Send signal to ESC.
    }
  } 
}
void motor3()
{
  while (Serial.available() == 0);
  {
    int val = Serial.parseInt(); 
    if(val < 1100 || val > 1900)
    {
      Serial.println("not valid");
    }
    else
    {
      Motor3.writeMicroseconds(val); // Send signal to ESC.
    }
  } 
}
void motor4()
{
  while (Serial.available() == 0);
  {
    int val = Serial.parseInt(); 
    if(val < 1100 || val > 1900)
    {
      Serial.println("not valid");
    }
    else
    {
      Motor4.writeMicroseconds(val); // Send signal to ESC.
    }
  } 
}
void loop() 
{
  int readouts[nValCnt];
  int dist=Get_LidarDatafromIIC(0x10);
  getAltidata();
  ReadAccGyr(readouts);
  float realVals[7];
  Rectify(readouts, realVals);
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm);
  if (realVals[1] > 0) {
  fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm);
  if (realVals[0] < 0) {
  fPitch = -fPitch;
  }

  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;

  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);

  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;

  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;

  //打印数据
  /*Serial.print(realVals[0]*9.81);
  Serial.print(" ");
  Serial.print(realVals[1]*9.81);
  Serial.print(" ");
  Serial.println(realVals[2]*9.81);
  Serial.print("Roll:");
  Serial.print(fNewRoll); Serial.print('(');
  Serial.print(fRollRate); Serial.print("),\tPitch:");
  Serial.print(fNewPitch); Serial.print('(');
  Serial.print(fPitchRate); Serial.print(")");
  Serial.print("temp:");Serial.println(readouts[6]);*/

  //发送数据
  //发送数据
  float frealVal0 = realVals[0]*9.81;
  float frealVal1 = realVals[1]*9.81;
  float frealVal2 = realVals[2]*9.81;

  payload[0] = 0xFE;
  memcpy(&payload[1], &frealVal0, sizeof(float));
  memcpy(&payload[5], &frealVal1, sizeof(float));
  memcpy(&payload[9], &frealVal2, sizeof(float));
  memcpy(&payload[13], &fNewRoll, sizeof(float));
  memcpy(&payload[17], &fRollRate, sizeof(float));
  memcpy(&payload[21], &fNewPitch, sizeof(float));
  memcpy(&payload[25], &fPitchRate, sizeof(float));
  memcpy(&payload[29], &readouts[6], sizeof(int));
  memcpy(&payload[33], &dist, sizeof(int));
  payload[35] = 0xEF;
  NRF24L01sender();
  delay(10);  
}
