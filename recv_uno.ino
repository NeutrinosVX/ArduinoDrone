#define RC_TYPE  0 //设备类型：遥控发射器=1，接收机=0
#define NRF24L01_CE_PIN   8
#define NRF24L01_CSN_PIN   10
#define NRF24L01_MOSI_PIN   11
#define NRF24L01_MISO_PIN   12
#define NRF24L01_SCK_PIN   13
#include <SPI.h>
#include <printf.h>
#include <RF24.h>

// 实例化nRF24L01收发器的对象
RF24 radio(NRF24L01_CE_PIN, NRF24L01_CSN_PIN);

//要在一对无线电上使用不同的地址，让这些地址用于配对
uint8_t address[][6] = { "1Node", "2Node" };// 0 uses address[0] to transmit, 1 uses address[1] to transmit

//发送与接收缓冲区
uint8_t payload[sizeof(float)*7+4];// // only using 4 bytes for TX & ACK payloads

uint16_t NRF24L01receiver() {
  // This device is a RX node
  uint16_t ret = 0xFEEF;
  uint8_t pipe;
  if (radio.available(&pipe)) {		// is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize();  // get the size of the payload

    radio.read(&payload, sizeof(payload));  // get incoming payload

    Serial.print(F("Received "));
    Serial.print(bytes);  // print the size of the payload
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);  // print the pipe number
    Serial.print(F(": "));
    for(int i=0;i<bytes ;i++){
  	Serial.print(payload[i],HEX);  // print incoming message
  	Serial.print(' ');
    }
    Serial.println("");

    //验证接收到的数据的有效性：信息块头0xFE, 信息块尾0xEF
    if(payload[0]==0xFE && payload[31]==0xEF){
  	ret = 0;
    }

    // 返回确认收到信息
    //payload[0] = 0xEE;
    //radio.writeAckPayload(1, &payload, 1);


  }

  return ret;
}

void setup(){
  Serial.begin(115200);
  //调试硬件串口初始化波特率
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
    } else {
      // 设置ACK有效载荷并将第一个响应加载到FIFO

      //memcpy(payload.message, "World ", 6);  // set the payload message
      // load the payload for the first received transmission on pipe 0
      radio.writeAckPayload(1, &payload, sizeof(payload));

      radio.startListening();  // put radio in RX mode
    }
}

void loop(){
  //无线接收开关锁指令
    uint16_t recv = NRF24L01receiver();
    if (recv != 0xFEEF) {
      //解析数据
      float frealVal0,frealVal1,frealVal2;
      float fNewRoll,fRollRate;
      float fNewPitch,fPitchRate;
      int temp;

      memcpy(&frealVal0, &payload[1], sizeof(float));
      memcpy(&frealVal1, &payload[5], sizeof(float));
      memcpy(&frealVal2, &payload[9], sizeof(float));
      memcpy(&fNewRoll, &payload[13], sizeof(float));
      memcpy(&fRollRate, &payload[17], sizeof(float));
      memcpy(&fNewPitch, &payload[21], sizeof(float));
      memcpy(&fPitchRate, &payload[25], sizeof(float));
      memcpy(&temp, &payload[29], sizeof(int));

      //打印数据
      Serial.print(frealVal0);
      Serial.print(" ");
      Serial.print(frealVal1);
      Serial.print(" ");
      Serial.println(frealVal2);
      Serial.print("Roll:");
      Serial.print(fNewRoll); Serial.print('(');
      Serial.print(fRollRate); Serial.print("),\tPitch:");
      Serial.print(fNewPitch); Serial.print('(');
      Serial.print(fPitchRate); Serial.print(")");
      Serial.print("temp:");Serial.println(temp);


    }

}