#include <Arduino.h>
#include <ArduinoRS485.h>
#include "TinyFrame.h"
#include "utils.h"



TinyFrame* tfapp;




void TF_WriteImpl(TinyFrame * const tf, const uint8_t *buff, uint32_t len)
{
  RS485.noReceive();
  RS485.beginTransmission();
  RS485.println(String(buff, len));
  RS485.endTransmission();
  RS485.receive();
  //Serial.println(String(buff, len));
}


TF_Result GEN_Listener(TinyFrame *tf, TF_Msg *msg)
{
  Serial.println(msg->data[0]);
  Serial.println(msg->data[1]);
  Serial.println(msg->data[2]);
  Serial.println(msg->data[3]);
  //Serial.println(String(msg->data, msg->len));
  return TF_STAY;
}


TF_Result ID_Listener(TinyFrame *tf, TF_Msg *msg){
    return TF_CLOSE;
}



void setup() {

  tfapp = TF_Init(TF_SLAVE);
  TF_AddGenericListener(tfapp, GEN_Listener);

  Serial.begin(115200);
  while (!Serial){}
  RS485.begin(115200);
  //Serial.println("started");
  uint8_t data[] = {0xdf, 255, 0, 1, 1};
  //Serial.println(String(data, 4));
  RS485.noReceive();
  RS485.beginTransmission();
  TF_Msg msg;
  msg.data = data;
  msg.len = 4;
  msg.type = 1;
  TF_Query(tfapp, &msg, ID_Listener, NULL, 0);
  RS485.endTransmission();
  RS485.receive();
}

void loop() {
  
  /*while (Serial.available())
  {
    TF_AcceptChar(tfapp, Serial.read());
  }*/

  while (RS485.available())
  {
    TF_AcceptChar(tfapp, RS485.read());
  }
}