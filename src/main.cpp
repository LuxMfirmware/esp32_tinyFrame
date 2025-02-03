/*
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
  Serial.println("till podaci");
   Serial.println("till podaci");
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
/*
  while (RS485.available())
  {
    Serial.println("till podaci");
    int dat = RS485.read();
    Serial.println(dat);
    TF_AcceptChar(tfapp, dat);
  }
} */
#include <Arduino.h>
#include <TinyFrame.h>

#define KOLIKO_CEKAM_ODGOVOR_U_MS 100
#define RS485_DE_PIN 32  // Pin za kontrolu RS485 transceivera

HardwareSerial &rs485Serial = Serial2; // RS485 port
HardwareSerial &serialMonitor = Serial; // Serijski monitor
TinyFrame tf;
//
// kad trebadne slat
//
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    digitalWrite(RS485_DE_PIN, HIGH); // Omogući slanje na RS485
    delay(1); // cek cek dok se smiri
    rs485Serial.write(buff, len);
    rs485Serial.flush();
    digitalWrite(RS485_DE_PIN, LOW); // Vrati u režim prijema
}

TF_Result listener(TinyFrame *tf, TF_Msg *msg) {
    serialMonitor.print("Primljeni podaci (text): ");
    for (uint16_t i = 0; i < msg->len; i++) {
        serialMonitor.write(msg->data[i]);
    }
    serialMonitor.println();
    
    serialMonitor.print("Primljeni podaci (hex): ");
    for (uint16_t i = 0; i < msg->len; i++) {
        serialMonitor.printf("%02X ", msg->data[i]);
    }
    serialMonitor.println();

    //return TF_CLOSE; // primi samo jedan odgovor
    return TF_STAY; // nastavi primat
}

void setup() {
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW); // Početno u režimu prijema
    
    serialMonitor.begin(115200, SERIAL_8N1);
    rs485Serial.begin(115200, SERIAL_8N1);
    
    TF_InitStatic(&tf, TF_MASTER);
    TF_AddGenericListener(&tf, listener);
    
    delay(1000);
    serialMonitor.println("Slanje tf_query...");
    
    TF_Msg msg;
    msg.type = 0x01;
    msg.data = (uint8_t *)"QUERY";
    msg.len = 5;

    TF_Query(&tf, &msg, listener, NULL, KOLIKO_CEKAM_ODGOVOR_U_MS); // treba implementirat tinyframe 1ms clock funkciju  TF_Tick(TinyFrame *tf)
}

void loop() {
    while (rs485Serial.available()) {
        uint8_t byte = rs485Serial.read();
        TF_AcceptChar(&tf, byte);
    }
}
