#include <Arduino.h>
#include <CAN.h>
#include <BluetoothSerial.h>

// Enlever le commentaire ci-dessous pour passer de BT à USB
//#define MODE_USB

#define CAN_MAX 256
#define BT_MAX 1024

#ifdef MODE_USB

#else
BluetoothSerial SerialBT;
#endif

typedef enum
{
  CANStandard = 0,
  CANExtended = 1,
  CANAny = 2
} CANFormat;

typedef enum
{
  CANData = 0,
  CANRemote = 1
} CANType;

typedef struct
{
  unsigned long id;      // 29 bit identifier
  unsigned char data[8]; // Data field
  unsigned char len;     // Length of data field in bytes
  CANFormat format;      // Format ::CANFormat
  CANType type;          // Type ::CANType
} CANMessage;

CANMessage canBuffer[CAN_MAX];
int canRempli = 0;
int canVide = 0;
int canPerdu = 0;

uint8_t btBuffer[BT_MAX];
int btRempli = 0;
int btVide = 0;
int btPerdu = 0;

int etatBT = 0;
CANMessage canBT;
uint8_t checksumBT = 0;
uint8_t lenBT = 0;

uint8_t msgBT[15];
int lenMsgBT = 0;

void envoieBT(const CANMessage &msg)
{
  uint8_t checksum;
  if (msg.format == CANStandard)
  {
    msgBT[0] = (msg.type == CANData) ? 0x55 : 0x5A;
    msgBT[1] = (msg.id & 0x00000700) >> 8;
    msgBT[2] = msg.id & 0x000000FF;
    msgBT[3] = msg.len;
    checksum = msgBT[0] + msgBT[1] + msgBT[2] + msgBT[3];
    lenMsgBT = 4;
    if (msg.type == CANData)
    {
      for (int i = 0; i < msg.len; i++)
      {
        msgBT[4 + i] = msg.data[i];
        checksum += msg.data[i];
      }
      lenMsgBT += msg.len;
    }
  }
  else
  {
    msgBT[0] = (msg.type == CANData) ? 0xA5 : 0xAA;
    msgBT[1] = (msg.id & 0x1F000000) >> 24;
    msgBT[2] = (msg.id & 0x00FF0000) >> 16;
    msgBT[3] = (msg.id & 0x0000FF00) >> 8;
    msgBT[4] = msg.id & 0x000000FF;
    msgBT[5] = msg.len;
    checksum = msgBT[0] + msgBT[1] + msgBT[2] + msgBT[3] + msgBT[4] + msgBT[5];
    lenMsgBT = 6;
    if (msg.type == CANData)
    {
      for (int i = 0; i < msg.len; i++)
      {
        msgBT[6 + i] = msg.data[i];
        checksum += msg.data[i];
      }
      lenMsgBT += msg.len;
    }
  }
  msgBT[lenMsgBT++] = ~checksum;
#ifdef MODE_USB
  Serial.write(msgBT, lenMsgBT);
#else
  SerialBT.write(msgBT, lenMsgBT);
#endif
}

void btRead()
{
#ifdef MODE_USB
  while (Serial.available())
  {
    btBuffer[btRempli++] = Serial.read();
#else
  while (SerialBT.available())
  {
    btBuffer[btRempli++] = SerialBT.read();
#endif
    if (btRempli == BT_MAX)
    {
      btRempli = 0;
    }
    if (btRempli == btVide)
    { // buffer plein on perd un caractère
      btVide++;
      if (btVide == BT_MAX) {
        btVide = 0;
      }
      btPerdu++;
    }
  }
}

void btMachine()
{
  uint8_t c = btBuffer[btVide++];
  if (btVide == BT_MAX)
    btVide = 0;
  switch (etatBT)
  {
  case 0:
    checksumBT = c;
    if (c == 0x55)
    {
      etatBT = 1;
      canBT.format = CANStandard;
      canBT.type = CANData;
    }
    else if (c == 0x5A)
    {
      etatBT = 1;
      canBT.format = CANStandard;
      canBT.type = CANRemote;
    }
    else if (c == 0xA5)
    {
      etatBT = 1;
      canBT.format = CANExtended;
      canBT.type = CANData;
    }
    else if (c == 0xAA)
    {
      etatBT = 1;
      canBT.format = CANExtended;
      canBT.type = CANRemote;
    }
    break;
  case 1:
    checksumBT += c;
    canBT.id = c;
    etatBT = 2;
    break;
  case 2:
    checksumBT += c;
    canBT.id = (canBT.id << 8) + c;
    etatBT = (canBT.format == CANExtended) ? 3 : 5;
    break;
  case 3:
    checksumBT += c;
    canBT.id = (canBT.id << 8) + c;
    etatBT = 4;
    break;
  case 4:
    checksumBT += c;
    canBT.id = (canBT.id << 8) + c;
    etatBT = 5;
    break;
  case 5:
    checksumBT += c;
    canBT.len = c;
    if (c > 8)
    {
      etatBT = 0; // erreur
    }
    else
    {
      etatBT = ((canBT.type == CANData) && (c != 0)) ? 6 : 7;
      lenBT = 0;
    }
    break;
  case 6:
    checksumBT += c;
    canBT.data[lenBT++] = c;
    if (lenBT == canBT.len)
    {
      etatBT = 7;
    }
    break;
  case 7:
    checksumBT = (~checksumBT);
    etatBT = 0;
    if (c == checksumBT)
    {
      bool rtr = (canBT.type == CANRemote);
      if (canBT.format == CANStandard) {
        CAN.beginPacket(canBT.id, canBT.len, rtr);        
      } else {
        CAN.beginExtendedPacket(canBT.id, canBT.len, rtr);
      }
      if (!rtr) {
        CAN.write(canBT.data, canBT.len);
      }
      CAN.endPacket();
    }
    break;
  }
}

void canRead(int packetSize)
{
  canBuffer[canRempli].id = CAN.packetId();
  canBuffer[canRempli].format = CAN.packetExtended() ? CANExtended : CANStandard;
  canBuffer[canRempli].len = CAN.packetDlc();
  if (CAN.packetRtr()) {
    canBuffer[canRempli].type = CANRemote;
  } else {
    canBuffer[canRempli].type = CANData;
    int i = 0;
    while (CAN.available()) {
      canBuffer[canRempli].data[i++] = CAN.read();
    }
  }
  canRempli++;
  if (canRempli == CAN_MAX)
  {
    canRempli = 0;
  }
  if (canRempli == canVide)
  { // buffer plein on perd un message
    canVide++;
    if (canVide == CAN_MAX) {
      canVide = 0;
    }
    canPerdu++;
  }
}

void initCanBus()
{
  // start the CAN bus at 1000 kbps
  while (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
  }
  CAN.onReceive(canRead);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(921600);
  initCanBus();
#ifndef MODE_USB
  SerialBT.begin("ESP32debug");
#endif
}

void loop()
{
  // put your main code here, to run repeatedly:
#ifdef MODE_USB
  while (Serial.available()) {
#else
  while (SerialBT.available()) {
#endif
    btRead();
  }
  if (canRempli != canVide)
  {
#ifndef MODE_USB
    if (SerialBT.connected())
    {
#endif
      envoieBT(canBuffer[canVide]);
      canVide++;
      if (canVide == CAN_MAX)
      {
        canVide = 0;
      }
#ifndef MODE_USB
    } else {
      Serial.printf("BT not connected\n");
    }
#endif
  }
  if (btRempli != btVide)
  {
    btMachine();
  }
}
