#include <SoftwareSerial.h>

// Serial Definitions
#define HOVER_SERIAL_BAUD 57600
#define SERIAL_BAUD       115200
#define START_FRAME       0xABCD
#define TIME_SEND         100

// SoftwareSerial for hoverboard drivers
SoftwareSerial HoverSerialRight(3, 4);  // Right driver
SoftwareSerial HoverSerialLeft(9, 10);  // Left driver

// SoftwareSerial for Mega communication (RX=A0, TX=A1)
SoftwareSerial megaSerial(A0, A1);

// Structs for command and feedback
typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;

typedef struct {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

SerialCommand Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

byte incomingByte, incomingBytePrev;
byte* p;
uint8_t idx = 0;
uint16_t bufStartFrame;

int16_t speedLeftCmd = 0;    // Will be set by Mega
int16_t speedRightCmd = 0;   // Will be set by Mega
int16_t speedLeftFeedback = 0;
int16_t speedRightFeedback = 0;

unsigned long iTimeSend = 0;

// ------------------ Setup ------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  HoverSerialRight.begin(HOVER_SERIAL_BAUD);
  HoverSerialLeft.begin(HOVER_SERIAL_BAUD);
  megaSerial.begin(57600);    // Communication with Mega

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("UNO Dual Hoverboard Driver - Mega Control Mode");
}

// ------------------ Send Command ------------------
void SendCommand(SoftwareSerial &port, int16_t speed) {
  Command.start = START_FRAME;
  Command.steer = 2 * speed;
  Command.speed = 0;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  port.write((uint8_t*)&Command, sizeof(Command));
}

// ------------------ Receive Feedback ------------------
bool ReceiveFeedback(SoftwareSerial &port, int16_t &avgSpeed) {
  static SerialFeedback NewFeedback;
  while (port.available()) {
    incomingByte = port.read();
    bufStartFrame = ((uint16_t)incomingByte << 8) | incomingBytePrev;

    if (bufStartFrame == START_FRAME) {
      p = (byte*)&NewFeedback;
      *p++ = incomingBytePrev;
      *p++ = incomingByte;
      idx = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
      *p++ = incomingByte;
      idx++;
    }

    if (idx == sizeof(SerialFeedback)) {
      uint16_t checksum = NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
                          NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
                          NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;

      if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
        memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
        avgSpeed = (Feedback.speedR_meas + Feedback.speedL_meas) / 2;
        idx = 0;
        return true;
      }
      idx = 0;
    }
    incomingBytePrev = incomingByte;
  }
  return false;
}

// ------------------ Read Mega Commands ------------------
void ReadMegaCommands() {
  if (megaSerial.available() >= 6) {  // Expecting "LxxxRxxx" (6 bytes)
    if (megaSerial.read() == 'L') {
      speedLeftCmd = megaSerial.parseInt();  // Read left speed
      if (megaSerial.read() == 'R') {
        speedRightCmd = megaSerial.parseInt();  // Read right speed
      }
    }
    // Optional: Debug received speeds
    Serial.print("Mega Cmd: L=");
    Serial.print(speedLeftCmd);
    Serial.print(" R=");
    Serial.println(speedRightCmd);
  }
}
void SendFeedbackToMega() {
  megaSerial.print("FB");  // Feedback header
  megaSerial.print("L");
  megaSerial.print(speedLeftFeedback);
  megaSerial.print("R");
  megaSerial.print(speedRightFeedback);
  megaSerial.println();  // End of message
}

// ------------------ Main Loop ------------------
void loop() {
  unsigned long now = millis();

  ReadMegaCommands();  // Receive speeds from Mega

  if (now >= iTimeSend) {
    iTimeSend = now + TIME_SEND;

    // Send motor commands
    SendCommand(HoverSerialRight, -speedRightCmd);
    SendCommand(HoverSerialLeft, speedLeftCmd);

    // Alternate reading feedback
    static bool readNow = false;
    static unsigned long lastReadTime = 0;
    if (now - lastReadTime >= 200) {
      lastReadTime = now;
      readNow = !readNow;

      if (readNow) {
        HoverSerialLeft.listen();
        delay(10);
        ReceiveFeedback(HoverSerialLeft, speedLeftFeedback);
      } else {
        HoverSerialRight.listen();
        delay(10);
        int16_t rawFeedback;
        if (ReceiveFeedback(HoverSerialRight, rawFeedback)) {
          speedRightFeedback = -rawFeedback;
        }
      }
      SendFeedbackToMega();  // Send feedback after update
    }

    // Debug output (optional)
    Serial.print("L_cmd: "); Serial.print(speedLeftCmd);
    Serial.print(" | L_fb: "); Serial.print(speedLeftFeedback);
    Serial.print(" || R_cmd: "); Serial.print(speedRightCmd);
    Serial.print(" | R_fb: "); Serial.println(speedRightFeedback);
  }

  digitalWrite(LED_BUILTIN, (now % 1000) < 500);
}