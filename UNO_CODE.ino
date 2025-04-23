#include <SoftwareSerial.h>

// Serial Definitions
#define HOVER_SERIAL_BAUD 57600
#define SERIAL_BAUD       57600
#define START_FRAME       0xABCD
#define TIME_SEND         100

// SoftwareSerial for hoverboard drivers
SoftwareSerial HoverSerialRight(3, 4);  // Right driver
SoftwareSerial HoverSerialLeft(9, 10);  // Left driver

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

int16_t speedLeftCmd = 0;
int16_t speedRightCmd = 0;
int16_t speedLeftFeedback = 0;
int16_t speedRightFeedback = 0;

unsigned long iTimeSend = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);  // Use hardware serial for Mega
  HoverSerialRight.begin(HOVER_SERIAL_BAUD);
  HoverSerialLeft.begin(HOVER_SERIAL_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("UNO Dual Hoverboard Driver - Mega Control Mode (Hardware Serial)");
}

void SendCommand(SoftwareSerial &port, int16_t speed) {
  Command.start = START_FRAME;
  Command.steer = 2 * speed;
  Command.speed = 0;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  port.write((uint8_t*)&Command, sizeof(Command));
}

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

void ReadMegaCommands() {
  static String input = "";

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n') {
      if (input.startsWith("L")) {
        int lPos = input.indexOf('L');
        int rPos = input.indexOf('R');
        if (lPos >= 0 && rPos > lPos) {
          speedLeftCmd = input.substring(lPos + 1, rPos).toInt();
          speedRightCmd = input.substring(rPos + 1).toInt();

          Serial.print("Updated Speeds: L=");
          Serial.print(speedLeftCmd);
          Serial.print(" R=");
          Serial.println(speedRightCmd);
        }
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void SendFeedbackToMega() {
  Serial.print("FB");
  Serial.print("L");
  Serial.print(speedLeftFeedback);
  Serial.print("R");
  Serial.println(speedRightFeedback);
}

void loop() {
  unsigned long now = millis();

  ReadMegaCommands();

  if (now >= iTimeSend) {
    iTimeSend = now + TIME_SEND;

    // Send speed commands
    SendCommand(HoverSerialRight, -speedRightCmd);
    SendCommand(HoverSerialLeft, speedLeftCmd);

    // Always try to read both feedbacks every cycle
    HoverSerialLeft.listen();
    delay(5);
    ReceiveFeedback(HoverSerialLeft, speedLeftFeedback);

    HoverSerialRight.listen();
    delay(5);
    int16_t rawFeedback;
    if (ReceiveFeedback(HoverSerialRight, rawFeedback)) {
      speedRightFeedback = -rawFeedback;
    }

    // Send both feedbacks to Mega
    SendFeedbackToMega();

    // Blink LED
    digitalWrite(LED_BUILTIN, (now % 1000) < 500);
  }
}
