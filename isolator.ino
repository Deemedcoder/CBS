#include <Arduino.h>
#include "STM32_CAN.h"

HardwareSerial Serial1(PA10, PA9);  // Debug via Serial1

// Use PA11/PA12 pins for CAN1
STM32_CAN Can(CAN1, DEF);
static CAN_message_t CAN_TX_msg;

// Analog input pins
const uint8_t analogPins[7] = { PC1, PC2, PC3, PA0, PA1, PA2, PA3 };
String pinNames[7] = { "PC1", "PC2", "PC3", "PA0", "PA1", "PA2", "PA3" };

// Convert voltage to status
int interpretVoltage(float voltage) {
  if (voltage >= 0.500 && voltage <= 0.650) return 0;  // Connected
  if (voltage >= 0.200 && voltage <= 0.300) return 1;  // Trip
  if (voltage >= 0.700 && voltage <= 0.900) return 2;  // Disconnected
  return 9;  // Invalid
}

void setup() {
  Serial1.begin(115200);
  delay(100);

  for (int i = 0; i < 7; i++) {
    pinMode(analogPins[i], INPUT);
  }

  Can.begin();
  Can.setBaudRate(500000);

  Serial1.println("✅ Starting CAN Status Sender...");
}

void loop() {
  String statusString = "";

  // Build status string and CAN message
  CAN_TX_msg.id = 0x321;
  CAN_TX_msg.len = 7;

  for (int i = 0; i < 7; i++) {
    int adc = analogRead(analogPins[i]);
    float voltage = (adc * 3.3) / 4095.0;
    int status = interpretVoltage(voltage);

    CAN_TX_msg.buf[i] = status;
    statusString += String(status);

    Serial1.print(pinNames[i]); Serial1.print(": ");
    Serial1.print(voltage, 3); Serial1.print(" V -> ");
    Serial1.println(status);
  }

  Can.write(CAN_TX_msg);

  Serial1.print("📤 Sent StatusString: ");
  Serial1.println(statusString);
  Serial1.println("---------------------------");

  //delay(200);  // Send every 200 ms
}
