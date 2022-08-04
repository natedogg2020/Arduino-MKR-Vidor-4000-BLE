/**
 * @file main.cpp
 * @author Nate McKelvey
 * @brief This is a stripped and revised version of Arduino's Nina W102 passthrough program. 
 *        The change from the original is using a buffer for the serial, which was added
 *        by GitHub user sameer and copied/stripped down here.
 *        (See https://github.com/arduino-libraries/WiFiNINA/pull/112/commits/cced9c5e9aa7ceb3d46032cb4a7acf207a8e6711)
 */

#include <Arduino.h>

/*
  SerialNINAPassthrough - Use esptool to flash the u-blox NINA (ESP32) module
  Arduino MKR WiFi 1010, Arduino MKR Vidor 4000, and Arduino UNO WiFi Rev.2.
  Copyright (c) 2018 Arduino SA. All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <VidorPeripherals.h>

unsigned long baud = 119400;

#define BUFFER_SIZE 512
static unsigned int buffer_length;
static char buffer[BUFFER_SIZE];

int rts = -1;
int dtr = -1;

void setup() {
  Serial.begin(baud);
  FPGA.begin();   // Startup FPGA with its preloaded VidorPeripheral binary data
  SerialNina.begin(baud); // Start FPGA's UART passthrough to Nina W102
  // Setup flashing pins for Nina W102
  FPGA.pinMode(FPGA_NINA_GPIO0, OUTPUT);
  FPGA.pinMode(FPGA_SPIWIFI_RESET, OUTPUT);
}

void loop() {
  // When Serial states rts() [Indicating Serial via USB wants to send data], reset the Nina W102
  if (rts != Serial.rts()) {
    FPGA.digitalWrite(FPGA_SPIWIFI_RESET, (Serial.rts() == 1) ? LOW : HIGH);
    rts = Serial.rts();
  }

  // When Serial states dtr() [Indicates Serial via USB is ready to receive data], put Nina W102 into upload state
  if (dtr != Serial.dtr()) {
    FPGA.digitalWrite(FPGA_NINA_GPIO0, (Serial.dtr() == 1) ? HIGH : LOW);
    dtr = Serial.dtr();
  }

  // When Serial via USB is received, write this to the Nina W102
  if (Serial.available() > 0) {
    buffer_length = min(Serial.available(), BUFFER_SIZE);
    Serial.readBytes(buffer, buffer_length);
    SerialNina.write(buffer, buffer_length);
  }

  // When Nina W102 Serial is received, write this to the USB Serial
  if (SerialNina.available() > 0) {
    buffer_length = min(SerialNina.available(), BUFFER_SIZE);
    SerialNina.readBytes(buffer, buffer_length);
    Serial.write(buffer, buffer_length);
  }

  // Update baud if the USB virtual serial wants a new baud rate
  if (Serial.baud() != baud) {
    // Reset rts/dtr to reset Nina W102 back into upload mode on next iteration
    rts = -1;
    dtr = -1;
    // Update baud
    baud = Serial.baud();
  }
}