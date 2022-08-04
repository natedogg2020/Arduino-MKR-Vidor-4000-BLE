/**
 * @file main.cpp
 * @author Nate McKelvey
 * @brief This is a very bare ESP32 beacon using the Arduino framework within Platform IO. This was initially implemented by 
 *        various authors attributed below, which was then refactored into the code you see below. This allows the ESP32 to 
 *        be seen as a BLE device, which can be found by it's BLE local name, which is ESP32 SimpleBLE initially. 
 * 
 *        Due to the small flash size of the Nina W102 chip and overarching limitations, using BLEDevice and BLEBeacon are 
 *        not possible as of writing this.
 */

#include "Arduino.h"
// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Sketch shows how to use SimpleBLE to advertise the name of the device and change it on the press of a button
// Useful if you want to advertise some sort of message
// Button is attached between GPIO 0 and GND, and the device name changes each time the button is pressed

#include "SimpleBLE.h"
#include <driver/gpio.h>

// Pins for the On-Board Vidor LEDs, which are active low, meaning LOW turns them on, HIGH turns them off
#define GREEN_LED 26
#define BLUE_LED  25

// Globals to be initialized once, since Platform IO doesn't global-ify the setup() vars by default.
SimpleBLE ble;
static uint8_t lastPinState = 1;
uint8_t pinState = 1;

void onButton(){
    String out = "BLE32 name: ";
    out += String(millis() / 1000);
    Serial.println(out);
    ble.begin(out);
}

void setup() {
    delay(3000);    // Wait 3 sec, since too much power draw occurs when NinaW102 starts immediately, causing boot loop

    // Setup LEDs as GPIO, since the LED pins aren't GPIO pins by default
    gpio_set_direction((gpio_num_t)BLUE_LED, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode((gpio_num_t)BLUE_LED, GPIO_FLOATING);
    gpio_set_direction((gpio_num_t)GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode((gpio_num_t)GREEN_LED, GPIO_FLOATING);
    
    digitalWrite(BLUE_LED, LOW);    // LEDs are active low (turned on when driven to 0 volts)
    digitalWrite(GREEN_LED, LOW);

    Serial.begin(119400);
    Serial.setDebugOutput(true);
    pinMode(0, INPUT_PULLUP);
    Serial.print("ESP32 SDK: ");
    Serial.println(ESP.getSdkVersion());
    ble.begin("ESP32 SimpleBLE");
    Serial.println("Press the button to change the device's name");
}

void loop() {
    pinState = digitalRead(0);
    if(!pinState && lastPinState){
        onButton();
    }
    lastPinState = pinState;
    while(Serial.available()) Serial.write(Serial.read());
}
