#include "funk_serial.h"
#include "Arduino.h"
#include <string.h>

void serial_setup() {
    Serial.setTimeout(5000);
}

void serial_loop() {
    if (Serial.available()) {
        String readString = Serial.readStringUntil('\n');
        Serial.printf("Readback: %s\n", readString);
    }
}