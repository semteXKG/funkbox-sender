/**
 * Send and receive LoRa-modulation packets with a sequence number, showing RSSI
 * and SNR for received packets on the little display.
 *
 * Note that while this send and received using LoRa modulation, it does not do
 * LoRaWAN. For that, see the LoRaWAN_TTN example.
 *
 * This works on the stick, but the output on the screen gets cut off.
*/
// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include "heltec_unofficial.h"
//#include "communicator.h"
#include "funk_serial.h"

void setup() {
  heltec_setup();
  serial_setup();
  //comm_setup();
}

void loop() {
  heltec_loop();
  serial_loop();
}


