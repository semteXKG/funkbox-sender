#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include "heltec_unofficial.h"

#include "funk_serial.h"
#include "messages.h"

#define MAX_ITEMS 32
#define MAX_READ_BYTES 100

#define INTERVAL 5000
#define MAX_TIMEOUT 8000

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
#define FREQUENCY 866.3 // for Europe
// #define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH 250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference.
#define SPREADING_FACTOR 12

// This value can be set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). 
// Note that the maximum ERP (which is what your antenna maximally radiates) on the 
// EU ISM band is 25 mW, and that transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER 22

payloaded_message *tx_queue[MAX_ITEMS];
uint64_t message_counter = 0;
uint64_t tx_time = 0;
uint64_t last_rx = 0;

boolean received = false;
String rxString;

//
//
// COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM 
//
//

void rx()
{
    received = true;
}

void comm_setup()
{
    memset(tx_queue, 0, sizeof(payloaded_message*) * MAX_ITEMS);
    both.println("Radio init");
    RADIOLIB_OR_HALT(radio.begin());
    // Set the callback function for received packets
    radio.setDio1Action(rx);
    // Set radio parameters
    both.printf("Frequency: %.2f MHz\n", FREQUENCY);
    RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
    both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
    RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
    both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
    RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
    both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
    RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
    // Start receiving
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

void comm_enqueue_message(struct payloaded_message *message)
{
    message->seq_nr = message_counter;
    
    tx_queue[message_counter % MAX_ITEMS] = message;
   
    message_counter++;
    if (message_counter > INT16_MAX)
        message_counter = 0;
    }

void transmit_message(struct payloaded_message *message)
{
    Serial.printf("Transmitting MSG: %s, %d", message->command, message->seq_nr);

    radio.clearDio1Action();
    heltec_led(50);
    tx_time = millis();
    RADIOLIB(radio.transmit(message->command));
    tx_time = millis() - tx_time;
    heltec_led(0);
    if (_radiolib_status == RADIOLIB_ERR_NONE)
    {
        both.printf("OK (%i ms)\n", (int)tx_time);
    }
    else
    {
        both.printf("fail (%i)\n", _radiolib_status);
    }

    radio.setDio1Action(rx);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

void send_ack(uint8_t seqNr) {
    Serial.printf("Sending ack for %d\n", seqNr);
    payloaded_message* message = (payloaded_message*) malloc(sizeof(payloaded_message));
    
    strcpy(message->command, "ACK");
    sprintf(message->payload, "%d", seqNr);

    comm_enqueue_message(message);
}

void process_received_message(const char* message) {
    char updateable_message[strlen(message)];
    strcpy(updateable_message, message);

    char* message_type = strtok(updateable_message, "|"); 
    u_int8_t sequence_number = atoi(strtok(NULL, "|"));
    char* payload = strtok(NULL, "|");
    both.printf("%s(%d): %s\n", message_type, sequence_number, payload);

    send_ack(sequence_number);
}

void comm_loop()
{
    for (int i = 0; i < MAX_ITEMS; i++)
    {
        if (tx_queue[i] != NULL &&
            tx_queue[i]->last_sent < (millis() - 2000))
        {
            transmit_message(tx_queue[i]);
            tx_queue[i]->last_sent = millis();
        }
    }

    if (received)
    {
        radio.readData(rxString);
        if (_radiolib_status == RADIOLIB_ERR_NONE)
        {
            both.printf("RX [%s]\n", rxString.c_str());
            both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
            both.printf("  SNR: %.2f dB\n", radio.getSNR());
            process_received_message(rxString.c_str());
        }
        last_rx = millis();
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    }
}

//
//
// SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL  SERIAL 
//
//

void serial_setup() {
    Serial.setTimeout(5000);
}

void serial_loop() {
    if (Serial.available()) {
        String readString = Serial.readStringUntil('\n');
        Serial.printf("Readback: %s\n", readString);
    }
}


//
//
// MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN 
//
//



void setup() {
  heltec_setup();
  serial_setup();
  //comm_setup();
}

void loop() {
  heltec_loop();
  serial_loop();
}


