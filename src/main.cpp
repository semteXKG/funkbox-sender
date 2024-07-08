#define RADIOLIB_DEBUG_BASIC true 
#define RADIOLIB_DEBUG_HEXDUMP true
#define HELTEC_POWER_BUTTON // must be before "#include <heltec_unofficial.h>"
#include "heltec_unofficial.h"
#include "esp32-hal.h"
#include "messages.h"

#define MAX_ITEMS 32
#define MAX_READ_BYTES 100

#define INTERVAL 2000
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
#define SPREADING_FACTOR 10

// This value can be set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW).
// Note that the maximum ERP (which is what your antenna maximally radiates) on the
// EU ISM band is 25 mW, and that transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER 0

payloaded_message *tx_queue[MAX_ITEMS];
uint64_t message_counter = 0;
unsigned long tx_time = 0;
unsigned long last_rx = 0;
unsigned long last_tx = 0;

boolean rx_in_progress = false;
volatile boolean received = false;
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
  memset(tx_queue, 0, sizeof(payloaded_message *) * MAX_ITEMS);
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
  message->last_sent = 0;
  int enq_pos = message_counter % MAX_ITEMS;
  Serial.printf("Enqueing at pos: %d\n", enq_pos);
  tx_queue[enq_pos] = message;

  message_counter++;
  if (message_counter > INT16_MAX)
    message_counter = 0;
}

void transmit_message(payloaded_message *messages[], int size) {
  char combined_output[1000];
  memset(combined_output, 0, 1000);

  for (int i = 0; i < size; i++) {
    char output[200];
    sprintf(output, "%s|%d|%s>", messages[i]->command, messages[i]->seq_nr, messages[i]->payload);
    strcat(combined_output, output);
  }

  tx_time = millis();
  RADIOLIB(radio.transmit(combined_output));
  tx_time = millis() - tx_time;
  heltec_led(0);
  if (_radiolib_status == RADIOLIB_ERR_NONE) {
    both.printf("OK (%i ms)\n", (int)tx_time);
  }
  else {
    both.printf("fail (%i)\n", _radiolib_status);
  }
  delay(500);

  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

void send_ack(uint8_t seqNr)
{
  Serial.printf("Sending ack for %d\n", seqNr);
  payloaded_message *message = (payloaded_message *)malloc(sizeof(payloaded_message));

  strcpy(message->command, "ACK");
  sprintf(message->payload, "%d", seqNr);

  comm_enqueue_message(message);
}

// handle a received message. right now this is just removing messages from the queue if they were ACKed
void handle_received_message(char* message_type, u_int8_t seq_nr, char* payload) {
  if(strcmp(message_type, "ACK") == 0) {
    Serial.printf("Got ACK Message for message %s\n", payload);
    u_int8_t acked_seq_nr = atoi(payload);
    for (int i = 0; i < MAX_ITEMS; i++) {      
      if (tx_queue[i] != NULL && acked_seq_nr == tx_queue[i]->seq_nr) {
        Serial.printf("Found element, removing\n");
        free(tx_queue[i]);
        tx_queue[i] = NULL;
        break;
      }
    }
  }
}

// splits the string into various sub - messages and processes them
// 
void process_received_message(const char *message)
{
  if(message == NULL || strlen(message) == 0) {
    return ;
  }

  both.printf("Rcv: %s\n", message);
  char updateable_message[strlen(message)];
  strcpy(updateable_message, message);

  char *saveptr1, *saveptr2;
  char *single_msg = strtok_r(updateable_message, ">", &saveptr1);

  while(single_msg != NULL) {
    char *message_type = strtok_r(single_msg, "|", &saveptr2);
    u_int8_t sequence_number = atoi(strtok_r(NULL, "|", &saveptr2));
    char *payload = strtok_r(NULL, "|", &saveptr2);
    
    both.printf("Received: %s(%d): %s\n", message_type, sequence_number, payload);
    if (strcmp(message_type, "ACK") == 0) {
      Serial.printf("not ACKING the message %s\n", message);
    } else {
      Serial.printf("ACKING the message %s\n", message);
      send_ack(sequence_number);
    }
    handle_received_message(message_type, sequence_number, payload);
    single_msg = strtok_r(NULL, ">", &saveptr1);
  }
}

// main communicator loop. processes the queue by sending out messages and reacts
// if the received flag was raised by interrupt.
void comm_loop()
{
  if(millis() > (last_tx + INTERVAL)) {
    payloaded_message* outgoing_messages[MAX_ITEMS];
    int index = 0;
    for (int i = 0; i < MAX_ITEMS; i++) {
      if (tx_queue[i] != NULL) {
        tx_queue[i]->last_sent = millis();
        outgoing_messages[index] = tx_queue[i];
        index++;
        if (strcmp(tx_queue[i]->command, "ACK") == 0) {
          tx_queue[i] = NULL;
        }
      }
    }

    if(index > 0) {
      Serial.printf("Sending %d messages", index);
      transmit_message(outgoing_messages, index);
    }
 
    for (int i = 0; i < index; i++) {
      if (strcmp(outgoing_messages[i]->command, "ACK") == 0) {
        Serial.println("Not waiting for ACK for ACK");
        free(tx_queue[i]);
      }
    }
    last_tx = millis();
  }

  if (received)
  {
    received = false;

    size_t packet_length = radio.getPacketLength();
    Serial.printf("received %d bytes\n", packet_length);
    
    if (packet_length == 0) {
      RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
      return;
    }

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

char incomingMessage[200];
char currentMessage[200];
int pos = 0;

void serial_parse_input(char *message)
{
  if (strlen(message) < 3) {
    return;
  }
  char* message_type = strtok(message, "|");
  char* payload = strtok(NULL, "|");
  payloaded_message* payload_message = (payloaded_message*)malloc(sizeof(payloaded_message));
  strcpy(payload_message->command, message_type);
  strcpy(payload_message->payload, payload);
  Serial.printf("Enqueuing message %s - %s\n", message_type, payload);  
  comm_enqueue_message(payload_message);
}

void serial_setup()
{
  memset(incomingMessage, 0, 200);
  memset(currentMessage, 0, 200);
  Serial.setTimeout(100);
}

void serial_loop()
{
  if (Serial.available())
  {
    int read = Serial.read(incomingMessage, 200);
    incomingMessage[read] = 0;
    Serial.print(incomingMessage);

    memcpy(currentMessage + pos, incomingMessage, read);
    pos += read;

    if (currentMessage[pos - 1] == '\n')
    {
      currentMessage[pos - 1] = 0;
      pos = 0;

      serial_parse_input(currentMessage);
    }
  }
}

//
//
// MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN
//
//

void setup()
{
  heltec_setup();
  serial_setup();
  comm_setup();
}

void loop()
{
  heltec_loop();
  comm_loop();
  serial_loop();
}
