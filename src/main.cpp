#define HELTEC_POWER_BUTTON // must be before "#include <heltec_unofficial.h>"
#include "heltec_unofficial.h"
#include "esp32-hal.h"
#include "messages.h"
#include <stdint.h>
#include <WiFi.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define SSID        "funkbox-car-main"
#define PASSWORD    "funkbox-car-main"

#define UDP_PORT 3333

#define MULTICAST_TTL 1
#define MULTICAST_IPV4_ADDR "232.10.11.12"

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

void socketserver_send(char *data);

//
//
// COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM
//
//

void freePayloadedMessage(payloaded_message* payloaded_message) {
  free(payloaded_message->command);
  free(payloaded_message->payload);
  free(payloaded_message);
}

void printBuffer(char* buffer, size_t start, size_t count) {
    for (int i = start; i < start + count; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
}

void rx()
{
  received = true;
}

void comm_setup()
{
  //memset(tx_queue, 0, sizeof(payloaded_message *) * MAX_ITEMS);
  for (int i = 0; i < MAX_ITEMS; i ++) {
    tx_queue[i] = NULL;
  }

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

int getMessageLen(payloaded_message* message) {
  return strlen(message->command) + strlen(message->payload) + 10 + 4;
}

void transmit_message(payloaded_message *messages[], int size) {
  int len = 0;
  for (int i = 0; i < size; i++) {
    len += getMessageLen(messages[i]);
  }

  char* combined_output = (char*) malloc(sizeof(char) * len+1);
  memset(combined_output, 0, len);

  for (int i = 0; i < size; i++) {
    char output[getMessageLen(messages[i])];
    sprintf(output, "%s|%d|%s>", messages[i]->command, messages[i]->seq_nr, messages[i]->payload);
    strcat(combined_output, output);
  }

  printBuffer(combined_output, 0, strlen(combined_output)+1);

  radio.clearDio1Action();
  heltec_led(50);

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
  free(combined_output);
  radio.setDio1Action(rx);
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

void send_ack(uint8_t seqNr)
{
  Serial.printf("Sending ack for %d\n", seqNr);
  payloaded_message *message = (payloaded_message *)malloc(sizeof(payloaded_message));
  message->command = (char *) malloc(sizeof(char) * 4);
  message->payload = (char *) malloc(sizeof(10));
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
        freePayloadedMessage(tx_queue[i]);
        tx_queue[i] = NULL;
        break;
      }
    }
    return;
  }
  char* complete_message = (char*) malloc((strlen(message_type) + strlen(payload) + 3)*sizeof(char));
  sprintf(complete_message, "%s|%s", message_type, payload);
  socketserver_send(complete_message);
}

// splits the string into various sub - messages and processes them
// 
void process_received_message(const char *message)
{
  if(message == NULL || strlen(message) == 0) {
    return ;
  }


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
        freePayloadedMessage(outgoing_messages[i]);
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

    char* incoming_message = (char*)malloc(sizeof(char) * packet_length + 1);

    radio.readData((uint8_t *)incoming_message , packet_length);
    incoming_message[packet_length] = '\0';

    if (_radiolib_status == RADIOLIB_ERR_NONE)
    {
      both.printf("RX [%s]\n", incoming_message);
      both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
      both.printf("  SNR: %.2f dB\n", radio.getSNR());
      process_received_message(incoming_message);
    }
    last_rx = millis();
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    free(incoming_message);
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

  if(payload[strlen(payload) - 1] == 0x0D) {
    payload[strlen(payload) - 1] = '\0';
  }

  payloaded_message* payload_message = (payloaded_message*)malloc(sizeof(payloaded_message));
  payload_message->command = (char *) malloc(sizeof(char) * strlen(message_type) + 1);
  payload_message->payload = (char *) malloc(sizeof(char) * strlen(payload) + 1);
    
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
// WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN
//
//

void wlan_setup() {
  both.print("Wifi ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  do {
    delay(2000);
    both.print(".");
  } while (WiFi.status() != WL_CONNECTED);

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  display.printf("Wlan connected\nIP: %s\n", WiFi.localIP().toString().c_str());
}


int sock;

/* Add a socket to the IPV4 multicast group */
static int socket_add_ipv4_multicast_group(esp_netif_t* interface, int sock, bool assign_source_if)
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface

    esp_netif_ip_info_t ip_info = { 0 };
    err = esp_netif_get_ip_info(interface, &ip_info);
    if (err != ESP_OK) {
        both.printf("Failed to get IP address info. Error 0x%x\n", err);
        goto err;
    }
    inet_addr_from_ip4addr(&iaddr, &ip_info.ip);

    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        both.printf("Configured IPV4 multicast address '%s' is invalid.\n", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        goto err;
    }
    both.printf("Configured IPV4 Multicast address %s\n", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        both.printf("Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.\n", MULTICAST_IPV4_ADDR);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            both.printf("Failed to set IP_MULTICAST_IF. Error %d\n", errno);
            goto err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        both.printf("Failed to set IP_ADD_MEMBERSHIP. Error %d\n", errno);
        goto err;
    }

 err:
    return err;
}
static int create_multicast_ipv4_socket(esp_netif_t* interface)
{
    struct sockaddr_in saddr = { 0 };
    int sock = -1;
    int err = 0;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        both.printf("Failed to create socket. Error %d\n", errno);
        return -1;
    }

    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));

    uint8_t ttl = MULTICAST_TTL;
    if (err < 0) {
        both.printf("Failed to bind socket. Error %d\n", errno);
        goto err;
    }


    // Assign multicast TTL (set separately from normal interface TTL)
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        both.printf("Failed to set IP_MULTICAST_TTL. Error %d\n", errno);
        goto err;
    }

    // this is also a listening socket, so add it to the multicast
    // group for listening...
    err = socket_add_ipv4_multicast_group(interface, sock, true);
    if (err < 0) {
        goto err;
    }

    // All set, socket is configured for sending and receiving
    return sock;

err:
    close(sock);
    return -1;
}

static void listen(void *pvParameters)
{
    esp_netif_t* interface = (esp_netif_t*) pvParameters;
    while (1) {


        sock = create_multicast_ipv4_socket(interface);
        if (sock < 0) {
            both.printf("Failed to create IPv4 multicast socket\n");
        }

        if (sock < 0) {
            // Nothing to do!
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }

        // set destination multicast addresses for sending from these sockets
        struct sockaddr_in sdestv4 = {
            .sin_family = PF_INET,
            .sin_port = htons(UDP_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);

        // Loop waiting for UDP received, and sending UDP packets if we don't
        // see any.
        int err = 1;
        while (err > 0) {
            struct timeval tv = {
                .tv_sec = 2,
                .tv_usec = 0,
            };
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);

            int s = select(sock + 1, &rfds, NULL, NULL, &tv);
            if (s < 0) {
                both.printf("Select failed: errno %d\n", errno);
                err = -1;
                break;
            } else if (s > 0) {
                if (FD_ISSET(sock, &rfds)) {
                    // Incoming datagram received
                    char recvbuf[200];
                    char raddr_name[32] = { 0 };
                    char empty[2] = "";
                    struct sockaddr_storage raddr; // Large enough for both IPv4 or IPv6
                    socklen_t socklen = sizeof(raddr);
                    int len = recvfrom(sock, recvbuf, sizeof(recvbuf)-1, 0,
                                       (struct sockaddr *)&raddr, &socklen);
                    if (len < 0) {
                        both.printf("multicast recvfrom failed: errno %d\n", errno);
                        err = -1;
                        break;
                    }

                    if (raddr.ss_family == PF_INET) {
                        inet_ntoa_r(((struct sockaddr_in *)&raddr)->sin_addr,
                                    raddr_name, sizeof(raddr_name)-1);
                    }
                    both.printf("received %d bytes from %s: ", len, raddr_name);

                    recvbuf[len] = 0; // Null-terminate whatever we received and treat like a string...
                    both.printf("%s\n", recvbuf);

                    char* message_type = strtok(recvbuf, "|");
                    char* payload = strtok(NULL, "|");
                    if (payload == NULL) {
                      payload = &empty[0];
                    }

                    payloaded_message* payload_message = (payloaded_message*)malloc(sizeof(payloaded_message));
                    payload_message->command = (char *) malloc(sizeof(char) * strlen(message_type) + 1);
                    payload_message->payload = (char *) malloc(sizeof(char) * strlen(payload) + 1);

                    strcpy(payload_message->command, message_type);
                    strcpy(payload_message->payload, payload);
                    Serial.printf("Enqueuing message %s - %s\n", message_type, payload);  
                    comm_enqueue_message(payload_message);
                }
            }
        }

        both.printf("Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }

}

void socketserver_send(char *data) {
  static int send_count;
  
  char addrbuf[32] = {0};
  int len = strlen(data);

  struct addrinfo hints = {
      .ai_flags = AI_PASSIVE,
      .ai_socktype = SOCK_DGRAM,
  };
  struct addrinfo *res;

  hints.ai_family = AF_INET; // For an IPv4 socket

  int err = getaddrinfo(MULTICAST_IPV4_ADDR,
                        NULL,
                        &hints,
                        &res);
  if (err < 0)
  {
    both.printf("getaddrinfo() failed for IPV4 destination address. error: %d\n", err);
    return;
  }
  if (res == 0)
  {
    both.printf("getaddrinfo() did not return any addresses\n");
    return;
  }

  ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
  inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf) - 1);
  both.printf("Sending to IPV4 multicast address %s:%d...\n", addrbuf, UDP_PORT);
  err = sendto(sock, data, len, 0, res->ai_addr, res->ai_addrlen);
  freeaddrinfo(res);
  if (err < 0)
  {
    both.printf("IPV4 sendto failed. errno: %d\n", errno);
    return;
  }
}

void socketserver_start() {
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  xTaskCreate(&listen, "listen_task", 4096, netif, 5, NULL);
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
  wlan_setup();
  socketserver_start();
}

void loop()
{
  heltec_loop();
  comm_loop();
  serial_loop();
}
