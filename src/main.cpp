#define HELTEC_POWER_BUTTON // must be before "#include <heltec_unofficial.h>"
#include "heltec_unofficial.h"
#include "esp32-hal.h"
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
#include "proto/message.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

char WLAN_SSID_RES[50] = {};
char WLAN_PWD_RES[50] = {};


#define UDP_PORT 3333

#define MULTICAST_TTL 1
#define MULTICAST_IPV4_ADDR "232.10.11.12"

#define MAX_ITEMS 32
#define MAX_READ_BYTES 100

#define INTERVAL 5000
#define MAX_TIMEOUT 15000

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
#define FREQUENCY 866.3 // for Europe
// #define FREQUENCY           905.2       // for US

// LR bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
//#define BANDWIDTH 250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference.
//#define SPREADING_FACTOR 10

// This value can be set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW).
// Note that the maximum ERP (which is what your antenna maximally radiates) on the
// EU ISM band is 25 mW, and that transmissting without an antenna can damage your hardware.
//#define TRANSMIT_POWER 0

Proto_LoRa_Data* tx_queue[MAX_ITEMS];
uint64_t message_counter = 0;
unsigned long tx_time = 0;
unsigned long last_rx = 0;
unsigned long last_tx = 0;

boolean setup_done = false;
boolean rx_in_progress = false;
volatile boolean received = false;
String rxString;

char empty[2] = "";

Proto_Mcu_Data persistent_state;
Proto_Mcu_Data incoming_state; 

void socketserver_send_lora(Proto_LoRa_Data message);
void socketserver_send(Proto_Message message);
void handle_proto(uint8_t* message, size_t length);
void handle_command(char* message);
Proto_Update_Data create_status_update();
//
//
// COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM  COMM
//
//

void printBuffer(char* buffer, size_t start, size_t count) {
    for (int i = start; i < start + count; i++) {
        Serial.printf("%02X ", buffer[i]);
    }
}

bool do_not_ack_msg(Proto_LoRa_Data* msg) {
  return !msg->requires_ack;
}

void rx()
{
  received = true;
}

void queue_setup() {
  for (int i = 0; i < MAX_ITEMS; i ++) {
    tx_queue[i] = NULL;
  }

}

void comm_start(double bandwith, int16_t spreading_factor, int16_t power)
{
  Serial.println("Radio init");
  radio.begin();
  // Set the callback function for received packets
  radio.setDio1Action(rx);
  // Set radio parameters
  Serial.printf("[LR] Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  Serial.printf("[LR] Bandwidth: %.1f kHz\n", bandwith);
  RADIOLIB_OR_HALT(radio.setBandwidth(bandwith));
  Serial.printf("[LR] Spreading Factor: %i\n", spreading_factor);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(spreading_factor));
  Serial.printf("[LR] TX power: %i dBm\n", power);
  RADIOLIB_OR_HALT(radio.setOutputPower(power));
  // Start receiving
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  setup_done = true;
}

void free_message(Proto_LoRa_Data* data) {
  free(data);
}

bool queue_has_command(Proto_Command_Type command_type) {
  for (int i = 0; i < MAX_ITEMS; i++) {
    if(tx_queue[i] != NULL 
        && tx_queue[i]->has_command_data 
        && tx_queue[i]->command_data.type == command_type) {
          return true;
        }
  }
  return false;
}


void comm_enqueue_message(Proto_LoRa_Data *message)
{
  if (message->has_command_data) {
    if(queue_has_command(message->command_data.type)) {
      Serial.printf("[TX QUEUE] Command already enqueued, skipping: %d\n", message->command_data.type);    
      return;
    }
  }

  message->has_seq_nr = true;
  message->seq_nr = message_counter;
  int enq_pos = message_counter % MAX_ITEMS;
  Serial.printf("[TX QUEUE] Enqueing at pos: %d\n", enq_pos);
  tx_queue[enq_pos] = message;

  message_counter++;
  if (message_counter > INT16_MAX)
    message_counter = 0;
}

void transmit_message(Proto_LoRa_Data* message) {
  uint8_t buffer[128];
  size_t message_length;
  bool status;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  status = pb_encode(&stream, Proto_LoRa_Data_fields, message);
  radio.clearDio1Action();
  heltec_led(50);

  message->has_send_timestamp = true;
  message->send_timestamp = esp_timer_get_time() / 1000;
  tx_time = millis();
  Serial.printf("[LR OUT] sending %d byte\n", stream.bytes_written);
  radio.transmit(buffer, stream.bytes_written);
  tx_time = millis() - tx_time;
  heltec_led(0);
  if (_radiolib_status == RADIOLIB_ERR_NONE) {
    Serial.printf("[LR OUT] msg sent (%i ms)\n", (int)tx_time);
  }
  else {
    Serial.printf("[LR OUT] msg failed (%i)\n", _radiolib_status);
  }
  radio.setDio1Action(rx);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}

void send_ack(uint8_t seqNr)
{
  Serial.printf("[LR OUT] Sending ack for %d\n", seqNr);
  Proto_LoRa_Data* data = (Proto_LoRa_Data*) malloc(sizeof(Proto_LoRa_Data));
  *data = Proto_LoRa_Data_init_zero;
  data->ack_data.seq_nr = seqNr;
  data->ack_data.has_seq_nr = true;
  data->has_ack_data = true;
  data->requires_ack = false;
  data->has_requires_ack = true;
  comm_enqueue_message(data);
}

// handle a received message. right now this is just removing messages from the queue if they were ACKed
void handle_received_message(Proto_LoRa_Data incoming_data) {
  if(incoming_data.has_ack_data) {
    int acked_seq_nr = incoming_data.ack_data.seq_nr;
    Serial.printf("[LR IN] Got ACK Message for message %d\n", acked_seq_nr);
    for (int i = 0; i < MAX_ITEMS; i++) {      
      if (tx_queue[i] != NULL && acked_seq_nr == tx_queue[i]->seq_nr) {
        free_message(tx_queue[i]);
        tx_queue[i] = NULL;
        break;
      }
    }
    return;
  }

  Serial.printf("[LR IN] Received hasMcu: %d hasCommand %d with seqno %d, requires ack: %d\n", incoming_data.has_update_data, incoming_data.has_command_data, incoming_data.seq_nr, incoming_data.requires_ack);
  if(incoming_data.has_command_data) {
    Serial.printf("[LR IN] Command type: %d\n", incoming_data.command_data.type);
  }
  
  if(incoming_data.requires_ack) {
    Serial.printf("[LR IN] Acking message\n");
    send_ack(incoming_data.seq_nr);
  }
  
  if(WiFi.status() != WL_CONNECTED) {
    Serial.printf("Not forwarding message as WLAN is not avail\n");
    return; 
  }

  socketserver_send_lora(incoming_data);
}

// splits the string into various sub - messages and processes them
// 
void process_received_message(const uint8_t *data, size_t length)
{
  bool status;
  if(data == NULL || length == 0) {
    return ;
  }

  Proto_LoRa_Data incoming_message = Proto_LoRa_Data_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(data, length);
  status = pb_decode(&stream, Proto_LoRa_Data_fields, &incoming_message);
  if (status) {
    handle_received_message(incoming_message);
  } else {
    Serial.printf("Could not decode \n");
  }
}

void send_update(Proto_Update_Data update_data) {
  Proto_LoRa_Data lora_data = Proto_LoRa_Data_init_zero;
  lora_data.has_update_data = true;
  lora_data.update_data = update_data;
  lora_data.requires_ack = false;
  lora_data.has_requires_ack = true;
  transmit_message(&lora_data);
}

bool should_send_message(Proto_LoRa_Data* data) {
  if(!data->has_send_timestamp) {
    Serial.printf("[LR OUT] Never Sent, Sending\n");
    return true; 
  }
  int time_since_send = (esp_timer_get_time() / 1000) - data->send_timestamp;
  Serial.printf("[LR OUT] time to resend: %d\n", time_since_send);
  if(time_since_send > 4000) {
    Serial.printf("[LR OUT] resending.\n");
    return true;
  }
  return false;
}



// main communicator loop. processes the queue by sending out messages and reacts
// if the received flag was raised by interrupt.
void comm_loop()
{
  if(!setup_done) {
    return;
  }
   
  if(millis() > (last_tx + INTERVAL)) {
    bool message_sent = false;
    long start = millis();
    for (int i = 0; i < MAX_ITEMS; i++) {
      if (tx_queue[i] != NULL && should_send_message(tx_queue[i])) {
        Serial.printf("[LR OUT] Sending idx: %d with commandData: %d, updateData: %d, ackData: %d\n", i, tx_queue[i]->has_command_data, tx_queue[i]->has_update_data, tx_queue[i]->has_ack_data);
        transmit_message(tx_queue[i]);
        message_sent = true;
        if (do_not_ack_msg(tx_queue[i])) {
          Serial.printf("[LR OUT] Freeing message\n");
          free_message(tx_queue[i]);
          tx_queue[i] = NULL;
        }
      }
    }
    #if (PRIMARY)
      if(!message_sent) {
        Serial.printf("[LR OUT] Sending out of turn\n");
        send_update(create_status_update());
      }
    #endif

    last_tx = millis();
    Serial.printf("[LR OUT] run took: %d\n", last_tx - start);
  }

  if (received)
  {
    received = false;

    size_t packet_length = radio.getPacketLength();
    Serial.printf("[LR IN] received %d bytes\n", packet_length);
    
    if (packet_length == 0) {
      radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
      return;
    }

    uint8_t incoming_message[packet_length];
    radio.readData(incoming_message , packet_length);

    if (_radiolib_status == RADIOLIB_ERR_NONE)
    {
      Serial.printf("[LR IN]RX \n");
      Serial.printf("[LR IN]  RSSI: %.2f dBm\n", radio.getRSSI());
      Serial.printf("[LR IN]  SNR: %.2f dB\n", radio.getSNR());
      
      Proto_Message message = Proto_Message_init_default;
      message.has_lora_stats = true;
      Proto_Lora_Stats lora_stats = Proto_Lora_Stats_init_default;
      lora_stats.has_rssi = true;
      lora_stats.has_snr = true;
      lora_stats.snr = radio.getSNR();
      lora_stats.rssi = radio.getRSSI();
      message.lora_stats = lora_stats;

      socketserver_send(message);
      process_received_message(incoming_message, packet_length);
    }
    last_rx = millis();
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }
}

//
//
// WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN WLAN
//
//

void extract_credentials() {
        strcpy(WLAN_SSID_RES, CONFIG_SSID);
        strcpy(WLAN_PWD_RES, CONFIG_PWD);
        
        #if PRIMARY
            strcat(WLAN_SSID_RES, "prim");
            strcat(WLAN_PWD_RES, "prim");
        #else
            strcat(WLAN_SSID_RES, "sec");
            strcat(WLAN_PWD_RES, "sec");
        #endif
}

void wlan_setup() {
  Serial.print("[WIN] Starting");
  extract_credentials();
  Serial.printf("User: %s, Pass: %s", WLAN_SSID_RES, WLAN_PWD_RES);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID_RES, WLAN_PWD_RES);
  int retryCnt = 0;
  
  do {
    delay(2000);
    Serial.print(".");
  } while (WiFi.status() != WL_CONNECTED || ++retryCnt == 10);

  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  display.printf("[WIN] Wlan connected\nIP: %s\n", WiFi.localIP().toString().c_str());
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
        Serial.printf("[WIN] Failed to get IP address info. Error 0x%x\n", err);
        goto err;
    }
    inet_addr_from_ip4addr(&iaddr, &ip_info.ip);

    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        Serial.printf("[WIN] Configured IPV4 multicast address '%s' is invalid.\n", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        goto err;
    }
    Serial.printf("[WIN] Configured IPV4 Multicast address %s\n", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        Serial.printf("[WIN] Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.\n", MULTICAST_IPV4_ADDR);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            Serial.printf("[WIN] Failed to set IP_MULTICAST_IF. Error %d\n", errno);
            goto err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        Serial.printf("[WIN] Failed to set IP_ADD_MEMBERSHIP. Error %d\n", errno);
        goto err;
    }

 err:
    return err;
}
static int create_multicast_ipv4_socket(esp_netif_t *interface)
{
  struct sockaddr_in saddr = {0};
  int sock = -1;
  int err = 0;

  sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0)
  {
    Serial.printf("[WIN] Failed to create socket. Error %d\n", errno);
    return -1;
  }

  // Bind the socket to any address
  saddr.sin_family = PF_INET;
  saddr.sin_port = htons(UDP_PORT);
  saddr.sin_addr.s_addr = htonl(INADDR_ANY);
  err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));

  uint8_t ttl = MULTICAST_TTL;
  if (err < 0)
  {
    Serial.printf("[WIN] Failed to bind socket. Error %d\n", errno);
    goto err;
  }

  // Assign multicast TTL (set separately from normal interface TTL)
  setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
  if (err < 0)
  {
    Serial.printf("[WIN] Failed to set IP_MULTICAST_TTL. Error %d\n", errno);
    goto err;
  }

  // this is also a listening socket, so add it to the multicast
  // group for listening...
  err = socket_add_ipv4_multicast_group(interface, sock, true);
  if (err < 0)
  {
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
  esp_netif_t *interface = (esp_netif_t *)pvParameters;
  while (1)
  {

    sock = create_multicast_ipv4_socket(interface);
    if (sock < 0)
    {
      Serial.printf("[WIN] Failed to create IPv4 multicast socket\n");
    }

    if (sock < 0)
    {
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
    while (err > 0)
    {
      struct timeval tv = {
          .tv_sec = 2,
          .tv_usec = 0,
      };
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(sock, &rfds);

      int s = select(sock + 1, &rfds, NULL, NULL, &tv);
      if (s < 0)
      {
        Serial.printf("[WIN] Select failed: errno %d\n", errno);
        err = -1;
        break;
      }
      else if (s > 0)
      {
        if (FD_ISSET(sock, &rfds))
        {
          // Incoming datagram received
          uint8_t recvbuf[500];
          char raddr_name[32] = {0};
          struct sockaddr_storage raddr; // Large enough for both IPv4 or IPv6
          socklen_t socklen = sizeof(raddr);
          int len = recvfrom(sock, recvbuf, sizeof(recvbuf) - 1, 0,
                             (struct sockaddr *)&raddr, &socklen);
          if (len < 0)
          {
            Serial.printf("[WIN] multicast recvfrom failed: errno %d\n", errno);
            err = -1;
            break;
          }

          if (raddr.ss_family == PF_INET)
          {
            inet_ntoa_r(((struct sockaddr_in *)&raddr)->sin_addr,
                        raddr_name, sizeof(raddr_name) - 1);
          }
          //Serial.printf("[WIN] Rcv %dB from %s: \n", len, raddr_name);
          handle_proto(recvbuf, len);
        }
      }
    }
  }

  Serial.printf("Shutting down socket and restarting...");
  shutdown(sock, 0);
  close(sock);
}

void handle_lora_config(Proto_Lora_Config lora_config) {
  if (persistent_state.lora_config.bandwidth != lora_config.bandwidth ||
      persistent_state.lora_config.spreading_factor != lora_config.spreading_factor || 
      persistent_state.lora_config.output_power != lora_config.output_power) {
    if (setup_done) {
      Serial.println("Config changed but setup was done, restarting");
      esp_restart();
    } else {
      Serial.println("Starting LoRa");
      comm_start(lora_config.bandwidth, lora_config.spreading_factor, lora_config.output_power);
    }
  }
}

void handle_proto(uint8_t* message, size_t length) {
  Proto_Message decoded_message = Proto_Message_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(message, length);
  bool status = pb_decode(&stream, Proto_Message_fields, &decoded_message);
  if(!status) {
    Serial.printf("Could not decode message");
    return;
  }
  if(decoded_message.has_mcu_data) {
    handle_lora_config(decoded_message.mcu_data.lora_config);
    persistent_state = decoded_message.mcu_data;
  } else if (decoded_message.has_command_data) {
    Serial.printf("Command Data Received from MCU, type %d\n", decoded_message.command_data.type);
    Proto_LoRa_Data* lora_data = (Proto_LoRa_Data*) malloc(sizeof(Proto_LoRa_Data));
    *lora_data = Proto_LoRa_Data_init_zero;
    lora_data->has_requires_ack = true;
    lora_data->requires_ack = true;
    lora_data->has_command_data = true;
    lora_data->command_data = decoded_message.command_data;
    comm_enqueue_message(lora_data);
  }
}

Proto_Update_Data create_status_update() {
  Proto_Update_Data data = Proto_Update_Data_init_zero;
  data.has_gas_sensor = persistent_state.has_gas;
  data.has_oil_sensor = persistent_state.has_oil;
  data.has_water_sensor = persistent_state.has_water;
  data.has_stint_data = persistent_state.has_stint;
  data.has_lap_data = persistent_state.has_lap_data;

  data.has_gps_data = persistent_state.has_gps;
  data.gps_data = persistent_state.gps;

  data.gas_sensor = persistent_state.gas;
  data.water_sensor = persistent_state.water;
  data.oil_sensor = persistent_state.oil;
  data.stint_data = persistent_state.stint;
  data.lap_data = persistent_state.lap_data;

  return data;
}

void socketserver_send_lora(Proto_LoRa_Data data) {
  Proto_Message proto_message = Proto_Message_init_zero;
  proto_message.has_lora_data = true;
  proto_message.lora_data = data;
  socketserver_send(proto_message);
}

void socketserver_send(Proto_Message data) {
  static int send_count;
  uint8_t buffer[500];
  char addrbuf[32] = {0};

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  bool status = pb_encode(&stream, Proto_Message_fields, &data);

  if(!status) {
    Serial.println("Could not encode Proto");
  } 

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
    Serial.printf("[WOUT] getaddrinfo() failed for IPV4 destination address. error: %d\n", err);
    return;
  }
  if (res == 0)
  {
    Serial.printf("[WOUT] getaddrinfo() did not return any addresses\n");
    return;
  }

  ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
  inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf) - 1);
  err = sendto(sock, buffer, stream.bytes_written, 0, res->ai_addr, res->ai_addrlen);
  freeaddrinfo(res);
  if (err < 0)
  {
    Serial.printf("[WOUT] IPV4 sendto failed. errno: %d\n", errno);
    return;
  }
}

void socketserver_start() {
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  xTaskCreate(&listen, "listen_task", 8192, netif, 5, NULL);
}


//
//
// MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN  MAIN
//
//

void print_status() {
    Serial.printf("\n\n----------------------------------------------------------\n");
    Serial.printf("|--------------------------------------------------------|\n");
    #if PRIMARY
        Serial.printf("|----PRIMARY---PRIMARY---PRIMARY---PRIMARY---PRIMARY-----|\n");
    #else
        Serial.printf("|-----SECONDARY---SECONDARY---SECONDARY---SECONDARTY-----|\n");
    #endif
    Serial.printf("|--------------------------------------------------------|\n");
    Serial.printf("----------------------------------------------------------\n");
}

SET_LOOP_TASK_STACK_SIZE(1024 * 16);

void setup()
{
  heltec_setup();
  queue_setup();
  wlan_setup();
  socketserver_start();
  print_status();

}

void loop()
{
  heltec_loop();
  comm_loop();
}
