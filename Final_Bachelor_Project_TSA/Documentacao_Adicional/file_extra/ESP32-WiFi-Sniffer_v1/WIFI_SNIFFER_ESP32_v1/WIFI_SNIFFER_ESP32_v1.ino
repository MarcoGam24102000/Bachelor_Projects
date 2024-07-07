#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define LED_GPIO_PIN                     5
#define WIFI_CHANNEL_SWITCH_INTERVAL  (500)
#define WIFI_CHANNEL_MAX               (13)

uint8_t level = 0, channel = 1;

int higherRSSI = -200;

int counter_RSSI = 0;

static wifi_country_t wifi_country = {.cc="CN", .schan = 1, .nchan = 13}; //Most recent esp32 library struct

typedef struct {
  unsigned frame_ctrl:16;
  unsigned duration_id:16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl:16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}

void wifi_sniffer_init(void)
{
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country) ); /* set country for channel range [1, 13] */
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );
  ESP_ERROR_CHECK( esp_wifi_start() );
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel)
{
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char * wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
  switch(type) {
  case WIFI_PKT_MGMT: return "MGMT";
  case WIFI_PKT_DATA: return "DATA";
  default:  
  case WIFI_PKT_MISC: return "MISC";
  }
}

void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)
{  
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

//  printf("PACKET TYPE=%s, CHAN=%02d, RSSI=%02d,"
//    " ADDR1=%02x:%02x:%02x:%02x:%02x:%02x,"
//    " ADDR2=%02x:%02x:%02x:%02x:%02x:%02x,"
//    " ADDR3=%02x:%02x:%02x:%02x:%02x:%02x\n",
//    wifi_sniffer_packet_type2str(type),
//    ppkt->rx_ctrl.channel,
//    ppkt->rx_ctrl.rssi,
//    /* ADDR1 */
//    hdr->addr1[0],hdr->addr1[1],hdr->addr1[2],
//    hdr->addr1[3],hdr->addr1[4],hdr->addr1[5],
//    /* ADDR2 */
//    hdr->addr2[0],hdr->addr2[1],hdr->addr2[2],
//    hdr->addr2[3],hdr->addr2[4],hdr->addr2[5],
//    /* ADDR3 */
//    hdr->addr3[0],hdr->addr3[1],hdr->addr3[2],
//    hdr->addr3[3],hdr->addr3[4],hdr->addr3[5]
//  ); 
  
  //---------------------------------------------MG------------//  
  
  char MAC_addr[11];
  // MAC_addr[0] = hdr->addr3[0];
  Serial.println("-----------");
  //Serial.print(char(int(MAC_addr[0])));

  for (int cnt = 0; cnt < sizeof(hdr->addr3); cnt++)
  {
    // convert byte to its ascii representation
    sprintf(&MAC_addr[cnt * 2], "%02X", hdr->addr3[cnt]);
  }
  Serial.println("\n");
  for(int i=0; i<sizeof(MAC_addr); i++){
    Serial.print(MAC_addr[i]);
  }
  
//  MAC_addr[1] = ':';
//  MAC_addr[2] = hdr->addr3[1];
//  MAC_addr[3] = ':';
//  MAC_addr[4] = hdr->addr3[2];
//  MAC_addr[5] = ':';
//  MAC_addr[6] = hdr->addr3[3];
//  MAC_addr[7] = ':';
//  MAC_addr[8] = hdr->addr3[4];
//  MAC_addr[9] = ':';
//  MAC_addr[10] = hdr->addr3[5];
  

  if(counter_RSSI < 20){
    if(ppkt->rx_ctrl.rssi >= higherRSSI){
      higherRSSI = ppkt->rx_ctrl.rssi;

      if(counter_RSSI == 19){
        //uint8_t broadcastAddress[] = {MAC_addr[0],MAC_addr[1],MAC_addr[2],MAC_addr[3],MAC_addr[4],MAC_addr[5],MAC_addr[6],MAC_addr[7],MAC_addr[8],MAC_addr[9],MAC_addr[10]}; 
        Serial.printf("\nBroadcast Address catched: \t %u", MAC_addr); 
      }
    }
    counter_RSSI++;
  }else{
    counter_RSSI = 0;
  }


  

  
  //Serial.print(broadcastAddress);

}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 5 as an output.
  Serial.begin(115200);
  delay(10);
  wifi_sniffer_init();
  pinMode(LED_GPIO_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  //Serial.print("inside loop");
  delay(1000); // wait for a second
  
  if (digitalRead(LED_GPIO_PIN) == LOW)
    digitalWrite(LED_GPIO_PIN, HIGH);
  else
    digitalWrite(LED_GPIO_PIN, LOW);
  vTaskDelay(WIFI_CHANNEL_SWITCH_INTERVAL / portTICK_PERIOD_MS);
  wifi_sniffer_set_channel(channel);
  channel = (channel % WIFI_CHANNEL_MAX) + 1;
}
