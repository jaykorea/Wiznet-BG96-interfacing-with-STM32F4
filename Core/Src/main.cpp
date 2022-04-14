/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ArduinoWrapper.h"
#include "UartRingbuffer.h"
#include "at_cmd_parser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
ATCmdParser m_parser = ATCmdParser();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RESP_OK        "OK\r\n"
#define RET_OK         1
#define RET_NOK        -1
#define DEBUG_ENABLE   1
#define DEBUG_DISABLE  0
#define ON     1
#define OFF    0

#define PWR_PIN 32
#define RST_PIN 33

#define MAX_BUF_SIZE        1024

#define BG96_APN_PROTOCOL_IPv4     1
#define BG96_APN_PROTOCOL_IPv6     2
#define BG96_DEFAULT_TIMEOUT       1000
#define BG96_WAIT_TIMEOUT    3000
#define BG96_CONNECT_TIMEOUT       15000
#define BG96_SEND_TIMEOUT    500
#define BG96_RECV_TIMEOUT    500

#define BG96_APN_PROTOCOL    BG96_APN_PROTOCOL_IPv6
#define WM_N400MSE_DEFAULT_BAUD_RATE       115200
#define BG96_PARSER_DELIMITER      "\r\n"

#define CATM1_APN_SKT  "lte-internet.sktelecom.com"

#define CATM1_DEVICE_NAME_BG96     "BG96"
#define DEVNAME         CATM1_DEVICE_NAME_BG96

#define LOGDEBUG(x, ...)     if(CATM1_DEVICE_DEBUG == DEBUG_ENABLE) { printf("\r\n[%s] ", DEVNAME);  printf((x), ##__VA_ARGS__); }
#define MYPRINTF(x, ...)     {printf("\r\n[MAIN] ");  printf((x), ##__VA_ARGS__);}

// Sensors
#define MBED_CONF_IOTSHIELD_SENSOR_CDS     A0
#define MBED_CONF_IOTSHIELD_SENSOR_TEMP    A1

// Debug message settings
#define BG96_PARSER_DEBUG    DEBUG_DISABLE
#define CATM1_DEVICE_DEBUG         DEBUG_ENABLE

#define REQUESTED_PERIODIC_TAU    "10100101"
#define REQUESTED_ACTIVE_TIME     "00100100"

/* MQTT */
#define MQTT_EOF        0x1A
#define MQTT_QOS0       0
#define MQTT_QOS1       1
#define MQTT_QOS2       2
#define MQTT_RETAIN     0

/* SSL/TLS */
// Ciphersuites
#define BG96_TLS_RSA_WITH_AES_256_CBC_SHA           "0x0035"
#define BG96_TLS_RSA_WITH_AES_128_CBC_SHA           "0x002F"
#define BG96_TLS_RSA_WITH_RC4_128_SHA       "0x0005"
#define BG96_TLS_RSA_WITH_RC4_128_MD5       "0x0004"
#define BG96_TLS_RSA_WITH_3DES_EDE_CBC_SHA          "0x000A"
#define BG96_TLS_RSA_WITH_AES_256_CBC_SHA256        "0x003D"
#define BG96_TLS_ECDHE_RSA_WITH_RC4_128_SHA         "0xC011"
#define BG96_TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA    "0xC012"
#define BG96_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA     "0xC013"
#define BG96_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA     "0xC014"
#define BG96_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256  "0xC027"
#define BG96_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384  "0xC028"
#define BG96_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256  "0xC02F"
#define BG96_TLS_SUPPORT_ALL    "0xFFFF"

// SSL/TLS version
#define BG96_TLS_VERSION_SSL30  0   // SSL3.0
#define BG96_TLS_VERSION_TLS10  1   // TLS1.0
#define BG96_TLS_VERSION_TLS11  2   // TLS1.1
#define BG96_TLS_VERSION_TLS12  3   // TLS1.2
#define BG96_TLS_VERSION_ALL    4

/* Debug message settings */
#define BG96_PARSER_DEBUG           DEBUG_DISABLE
#define CATM1_DEVICE_DEBUG          DEBUG_ENABLE


/* MQTT Sample */
// MQTT connection state
enum {
  MQTT_STATE_OPEN = 0,
  MQTT_STATE_CONNECT,
  MQTT_STATE_CONNECTED,
  MQTT_STATE_DISCON
};


/* BG96 Config for connect to AWS IoT */
#define AWS_IOT_BG96_SSLTLS_VERSION      4           // 4: All
#define AWS_IOT_BG96_SSLTLS_SECLEVEL     2           // 2: Manage server and client authentication if requested by the remote server
#define AWS_IOT_BG96_SSLTLS_IGNORELOCALTIME      1           // 1: Ignore validity check for certification
#define AWS_IOT_BG96_SSLTLS_CIPHERSUITE          BG96_TLS_RSA_WITH_AES_256_CBC_SHA

        /* AWS IoT MQTT Client */
    char aws_iot_sub_topic[128] = {0, };
    char aws_iot_pub_topic[128] = {0, };
    char aws_iot_sub_topic1[128] = {0, };
    char aws_iot_pub_topic1[128] = {0, };
    char aws_iot_sub_topic2[128] = {0, };
    char aws_iot_pub_topic2[128] = {0, };
    char buf_mqtt_topic[128] = {0, };
    char buf_mqtt_topic1[128] = {0, };
    char buf_mqtt_topic2[128] = {0, };
    char buf_mqtt_recv[AWS_IOT_MQTT_RX_BUF_LEN] = {0, };
    char buf_mqtt_send[AWS_IOT_MQTT_TX_BUF_LEN] = {0, };
    char buf_mqtt_recv1[AWS_IOT_MQTT_RX_BUF_LEN] = {0, };
    char buf_mqtt_send1[AWS_IOT_MQTT_TX_BUF_LEN] = {0, };
    char buf_mqtt_recv2[AWS_IOT_MQTT_RX_BUF_LEN] = {0, };
    char buf_mqtt_send2[AWS_IOT_MQTT_TX_BUF_LEN] = {0, };
    int mqtt_len = 0;
    int mqtt_len1 = 0;
    int mqtt_len2 = 0;
    int mqtt_msgid = 0;
    int mqtt_msgid1 = 0;
    int mqtt_masgid2 = 0;

unsigned long getLocationTime = 0;

char dateBuf[30];
static char last_dateBuf[30];
char utcBuf[30];
static char last_utcBuf[30];
char latBuf[50];
static char last_latBuf[30] = "37.40002";
char lonBuf[50];
static char last_lonBuf[30] = "126.94254";
char timestampBuf[20] = "12345678";

int rstCheck;
typedef struct gps_data_t {
  float utc;      // hhmmss.sss
  float lat;      // latitude. (-)dd.ddddd
  float lon;      // longitude. (-)dd.ddddd
  float hdop;     // Horizontal precision: 0.5-99.9
  float altitude; // altitude of antenna from sea level (meters)
  int fix;        // GNSS position mode 2=2D, 3=3D
  float cog;      // Course Over Ground ddd.mm
  float spkm;     // Speed over ground (Km/h) xxxx.x
  float spkn;     // Speed over ground (knots) xxxx.x
  char date[7];   // data: ddmmyy
  int nsat;       // number of satellites 0-12
} gps_data;

gps_data gps_info;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //UART에 수신인터럽트 일어나면 이놈이 호출됨
//{
// if (huart->Instance == USART1) //current UART
//  {
//	 m_parser.push(m_parser.rxData);
//	 m_parser.rxFlag = true;
//  }
// 	 HAL_UART_Receive_IT(&huart1, &m_parser.rxData, 1); //다음 수신인터럽트를 준비
// }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//-----------------------------------------------Cat.M1.Device Function-------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

//void serialPcInit(void)
//{
//  Serial.begin(115200);
//}
//
//void serialDeviceInit()
//{
//  Serial2.begin(WM_N400MSE_DEFAULT_BAUD_RATE,SERIAL_8N1,16,17);
//}

int8_t setEchoStatus_BG96(bool onoff)
{
  if ( onoff == true )
  {
    if ( !(m_parser.send("ATE1") && m_parser.recv(RESP_OK)) ) {
      LOGDEBUG("Echo On: Failed\r\n");
      return RET_NOK;
    }
    else
    {
      LOGDEBUG("Echo On: Success\r\n");
      return RET_OK;
    }

  }
  else if ( onoff == false )
  {
    if ( !(m_parser.send("ATE0") && m_parser.recv(RESP_OK)) ) {
      LOGDEBUG("Echo Off: Failed\r\n");
      return RET_NOK;
    }
    else
    {
      LOGDEBUG("Echo Off: Success\r\n");
      return RET_OK;
    }
  }
}



int8_t getUsimStatus_BG96(void)
{
  char usim_stat[10], detail[10];
  char buf[40];

  if ( m_parser.send("AT+CPIN?") &&
       m_parser.recv("+CPIN: READY\n") &&
       m_parser.recv(RESP_OK) ) {
    LOGDEBUG("USIM Status: READY\r\n");
    return RET_OK;
  }

  else if ( m_parser.send("AT+CPIN?") &&
    m_parser.recv("+CPIN: %[^,],%[^\n]\n", usim_stat, detail) &&
    m_parser.recv(RESP_OK) ) {
    sprintf((char *)buf, "USIM Satatus: %s, %s", usim_stat, detail);
    LOGDEBUG(buf);
    return RET_NOK;
  }
}

int8_t getNetworkStatus_BG96(void)
{
  char mode[40], stat[40];
  char buf[40];

  if ( m_parser.send("AT+CEREG?") &&
       m_parser.recv("+CEREG: %[^,],%[^\n]\n", mode, stat) &&
       m_parser.recv(RESP_OK) ) {

    if ( (atoi(mode) == 0) && (atoi(stat) == 1) ) {
      LOGDEBUG("Network Status: Attach\r\n");
      return RET_OK;
    }

    else if (( atoi(stat) != 1 )) {
      sprintf((char *)buf, "Network Status: %d, %d", atoi(mode), atoi(stat));
      LOGDEBUG(buf);
      return RET_NOK;
    }
  }
  return RET_NOK;
}

void serialAtParserInit()
{
  m_parser.set_timeout(1000);
  m_parser.set_delimiter("\r");
}
void catm1DeviceInit()
{
//  serialDeviceInit();
  serialAtParserInit();
}

// ----------------------------------------------------------------
// Functions: Cat.M1 Status
// ----------------------------------------------------------------

int8_t waitCatM1Ready()
{
  while (1)
  {
    if (m_parser.recv("RDY")) {
      MYPRINTF("BG96 ready\r\n");
      return RET_OK;
    }
    else if (m_parser.send("AT") && m_parser.recv(RESP_OK))
    {
      MYPRINTF("BG96 already available\r\n");
      return RET_OK;
    }
  }
  return RET_NOK;
}


bool initStatus()
{

  if ( setEchoStatus_BG96(false) != RET_OK )
  {
    return false;
  }

  if ( getUsimStatus_BG96() != RET_OK )
  {
    return false;
  }

  if ( getNetworkStatus_BG96() != RET_OK )
  {
    return false;
  }

  return true;
}



int8_t checknSetApn_BG96(const char * apn) // Configure Parameters of a TCP/IP Context
{
  char resp_str[100];
  char buf[25];
  char buf1[25];

  uint16_t i = 0;
  char * search_pt;

  memset(resp_str, 0, sizeof(resp_str));

  LOGDEBUG("Checking APN...\r\n");


  m_parser.send("AT+QICSGP=1");
  while (1)
  {
    m_parser.read(&resp_str[i++], 1);
    search_pt = strstr(resp_str, "OK\r\n");
    if (search_pt != 0)
    {
      break;
    }
  }

  search_pt = strstr(resp_str, apn);
  if (search_pt == 0)
  {
    sprintf((char *)buf, "Mismatched APN: %s\r\n", resp_str);
    sprintf((char *)buf1, "Storing APN %s...\r\n", apn);
    LOGDEBUG(buf);
    LOGDEBUG(buf1);

    if (!(m_parser.send("AT+QICSGP=1,%d,\"%s\",\"\",\"\",0", BG96_APN_PROTOCOL, apn) && m_parser.recv("OK")))
    {
      return RET_NOK; // failed
    }
  }
  LOGDEBUG("APN Check Done\r\n");

  return RET_OK;
}





void getIMEIInfo_BG96(void)
{
  char m_imei[30];
  char buf[25];

  if ( (m_parser.send("AT*MINFO")
        && m_parser.recv("*MINFO:%*[^,],%*[^,],%[^,],%*[^\n]\n", m_imei)
        && m_parser.recv(RESP_OK)) ) {
    sprintf((char *)buf, "Module IME: %s\r\n", m_imei);
    LOGDEBUG(buf);
  }
}

int8_t getFirmwareVersion_BG96(char * version)
{
  int8_t ret = RET_NOK;

  if (m_parser.send("AT+QGMR") && m_parser.recv("%s\n", version) && m_parser.recv("OK"))
  {
    ret = RET_OK;
  }
  return ret;
}

int8_t getImeiNumber_BG96(char * imei)
{
  int8_t ret = RET_NOK;

  if (m_parser.send("AT+CGSN") && m_parser.recv("%s\n", imei) && m_parser.recv("OK"))
  {
    ret = RET_OK;
  }
  return ret;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 DNS
// ----------------------------------------------------------------

int8_t getIpAddressByName_BG96(const char * name, char * ipstr)
{
  char buf2[50];
  bool ok;
  int  err, ipcount, dnsttl;

  int8_t ret = RET_NOK;

  ok = ( m_parser.send("AT+QIDNSGIP=1,\"%s\"", name)
         && m_parser.recv("OK")
         && m_parser.recv("+QIURC: \"dnsgip\",%d,%d,%d", &err, &ipcount, &dnsttl)
         && err == 0
         && ipcount > 0
       );

  if ( ok ) {
    m_parser.recv("+QIURC: \"dnsgip\",\"%[^\"]\"", ipstr);       //use the first DNS value
    for ( int i = 0; i < ipcount - 1; i++ )
      m_parser.recv("+QIURC: \"dnsgip\",\"%[^\"]\"", buf2);   //and discrard the rest  if >1

    ret = RET_OK;
  }
  return ret;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 PDP context activate / deactivate
// ----------------------------------------------------------------

void setContextActivate_BG96(void)
{
  if ( (m_parser.send("AT+QIACT=1")
        && m_parser.recv(RESP_OK)) ) {
    LOGDEBUG("PDP Context Activation: Success\r\n");
  }
}

int8_t setContextDeactivate_BG96(void) // Deactivate a PDP Context
{
  if ( (m_parser.send("AT+QIDEACT=1")
        && m_parser.recv(RESP_OK)) ) {
    LOGDEBUG("PDP Context Deactivation: Success\r\n");
  }
}

int8_t getIpAddress_BG96(char * ipstr) // IPv4 or IPv6
{
  int8_t ret = RET_NOK;
  int id, state, type; // not used

  m_parser.send("AT+QIACT?");
  if (m_parser.recv("+QIACT: %d,%d,%d,\"%[^\"]\"", &id, &state, &type, ipstr)
      && m_parser.recv("OK")) {
    ret = RET_OK;
  }
  return ret;
}
// ----------------------------------------------------------------
// Functions: Cat.M1 MQTT Publish & Subscribe
// ----------------------------------------------------------------

int8_t openMqttBroker_BG96(char * url, int port)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  unsigned long lastOpenedTime = 0;         // last time you connected to the server, in milliseconds
  bool done = false;
  //Timer t;

  //t.start();
  lastOpenedTime = millis();

  if (m_parser.send("AT+QMTOPEN=%d,\"%s\",%d", id, url, port) && m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTOPEN: %d,%d", &id, &result) && (result == 0));

      // MQTT Open: result code sample, refer to BG96_MQTT_Application_Note
      if (result == 1) {
        LOGDEBUG("AT+QMTOPEN result[1]: Wrong parameter");
      } else if (result == 2) {
        LOGDEBUG("AT+QMTOPEN result[2]: MQTT identifier is occupied");
      } else if (result == 3) {
        LOGDEBUG("AT+QMTOPEN result[3]: Failed to activate PDP");
      } else if (result == 4) {
        LOGDEBUG("AT+QMTOPEN result[4]: Failed to parse domain name");
      } else if (result == 5) {
        LOGDEBUG("AT+QMTOPEN result[5]: Network disconnection error");
      }
    } while (!done && (millis() - lastOpenedTime) < BG96_CONNECT_TIMEOUT);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}

int8_t connectMqttBroker_BG96(char * clientid, char * userid, char * password)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  int ret_code = 0;

  unsigned long lastConnectedTime = 0;         // last time you connected to the server, in milliseconds
  char buf[100];

  bool done = false;
  //Timer t;

  if ((userid != NULL) && (password != NULL)) {
    m_parser.send("AT+QMTCONN=%d,\"%s\",\"%s\",\"%s\"", id, clientid, userid, password);
  } else {
    m_parser.send("AT+QMTCONN=%d,\"%s\"", id, clientid);
  }

  //t.start();
  lastConnectedTime = millis();

  if (m_parser.recv("OK"))
  {
    do {
      done = (m_parser.recv("+QMTCONN: %d,%d,%d", &id, &result, &ret_code)
      && (result == 0) && (ret_code == 0));

      // MQTT Connect: result sample, refer to BG96_MQTT_Application_Note
      if (result == 1) {
        LOGDEBUG("AT+QMTCONN result[1]: Packet retransmission");
      } else if (result == 2) {
        LOGDEBUG("AT+QMTCONN result[2]: Failed to send packet");
      }

      // MQTT Connect: ret_code sample, refer to BG96_MQTT_Application_Note
      if (result == 1) {
        LOGDEBUG("AT+QMTCONN ret_code[1]: Connection Refused: Unacceptable Protocol Version");
      } else if (result == 2) {
        LOGDEBUG("AT+QMTCONN ret_code[2]: Connection Refused: Identifier Rejected");
      } else if (result == 3) {
        LOGDEBUG("AT+QMTCONN ret_code[3]: Connection Refused: Server Unavailable");
      } else if (result == 4) {
        LOGDEBUG("AT+QMTCONN ret_code[4]: Connection Refused: Bad User Name or Password");
      } else if (result == 5) {
        LOGDEBUG("AT+QMTCONN ret_code[5]: Connection Refused: Not Authorized");
      }
    } while (!done &&(millis() - lastConnectedTime) < BG96_CONNECT_TIMEOUT * 2);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}

int8_t closeMqttBroker_BG96(void)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  unsigned long lastClosedTime = 0;    // last time you connected to the server, in milliseconds

  bool done = false;
  //Timer t;

  //t.start();
  lastClosedTime = millis();

  if (m_parser.send("AT+QMTDISC=%d", id) && m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTDISC: %d,%d", &id, &result));
    } while (!done && (millis() - lastClosedTime) < BG96_CONNECT_TIMEOUT * 2);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}


int8_t sendMqttPublishMessage_BG96(char * topic, int qos, int retain, char * msg, int len)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;
  int sent_msgid = 0;
  static int msgid = 0;

  unsigned long lastSentTime = 0;    // last time you connected to the server, in milliseconds
  char buf[100];

  bool done = false;
  //Timer t;

  if (qos != 0) {
    if (msgid < 0xffff)
      msgid++;
    else
      msgid = 0;
  }

  //t.start();
  lastSentTime = millis();

  m_parser.send("AT+QMTPUB=%d,%d,%d,%d,\"%s\"", id, qos ? msgid : 0, qos, retain, topic);

  if ( !done && m_parser.recv(">") )
    done = (m_parser.write(msg, len) <= 0) & m_parser.send("%c", MQTT_EOF);

  if (m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTPUB: %d,%d,%d", &id, &sent_msgid, &result));
    } while (!done && (millis() - lastSentTime) < BG96_CONNECT_TIMEOUT * 2); //BG96_CONNECT_TIMEOUT * 2

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.flush();

  return ret;
}

int8_t setMqttSubscribeTopic_BG96(char * topic, int msgid, int qos)
{
  int8_t ret = RET_NOK;
  int id = 0;
  int result = 0;

  int sent_msgid = 0;
  int qos_level = 0;
  unsigned long lastSetTime = 0;    // last time you connected to the server, in milliseconds

  bool done = false;
  //Timer t;

  m_parser.set_timeout(BG96_CONNECT_TIMEOUT);

  //t.start();
  lastSetTime = millis();

  if (m_parser.send("AT+QMTSUB=%d,%d,\"%s\",%d", id, msgid, topic, qos) && m_parser.recv("OK")) {
    do {
      done = (m_parser.recv("+QMTSUB: %d,%d,%d,%d", &id, &sent_msgid, &result, &qos_level));
    } while (!done && (millis() - lastSetTime) < BG96_CONNECT_TIMEOUT);

    if (done) {
      ret = RET_OK;
    }
  }
  m_parser.set_timeout(BG96_DEFAULT_TIMEOUT);
  m_parser.flush();

  return ret;
}

int8_t checkRecvMqttMessage_BG96(char * topic, int * msgid, char * msg)
{
  int8_t ret = RET_NOK;
  int id = 0;
  bool received = false;

  m_parser.set_timeout(1);
  received = m_parser.recv("+QMTRECV: %d,%d,\"%[^\"]\",\"%[^\"]\"", &id, msgid, topic, msg);
  m_parser.set_timeout(BG96_DEFAULT_TIMEOUT);

  if (received) ret = RET_OK;
  return ret;
}

void dumpMqttSubscribeTopic_BG96(char * topic, int msgid, int qos)
{
  char buf[100];
  sprintf((char *)buf, "[MQTT] Subscribe Topic: \"%s\", ID: %d, QoS: %d\r\n", topic, msgid, qos);
  MYPRINTF(buf);
}

void dumpMqttPublishMessage_BG96(char * topic, char * msg)
{
  char buf[100];
  sprintf((char *)buf, "[MQTT] Published Topic: \"%s\", Message: \"%s\"\r\n", topic, msg);
  MYPRINTF(buf);
}

// ----------------------------------------------------------------
// Functions: MQTT SSL/TLS enable
// ----------------------------------------------------------------

int8_t setMqttTlsEnable_BG96(bool enable)
{
    int8_t ret = RET_NOK;

    int id = 0; // tcp connection id (0 - 6)
    int tls_ctxindex = 0; // ssl context index (0 - 5)

    if(m_parser.send("AT+QMTCFG=\"SSL\",%d,%d,%d", id, enable?1:0, tls_ctxindex) && m_parser.recv("OK")) {
        ret = RET_OK;
    } else {
        LOGDEBUG("MQTT SSL/TLS enable failed\r\n");
    }
    return ret;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 File system
// ----------------------------------------------------------------

int8_t saveFileToStorage_BG96(char * path, const char * buf, int len)
{
    int8_t ret = RET_NOK;
    int timeout_sec = 30;

    bool done = false;
    int upload_size = 0;
    char checksum[10] = {0, };

    m_parser.set_timeout(BG96_WAIT_TIMEOUT);

    if(m_parser.send("AT+QFUPL=\"%s\",%d,%d", path, len, timeout_sec) && m_parser.recv("CONNECT")) {
        done = m_parser.write(buf, len);
        if(done) {
    if(m_parser.recv("+QFUPL: %d,%s\r\n", &upload_size, checksum) && m_parser.recv("OK")) {
        if(len == upload_size) {
LOGDEBUG("File saved: %s, %d, %s\r\n", path, upload_size, checksum);
ret = RET_OK;
        }
    }
        }
    }
    m_parser.set_timeout(BG96_DEFAULT_TIMEOUT);
    if(ret != RET_OK) {
        LOGDEBUG("Save a file to storage failed: %s\r\n", path);
    }
    m_parser.flush();

    delay(100);
    return ret;
}

int8_t eraseFileStorageAll_BG96(void)
{
    int8_t ret = RET_NOK;

    if(m_parser.send("AT+QFDEL=\"*\"") && m_parser.recv("OK")) {
        ret = RET_OK;
    } else {
        LOGDEBUG("Erase storage failed\r\n");
    }

    delay(100);
    return ret;
}

#define MAX_FILE_LIST           10
void dumpFileList_BG96(void)
{
    char _buf[30] = {0, };
    int flen = {0, };
    int fcnt = 0;

    unsigned long lastSentTime = 0;

    bool done = false;
    //Timer t;

    //t.start();
    lastSentTime = millis();

    if(m_parser.send("AT+QFLST")) {
        do {
    if(m_parser.recv("+QFLST: \"%[^\"]\",%d\r\n", _buf, &flen)) {
        LOGDEBUG("File[%d]: %s, %d\r\n", fcnt++, _buf, flen);
        memset(_buf, 0x00, sizeof(_buf));
    }
    else if(m_parser.recv("OK")) {
        done = true;
    }
        } while(!done && (millis() - lastSentTime) < BG96_WAIT_TIMEOUT);
    }
    m_parser.flush();
}


// ----------------------------------------------------------------
// Functions: SSL/TLS config
// ----------------------------------------------------------------

int8_t setTlsCertificatePath_BG96(char * param, char * path)
{
    int8_t ret = RET_NOK;
    int tls_ctxindex = 0;       // ssl context index (0 - 5)

    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,\"%s\"", param, tls_ctxindex, path) && m_parser.recv("OK")) {
        ret = RET_OK;
    } else {
        LOGDEBUG("Set SSL/TLS certificate path failed: %s\r\n", param);
    }
    return ret;
}

// 0: SSL3.0, 1: TLS1.0, 2: TLS1.1, 3: TLS1.2, 4: All
int8_t setTlsConfig_sslversion_BG96(int ver)
{
    int8_t ret = RET_NOK;
    int tls_ctxindex = 0;     // ssl context index (0 - 5)
    char param[] = "sslversion";    // ssl config paramter type

    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%d", param, tls_ctxindex, ver) && m_parser.recv("OK")) {
        ret = RET_OK;
    } else {
        LOGDEBUG("Set SSL/TLS version failed: %d\r\n", ver);
    }
    return ret;
}

int8_t setTlsConfig_ciphersuite_BG96(char * ciphersuite)    // refer to SSL manual
{
    int8_t ret = RET_NOK;
    int tls_ctxindex = 0;     // ssl context index (0 - 5)
    char param[] = "ciphersuite";           // ssl config paramter type

    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%s", param, tls_ctxindex, ciphersuite) && m_parser.recv("OK")) {
        ret = RET_OK;
    } else {
        LOGDEBUG("Set SSL/TLS Ciphersuite failed: %d\r\n", ciphersuite);
    }
    return ret;
}

int8_t setTlsConfig_seclevel_BG96(int seclevel)
{
    int8_t ret = RET_NOK;
    int tls_ctxindex = 0;     // ssl context index (0 - 5)
    char sslconfig[] = "seclevel";          // ssl config paramter type

    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%d", sslconfig, tls_ctxindex, seclevel) && m_parser.recv("OK")) {
        ret = RET_OK;
    } else {
        LOGDEBUG("Set SSL/TLS authentication mode failed: %d\r\n", seclevel);
    }
    return ret;
}

int8_t setTlsConfig_ignoreltime_BG96(bool enable)
{
    int8_t ret = RET_NOK;
    int tls_ctxindex = 0;     // ssl context index (0 - 5)
    char sslconfig[] = "ignorelocaltime";   // ssl config paramter type

    if(m_parser.send("AT+QSSLCFG=\"%s\",%d,%d", sslconfig, tls_ctxindex, enable?1:0) && m_parser.recv("OK")) {
        ret = RET_OK;
    } else {
        LOGDEBUG("Set SSL/TLS ignore validity check option failed: %s\r\n", sslconfig);
    }
    return ret;
}


// ----------------------------------------------------------------
// Functions: AWS IoT samples
// ----------------------------------------------------------------

int8_t aws_iot_connection_process(void)
{
    static int8_t mqtt_state;

    switch(mqtt_state) {
        case MQTT_STATE_CONNECTED:
    break;

        case MQTT_STATE_OPEN:
    if(openMqttBroker_BG96(AWS_IOT_MQTT_HOST, AWS_IOT_MQTT_PORT) == RET_OK) {
        MYPRINTF("[MQTT] Socket open success\r\n");
        mqtt_state = MQTT_STATE_CONNECT;
    } else {
        MYPRINTF("[MQTT] Socket open failed\r\n");
    }
    break;

        case MQTT_STATE_CONNECT:
    if(connectMqttBroker_BG96(AWS_IOT_MQTT_CLIENT_ID, NULL, NULL) == RET_OK) {
        MYPRINTF("[MQTT] Connected, ClientID: \"%s\"\r\n", AWS_IOT_MQTT_CLIENT_ID);
        mqtt_state = MQTT_STATE_CONNECTED;
    } else {
        MYPRINTF("[MQTT] Connect failed\r\n");
        mqtt_state = MQTT_STATE_DISCON;
    }
    break;

        case MQTT_STATE_DISCON:
    if(closeMqttBroker_BG96() == RET_OK) {
        MYPRINTF("[MQTT] Disconnected\r\n");
    }
    mqtt_state = MQTT_STATE_OPEN;
    break;

        default:
    mqtt_state = MQTT_STATE_OPEN;
    break;
    }

    return mqtt_state;
}

// ----------------------------------------------------------------
// Functions: Cat.M1 GPS
// ----------------------------------------------------------------

int8_t setGpsOnOff_BG96(bool onoff)
{
  int8_t ret = RET_NOK;
  char _buf[15];
  char _buf1[30];
  char _buf2[30];

  sprintf((char *)_buf, "%s", onoff ? "AT+QGPS=2" : "AT+QGPSEND");

  if (m_parser.send(_buf) && m_parser.recv("OK")) {
    sprintf((char *)_buf1, "GPS Power: %s\r\n", onoff ? "On" : "Off");
    LOGDEBUG(_buf1);
    ret = RET_OK;
  } else {
    sprintf((char *)_buf2, "Set GPS Power %s failed\r\n", onoff ? "On" : "Off");
    LOGDEBUG(_buf2);
  }
  return ret;
}



int8_t getGpsLocation_BG96(gps_data data)
{
  int8_t ret = RET_NOK;
  char buf1[200];

  bool ok = false;
  //Timer t;

  // Structure init: GPS info
  //data->utc = data->lat = data->lon = data->hdop = data->altitude = data->cog = data->spkm = data->spkn = data->nsat = 0.0;
  //data->fix = 0;
  data.utc = data.lat = data.lon = data.hdop = data.altitude = data.cog = data.spkm = data.spkn = data.nsat = 0.0;
  data.fix = 0;
  //data->date = 0;
  //memset(&data->date, 0x00, 7);
  //strcpy(data->date, "");
  strcpy(data.date, "");

  // timer start
  //t.start();
  getLocationTime = millis();

    //while ( !ok && ( (millis() - getLocationTime) < BG96_CONNECT_TIMEOUT ) ) {
    if (!ok) {
    m_parser.flush();
    m_parser.send((char*)"AT+QGPSLOC=2"); // MS-based mode
    ok = m_parser.recv("+QGPSLOC: ");
    if (ok) {
      m_parser.recv("%s\r\n", buf1);
      sscanf(buf1, "%f,%f,%f,%f,%f,%d,%f,%f,%f,%6s,%d",
     &data.utc, &data.lat, &data.lon, &data.hdop,
     &data.altitude, &data.fix, &data.cog,
     &data.spkm, &data.spkn, &data.date, &data.nsat);

     //dtostrf(data.lat, 7, 5, latBuf);
     //dtostrf(data.lon, 8, 5, lonBuf);
     sprintf((char*)utcBuf, "%f", data.utc);
     sprintf((char*)latBuf, "%f", data.lat);
     sprintf((char*)lonBuf, "%f", data.lon);
     sprintf((char*)dateBuf, "%s", data.date);
     sprintf((char*)timestampBuf, "%.2f,%s", data.utc, data.date);

//     Serial.print("utcBuf 값 : ");
//     Serial.println(utcBuf);
//     Serial.print("latBuf 값 : ");
//     Serial.println(latBuf);
//     Serial.print("lonBuf 값 : ");
//     Serial.println(lonBuf);
//     Serial.print("dateBuf 값 : ");
//     Serial.println(dateBuf);
//     Serial.print("timestampBuf 값 : ");
//     Serial.println(timestampBuf);
//     Serial.println("");
//     Serial.print("buf1 값 : ");
//     Serial.println((char*)buf1);


      ok = m_parser.recv("OK");
    }
  }

  if (ok == true) ret = RET_OK;

  return ret;
}

/*
int bg96_ON(int pwrCheck)
{
  pwrCheck = 0;
 if (pwrCheck == 0) {
  digitalWrite(PWR_PIN, HIGH);
  digitalWrite(RST_PIN, HIGH);
  delay(300); // Setup Time : Greater than or equal to 30ms
  digitalWrite(PWR_PIN, LOW);
  digitalWrite(RST_PIN, LOW);
  delay(600); // Hold Time : Greater than or equal to 500ms
  digitalWrite(PWR_PIN, HIGH);
  delay(5000); // Release Time : Greater than or equal to 4800ms

  pwrCheck = 1;
  }
  return pwrCheck;
}
*/
int bg96_DeviceRST(int rstCheck)
{
  rstCheck = 0;

 if (rstCheck == 0) {
  HAL_GPIO_WritePin(GPIOA, bg96_rst_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, bg96_pwr_Pin, GPIO_PIN_SET);
  delay(300); // Setup Time : Greater than or equal to 30ms
  HAL_GPIO_WritePin(GPIOA, bg96_rst_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, bg96_pwr_Pin, GPIO_PIN_RESET);
  delay(400); // Hold Time : Greater than or equal to 500ms
  HAL_GPIO_WritePin(GPIOA, bg96_rst_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, bg96_pwr_Pin, GPIO_PIN_SET);
  delay(1000); // Release Time : Greater than or equal to 4800ms

  rstCheck = 1;
  }
  return rstCheck;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_IT(&huart1, &m_parser.rxData, 1);
  Ringbuf_init();


  if (bg96_DeviceRST(rstCheck) == 1 ) {

      //aws_flip();

      //char buf[100];
      //char buf1[40];

      //serialPcInit();
      catm1DeviceInit();

      MYPRINTF("Waiting for Cat.M1 Module Ready...\r\n");

      waitCatM1Ready();

      delay(2000);

      MYPRINTF("System Init Complete\r\n");


    if ( (setEchoStatus_BG96(false)) && (getUsimStatus_BG96()) && (getNetworkStatus_BG96()) )
    	MYPRINTF("System Operation Complete\r\n");
// -> 코드 수정 필요




    checknSetApn_BG96(CATM1_APN_SKT);

    MYPRINTF("[FILE] Save and check AWS certificates\r\n");

    setContextActivate_BG96();

    int mqtt_state = MQTT_STATE_OPEN;

   /* Erase BG96 file storage */
      if(eraseFileStorageAll_BG96() == RET_OK) {
          MYPRINTF("[FILE] Erase BG96 storage complete\r\n");
      };

      /* Store AWS IoT certificate files to BG96 storage */
      saveFileToStorage_BG96(AWS_IOT_ROOT_CA_FILENAME, aws_iot_rootCA, strlen(aws_iot_rootCA));

      saveFileToStorage_BG96(AWS_IOT_CERTIFICATE_FILENAME, aws_iot_certificate, strlen(aws_iot_certificate));

      saveFileToStorage_BG96(AWS_IOT_PRIVATE_KEY_FILENAME, aws_iot_private_key, strlen(aws_iot_private_key));

  #if 0
      dumpFileList_BG96(); // file list dump
  #endif

      MYPRINTF("[SSL/TLS] Set BG96 SSL/TLS configuration\r\n")

      /* BG96 SSL/TLS config */
      // Set AWS IoT Certificate files
      setTlsCertificatePath_BG96("cacert", AWS_IOT_ROOT_CA_FILENAME);     // Root CA
      setTlsCertificatePath_BG96("clientcert", AWS_IOT_CERTIFICATE_FILENAME);     // Client certificate
      setTlsCertificatePath_BG96("clientkey", AWS_IOT_PRIVATE_KEY_FILENAME);      // Client privatekey

      // Set SSL/TLS config
      setTlsConfig_sslversion_BG96(AWS_IOT_BG96_SSLTLS_VERSION);
      setTlsConfig_ciphersuite_BG96(AWS_IOT_BG96_SSLTLS_CIPHERSUITE);
      setTlsConfig_seclevel_BG96(AWS_IOT_BG96_SSLTLS_SECLEVEL);
      setTlsConfig_ignoreltime_BG96(AWS_IOT_BG96_SSLTLS_IGNORELOCALTIME);


      /* BG96 MQTT config: SSL/TLS enable */
      setMqttTlsEnable_BG96(true);

      MYPRINTF("[MQTT] Connect to AWS IoT \"%s:%d\"\r\n", AWS_IOT_MQTT_HOST, AWS_IOT_MQTT_PORT);

      if (setGpsOnOff_BG96(ON) == RET_OK) {
      MYPRINTF("GPS On\r\n")
  #if 0
      if (setGpsOnOff_BG96(OFF) == RET_OK) {
        MYPRINTF("GPS Off\r\n")
      }
  #endif
   }  else {
      MYPRINTF("GPS On failed\r\n")
    }

    }
    else
    	MYPRINTF("GPS On failed\r\n")

      //bool subscribe_complete = false;

      sprintf(aws_iot_sub_topic, "$aws/things/%s/shadow/update/accepted", AWS_IOT_MY_THING_NAME);
      sprintf(aws_iot_pub_topic, "$aws/things/%s/shadow/update", AWS_IOT_MY_THING_NAME);
      sprintf(aws_iot_pub_topic1, "$aws/things/%s/shadow/update", AWS_IOT_MY_THING_NAME);
      sprintf(aws_iot_pub_topic2, "dt/cardata/logger/%s/car-data", AWS_IOT_MY_THING_NAME);

//      reset_trip();
//      flipper.attach(30.0, aws_flip); // Publish messages every 3 seconds
      //flipper.attach(2.0,tripDataSave);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (getGpsLocation_BG96(gps_info) == RET_OK) {
	         //MYPRINTF("Get GPS information >>>");
	         //sprintf((char *)utcBuf, "gps_info - utc: %6.3f", gps_info.utc);
	         //MYPRINTF(utcBuf)     // utc: hhmmss.sss
	         //sprintf((char *)latBuf, "gps_info - lat: %2.5f", gps_info.lat);
	         //MYPRINTF(latBuf)     // latitude: (-)dd.ddddd
	         //sprintf((char *)lonBuf, "gps_info - lon: %2.5f", gps_info.lon);
	         //MYPRINTF(lonBuf)     // longitude: (-)dd.ddddd
	         //MYPRINTF("gps_info - hdop: %2.1f", gps_info.hdop)           // Horizontal precision: 0.5-99.9
	         //MYPRINTF("gps_info - altitude: %2.1f", gps_info.altitude)   // altitude of antenna from sea level (meters)
	         //MYPRINTF("gps_info - fix: %d", gps_info.fix)        // GNSS position mode: 2=2D, 3=3D
	         //MYPRINTF("gps_info - cog: %3.2f", gps_info.cog)     // Course Over Ground: ddd.mm
	         //MYPRINTF("gps_info - spkm: %4.1f", gps_info.spkm)           // Speed over ground (Km/h): xxxx.x
	         //MYPRINTF("gps_info - spkn: %4.1f", gps_info.spkn)           // Speed over ground (knots): xxxx.x
	         //MYPRINTF("gps_info - date: %s", gps_info.date)      // data: ddmmyy
	         //MYPRINTF("gps_info - nsat: %d\r\n", gps_info.nsat)          // number of satellites: 0-12


	        //char* last_utcBuf = utcBuf;
	        //char* last_latBuf = latBuf;
	        //char* last_lonBuf = lonBuf;
	        //char* last_dateBuf = dateBuf;
	        strcpy(last_utcBuf, utcBuf);
	        strcpy(last_latBuf, latBuf);
	        strcpy(last_lonBuf, lonBuf);
	        strcpy(last_dateBuf, dateBuf);
	        /*Serial.print("last_utcBuf 값 : ");
	        Serial.println(last_utcBuf);
	        Serial.print("last_latBuf 값 : ");
	        Serial.println(last_latBuf);
	        Serial.print("last_lonBuf 값 : ");
	        Serial.println(last_lonBuf);
	        Serial.print("last_dateBuf 값 : ");
	        Serial.println(last_dateBuf);*/

	       } else delay(1);
	         MYPRINTF("Failed to get GPS information\r\n");

	        if(aws_iot_connection_process() == MQTT_STATE_CONNECTED) {

	        	char car_state[] = "hello soomin~";
	        	uint32_t rpm = 1;
	        	uint32_t kph = 2;
	        	uint32_t engineLoad = 3;
	        	uint16_t runTime = 4;
	        	uint8_t fuelType = 5;
	        	int32_t oilTemp = 6;
	        	uint32_t relativePedalPos = 7;
	        	uint32_t throttle = 8;
	        	uint32_t relativeThrottle = 9;
	        	int32_t intakeAirTemp = 10;
	        	uint32_t fuelLevel = 11;
	        	float mafRate = 12; // uint32_t
	        	uint8_t obdStandards = 13;
	        	uint16_t distTravelWithMIL = 14;
	        	uint16_t distSinceCodesCleared = 15;
	        	uint32_t ctrlModVoltage = 16;
	        	int16_t ambientAirTemp = 17;
	        	uint32_t manifoldPressure = 18;
	        	int32_t engineCoolantTemp = 19;
	        	float commandedThrottleActuator = 20;
	        	static float efficiencyScore = 5.7;
	        	static float safetyScore = 8.3;
	        	static float drivingScore = 10.0;
	        	static float averagedrivingScore = 10.0;



	           // MQTT Subscribe
	       if(setMqttSubscribeTopic_BG96(aws_iot_sub_topic, 1, MQTT_QOS1) == RET_OK) {
	     MYPRINTF("[MQTT] Topic subscribed: \"%s\"\r\n", aws_iot_sub_topic);
	       }

	           // MQTT Publish
	       mqtt_len = sprintf(buf_mqtt_send, "{\"state\":{\"reported\":{\"name\":\"%s\",\"enabled\":\"%s\",\"geo\":{\"latitude\":\"%s\",\"longitude\":\"%s\"}}}}", AWS_IOT_MY_THING_NAME, car_state , last_latBuf, last_lonBuf);
	       if(sendMqttPublishMessage_BG96(aws_iot_pub_topic, MQTT_QOS1, MQTT_RETAIN, buf_mqtt_send, mqtt_len) == RET_OK) {
	     MYPRINTF("[MQTT] Message published: \"%s\", Message: %s\r\n", aws_iot_pub_topic, buf_mqtt_send);
	       }

	        delay(1);

	       mqtt_len2 = sprintf(buf_mqtt_send2, "{\"efficiencyScore\": %f,\"safetyScore\": %f,\"drivingScore\": %f,\"averagedrivingScore\": %f,\"rpm\": %u,\"kph\": %u,\"engineLoad\": %u,\"runTime\": %u,\"fuelType\": %d,\"oilTemp\": %d,\"relativePedalPos\": %u,\"throttle\": %u,\"relativeThrottle\": %u,\"intakeAirTemp\": %d,\"fuelLevel\": %u,\"mafRate\": %f,\"obdStandards\": %u,\"distTravelWithMIL\": %u,\"distSinceCodesCleared\": %u,\"ctrlModVoltage\": %d,\"ambientAirTemp\": %d,\"manifoldPressure\": %d,\"engineCoolantTemp\": %d,\"commandedThrottleActuator\": %f,\"timestamp\": %s}", efficiencyScore, safetyScore, drivingScore, averagedrivingScore, rpm, kph, engineLoad, runTime, fuelType, oilTemp, relativePedalPos, throttle, relativeThrottle, intakeAirTemp, fuelLevel, mafRate, obdStandards, distTravelWithMIL, distSinceCodesCleared, ctrlModVoltage, ambientAirTemp, manifoldPressure, engineCoolantTemp, commandedThrottleActuator, timestampBuf);
	       if(sendMqttPublishMessage_BG96(aws_iot_pub_topic2, MQTT_QOS1, MQTT_RETAIN, buf_mqtt_send2, mqtt_len2) == RET_OK) {
	     MYPRINTF("[MQTT] Message published: \"%s\", Message: %s\r\n", aws_iot_pub_topic2, buf_mqtt_send2);
	       }

	           // MQTT message received
	       if(checkRecvMqttMessage_BG96(buf_mqtt_topic, &mqtt_msgid, buf_mqtt_recv) == RET_OK) {
	     MYPRINTF("[MQTT] Message arrived: Topic \"%s\" ID %d, Message %s\r\n", buf_mqtt_topic, mqtt_msgid, buf_mqtt_recv);
	           }
	       delay(5);
	    }

    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
