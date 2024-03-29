// Copyright © 2017 The Things Network
// Use of this source code is governed by the MIT license that can be found in the LICENSE file.

#ifndef _THETHINGSNETWORK_H_
#define _THETHINGSNETWORK_H_

#include <Arduino.h>
#include <Stream.h>
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_SAMD)
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif

#define TTN_DEFAULT_SF 7
#define TTN_DEFAULT_FSB 2
#define TTN_RETX "7"

#define TTN_PWRIDX_EU868 "1"
#define TTN_PWRIDX_US915 "5"
#define TTN_PWRIDX_AU915 "5"
#define TTN_PWRIDX_AS920_923 "1" // TODO: should be 0, but the current RN2903AS firmware doesn't accept that value (probably still using EU868: 1=14dBm)
#define TTN_PWRIDX_AS923_925 "1" // TODO: should be 0
#define TTN_PWRIDX_KR920_923 "1" // TODO: should be 0
#define TTN_PWRIDX_IN865_867 "1" // TODO: should be 0

#define TTN_BUFFER_SIZE 300
#define TTN_DEFAULT_TOUT 10000	// Default modem timeout in ms

typedef uint8_t port_t;

enum ttn_response_t
{
  TTN_ERROR_SEND_COMMAND_FAILED = (-1),
  TTN_ERROR_UNEXPECTED_RESPONSE = (-10),
  TTN_SUCCESSFUL_TRANSMISSION = 1,
  TTN_SUCCESSFUL_RECEIVE = 2,
  TTN_UNSUCCESSFUL_RECEIVE = 3
};

enum ttn_fp_t
{
  TTN_FP_EU868,
  TTN_FP_US915,
  TTN_FP_AU915,
  TTN_FP_AS920_923,
  TTN_FP_AS923_925,
  TTN_FP_KR920_923,
  TTN_FP_IN865_867
};

enum lorawan_class_t
{
  CLASS_A,
  CLASS_B,
  CLASS_C
};

enum ttn_response_code_t
{
	TTN_OK,
	TTN_ERR_BUSY = (-1),
	TTN_ERR_FRMCNT = (-2),
	TTN_ERR_INVCLS = (-3),
	TTN_ERR_INVDLEN = (-4),
	TTN_ERR_INVPAR = (-5),
	TTN_ERR_NKEYINT = (-6),
	TTN_ERR_MACPAUSE = (-7),
	TTN_ERR_NKYMLTCST = (-8),
	TTN_ERR_NFRCHN = (-9),
	TTN_ERR_NJOIN = (-10),
	TTN_ERR_SILENT = (-11),
	TTN_ERR_ERR = (-12),
};

enum ttn_modem_status_t
{
	TTN_MDM_READERR = -1,
	TTN_MDM_IDLE = 0,
	TTN_MDM_TX,
	TTN_MDM_BEFORE_RX,
	TTN_MDM_RX1,
	TTN_MDM_BEFORE_RX2,
	TTN_MDM_RETX_DELAY,
	TTN_MDM_APB_DELAY,
	TTN_MDM_C_RX1,
	TTN_MDM_C_RX2
};

class TheThingsNetwork
{
private:
  Stream *modemStream;
  Stream *debugStream = NULL;
  ttn_fp_t fp;
  uint8_t sf;
  uint8_t fsb;
  bool adr;
  char buffer[64]; // was 512 MAX rx/tx lenght = "mac tx uncnf 242 " + 242 bytes + "\n"
  bool baudDetermined = false;
  void (*messageCallback)(const uint8_t *payload, size_t size, port_t port);
  void (*beforeTxCallback)(void);
  void (*afterTxCallback)(void);
  void (*afterRxCallback)(void);
  lorawan_class_t lw_class = CLASS_A;

  void clearReadBuffer();
  size_t readLine(char *buffer, size_t size, uint8_t attempts = 3);
  size_t readResponse(uint8_t prefixTable, uint8_t indexTable, uint8_t index, char *buffer, size_t size);
  size_t readResponse(uint8_t table, uint8_t index, char *buffer, size_t size);

  void debugPrintIndex(uint8_t index, const char *value = NULL);
  void debugPrintMessage(uint8_t type, uint8_t index, const char *value = NULL);

  void autoBaud();
  void configureEU868();
  void configureUS915(uint8_t fsb);
  void configureAU915(uint8_t fsb);
  void configureAS920_923();
  void configureAS923_925();
  void configureKR920_923();
  void configureIN865_867();
  void configureChannels(uint8_t fsb);
  bool setSF(uint8_t sf);
  bool waitForOk();

  ttn_response_t parseBytes();
  void sendCommand(uint8_t table, uint8_t index, bool appendSpace, bool print = true);
  bool sendMacSet(uint8_t index, uint8_t value1, unsigned long value2);
  bool sendMacSet(uint8_t index, const char *value);
  bool sendChSet(uint8_t index, uint8_t channel, unsigned long value);
  bool sendChSet(uint8_t index, uint8_t channel, const char *value);
  bool sendJoinSet(uint8_t type);
  bool sendPayload(uint8_t mode, uint8_t port, uint8_t *payload, size_t len);
  void sendGetValue(uint8_t table, uint8_t prefix, uint8_t index);

public:
  bool needsHardReset = false;

  TheThingsNetwork(Stream &modemStream, Stream &debugStream, ttn_fp_t fp, uint8_t sf = TTN_DEFAULT_SF, uint8_t fsb = TTN_DEFAULT_FSB);
  void reset(bool adr = true);
  void resetHard(uint8_t resetPin);
  void showStatus();
  size_t getHardwareEui(char *buffer, size_t size);
  size_t getAppEui(char *buffer, size_t size);
  enum ttn_modem_status_t getStatus();
  uint16_t getVDD();
  int16_t getRSSI();
  uint32_t getFrequency();
  uint32_t getWatchDogTimer();
  uint32_t getFCU();
  uint32_t getFCD();
  uint8_t getBW();
  uint8_t getCR();
  int8_t getPower();
  int8_t getSNR();
  int8_t getDR();
  int8_t getPowerIndex();
  bool getChannelStatus (uint8_t ch);
  ttn_response_code_t getLastError();
  void onMessage(void (*cb)(const uint8_t *payload, size_t size, port_t port));
  void onBeforeTx(void (*cb)(void));
  void onAfterTx(void (*cb)(void));
  void onAfterRx(void (*cb)(void));
  bool provision(const char *appEui, const char *appKey);
  bool join(const char *appEui, const char *appKey, int8_t retries = -1, uint32_t retryDelay = 10000, lorawan_class_t = CLASS_A);
  bool join(int8_t retries = -1, uint32_t retryDelay = 10000);
  bool personalize(const char *devAddr, const char *nwkSKey, const char *appSKey);
  bool personalize();
  bool setClass(lorawan_class_t p_lw_class);
  ttn_response_t sendBytes(const uint8_t *payload, size_t length, port_t port = 1, bool confirm = false, uint8_t sf = 0);
  ttn_response_t poll(port_t port = 1, bool confirm = false, bool modem = false);
  void sleep(uint32_t mseconds);
  void wake();
  void saveState();
  void linkCheck(uint16_t seconds);
  uint8_t getLinkCheckGateways();
  uint8_t getLinkCheckMargin();
  bool setChannel(uint8_t ch, uint32_t freq = 0l, uint8_t drmin = 255, uint8_t drmax = 255);
  bool setRx2Channel(uint32_t freq, uint8_t dr);
  bool setChannelDCycle (uint8_t ch, float dcycle);
  bool setChannelStatus (uint8_t ch, bool status);
  bool setPowerIndex(uint8_t idx);
  bool setDR(uint8_t dr);
  bool setADR(bool adr);
  bool setRX1Delay(uint16_t delay);
  bool setFCU(uint32_t fcu);
  bool setFCD(uint32_t fcd);
};
#endif
