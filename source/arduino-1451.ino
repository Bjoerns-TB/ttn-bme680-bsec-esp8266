  *  For BSEC_1.4.5.1_Generic_Release_20171116
/*
   Copyright (C) 2017 Robert Bosch. All Rights Reserved.

   Disclaimer

   Common:
   Bosch Sensortec products are developed for the consumer goods industry. They may only be used
   within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
   provided with the express understanding that there is no warranty of fitness for a particular purpose.
   They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
   that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
   Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
   The resale and/or use of products are at the purchasers own risk and his own responsibility. The
   examination of fitness for the intended use is the sole responsibility of the Purchaser.

   The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
   incidental, or consequential damages, arising from any product use not covered by the parameters of
   the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
   Sensortec for all costs in connection with such claims.

   The purchaser must monitor the market for the purchased products, particularly with regard to
   product safety and inform Bosch Sensortec without delay of all security relevant incidents.

   Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
   technical specifications of the product series. They are therefore not intended or fit for resale to third
   parties or for use in end products. Their sole purpose is internal client testing. The testing of an
   engineering sample may in no way replace the testing of a product series. Bosch Sensortec
   assumes no liability for the use of engineering samples. By accepting the engineering samples, the
   Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
   samples.

   Special:
   This software module (hereinafter called "Software") and any information on application-sheets
   (hereinafter called "Information") is provided free of charge for the sole purpose to support your
   application work. The Software and Information is subject to the following terms and conditions:

   The Software is specifically designed for the exclusive use for Bosch Sensortec products by
   personnel who have special experience and training. Do not use this Software if you do not have the
   proper experience or training.

   This Software package is provided `` as is `` and without any expressed or implied warranties,
   including without limitation, the implied warranties of merchantability and fitness for a particular
   purpose.

   Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
   of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
   representatives and agents shall not be liable for any direct or indirect damages or injury, except as
   otherwise stipulated in mandatory applicable law.

   The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
   responsibility for the consequences of use of such Information nor for any infringement of patents or
   other rights of third parties which may result from its use. No license is granted by implication or
   otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
   subject to change without notice.

   It is not allowed to deliver the source code of the Software to any third party without permission of
   Bosch Sensortec.

*/

/*!
   @file bsec_iot_example.ino

   @brief
   Example for using of BSEC library in a fixed configuration with the BME680 sensor.
   This works by running an endless loop in the bsec_iot_loop() function.
*/

/*!
   @addtogroup bsec_examples BSEC Examples
   @brief BSEC usage examples
   @{*/
/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 * 
 * 
 * 
 * Connect your NODE MCU as this
 *  D1 (GPIO5)  - DIO0 (SX1275/RFM95)
 *  D2 (GPIO4)  - DIO1
 *  D5 (GPIO14) - SCK 
 *  D6 (GPIO12) - MISO
 *  D7 (GPIO13) - MOSI
 *  DO (GPIO16) - NSS
 *  GND         - GND
 *  3.3v        - VCC
 * 
 * Compile this code as NODEMCU 1.0
 *******************************************************************************/

 * Connect your BME680 as this
 *  D3 (GPIO0)  - SDA (BME680)
 *  D4 (GPIO2)  - SCL
 *  GND         - GND
 *  3.3v        - VCC
 *******************************************************************************/
/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include "bsec_integration.h"
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <ESP8266WiFi.h>

/**********************************************************************************************************************/
/* LoRaWAN */
/**********************************************************************************************************************/
uint8_t mydata[9];


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xB9, 0x10, 0x1F, 0x43, 0xB7, 0x23, 0x13, 0x04, 0x81, 0xCE, 0x54, 0x1E, 0x24, 0x4F, 0xAC, 0xA3 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const u1_t PROGMEM APPSKEY[16] = { 0x82, 0x44, 0x01, 0xE7, 0xD5, 0x83, 0x14, 0xEB, 0x4D, 0xCD, 0xD7, 0xF7, 0xB3, 0x10, 0x46, 0x6A };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x26011BEB; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 16,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,  //5
  .dio = {5, 4, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
   @brief           Write operation in either Wire or SPI

   param[in]        dev_addr        Wire or SPI device address
   param[in]        reg_addr        register address
   param[in]        reg_data_ptr    pointer to the data to be written
   param[in]        data_len        number of bytes to be written

   @return          result of the bus communication function
*/
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);    /* Set register address to start writing to */

  /* Write the data */
  for (int index = 0; index < data_len; index++) {
    Wire.write(reg_data_ptr[index]);
  }

  return (int8_t)Wire.endTransmission();
}

/*!
   @brief           Read operation in either Wire or SPI

   param[in]        dev_addr        Wire or SPI device address
   param[in]        reg_addr        register address
   param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
   param[in]        data_len        number of bytes to be read

   @return          result of the bus communication function
*/
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
  int8_t comResult = 0;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);                    /* Set register address to start reading from */
  comResult = Wire.endTransmission();

  delayMicroseconds(150);                 /* Precautionary response delay */
  Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */

  int index = 0;
  while (Wire.available())  /* The slave device may send less than requested (burst read) */
  {
    reg_data_ptr[index] = Wire.read();
    index++;
  }

  return comResult;
}

/*!
   @brief           System specific implementation of sleep function

   @param[in]       t_ms    time in milliseconds

   @return          none
*/
void sleep(uint32_t t_ms)
{
  delay(t_ms);
}

/*!
   @brief           Capture the system time in microseconds

   @return          system_current_time    current system timestamp in microseconds
*/
int64_t get_timestamp_us()
{
  return (int64_t) millis() * 1000;
}

/*!
   @brief           Handling of the ready outputs

   @param[in]       timestamp       time in nanoseconds
   @param[in]       iaq             IAQ signal
   @param[in]       iaq_accuracy    accuracy of IAQ signal
   @param[in]       temperature     temperature signal
   @param[in]       humidity        humidity signal
   @param[in]       pressure        pressure signal
   @param[in]       raw_temperature raw temperature signal
   @param[in]       raw_humidity    raw humidity signal
   @param[in]       gas             raw gas sensor signal
   @param[in]       bsec_status     value returned by the bsec_do_steps() call

   @return          none
*/
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                  float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status)
{
  Serial.print("[");
  Serial.print(timestamp / 1e6);
  Serial.print("] T: ");
  Serial.print(temperature);
  Serial.print("| rH: ");
  Serial.print(humidity);
  Serial.print("| P: ");
  Serial.print(pressure / 100);
  Serial.print("| IAQ: ");
  Serial.print(iaq);
  Serial.print(" (");
  Serial.print(iaq_accuracy);
  Serial.println(")");

  int16_t temp = temperature * 100;
  int16_t hum = humidity * 100;
  int32_t ia = iaq * 100;
  int16_t pressu = round(pressure / 10);
  Serial.print(pressu);
  mydata[0] = temp;
  mydata[1] = temp >> 8;
  mydata[2] = hum;
  mydata[3] = hum >> 8;
  mydata[4] = ia;
  mydata[5] = ia >> 8;
  mydata[6] = pressu;
  mydata[7] = pressu >> 8;


  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);
}
  /*!
     @brief           Load previous library state from non-volatile memory

     @param[in,out]   state_buffer    buffer to hold the loaded state string
     @param[in]       n_buffer        size of the allocated state buffer

     @return          number of bytes copied to state_buffer
  */
  uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
  {
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    return 0;
  }

  /*!
     @brief           Save library state to non-volatile memory

     @param[in]       state_buffer    buffer holding the state to be stored
     @param[in]       length          length of the state string to be stored

     @return          none
  */
  void state_save(const uint8_t *state_buffer, uint32_t length)
  {
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
  }

  /*!
     @brief           Load library config from non-volatile memory

     @param[in,out]   config_buffer    buffer to hold the loaded state string
     @param[in]       n_buffer        size of the allocated state buffer

     @return          number of bytes copied to config_buffer
  */
  uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
  {
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    return 0;
  }

  /*!
     @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
                  on timer ticks

     @return      result of the processing
  */
  void setup()
  {
    return_values_init ret;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);
    /* Init I2C and serial communication */
    Wire.begin(0, 2); //GPIO SDA und SCL D3,D4
    Serial.begin(115200);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    /* Call to the function which initializes the BSEC library
       Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_ULP, 0.0f, bus_write, bus_read, sleep, state_load, config_load);
    if (ret.bme680_status)
    {
      /* Could not intialize BME680 */
      Serial.println("Error while initializing BME680");
      return;
    }
    else if (ret.bsec_status)
    {
      /* Could not intialize BSEC library */
      Serial.println("Error while initializing BSEC library");
      return;
    }

    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
  }

  void loop()
  {
  }

  /*! @}*/
