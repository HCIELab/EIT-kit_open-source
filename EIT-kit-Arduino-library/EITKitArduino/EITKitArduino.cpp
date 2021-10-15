/*
  EITKitArduino.cpp - Library for EIT-kit Sensing Board.
  Will be released into the public domain.
*/

#if ARDUINO>=100
   #include <Arduino.h>  // Arduino 1.0
 #else
   #include <WProgram.h>  // Arduino 0022
 #endif

#include "EITKitArduino.h"
#include "SPI.h"
#include "assert.h"
#include <string>
#include <iomanip>
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream

#if defined(ARDUINO_ARCH_ESP32)
#include "soc/gpio_struct.h"
#include "MCP23S17.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
//  SPIClass * vspi = new SPIClass(VSPI);
//     SPIClass * hspi = new SPIClass(HSPI);
SPIClass * vspi = new SPIClass(VSPI);
SPIClass * hspi = new SPIClass(HSPI);
MCP23S17 IOExpander(vspi, CHIP_SEL_MCP23S17, 0);
BLECharacteristic *pCharacteristic;
BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
int op = 0;
char cstr[300];
// Set the MCP23s17: IOExpander
#elif defined(__IMXRT1062__) // for Teensy 4.0

// State of MCP4252 TCON register
uint8_t tcon_reg = 0xFF;
#endif


int16_t sine_table[1024] = {
    0, 3, 6, 9, 12, 15, 18, 21, 25, 28, 31, 34, 37, 40, 43, 47,
    50, 53, 56, 59, 62, 65, 68, 72, 75, 78, 81, 84, 87, 90, 93, 96,
    99, 102, 106, 109, 112, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142, 145,
    148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 187, 190, 193,
    195, 198, 201, 204, 207, 210, 213, 216, 218, 221, 224, 227, 230, 233, 235, 238,
    241, 244, 246, 249, 252, 255, 257, 260, 263, 265, 268, 271, 273, 276, 279, 281,
    284, 287, 289, 292, 294, 297, 299, 302, 304, 307, 310, 312, 314, 317, 319, 322,
    324, 327, 329, 332, 334, 336, 339, 341, 343, 346, 348, 350, 353, 355, 357, 359,
    362, 364, 366, 368, 370, 372, 375, 377, 379, 381, 383, 385, 387, 389, 391, 393,
    395, 397, 399, 401, 403, 405, 407, 409, 411, 413, 414, 416, 418, 420, 422, 423,
    425, 427, 429, 430, 432, 434, 435, 437, 439, 440, 442, 443, 445, 447, 448, 450,
    451, 453, 454, 455, 457, 458, 460, 461, 462, 464, 465, 466, 468, 469, 470, 471,
    473, 474, 475, 476, 477, 478, 479, 481, 482, 483, 484, 485, 486, 487, 488, 489,
    489, 490, 491, 492, 493, 494, 495, 495, 496, 497, 498, 498, 499, 500, 500, 501,
    502, 502, 503, 503, 504, 504, 505, 505, 506, 506, 507, 507, 508, 508, 508, 509,
    509, 509, 510, 510, 510, 510, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
    512, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 510, 510, 510, 510, 509,
    509, 509, 508, 508, 508, 507, 507, 506, 506, 505, 505, 504, 504, 503, 503, 502,
    502, 501, 500, 500, 499, 498, 498, 497, 496, 495, 495, 494, 493, 492, 491, 490,
    489, 489, 488, 487, 486, 485, 484, 483, 482, 481, 479, 478, 477, 476, 475, 474,
    473, 471, 470, 469, 468, 466, 465, 464, 462, 461, 460, 458, 457, 455, 454, 453,
    451, 450, 448, 447, 445, 443, 442, 440, 439, 437, 435, 434, 432, 430, 429, 427,
    425, 423, 422, 420, 418, 416, 414, 413, 411, 409, 407, 405, 403, 401, 399, 397,
    395, 393, 391, 389, 387, 385, 383, 381, 379, 377, 375, 372, 370, 368, 366, 364,
    362, 359, 357, 355, 353, 350, 348, 346, 343, 341, 339, 336, 334, 332, 329, 327,
    324, 322, 319, 317, 314, 312, 310, 307, 304, 302, 299, 297, 294, 292, 289, 287,
    284, 281, 279, 276, 273, 271, 268, 265, 263, 260, 257, 255, 252, 249, 246, 244,
    241, 238, 235, 233, 230, 227, 224, 221, 218, 216, 213, 210, 207, 204, 201, 198,
    195, 193, 190, 187, 184, 181, 178, 175, 172, 169, 166, 163, 160, 157, 154, 151,
    148, 145, 142, 139, 136, 133, 130, 127, 124, 121, 118, 115, 112, 109, 106, 102,
    99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 68, 65, 62, 59, 56, 53,
    50, 47, 43, 40, 37, 34, 31, 28, 25, 21, 18, 15, 12, 9, 6, 3,
    0, -3, -6, -9, -12, -15, -18, -21, -25, -28, -31, -34, -37, -40, -43, -47,
    -50, -53, -56, -59, -62, -65, -68, -72, -75, -78, -81, -84, -87, -90, -93, -96,
    -99, -102, -106, -109, -112, -115, -118, -121, -124, -127, -130, -133, -136, -139, -142, -145,
    -148, -151, -154, -157, -160, -163, -166, -169, -172, -175, -178, -181, -184, -187, -190, -193,
    -195, -198, -201, -204, -207, -210, -213, -216, -218, -221, -224, -227, -230, -233, -235, -238,
    -241, -244, -246, -249, -252, -255, -257, -260, -263, -265, -268, -271, -273, -276, -279, -281,
    -284, -287, -289, -292, -294, -297, -299, -302, -304, -307, -310, -312, -314, -317, -319, -322,
    -324, -327, -329, -332, -334, -336, -339, -341, -343, -346, -348, -350, -353, -355, -357, -359,
    -362, -364, -366, -368, -370, -372, -375, -377, -379, -381, -383, -385, -387, -389, -391, -393,
    -395, -397, -399, -401, -403, -405, -407, -409, -411, -413, -414, -416, -418, -420, -422, -423,
    -425, -427, -429, -430, -432, -434, -435, -437, -439, -440, -442, -443, -445, -447, -448, -450,
    -451, -453, -454, -455, -457, -458, -460, -461, -462, -464, -465, -466, -468, -469, -470, -471,
    -473, -474, -475, -476, -477, -478, -479, -481, -482, -483, -484, -485, -486, -487, -488, -489,
    -489, -490, -491, -492, -493, -494, -495, -495, -496, -497, -498, -498, -499, -500, -500, -501,
    -502, -502, -503, -503, -504, -504, -505, -505, -506, -506, -507, -507, -508, -508, -508, -509,
    -509, -509, -510, -510, -510, -510, -511, -511, -511, -511, -511, -511, -511, -511, -511, -511,
    -512, -511, -511, -511, -511, -511, -511, -511, -511, -511, -511, -510, -510, -510, -510, -509,
    -509, -509, -508, -508, -508, -507, -507, -506, -506, -505, -505, -504, -504, -503, -503, -502,
    -502, -501, -500, -500, -499, -498, -498, -497, -496, -495, -495, -494, -493, -492, -491, -490,
    -489, -489, -488, -487, -486, -485, -484, -483, -482, -481, -479, -478, -477, -476, -475, -474,
    -473, -471, -470, -469, -468, -466, -465, -464, -462, -461, -460, -458, -457, -455, -454, -453,
    -451, -450, -448, -447, -445, -443, -442, -440, -439, -437, -435, -434, -432, -430, -429, -427,
    -425, -423, -422, -420, -418, -416, -414, -413, -411, -409, -407, -405, -403, -401, -399, -397,
    -395, -393, -391, -389, -387, -385, -383, -381, -379, -377, -375, -372, -370, -368, -366, -364,
    -362, -359, -357, -355, -353, -350, -348, -346, -343, -341, -339, -336, -334, -332, -329, -327,
    -324, -322, -319, -317, -314, -312, -310, -307, -304, -302, -299, -297, -294, -292, -289, -287,
    -284, -281, -279, -276, -273, -271, -268, -265, -263, -260, -257, -255, -252, -249, -246, -244,
    -241, -238, -235, -233, -230, -227, -224, -221, -218, -216, -213, -210, -207, -204, -201, -198,
    -195, -193, -190, -187, -184, -181, -178, -175, -172, -169, -166, -163, -160, -157, -154, -151,
    -148, -145, -142, -139, -136, -133, -130, -127, -124, -121, -118, -115, -112, -109, -106, -102,
    -99, -96, -93, -90, -87, -84, -81, -78, -75, -72, -68, -65, -62, -59, -56, -53,
    -50, -47, -43, -40, -37, -34, -31, -28, -25, -21, -18, -15, -12, -9, -6, -3
};

extern volatile uint32_t F_CPU_ACTUAL;

// GPIO Pin to analog channel mapping from Arduino\hardware\teensy\avr\cores\teensy4\analog.c
extern const uint8_t pin_to_channel[42];
uint32_t gpio_buf[MAX_SAMPLES*ADC_AVG];     // Store raw GPIO readings

// auto_calibration is default on
// visualize 3d is default off
EITKitArduino::EITKitArduino(){
  EITKitArduino(32,1,2, AD, AD, false);
}

EITKitArduino::EITKitArduino(int num_electrodes, int num_bands, int num_terminals, Meas_t drive_type, Meas_t meas_type, bool bluetooth_communication)
{
  if(serial_communication){
      Serial.begin(115200);
  }
  Serial.println("starting with ");
  
  Serial.println(SPI_FREQ_FAST);
  
  _num_electrodes = num_electrodes; // number of electrodes for measurement per band
  _num_meas = _num_electrodes*_num_electrodes;
  _num_bands = num_bands; // total number of bands used in measurement
  _num_terminals = num_terminals; // 2-terminal or 4-terminal measurement protocol
  _drive_type = drive_type; // protocol for electrodes used in excitation current
  _meas_type = meas_type; // protocol for electrodes used in voltage reading 
  _bluetooth_communication = bluetooth_communication;

  // Initialize CS on high and toggle low when communicating
  pinMode(CHIP_SEL_DRIVE, OUTPUT);
  pinMode(CHIP_SEL_MEAS, OUTPUT);
  pinMode(CHIP_SEL_AD5930, OUTPUT);
  digitalWrite(CHIP_SEL_DRIVE, HIGH);
  digitalWrite(CHIP_SEL_MEAS, HIGH);
  digitalWrite(CHIP_SEL_AD5930, HIGH);

  #if defined(ARDUINO_ARCH_ESP32)
  // Initialize HSPI
  hspi->setFrequency(SPI_FREQ_SLOW);
  hspi->begin();
  hspi->setHwCs(false);

  // Initialize VSPI along with IO expander
  vspi->setFrequency(SPI_FREQ_FAST);
  IOExpander.begin();
  vspi->setHwCs(false);

  // Set CS back to high in case HSPI initialization overwrote it
  pinMode(CHIP_SEL_MCP23S17, OUTPUT);
  digitalWrite(CHIP_SEL_MCP23S17, HIGH);

  // For the MUX connected via MC23S17 on port A
  IOExpander.pinMode(CHIP_SEL_MUX_SRC, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_SINK, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_VP, OUTPUT);
  IOExpander.pinMode(CHIP_SEL_MUX_VN, OUTPUT);
  IOExpander.digitalWrite(CHIP_SEL_MUX_SRC, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_VP, HIGH);
  IOExpander.digitalWrite(CHIP_SEL_MUX_VN, HIGH);
  
  // For the AD5930 signal generator, connected via MCP23S17 on port B
  IOExpander.pinMode(AD5930_INT_PIN, OUTPUT);
  IOExpander.pinMode(AD5930_CTRL_PIN, OUTPUT);
  IOExpander.pinMode(AD5930_STANDBY_PIN, OUTPUT);
  pinMode(AD5930_MSBOUT_PIN, INPUT);
  IOExpander.digitalWrite(AD5930_INT_PIN, LOW); 
  IOExpander.digitalWrite(AD5930_CTRL_PIN, LOW);
  IOExpander.digitalWrite(AD5930_STANDBY_PIN, LOW);

  // ADC input
  pinMode(ADC_BIT0, INPUT);
  pinMode(ADC_BIT1, INPUT);
  pinMode(ADC_BIT2, INPUT);
  pinMode(ADC_BIT3, INPUT);
  pinMode(ADC_BIT4, INPUT);
  pinMode(ADC_BIT5, INPUT);
  pinMode(ADC_BIT6, INPUT);
  pinMode(ADC_BIT7, INPUT);
  pinMode(ADC_BIT8, INPUT);
  pinMode(ADC_BIT9, INPUT);

  AD5930_Write(CTRL_REG, 0b011111110011);
  AD5930_Set_Start_Freq(TEST_FREQ);

  AD5270_LockUnlock(CHIP_SEL_DRIVE, 0);
  AD5270_LockUnlock(CHIP_SEL_MEAS, 0);

  /* Start the frequency sweep */
  IOExpander.digitalWrite(AD5930_CTRL_PIN, HIGH);
  delay(100);

  Serial.println("Board Initialized");

  // Start Bluetooth Initialization
  if(_bluetooth_communication){
      Serial.println("Starting BLE work!");
      BLEStart();            
      Serial.println("Characteristic defined! Now you can read it in your phone!");
  }

  #elif defined(__IMXRT1062__) // for Teensy 4.0
  // Teensy 4.0
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);

  
  pinMode(CHIP_SEL_MUX_SRC, OUTPUT);
  pinMode(CHIP_SEL_MUX_SINK, OUTPUT);
  pinMode(CHIP_SEL_MUX_VP, OUTPUT);
  pinMode(CHIP_SEL_MUX_VN, OUTPUT);


  pinMode(AD5930_INT_PIN, OUTPUT);
  pinMode(AD5930_CTRL_PIN, OUTPUT);
  pinMode(AD5930_STANDBY_PIN, OUTPUT);
  pinMode(AD5930_MSBOUT_PIN, INPUT);

  // ADC input
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);

 
  digitalWrite(CHIP_SEL_MUX_SRC, HIGH);
  digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
  digitalWrite(CHIP_SEL_MUX_VP, HIGH);
  digitalWrite(CHIP_SEL_MUX_VN, HIGH);
  
  digitalWrite(AD5930_INT_PIN, LOW);
  digitalWrite(AD5930_CTRL_PIN, LOW);
  digitalWrite(AD5930_STANDBY_PIN, LOW);

  AD5930_Write(CTRL_REG, 0b011111110011);
  AD5930_Set_Start_Freq(TEST_FREQ);

  AD5270_LockUnlock(CHIP_SEL_DRIVE, 0);
  AD5270_LockUnlock(CHIP_SEL_MEAS, 0);

  /* Start the frequency sweep */
  digitalWrite(AD5930_CTRL_PIN, HIGH);
  delay(100);

  // mux_write(CHIP_SEL_MUX_SRC, elec_to_mux[0], MUX_EN);
  // mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[1], MUX_EN);
  // mux_write(CHIP_SEL_MUX_VP, elec_to_mux[0], MUX_EN);
  // mux_write(CHIP_SEL_MUX_VN, elec_to_mux[1], MUX_EN);
  mux_write_to_electrode(SRC, 0, MUX_EN);
  mux_write_to_electrode(SINK, 1, MUX_EN);
  mux_write_to_electrode(VP, 0, MUX_EN);
  mux_write_to_electrode(VN, 1, MUX_EN);
  #endif 


  calibrateEIT();

  uint16_t i;

  /* Read resting impedance state for calibration */
  for (i = 0; i < 30; i++)
  {
    // read_frame(AD, AD, _signal_rms, _signal_phase, NUM_ELECTRODES);
    #if defined(ARDUINO_ARCH_ESP32)
    read_frame(_drive_type, _meas_type, _signal_rms, _num_electrodes);
    #elif defined(__IMXRT1062__) // for Teensy 4.0
    read_frame(AD, AD, _signal_rms, signal_mag, _signal_phase, _num_electrodes);
    #endif 

    uint16_t j;
    for (j = 0; j < _num_meas; j++)
    {
      if (_signal_rms[j] != 0)
        _cur_frame[j] = 0.80 * _cur_frame[j] + 0.20 * (_signal_rms[j]);
    }
  }

  Serial.println("origin frame");
  std::string cur_measurements = "origin";
  std::ostringstream streamObj;
  streamObj << std::fixed;
  // Set precision to 4 digits
  streamObj << std::setprecision(4);
  for (i = 0; i < _num_meas; i++)
  {
    // Serial.println(_cur_frame[i], 4);
    if (_cur_frame[i] != 0) {
        streamObj << _cur_frame[i];
        // streamObj << counter;
      }
  }
  cur_measurements.append(streamObj.str());
  measurements_to_send = cur_measurements;
  // Serial.println(cur_measurements.length());
  // Serial.println(cur_measurements.c_str());
  if(_bluetooth_communication){
    ensureBluetoothConnection();
  }
}

// Start Bluetooth Connection
void EITKitArduino::BLEStart() {
  #if defined(ARDUINO_ARCH_ESP32)
  Serial.println("starting blestart!");
  BLEDevice::init("EIT-kit's ESP32");
  Serial.println("starting creation server!");

  pServer = BLEDevice::createServer();
  Serial.println("created server!");

  pServer->setCallbacks(new MyServerCallbacks(&deviceConnected));
  Serial.println("created myservercallbacks!");
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  Serial.println("starting messagecallbacks!");
  pCharacteristic->setCallbacks(new MessageCallbacks(&measurements_to_send)); 
  pService->start();
  Serial.println("created messagecallbacks!");
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  #endif
}

void EITKitArduino::ensureBluetoothConnection(){
  #if defined(ARDUINO_ARCH_ESP32)
  if (deviceConnected) {
    //What is being sent to iPhone
    pCharacteristic->notify();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  #endif
}

void EITKitArduino::calibrateEIT(){
  Serial.println("Beginning Calibration");

  _phase_offset = 0;
  calibrate_samples();
  calibrate_gain(AD, AD);
  // removed without phase
  #if defined(__IMXRT1062__) // for Teensy 4.0
  calibrate_signal(AD, AD);
  #endif
  Serial.println("Calibrated new");

  // AD5270_Set(CHIP_SEL_DRIVE, 1023); // Junyi test setting
  // AD5270_Set(CHIP_SEL_MEAS, 1023);

  // Junyi uncommented
  mux_write_to_electrode(SRC, 0, MUX_EN);
  mux_write_to_electrode(SINK, 1, MUX_EN);
  mux_write_to_electrode(VP, 2, MUX_EN);
  mux_write_to_electrode(VN, 3, MUX_EN);

  Serial.print("Drive gain: ");
  Serial.println(_current_gain);
  Serial.print("Measurement gain: ");
  Serial.println(_voltage_gain);
  Serial.print("Sample rate (uS per reading): ");
  Serial.println(sample_rate, 4);
  Serial.print("Samples per period: ");
  Serial.println(samples_per_period);
  Serial.print("Reference signal phase offset (radians): ");
  Serial.println(_phase_offset, 4);
  
}

void EITKitArduino::take_measurements(Meas_t drive_type, Meas_t meas_type){
  uint16_t i;

  // phase removed
  // read_frame(drive_type, meas_type, _signal_rms, _signal_phase, _num_electrodes);
  #if defined(ARDUINO_ARCH_ESP32)
  read_frame(_drive_type, _meas_type, _signal_rms, _num_electrodes);
  #elif defined(__IMXRT1062__) // for Teensy 4.0
  read_frame(_drive_type, _meas_type, _signal_rms, signal_mag, _signal_phase, _num_electrodes);
  #endif 
  // read_frame_for_band(0,drive_type, meas_type, _signal_rms, _num_electrodes);

  // Serial.println("taking measurements");
  if (millis() - frame_delay > 500) {
    // Serial.println("framei");
    std::string cur_measurements = "framei";
    std::ostringstream streamObj;
    streamObj << std::fixed;
    // Set precision to 4 digits
    streamObj << std::setprecision(4);

    for (i = 0; i < _num_meas; i++) {
      if (_signal_rms[i] != 0) {
        _cur_frame[i] = 0.50 * _cur_frame[i] + 0.50 * (_signal_rms[i]);
      }
      // Serial.println(_cur_frame[i], 4);
      if (_cur_frame[i] != 0) {
        streamObj << _cur_frame[i];
        // streamObj << counter;
      }
    }
    cur_measurements.append(streamObj.str());
    measurements_to_send = cur_measurements;
    // Serial.println(cur_measurements.length());
    // Serial.println(cur_measurements.c_str());
    frame_delay = millis();
  }
  if(_bluetooth_communication){
    ensureBluetoothConnection();
  }
}

// public set/get methods 
// set the total number of electrodes for measurement per band
void EITKitArduino::set_num_electrodes(int num_electrodes){
  _num_electrodes = num_electrodes;
}

// get the total number of electrodes for measurement per band
int EITKitArduino::get_num_electrodes(){
  return _num_electrodes;
}

// set the total number of bands used in measurement
void EITKitArduino::set_num_bands(int num_bands){
  _num_bands = num_bands;
}

// get the total number of bands used in measurement
int EITKitArduino::get_num_bands(){
  return _num_bands;
}

// set the measurement protocol: 2-terminal or 4-terminal 
void EITKitArduino::set_num_terminals(int num_terminals){
  _num_terminals = num_terminals;
}

// get the measurement protocol: 2-terminal or 4-terminal 
int EITKitArduino::get_num_terminals(){
  return _num_terminals;
}

// set the protocol for electrodes used in voltage reading 
void EITKitArduino::set_meas_type(Meas_t meas_type){
  _meas_type = meas_type;
}

// get the protocol for electrodes used in voltage reading 
Meas_t EITKitArduino::get_meas_type(){
  return _meas_type;
}

// set the protocol for electrodes used in voltage reading 
void EITKitArduino::set_drive_type(Meas_t drive_type){
  _drive_type = drive_type;
}

// get the protocol for electrodes used in voltage reading 
Meas_t EITKitArduino::get_drive_type(){
  return _drive_type;
}

// set whether to create visualization in 3d 
void EITKitArduino::set_visualize_3d(bool visualize_3d){
  _visualize_3d = visualize_3d;
}

// get whether a visualization in 3d will be created
bool EITKitArduino::get_visualize_3d(){
  return _visualize_3d;
}

// set whether to use built-in calibration 
void EITKitArduino::set_auto_calibration(bool auto_calibration){
  _auto_calibration = auto_calibration;
}

// get whether to built-in calibration will be performed
bool EITKitArduino::get_auto_calibration(){
  return _auto_calibration;
}

/* set the AC current frequency
 default value is 50kHx */
void EITKitArduino::set_current_freq(uint16_t current_freq){
  _current_freq = current_freq;
}

// get the AC current frequency
uint16_t EITKitArduino::get_current_freq(){
  return _current_freq;
}

// set the AC current gain 
void EITKitArduino::set_current_gain(uint16_t current_gain){
  _current_gain = current_gain;
}

// get the AC current gain
uint16_t EITKitArduino::get_current_gain(){
  return _current_gain;
}

// set the voltage gain 
void EITKitArduino::set_voltage_gain(uint16_t voltage_gain){
  _voltage_gain = voltage_gain;
}

// get the voltage gain
uint16_t EITKitArduino::get_voltage_gain(){
  return _voltage_gain;
}

// get magnitude measurements
double* EITKitArduino::get_magnitude_array(){
  return _signal_rms;
}

// get phase measurements
double* EITKitArduino::get_phase_array(){
  return _signal_phase;
}

#if defined(ARDUINO_ARCH_ESP32)
// VSPI addresses devices through GPIO
void EITKitArduino::vspi_write_byte(const int chip_select, uint8_t data_to_send, const uint8_t bit_order, const uint8_t mode) {
  vspi->beginTransaction(SPISettings(SPI_FREQ_FAST, bit_order, mode));
  digitalWrite(chip_select, LOW);   //pull SS slow to prep other end for transfer
  vspi->transfer(data_to_send);
  digitalWrite(chip_select, HIGH);  //pull ss high to signify end of data transfer
  vspi->endTransaction();
}

void EITKitArduino::vspi_write_word(const int chip_select, uint16_t data_to_send, const uint8_t bit_order, const uint8_t mode) {
  vspi->beginTransaction(SPISettings(SPI_FREQ_FAST, bit_order, mode));
  digitalWrite(chip_select, LOW);   //pull SS slow to prep other end for transfer
  vspi->transfer16(data_to_send);
  digitalWrite(chip_select, HIGH);  //pull ss high to signify end of data transfer
  vspi->endTransaction();
}

// HSPI addresses devices through IO expander
void EITKitArduino::hspi_write_byte(const int chip_select, uint8_t data_to_send, const uint8_t bit_order, const uint8_t mode){
  hspi->beginTransaction(SPISettings(SPI_FREQ_SLOW, bit_order, mode));
  IOExpander.digitalWrite(chip_select, LOW);
  hspi->transfer(data_to_send);
  IOExpander.digitalWrite(chip_select, HIGH);
  hspi->endTransaction();
}

void EITKitArduino::hspi_write_word(const int chip_select, uint16_t data_to_send, const uint8_t bit_order, const uint8_t mode){
  hspi->beginTransaction(SPISettings(SPI_FREQ_SLOW, bit_order, mode));
  IOExpander.digitalWrite(chip_select, LOW);
  hspi->transfer16(data_to_send);
  IOExpander.digitalWrite(chip_select, HIGH);
  hspi->endTransaction();
}

/* Write a 4-bit command and a 10-bit data word */
void EITKitArduino::AD5270_Write(const int chip_select, uint8_t cmd, uint16_t data){
  uint16_t data_word = ((cmd & 0x0F) << 10) | (data & 0x03FF);
  vspi_write_word(chip_select, data_word, MSBFIRST, SPI_MODE1);
}

#elif defined(__IMXRT1062__) // for Teensy 4.0

/* Write a 4-bit command and a 10-bit data word */
void EITKitArduino::AD5270_Write(const int chip_sel, uint8_t cmd, uint16_t data)
{
    uint16_t data_word = ((cmd & 0x0F) << 10) | (data & 0x03FF);
  
    digitalWrite(chip_sel, LOW);
    delayMicroseconds(500);
    spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    delayMicroseconds(500);
    digitalWrite(chip_sel, HIGH);
}

/* Shift a byte out serially with the given frequency in Hz (<= 500kHz) */
void EITKitArduino::spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val){
    uint32_t period = (freq >= 500000) ? 1 : (500000 / freq);   // Half clock period in uS
    uint8_t cpol = (mode == SPI_MODE2 || mode == SPI_MODE3);
    uint8_t cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    uint8_t sck = cpol ? HIGH : LOW;

    uint8_t i;
    uint32_t start_time;

    // Set clock idle for 2 periods
    digitalWrite(clock_pin, sck);
    delayMicroseconds(period*4);

    for (i = 0; i < bits; i++)  {
        start_time = micros();

        // Shift bit out
        if (bit_order == LSBFIRST)
            digitalWrite(data_pin, !!(val & (1 << i)));
        else    
            digitalWrite(data_pin, !!(val & (1 << ((bits-1) - i))));

        // Toggle clock leading edge
        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while(micros() - start_time < period);
        } else {
            while(micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }

        // Toggle clock trailing edge
        start_time = micros();
        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while(micros() - start_time < period);
        } else {
            while(micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }
    }
}

uint16_t EITKitArduino::analog_read()
{
    // GPIO reg bit order: 2, 3, 16, 17, 18, 19, 22, 23, 24, 25, 26, 27
    // Teensy pin order:   1, 0, 19, 18, 14, 15, 17, 16, 22, 23, 20, 21
    // All pins are on GPIO6
    
    uint16_t gpio_reg = *(&GPIO6_DR + 2) >> 16;
    uint16_t val = ((gpio_reg & 0x0200) >> 9) | // Pin 23 (GPIO 25)
                   ((gpio_reg & 0x0100) >> 7) | // Pin 22 (GPIO 24)
                   ((gpio_reg & 0x0800) >> 9) | // Pin 21 (GPIO 27)
                   ((gpio_reg & 0x0400) >> 7) | // Pin 20 (GPIO 26)
                   ((gpio_reg & 0x0003) << 4) | // Pins 19,18 (GPIO 16,17)
                    (gpio_reg & 0x00C0) |       // Pins 17,16 (GPIO 22,23)
                   ((gpio_reg & 0x0008) << 5) | // Pin 15 (GPIO 19)
                   ((gpio_reg & 0x0004) << 7);  // Pin 14 (GPIO 18)
    return val;
}
// /* Return the time it takes to make num_samples measurements */
// uint32_t EITKitArduino::time_measurement(uint16_t num_samples)
// {   
//     uint16_t gpio_buf[num_samples][ADC_AVG];    // Store raw ADC samples of the input waveform
//     uint8_t ref_buf[num_samples];               // Store high-low values of the square output waveform
    
//     uint32_t time1, time2;
//     uint32_t count, num_cycles;
//     uint16_t i, j;
    
//     /* Collect samples */
//     time1 = micros();
//     for(i = 0; i < num_samples; i++)
//     { 
//         num_cycles = 20;
//         count = 0;

//         // Read GPIO pins
//         for (j = 0; j < ADC_AVG; j++)
//         {
//             while (ARM_DWT_CYCCNT - count < num_cycles);   // Wait set number of cycles since last count
//             count = ARM_DWT_CYCCNT;

//             gpio_buf[i][j] = gpio_read();
//         }
//         ref_buf[i] = digitalRead(AD5930_MSBOUT_PIN);
//     }
//     time2 = micros();

//     return (time2 - time1);
// }
#endif 

/* Enable/disable rheostat value changes */
void EITKitArduino::AD5270_LockUnlock(const int chip_select, uint8_t lock){
  AD5270_Write(chip_select, CMD_WR_CTRL, lock ? 0 : 0x002);
}

/* Enable/disable hardware shutdown */
void EITKitArduino::AD5270_Shutdown(const int chip_select, uint8_t shutdown){
  AD5270_Write(chip_select, CMD_SHTDN, shutdown ? 1 : 0);
}

/* Set the value of the digital rheostat - range is 0-0x3FF (0-100kOhm) */
void EITKitArduino::AD5270_Set(const int chip_select, uint16_t val)
{
  AD5270_Write(chip_select, CMD_WR_RDAC, val);
}

/* Write a 12-bit data word into a register. Register addresses are 4 bits */
void EITKitArduino::AD5930_Write(uint8_t reg, uint16_t data){
  uint16_t data_word = ((reg & 0x0F) << 12) | (data & 0x0FFF);

  #if defined(ARDUINO_ARCH_ESP32)
  vspi_write_word(CHIP_SEL_AD5930, data_word, MSBFIRST, SPI_MODE1);
  #elif defined(__IMXRT1062__) // for Teensy 4.0
  digitalWrite(CHIP_SEL_AD5930, LOW);
  spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
  digitalWrite(CHIP_SEL_AD5930, HIGH);
  #endif
}

/* Program the given frequency (in Hz) */
void EITKitArduino::AD5930_Set_Start_Freq(uint32_t freq){
  uint32_t scaled_freq = (freq * 1.0 / AD5930_CLK_FREQ) * 0x00FFFFFF;
  uint16_t freq_low = scaled_freq & 0x0FFF;
  uint16_t freq_high = (scaled_freq >> 12) & 0x0FFF;

  AD5930_Write(SFREQ_LOW_REG, freq_low);
  AD5930_Write(SFREQ_HIGH_REG, freq_high);
}

void EITKitArduino::mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable){
  #if defined(ARDUINO_ARCH_ESP32)
  uint8_t value;
  if (enable) {
      value = pin_sel & 0x1F;
   } else {
      value = 0xC0 | (pin_sel & 0x1F);
   }
   hspi_write_byte(chip_select, value, MSBFIRST, SPI_MODE1);
  
  #elif defined(__IMXRT1062__) // for Teensy 4.0
  digitalWrite(chip_select, LOW);
  if (enable)
      spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 8, pin_sel & 0x1F);
  else
      spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 8, 0xC0 | (pin_sel & 0x1F));
  digitalWrite(chip_select, HIGH);
  #endif
}

void EITKitArduino::mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable){
  if(electrode_sel<32){
    int cs_pin = 0;
    switch(chip_select){
      case SRC: 
        cs_pin = CHIP_SEL_MUX_SRC; break;
      case SINK: 
        cs_pin = CHIP_SEL_MUX_SINK; break;
      case VP: 
        cs_pin = CHIP_SEL_MUX_VP; break;
      case VN: 
        cs_pin = CHIP_SEL_MUX_VN; break;
    }
    mux_write(cs_pin, elec_to_mux[electrode_sel], enable);
  }else{
    #if defined(ARDUINO_ARCH_ESP32) // for Arduino ESP32

    int cs_pin = 0;
    switch(chip_select){
      case SRC: 
        cs_pin = CHIP_SEL_MUX_SRC_2; break;
      case SINK: 
        cs_pin = CHIP_SEL_MUX_SINK_2; break;
      case VP: 
        cs_pin = CHIP_SEL_MUX_VP_2; break;
      case VN: 
        cs_pin = CHIP_SEL_MUX_VN_2; break;
    }
    mux_write(cs_pin, elec_to_mux[electrode_sel], enable);
    #endif
  }
}

/* Read GPIO 0-31 (takes ~50.1ns) */
uint32_t EITKitArduino::gpio_read(){
  #if defined(ARDUINO_ARCH_ESP32)
  return REG_READ(GPIO_IN_REG);
  #elif defined(__IMXRT1062__) // for Teensy 4.0
  return (*(&GPIO6_DR + 2) >> 16);
  #endif
}

/* Convert GPIO reading to 10-bit unsigned integer */
uint16_t EITKitArduino::gpio_convert(uint32_t gpio_reg){
  #if defined(ARDUINO_ARCH_ESP32)
  const uint8_t bit_to_gpio[10] = { ADC_BIT0, ADC_BIT1, ADC_BIT2, ADC_BIT3, ADC_BIT4, ADC_BIT5, ADC_BIT6, ADC_BIT7, ADC_BIT8, ADC_BIT9 };
  uint16_t val = 0;
  
  for (int i = 0; i < 10; i++) {
    uint32_t gpio_val = gpio_reg & (1 << bit_to_gpio[i]);
    if (gpio_val) {
      uint32_t bit_val;
      if (bit_to_gpio[i] > i)
        bit_val = gpio_val >> (bit_to_gpio[i] - i);
      else
        bit_val = gpio_val << (i - bit_to_gpio[i]);
      val |= bit_val;
    }
  }
  #elif defined(__IMXRT1062__) // for Teensy 4.0
  uint16_t val = ((gpio_reg & 0x0200) >> 9) | // Pin 23 (GPIO 25)
                   ((gpio_reg & 0x0100) >> 7) | // Pin 22 (GPIO 24)
                   ((gpio_reg & 0x0800) >> 9) | // Pin 21 (GPIO 27)
                   ((gpio_reg & 0x0400) >> 7) | // Pin 20 (GPIO 26)
                   ((gpio_reg & 0x0003) << 4) | // Pins 19,18 (GPIO 17,16)
                    (gpio_reg & 0x00C0) |       // Pins 17,16 (GPIO 22,23)
                   ((gpio_reg & 0x0008) << 5) | // Pin 15 (GPIO 19)
                   ((gpio_reg & 0x0004) << 7);  // Pin 14 (GPIO 18)
  #endif
  return val;
}


/* Find the optimal number of samples to read the desired number of periods of the input signal */
void EITKitArduino::calibrate_samples(){

  /* Take many samples to determine sample rate */
  num_samples = MAX_SAMPLES;
  #if defined(ARDUINO_ARCH_ESP32)
  uint32_t sample_time = read_signal(NULL, NULL, 0);
  #elif defined(__IMXRT1062__) // for Teensy 4.0
  uint32_t sample_time = read_signal(NULL, NULL, NULL, NULL, 0);
  #endif
  /* Calculate sample rate and total number of samples */
  sample_rate = (float)sample_time / MAX_SAMPLES; // microsec to read each measurement ADV_AVG number of times
  samples_per_period = (1000000 / sample_rate) / TEST_FREQ; // [measurements read] / [current cycles]
  num_samples = samples_per_period * NUM_PERIODS; 
}

uint16_t EITKitArduino::sine_compare(uint16_t * signal, uint16_t pk_pk, uint16_t points_per_period, uint8_t num_periods){

  if (points_per_period == 0)
    return 0;

  uint16_t num_points = points_per_period * num_periods;

  uint16_t i;
  uint16_t point_error;
  uint32_t error_sum = 0;

  for (i = 0; i < num_points; i++) {
    // Scale sine wave to match input signal frequency and amplitude
    uint32_t ref_index = ((i * 1024) / points_per_period) % 1024;
    int32_t ref_point = (sine_table[ref_index] * pk_pk) / 1024;

    // Center input signal to 0
    int32_t signal_val = (int16_t)signal[i] - 512;

    point_error = abs(signal_val - ref_point);
    error_sum += point_error;
  }
  error_sum = error_sum / num_points;
  return error_sum;
}
#if defined(ARDUINO_ARCH_ESP32)
void EITKitArduino::read_frame(Meas_t drive_type, Meas_t meas_type, double * rms_array, uint8_t electrodes_per_band){
  for(int band = 0; band<_num_bands; band++){
    read_frame_for_band(band, drive_type, meas_type, rms_array, electrodes_per_band);
  }
}
void EITKitArduino::read_frame_for_band(uint8_t band, Meas_t drive_type, Meas_t meas_type, double * rms_array, uint8_t electrodes_per_band){
  int8_t tx_pair, rx_pair;
  uint8_t src_pin, sink_pin, vp_pin, vn_pin;
  uint16_t num_meas = 0;

  for (tx_pair = band*electrodes_per_band; tx_pair < electrodes_per_band*band+electrodes_per_band; tx_pair++)
  {
    // Serial.println("reading");
    switch (drive_type)
    {
      case AD:
        src_pin = tx_pair;
        sink_pin = (tx_pair + 1) % electrodes_per_band;
        break;
      case OP:
        src_pin = tx_pair;
        sink_pin = (tx_pair + electrodes_per_band / 2) % electrodes_per_band;
        break;
      case MONO:
        src_pin = tx_pair;
        //sink_pin = (tx_pair == 0 ? 31 : 0);
        sink_pin = 0;
        break;
    }
    mux_write_to_electrode(SRC, src_pin, MUX_EN);
    mux_write_to_electrode(SINK, sink_pin, MUX_EN);
    
    delayMicroseconds(200);

    for (rx_pair = band*electrodes_per_band; rx_pair < electrodes_per_band*band+electrodes_per_band; rx_pair++, num_meas++)
    {
      // For two terminal measurements, the excitation current and measurement comes from the same electrode pairs
      if(_num_terminals == 2){
        vp_pin = src_pin;
        vn_pin = sink_pin;
      }else{
        switch (meas_type)
        {
          case AD:
            vp_pin = rx_pair;
            vn_pin = (rx_pair + 1) % electrodes_per_band;
            break;
          case OP:
            vp_pin = rx_pair;
            vn_pin = (rx_pair + electrodes_per_band / 2) % electrodes_per_band;
            break;
          case MONO:
            vp_pin = rx_pair;
            vn_pin = sink_pin;
            break;
        }
      }
      if (meas_type == MONO)
      {
        if ((vp_pin == src_pin) || (vp_pin == vn_pin) || (src_pin == sink_pin))
        {
          rms_array[num_meas] = 0;
        }
        else
        {
          mux_write_to_electrode(VP, vp_pin, MUX_EN);
          mux_write_to_electrode(VN, vn_pin, MUX_EN);

          delayMicroseconds(100);

          read_signal(rms_array + num_meas, NULL, 0);
        }
      }
      else
      {
        if ((vp_pin == src_pin) || (vp_pin == sink_pin) || (vn_pin == src_pin) || (vn_pin == sink_pin))
        {
          rms_array[num_meas] = 0;
        }
        else
        { 
          mux_write_to_electrode(VP, vp_pin, MUX_EN);
          mux_write_to_electrode(VN, vn_pin, MUX_EN);

          delayMicroseconds(100);

          read_signal(rms_array + num_meas, NULL, 0);
        }
      }
    }
  }
}
uint32_t EITKitArduino::read_signal(double * rms, uint16_t * error_rate, uint8_t debug){
  uint16_t i, j;
  uint8_t adc_half_period_count = 0;
  uint16_t zero_cross_index = 0;

  uint32_t time1, time2;
  uint32_t count, num_cycles;
  uint32_t sample_sum, total_sum = 0;
  uint32_t phase_val;

  time1 = micros();

  /* Collect samples */
  for (i = 0; i < num_samples*ADC_AVG; i++)
  {
    gpio_buf[i] = gpio_read();
  }

  time2 = micros();

  /* Process samples */
  for (i = 0; i < num_samples; i++)
  {
    /* Extract integer from GPIO reading */
    for (j = 0, sample_sum = 0; j < ADC_AVG; j++)
      sample_sum += gpio_convert(gpio_buf[i*ADC_AVG+j]);    // Get 10-bit ADC value from raw GPIO value
    adc_buf[i] = sample_sum / ADC_AVG;

    /* Store product for RMS calculation */
    int16_t adc_val = (int16_t)adc_buf[i] - 512;
    total_sum += adc_val * adc_val;

    if (i > 0)
    {
      /* Signal at midpoint, entering peak or trough */
      if ((adc_buf[i] > 512 && adc_buf[i - 1] <= 512) || (adc_buf[i] < 512 && adc_buf[i - 1] >= 512))
      {
        adc_half_period_count++;

        /* Record index of first rising zero point */
        if (adc_buf[i] > 512 && zero_cross_index == 0)
          zero_cross_index = i;
      }
    }
  }

  /* Calculate peak-to-peak magnitude and RMS */
  uint16_t rms_10bit = sqrt(total_sum / num_samples);
  uint16_t pk_pk_10bit = rms_10bit * sqrt(2) * 2;
  double rms_result = (double)rms_10bit * 2.2 / 1024;

  if (rms)
    *rms = rms_result;

  if (error_rate)
  {
    // Compare measured signal to sine wave (only if a whole period of samples is available)
    uint8_t compare_periods = 2;
    if ((num_samples - zero_cross_index) >= (samples_per_period * compare_periods))
      *error_rate = sine_compare(adc_buf + zero_cross_index, pk_pk_10bit, samples_per_period, compare_periods);
  }

  if (debug)
  {  
    for (int i = 0; i < num_samples; i++){
      Serial.println(adc_buf[i]);
    }
  }
  return (time2 - time1);
}

/* Find the gains that produce the highest sinusoidal current and voltage measurements */
void EITKitArduino::calibrate_gain(Meas_t drive_type, Meas_t meas_type){
  uint16_t i, j, k;
  uint16_t gain;
  uint32_t error_sum;
  double rms_sum;
  uint16_t min_current_gain = 0;    // 0 is highest gain, 1023 is lowest
  uint16_t min_voltage_gain = 0;

  // Calibrate current gain
  for (i = 0; i < _num_electrodes; i++) {
    Serial.print(".");
    // Set current source electrodes to origin, set voltage measurement electrodes to overlap
    mux_write_to_electrode(SRC, i, MUX_EN);
    mux_write_to_electrode(VP, i, MUX_EN);
    mux_write_to_electrode(SINK, (i+1)%MAX_ELECTRODES, MUX_EN);
    mux_write_to_electrode(VN, (i+1)%MAX_ELECTRODES, MUX_EN);
    // TODO Jackson?: Add check for drive and measurement type
    delay(1);

    // Set voltage measurement gain to 1 (maximum current (5V pk-pk) must be within ADC range)
    AD5270_Shutdown(CHIP_SEL_MEAS, 1);
  
    // Calibrate current source
    for (j = 0; j < 1024; j++) {
      gain = j;
      AD5270_Set(CHIP_SEL_DRIVE, gain);
      delayMicroseconds(50);
    
      double rms;
      uint16_t error;
      rms_sum = 0;
      error_sum = 0;
      for (k = 0; k < 10; k++) {
        read_signal(&rms, &error, 0);
        rms_sum += rms;
        error_sum += error;
      }
      rms_sum = rms_sum / 10;
      error_sum = error_sum / 10;
    
      // Accept the highest gain (lowest value) such that the reading is a valid sinusoid
      if (rms_sum > 0.2 && rms_sum < 1.5 && error_sum < 20)
        break;
    }
    if (gain > min_current_gain){
      min_current_gain = gain;
    }
  }
  _current_gain = min_current_gain;
  AD5270_Set(CHIP_SEL_DRIVE, _current_gain);
  Serial.println();
  Serial.println("done with current gain");
  // Calibrate voltage gain
  for (i = 0; i < _num_electrodes; i++) {
    Serial.print(".");
    // Set voltage measurement electrodes to the highest voltage differential point
    mux_write_to_electrode(SRC, i, MUX_EN);
    mux_write_to_electrode(SINK, (i+1)%MAX_ELECTRODES, MUX_EN);
    mux_write_to_electrode(VP, (i+2)%MAX_ELECTRODES, MUX_EN);
    mux_write_to_electrode(VN, (i+3)%MAX_ELECTRODES, MUX_EN);
    // TODO Jackson?: Add check for drive and measurement type
    
    delay(1);
    AD5270_Shutdown(CHIP_SEL_MEAS, 0);
    
    // Calibrate voltage measurement
    for (j = 0; j < 1024; j++) {
      gain = j;
      AD5270_Set(CHIP_SEL_MEAS, gain);
      delayMicroseconds(50);
    
      double rms;
      uint16_t error;
      rms_sum = 0;
      error_sum = 0;
      for (k = 0; k < 10; k++) {
        read_signal(&rms, &error, 0);
        rms_sum += rms;
        error_sum += error;
      }
      rms_sum = rms_sum / 10;
      error_sum = error_sum / 10;
    
      // Accept the highest gain (lowest value) such that reading is a valid sinusoid
      if (rms_sum > 0.2 && rms_sum < 1.5 && error_sum < 30)
        break;
    }
    if (gain > min_voltage_gain)
      min_voltage_gain = gain;
  }
  _voltage_gain = min_voltage_gain;
  AD5270_Set(CHIP_SEL_MEAS, _voltage_gain);
  Serial.println();
  
  mux_write_to_electrode(SRC, 0, MUX_DIS);
  mux_write_to_electrode(SINK, 0, MUX_DIS);
  mux_write_to_electrode(VP, 0, MUX_DIS);
  mux_write_to_electrode(VN, 0, MUX_DIS);
  Serial.println("done calibrate gain");
}
#elif defined(__IMXRT1062__) // for Teensy 4.0
void EITKitArduino::read_frame(Meas_t drive_type, Meas_t meas_type, double * rms_array, double * mag_array, double * phase_array, uint8_t num_elec)
{
  int8_t tx_pair, rx_pair;
  uint8_t src_pin, sink_pin, vp_pin, vn_pin;
  uint16_t num_meas = 0;

  for(tx_pair = 0; tx_pair < num_elec; tx_pair++)
  {
      switch (drive_type)
      {
          case AD:
              src_pin = tx_pair;
              sink_pin = (tx_pair + 1) % num_elec;
              break;
          case OP:
              src_pin = tx_pair;
              sink_pin = (tx_pair + num_elec/2) % num_elec;
              break;
          case MONO:
              src_pin = tx_pair;
              //sink_pin = (tx_pair == 0 ? 31 : 0);
              sink_pin = 0;
              break;
      }
      mux_write_to_electrode(SRC, src_pin, MUX_EN);
      mux_write_to_electrode(SINK, sink_pin, MUX_EN);

      delayMicroseconds(200);

      for(rx_pair = 0; rx_pair < num_elec; rx_pair++, num_meas++)
      {
          switch (meas_type)
          {
              case AD:
                  vp_pin = rx_pair;
                  vn_pin = (rx_pair + 1) % num_elec;
                  break;
              case OP:
                  vp_pin = rx_pair;
                  vn_pin = (rx_pair + num_elec/2) % num_elec;
                  break;
              case MONO:
                  vp_pin = rx_pair;
                  vn_pin = sink_pin;
                  break;
          }

          if (meas_type == MONO)
          {
              if ((vp_pin == src_pin) || (vp_pin == vn_pin) || (src_pin == sink_pin))
              {
                  mag_array[num_meas] = 0;
                  phase_array[num_meas] = 0;
              }
              else 
              {
                  // mux_write(CHIP_SEL_MUX_VP, elec_to_mux[vp_pin], MUX_EN);
                  // mux_write(CHIP_SEL_MUX_VN, elec_to_mux[vn_pin], MUX_EN);
                  mux_write_to_electrode(VP, vp_pin, MUX_EN);
                  mux_write_to_electrode(VN, vn_pin, MUX_EN);

                  delayMicroseconds(100);
      
                  read_signal(rms_array + num_meas, mag_array + num_meas, phase_array + num_meas, NULL, 0);
              }
          }
          else
          {
              if ((vp_pin == src_pin) || (vp_pin == sink_pin) || (vn_pin == src_pin) || (vn_pin == sink_pin))
              {
                  mag_array[num_meas] = 0;
                  phase_array[num_meas] = 0;
              } 
              else 
              {
                  // mux_write(CHIP_SEL_MUX_VP, elec_to_mux[vp_pin], MUX_EN);
                  // mux_write(CHIP_SEL_MUX_VN, elec_to_mux[vn_pin], MUX_EN);
                  mux_write_to_electrode(VP, vp_pin, MUX_EN);
                  mux_write_to_electrode(VN, vn_pin, MUX_EN);
                  
                  delayMicroseconds(100);

                  read_signal(rms_array + num_meas, mag_array + num_meas, phase_array + num_meas, NULL, 0);
              }
          }
      }
  }

}
/* Return the magnitude and phase offset of a sinusoidal input signal */
uint32_t EITKitArduino::read_signal(double * rms, double * mag, double * phase, uint16_t * error_rate, uint8_t debug)
{ 
    uint16_t i, j;
    uint16_t phase_count;
    uint16_t adc_min = 1023;
    uint16_t adc_max = 0;
    uint8_t adc_peak_count = 0;
    uint8_t adc_trough_count = 0;
    uint8_t ref_period_count = 0;
    uint8_t adc_period_count = 0;
    uint8_t phase_readings = 0;
    uint16_t phase_start_index = 0;

    uint16_t gpio_buf[num_samples][ADC_AVG];    // Store raw ADC samples of the input waveform
    uint16_t adc_buf[num_samples];              // Store converted ADC samples of the input waveform
    uint8_t ref_buf[num_samples];               // Store high-low values of the square output waveform
    uint16_t adc_peaks[num_samples];
    uint16_t adc_troughs[num_samples];
    uint16_t phase_cycles[num_samples];
    
    uint32_t time1, time2;
    uint32_t count, num_cycles;
    uint32_t sample_sum, total_sum = 0;
    
    time1 = micros();

    /* Collect samples */
    for(i = 0; i < num_samples; i++)
    { 
        //num_cycles = ((F_CPU_ACTUAL >> 16) * 50) / (1000000000UL >> 16);   // Number of systick cycles equal to 50ns
        num_cycles = 20;
        count = 0;

        // Read GPIO pins
        for (j = 0; j < ADC_AVG; j++)
        {
            while (ARM_DWT_CYCCNT - count < num_cycles);   // Wait set number of cycles since last count
            count = ARM_DWT_CYCCNT;

            gpio_buf[i][j] = gpio_read();
        }
        ref_buf[i] = digitalRead(AD5930_MSBOUT_PIN);
    }

    time2 = micros();

    /* Process samples */
    for(i = 0; i < num_samples; i++)
    {       
        for (j = 0, sample_sum = 0; j < ADC_AVG; j++)
            sample_sum += gpio_convert(gpio_buf[i][j]);    // Get 10-bit ADC value from raw GPIO value
        adc_buf[i] = sample_sum / ADC_AVG;

        /* Store product for RMS calculation */
        int16_t adc_val = (int16_t)adc_buf[i] - 512;
        total_sum += adc_val * adc_val;

        /* Store local max/min */
        if (adc_buf[i] > adc_max)
            adc_max = adc_buf[i];
        if (adc_buf[i] < adc_min)
            adc_min = adc_buf[i];

        if (i > 0)
        {
            /* Signal increasing, entering peak */
            if (adc_buf[i] > 512 && adc_buf[i-1] <= 512)
            {
                /* Ensure that a full half-cycle has been measured */
                if (adc_period_count > 0)
                {
                    adc_troughs[adc_trough_count] = adc_min;
                    adc_trough_count++;
                    adc_min = 1023;

                    /* Discard large phase offsets as error */
                    if (phase_count <= samples_per_period)
                    {
                        phase_cycles[phase_readings] = phase_count;
                        phase_readings++;
                    }
                }
                adc_period_count++;

                /* Record index of first rising zero point */
                if (phase_start_index == 0)
                    phase_start_index = i;
            }

            /* Signal decreasing, entering trough */
            else if (adc_buf[i] < 512 && adc_buf[i-1] >= 512)
            {
                if (adc_period_count > 0)
                {
                    adc_peaks[adc_peak_count] = adc_max;
                    adc_peak_count++;
                    adc_max = 0;

                    /* Discard large phase offsets as error */
                    if (phase_count <= samples_per_period)
                    {
                        phase_cycles[phase_readings] = phase_count;
                        phase_readings++;
                    }
                }
                adc_period_count++;
            }

            phase_count++;
            
            /* Reference signal transition */
            if ((ref_buf[i] && !ref_buf[i-1]) || (!ref_buf[i] && ref_buf[i-1]))
            {
                ref_period_count++;
                phase_count = 0;
            }
        }
    }

    /* Calculate average peaks and troughs */
    for (i = 0, sample_sum =  0; i < adc_peak_count; i++)
        sample_sum += adc_peaks[i];
    adc_max = sample_sum / adc_peak_count;
    for (i = 0, sample_sum = 0; i < adc_trough_count; i++)
        sample_sum += adc_troughs[i];
    adc_min = sample_sum / adc_trough_count;

//    for (i = 0, sample_sum =  0; i < NUM_PERIODS; i++)
//        sample_sum += adc_peaks[i];
//    adc_max = sample_sum / NUM_PERIODS;
//    for (i = 0, sample_sum = 0; i < NUM_PERIODS; i++)
//        sample_sum += adc_troughs[i];
//    adc_min = sample_sum / NUM_PERIODS;

    /* Calculate phase offset */
    int16_t phase_offset_cycles;
    for (i = 0, sample_sum = 0; i < phase_readings; i++)
        sample_sum += phase_cycles[i];
    phase_offset_cycles = sample_sum / phase_readings;

    /* Calculate peak-to-peak magnitude and RMS */
    uint16_t mag_10bit = adc_max - adc_min;
    uint16_t rms_10bit = sqrt(total_sum / num_samples);

    if (rms)
        *rms = (double)rms_10bit * 2.2 / 1024;
    if (mag)
        *mag = (double)mag_10bit * 2.2 / 1024;                                                      
    if (phase)
        *phase = (sample_rate * phase_offset_cycles / 1000000) * TEST_FREQ * 2*PI;   

    if (error_rate)
    {
        // Compare measured signal to sine wave (only if >=2 period samples are available)
        uint16_t compare_periods = 2;
        if ((num_samples - phase_start_index) >= (samples_per_period * compare_periods))
            *error_rate = sine_compare(adc_buf+phase_start_index, mag_10bit, samples_per_period, compare_periods);
    }

    return (time2 - time1);
}
/* Find the gains that produce the highest sinusoidal current and voltage measurements */
void EITKitArduino::calibrate_gain(Meas_t drive_type, Meas_t meas_type)
{
    // Set current source electrodes to origin, set voltage measurement electrodes to overlap
    mux_write_to_electrode(SRC, 0, MUX_EN);
    mux_write_to_electrode(VP, 0, MUX_EN);
    if (drive_type == AD) {
        mux_write_to_electrode(SINK, 1, MUX_EN);
        mux_write_to_electrode(VN, 1, MUX_EN);
        // mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[1], MUX_EN);
        // mux_write(CHIP_SEL_MUX_VN, elec_to_mux[1], MUX_EN);
    } else if (drive_type == OP) {
        mux_write_to_electrode(SINK, 16, MUX_EN);
        mux_write_to_electrode(VN, 16, MUX_EN);
        // mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[16], MUX_EN);
        // mux_write(CHIP_SEL_MUX_VN, elec_to_mux[16], MUX_EN);
    }

    delay(5);

    // Set voltage measurement gain to 1 (maximum current (5V pk-pk) must be within ADC range)
    AD5270_Shutdown(CHIP_SEL_MEAS, 1);

    uint16_t i, j;
    uint32_t error_sum;
    double mag_sum;

    // Calibrate current source
    for (i = 0; i < 1024; i++) {
        _current_gain = i;
        AD5270_Set(CHIP_SEL_DRIVE, _current_gain);
        delayMicroseconds(500);

        double mag;
        uint16_t error;
        mag_sum = 0;
        error_sum = 0;
        for (j = 0; j < 10; j++) {
            read_signal(NULL, &mag, NULL, &error, 0);
            mag_sum += mag;
            error_sum += error;
        }
        mag_sum = mag_sum / 10;
        error_sum = error_sum / 10;

        // Accept the highest gain such that reading a valid sinusoid
        if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 30)
            break;
    }

    // Set voltage measurement electrodes to the highest voltage differential point
    if (meas_type == AD) {
      mux_write_to_electrode(VP, 2, MUX_EN);
      mux_write_to_electrode(VN, 3, MUX_EN);
        // mux_write(CHIP_SEL_MUX_VP, elec_to_mux[2], MUX_EN);
        // mux_write(CHIP_SEL_MUX_VN, elec_to_mux[3], MUX_EN);
    } else if (meas_type == OP) {
      mux_write_to_electrode(VP, 1, MUX_EN);
      mux_write_to_electrode(VN, 17, MUX_EN);
        // mux_write(CHIP_SEL_MUX_VP, elec_to_mux[1], MUX_EN);
        // mux_write(CHIP_SEL_MUX_VN, elec_to_mux[17], MUX_EN);
    }

    delay(5);

    AD5270_Shutdown(CHIP_SEL_MEAS, 0);

    // Calibrate voltage measurement
    for (i = 0; i < 1024; i++) {
        _voltage_gain = i;
        AD5270_Set(CHIP_SEL_MEAS, _voltage_gain);
        delayMicroseconds(500);

        double mag;
        uint16_t error;
        mag_sum = 0;
        error_sum = 0;
        for (j = 0; j < 10; j++) {
            read_signal(NULL, &mag, NULL, &error, 0);
            mag_sum += mag;
            error_sum += error;
        }
        mag_sum = mag_sum / 10;
        error_sum = error_sum / 10;

        // Accept the highest gain such that reading a valid sinusoid
        if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 30)
            break;
    }
    mux_write_to_electrode(SRC, 0, MUX_DIS);  
    mux_write_to_electrode(SINK, 0, MUX_DIS);
    mux_write_to_electrode(VP, 0, MUX_DIS);
    mux_write_to_electrode(VN, 0, MUX_DIS);
    // mux_write(CHIP_SEL_MUX_SRC, 0, MUX_DIS);
    // mux_write(CHIP_SEL_MUX_SINK, 0, MUX_DIS);
    // mux_write(CHIP_SEL_MUX_VP, 0, MUX_DIS);
    // mux_write(CHIP_SEL_MUX_VN, 0, MUX_DIS);
}
#endif



#if defined(ARDUINO_ARCH_ESP32)
EITKitArduino::MyServerCallbacks::MyServerCallbacks(bool *_bptr) {
    connection_ptr = _bptr;
    BLEServerCallbacks();
}

void EITKitArduino::MyServerCallbacks::onConnect(BLEServer* pServer) {
    assert(connection_ptr && "connection_ptr not sent in MyServerCallbacks::onConnect()");
    *connection_ptr = true;
}

void EITKitArduino::MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    assert(connection_ptr && "connection_ptr not sent in MyServerCallbacks::onDisconnect()");
    *connection_ptr = false;
}

EITKitArduino::MessageCallbacks::MessageCallbacks(std::string *str_ptr) {
    _str_ptr = str_ptr;
    BLECharacteristicCallbacks();
}

void EITKitArduino::MessageCallbacks::onWrite(BLECharacteristic *characteristic) {
    // assert(_str_ptr && "connection_ptr not sent in MyServerCallbacks::onConnect()");
    // Serial.println("onWrite start");
    char reading[511];
    memset(reading, 0, 511);
    int j = 0;
    if (op * 510 < _str_ptr->length()) {
      for (int i = 510 * op; i < (510 * op) + 510; i++) {
        if(i<_str_ptr->length()){
          reading[j] = _str_ptr->at(i);
          j++;
        }else{
          break;
        }
      }
      pCharacteristic->setValue(reading);
      pCharacteristic->notify();
      // Serial.println(reading);
      // op++;
      if(_str_ptr->length()<511){
        *_str_ptr = "";
      }else{
        *_str_ptr = _str_ptr->substr(510);
      }
    }
    else {
      op = 0;
    }
}

void EITKitArduino::MessageCallbacks::onRead(BLECharacteristic *characteristic) {
}
#endif

/* Find the magnitude and phase offset of the highest voltage differental point */
void EITKitArduino::calibrate_signal(Meas_t drive_type, Meas_t meas_type){

  // Set current source electrodes to origin
  mux_write_to_electrode(SRC, 0, MUX_EN);
  if (drive_type == AD)
    mux_write_to_electrode(SINK, 1, MUX_EN);
  else if (drive_type == OP)
    mux_write_to_electrode(SINK, 16, MUX_EN);

  // Set voltage measurement electrodes to the highest voltage differential point
  if (meas_type == AD) {
    mux_write_to_electrode(VP, 30, MUX_EN);
    mux_write_to_electrode(VN, 31, MUX_EN);
  } else if (meas_type == OP) {
    mux_write_to_electrode(VP, 15, MUX_EN);
    mux_write_to_electrode(VN, 31, MUX_EN);
  }

  delay(5);

  /* Determine the phase offset of the reference signal */
  _phase_offset = 0;
  // uint32_t sample_time = read_signal(NULL, &_phase_offset, NULL, 0);

  mux_write_to_electrode(SRC, 0, MUX_DIS);
  mux_write_to_electrode(SINK, 0, MUX_DIS);
  mux_write_to_electrode(VP, 0, MUX_DIS);
  mux_write_to_electrode(VN, 0, MUX_DIS);
}