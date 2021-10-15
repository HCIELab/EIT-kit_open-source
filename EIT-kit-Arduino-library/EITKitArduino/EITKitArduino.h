/*
  EITKitArduino.h - Library for EIT-kit Sensing Board.
  Will be released into the public domain.
*/
#ifndef EITKitArduino_h
#define EITKitArduino_h


#if ARDUINO>=100
   #include <Arduino.h>  // Arduino 1.0
 #else
   #include <WProgram.h>  // Arduino 0022
 #endif

#include "SPI.h"

#define MUX_EN             1
#define MUX_DIS            0
#define NUM_ELECTRODES     32
#define NUM_MEAS           NUM_ELECTRODES*NUM_ELECTRODES //used only for instantiation of measurement arrays
#define MAX_ELECTRODES     64 // maximum electrodes that can be used

// AD5930 register addresses - signal generator
#define CTRL_REG           0x00
#define NUM_INCR_REG       0x01
#define DFREQ_LOW_REG      0x02
#define DFREQ_HIGH_REG     0x03
#define TIME_INCR_REG      0x04
#define TIME_BURST_REG     0x08
#define SFREQ_LOW_REG      0x0C
#define SFREQ_HIGH_REG     0x0D

typedef enum { AD, OP, MONO } Meas_t;
typedef enum { SRC, SINK, VP, VN } Mux_t;

#if defined(ARDUINO_ARCH_ESP32) // for Arduino ESP32
// defined constants 

#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "MCP23S17.h"


#define IS_ARDUINO        1
// SPI 
#define SPI_FREQ_FAST      4000000
#define SPI_FREQ_SLOW      500000
#define HSPI_MOSI_PIN      13
#define HSPI_SCK_PIN       14
#define VSPI_MOSI_PIN      23
#define VSPI_SCK_PIN       18

#define AD5930_CLK_FREQ    50000000
#define TEST_FREQ          50000
#define NUM_PERIODS        4        // Number of signal periods to measure
#define ADC_AVG            5        // Number of ADC samples to average for each analog reading

#define MAX_SAMPLES        2000     // Max number of samples allowed when reading signal (arbitrary, more samples uses more memory)
        
// AD5270 commands - new digital potentiometer
#define CMD_WR_RDAC        0x01
#define CMD_RD_RDAC        0x02
#define CMD_ST_RDAC        0x03
#define CMD_RST            0x04
#define CMD_RD_MEM         0x05
#define CMD_RD_ADDR        0x06
#define CMD_WR_CTRL        0x07
#define CMD_RD_CTRL        0x08
#define CMD_SHTDN          0x09

/* GPIO pin mappings */

#define CHIP_SEL_AD5930    25   // Chip select pin for AD5930: Signal generator
#define CHIP_SEL_MCP23S17  12   // chip select for MCP23517: IO Expander
#define CHIP_SEL_DRIVE     32   // Chip select pin for driving digital rheostat potentiometer
#define CHIP_SEL_MEAS      33   // Chip select pin for measuring digital rheostat potentiometer
#define CHIP_SEL_RHEO       // For Teensy: Chip select pin for digital dual rheostat


#define AD5930_MSBOUT_PIN  35

// Map ADC pins to GPIO pins
#define ADC_BIT0   27 // LSb
#define ADC_BIT1   26
#define ADC_BIT2   15
#define ADC_BIT3   4
#define ADC_BIT4   16
#define ADC_BIT5   17
#define ADC_BIT6   5
#define ADC_BIT7   19
#define ADC_BIT8   21
#define ADC_BIT9   22

/* IO expander pin mappings */

/* Chip Select for MUX for electrodes 0-31 */
#define CHIP_SEL_MUX_SRC   7    // Chip select pin for source electrodes MUX //top2
#define CHIP_SEL_MUX_SINK  6    // Chip select pin for sink electrodes MUX
#define CHIP_SEL_MUX_VP    5    // Chip select for voltage measurement positive electrodes MUX //bottom2
#define CHIP_SEL_MUX_VN    9    // Chip select for voltage measurement negative electrodes MUX

/* Chip Select for MUX for electrodes 32-63 */
#define CHIP_SEL_MUX_SRC_2  0    // Chip select pin for source electrodes MUX //top2
#define CHIP_SEL_MUX_SINK_2  1    // Chip select pin for sink electrodes MUX
#define CHIP_SEL_MUX_VP_2    2    // Chip select for voltage measurement positive electrodes MUX //bottom2
#define CHIP_SEL_MUX_VN_2    3    // Chip select for voltage measurement negative electrodes MUX

#define AD5930_INT_PIN     12  // Pulse high to reset internal state machine
#define AD5930_CTRL_PIN    11  // Pull high to start frequency sweep. Pull low to end the burst. Pull high again to increment frequency
#define AD5930_STANDBY_PIN 10  // Pull high to power down

// Define Bluetooth UUID
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"      

#elif defined(__IMXRT1062__) // for Teensy 4.0

#include <string>

#define IS_ARDUINO        0
// defined constants 
#define SPI_FREQ_FAST      500000
#define SPI_FREQ_SLOW      500000
#define HSPI_MOSI_PIN      
#define HSPI_SCK_PIN       
#define VSPI_MOSI_PIN      
#define VSPI_SCK_PIN       

#define MOSI_PIN           5
#define SCK_PIN            4

#define AD5930_CLK_FREQ    50000000
#define TEST_FREQ          40000
#define NUM_PERIODS        5        // Number of signal periods to measure
#define ADC_AVG            7        // Number of ADC samples to average for each analog reading

#define MAX_SAMPLES        2000     // Max number of samples allowed when reading signal (arbitrary, more samples uses more memory)

// AD5270 commands - new digital potentiometer
#define CMD_WR_RDAC        0x01
#define CMD_RD_RDAC        0x02
#define CMD_ST_RDAC        0x03
#define CMD_RST            0x04
#define CMD_RD_MEM         0x05
#define CMD_RD_ADDR        0x06
#define CMD_WR_CTRL        0x07
#define CMD_RD_CTRL        0x08
#define CMD_SHTDN          0x09

#define CHIP_SEL_AD5930    3  // Chip select pin for AD5930
#define CHIP_SEL_DRIVE     9  // Chip select pin for driving digital rheostat
#define CHIP_SEL_MEAS      13 // Chip select pin for measuring digital rheostat
#define CHIP_SEL_MUX_SRC   24 // Chip select pin for source electrodes MUX
#define CHIP_SEL_MUX_SINK  25 // Chip select pin for sink electrodes MUX
#define CHIP_SEL_MUX_VP    26 // Chip select for voltage measurement positive electrodes MUX
#define CHIP_SEL_MUX_VN    27 // Chip select for voltage measurement negative electrodes MUX

#define AD5930_MSBOUT_PIN  8
#define AD5930_INT_PIN     7  // Pulse high to reset internal state machine
#define AD5930_CTRL_PIN    6  // Pull high to start frequency sweep. Pull low to end the burst. Pull high again to increment frequency
#define AD5930_STANDBY_PIN 2  // Pull high to power down 

/* GPIO pin mappings */
#define CHIP_SEL_MCP23S17     // chip select for MCP23517: IO Expander

// Map ADC pins to GPIO pins
#define ADC_BIT0    // LSb
#define ADC_BIT1   
#define ADC_BIT2   
#define ADC_BIT3   
#define ADC_BIT4   
#define ADC_BIT5   
#define ADC_BIT6   
#define ADC_BIT7   
#define ADC_BIT8   
#define ADC_BIT9   

/* Chip Select for MUX for electrodes 32-63 */
#define CHIP_SEL_MUX_SRC_2       // Chip select pin for source electrodes MUX //top2
#define CHIP_SEL_MUX_SINK_2      // Chip select pin for sink electrodes MUX
#define CHIP_SEL_MUX_VP_2        // Chip select for voltage measurement positive electrodes MUX //bottom2
#define CHIP_SEL_MUX_VN_2        // Chip select for voltage measurement negative electrodes MUX

// Define Bluetooth UUID
#define SERVICE_UUID        
#define CHARACTERISTIC_UUID 

#endif

class EITKitArduino
{
  public:
    EITKitArduino();
    EITKitArduino(int num_electrodes, int num_bands, int num_terminals, Meas_t drive_type, Meas_t meas_type, bool bluetooth_communication);
    void ensureBluetoothConnection();
    void take_measurements(Meas_t drive_type, Meas_t meas_type); 
    void set_num_electrodes(int num_electrodes);
    int get_num_electrodes();
    void set_num_bands(int num_bands);
    int get_num_bands();
    void set_num_terminals(int num_terminals);
    int get_num_terminals();
    void set_meas_type(Meas_t meas_type);
    Meas_t get_meas_type();
    void set_drive_type(Meas_t drive_type);
    Meas_t get_drive_type();
    void set_visualize_3d(bool visualize_3d);
    bool get_visualize_3d();
    void set_auto_calibration(bool auto_calibration);
    bool get_auto_calibration();
    void set_current_freq(uint16_t current_freq);
    uint16_t get_current_freq();
    void set_current_gain(uint16_t current_gain);
    uint16_t get_current_gain();
    void set_voltage_gain(uint16_t voltage_gain);
    uint16_t get_voltage_gain();
    double* get_magnitude_array();
    double* get_phase_array();
 
  private:
    // Bluetooth Variables
    std::string measurements_to_send = "";
    // Mapping of electrode number (input) to MUX channel (output)
    // const uint8_t elec_to_mux[MAX_ELECTRODES] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16};
    const uint8_t elec_to_mux[MAX_ELECTRODES] = { 9, 10, 11, 8, 7, 6, 5, 4, 3, 2, 1, 0, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16};

    // Global calibration parameters
    float sample_rate;
    uint16_t samples_per_period;
    uint16_t num_samples = 1;
    double ref_signal_mag;
    // Buffers for use in read_signal
    uint16_t adc_buf[MAX_SAMPLES];              // Store converted ADC samples of the input waveform
    // Temporary test values
    uint8_t pin_num = 0;
    uint16_t rheo_val = 1023;

    // Measurement Settings
    int _num_electrodes = 8; // total number of electrodes for measurement per band
    int _num_meas = _num_electrodes*_num_electrodes; // total number of electrodes for measurement per band
    int _num_bands = 1; // total number of bands used in measurement
    int _num_terminals = 4; // 2-terminal or 4-terminal measurement protocol, 4 terminal is the default
    Meas_t _drive_type = AD; // protocol for electrodes used in voltage reading 
    Meas_t _meas_type = AD; // protocol for electrodes used in voltage reading 
    bool _visualize_3d = false; // whether to create visualization in 3d 
    bool _auto_calibration = true; // whether to use built-in calibration 

    // Signal reading results
    double _signal_rms[NUM_MEAS];    // Store signal RMS data
    double _signal_phase[NUM_MEAS];  // Store signal phase data
    double signal_mag[NUM_MEAS];    // Store signal magnitude data
    double _cur_frame[NUM_MEAS] = {0};
    double _phase_offset;
    uint32_t frame_delay = 0;
    uint16_t _current_amp = 0;
    uint16_t _current_freq = 0;
    uint16_t _current_gain = 0;
    uint16_t _voltage_gain = 0;
    bool serial_communication = true; // whether statements are printed in Serial monitor during execution
    bool _bluetooth_communication = false;

    void BLEStart();
    void calibrateEIT();
    void sendBluetoothMessage();
    void calibrate_samples();
    void calibrate_gain(Meas_t drive_type, Meas_t meas_type);
    void calibrate_signal(Meas_t drive_type, Meas_t meas_type);
    void AD5270_Write(const int chip_select, uint8_t cmd, uint16_t data);
    void AD5270_LockUnlock(const int chip_select, uint8_t lock);

    #if defined(ARDUINO_ARCH_ESP32)
      void vspi_write_byte(const int chip_select, uint8_t data_to_send, const uint8_t bit_order, const uint8_t mode);
      void vspi_write_word(const int chip_select, uint16_t data_to_send, const uint8_t bit_order, const uint8_t mode);
      void hspi_write_byte(const int chip_select, uint8_t data_to_send, const uint8_t bit_order, const uint8_t mode);
      void hspi_write_word(const int chip_select, uint16_t data_to_send, const uint8_t bit_order, const uint8_t mode);
      uint32_t read_signal(double * rms, uint16_t * error_rate, uint8_t debug);
      void read_frame(Meas_t drive_type, Meas_t meas_type, double * rms_array, uint8_t electrodes_per_band);
      void read_frame_for_band(uint8_t band, Meas_t drive_type, Meas_t meas_type, double * rms_array, uint8_t electrodes_per_band);
    #elif defined(__IMXRT1062__) // for Teensy 4.0
      void spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val);
      uint16_t analog_read();
      uint32_t read_signal(double * rms, double * mag, double * phase, uint16_t * error_rate, uint8_t debug);
      void read_frame(Meas_t drive_type, Meas_t meas_type, double * rms_array, double * mag_array, double * phase_array, uint8_t num_elec);
    #endif
    void AD5270_Shutdown(const int chip_select, uint8_t shutdown);
    void AD5270_Set(const int chip_select, uint16_t val);
    void AD5930_Write(uint8_t reg, uint16_t data);
    void AD5930_Set_Start_Freq(uint32_t freq);

    void mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable);
    void mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable);
    
    uint32_t gpio_read();
    uint16_t gpio_convert(uint32_t gpio_reg);
    
    
    uint16_t sine_compare(uint16_t * signal, uint16_t pk_pk, uint16_t points_per_period, uint8_t num_periods);
   
    #if defined(ARDUINO_ARCH_ESP32)
    class MessageCallbacks : public BLECharacteristicCallbacks {
        std::string *_str_ptr;
    public:
        MessageCallbacks(std::string *str_ptr);
        void onWrite(BLECharacteristic *characteristic);
        void onRead(BLECharacteristic *characteristic);
    };

    class MyServerCallbacks : public BLEServerCallbacks {
        bool *connection_ptr = nullptr;
    public:
        MyServerCallbacks(bool *_bptr);
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
    };
    #endif
};

#endif
