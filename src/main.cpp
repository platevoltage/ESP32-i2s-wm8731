
 
// Include I2S driver
#include <Arduino.h>
#include <driver/i2s.h>
#include <Wire.h>
 
// Connections to INMP441 I2S microphone
#define I2S_WS 26
#define I2S_SD 33
#define I2S_SCK 27

// i2s device configuration
#define LINVOL 23
#define RINVOL 23
#define LHPVOL 121
#define RHPVOL 121
#define ADCHPD 0
#define SIDEATT 0
#define SIDETONE 0
#define DACSEL 1
#define BYPASS 0
#define INSEL 1
#define MUTEMIC 0
#define MICBOOST 1
#define SAMPLE_RATE 44 

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0
 
// Define input buffer length
#define bufferLen 64
int16_t sBuffer[bufferLen];
void AudioCodec_init() {


        
        // setup i2c pins and configure codec
        // the new Wire library has trouble with 0x00, so (uint8_t) is added
        // To change the Wire interface speed, go to:
        // <path_from_arduino>\hardware\arduino\sam\libraries\Wire\Wire.h
        // and change the parameters TWI_CLOCK, RECV_TIMEOUT and XMIT_TIMEOUT
        // to the desire frequency.
        int temp_wire1;
        int temp_wire2;
        
        Wire.begin();
        Wire.beginTransmission(0x1a);
        Wire.write(0x0c); // power reduction register
        Wire.write((uint8_t)0x00); // turn everything on
        temp_wire1 = Wire.endTransmission();
        
        Wire.beginTransmission(0x1a);
        Wire.write(0x0e); // digital data format
        Wire.write(0x03); // 16b SPI mode
        temp_wire2 = Wire.endTransmission();
        
        Serial.println(temp_wire1);
        Serial.println(temp_wire2);
        
        Wire.beginTransmission(0x1a);
        Wire.write((uint8_t)0x00); // left in setup register
        Wire.write((uint8_t)LINVOL);
        Wire.endTransmission();
        
        Wire.beginTransmission(0x1a);
        Wire.write(0x02); // right in setup register
        Wire.write((uint8_t)RINVOL);
        Wire.endTransmission();
        
        Wire.beginTransmission(0x1a);
        Wire.write(0x04); // left headphone out register
        Wire.write((uint8_t)LHPVOL);
        Wire.endTransmission();
        
        Wire.beginTransmission(0x1a);
        Wire.write(0x06); // right headphone out register
        Wire.write((uint8_t)RHPVOL);
        Wire.endTransmission();
        
        Wire.beginTransmission(0x1a);
        Wire.write(0x0a); // digital audio path configuration
        Wire.write((uint8_t)ADCHPD);
        Wire.endTransmission();
        
        Wire.beginTransmission(0x1a);
        Wire.write(0x08); // analog audio pathway configuration
        Wire.write((uint8_t)((SIDEATT << 6)|(SIDETONE << 5)|(DACSEL << 4)|(BYPASS << 3)|(INSEL << 2)|(MUTEMIC << 1)|(MICBOOST << 0)));
        Wire.endTransmission();
        
        Wire.beginTransmission(0x1a);
        Wire.write(0x10); // clock configuration
        #if SAMPLE_RATE == 88
          Wire.write(0xbc);
        #elif SAMPLE_RATE == 44
          Wire.write(0xa0);
        #elif SAMPLE_RATE == 22
          Wire.write(0xe0);
        #elif SAMPLE_RATE == 8
          Wire.write(0xac);
        #elif SAMPLE_RATE == 2
          Wire.write(0xce);
        #endif
        Wire.endTransmission();
      
        Wire.beginTransmission(0x1a);
        Wire.write(0x12); // codec enable
        Wire.write(0x01);
        Wire.endTransmission();
        
} 
void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };
 
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}
 
void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  AudioCodec_init();
  i2s_set_pin(I2S_PORT, &pin_config);
}
 
void setup() {
 
  // Set up Serial Monitor
  Serial.begin(115200);
  Serial.println(" ");
 
  delay(1000);
 
  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
 
 
  delay(500);
}
 
void loop() {
 
  // False print statements to "lock range" on serial plotter display
  // Change rangelimit value to adjust "sensitivity"
  int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");
 
  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
 
  if (result == ESP_OK)
  {
    // Read I2S data buffer
    int16_t samples_read = bytesIn / 8;
    if (samples_read > 0) {
      float mean = 0;
      for (int16_t i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }
 
      // Average the data reading
      mean /= samples_read;
 
      // Print to serial plotter
      Serial.println(mean);
    }
  }
}