
 
// Include I2S driver
#include <Arduino.h>
#include <driver/i2s.h>
#include <Wire.h>
 
// Connections to INMP441 I2S microphone
#define I2S_WS 26
#define I2S_SCK 27
#define I2S_SD_OUT 25
#define I2S_SD_IN 33

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

// initialize wm8731
void AudioCodec_init() {

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


// Set up I2S Processor configuration
void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
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
 
// Set I2S pin configuration
void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_SD_OUT,
    .data_in_num = I2S_SD_IN
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
   // This code generates a square wave, the frequency is changed by changing the WaveLength var below, the higher the
  // number the lower the frequency. This var simply acts as a timer for how long we stay high (peak) and low (trough)
 
  static const uint16_t Volume=0x3ff;             // Highest value the peak will get to, bigger gives louder vol, but  
                                                  // too high can lead to distortion on cheap or small speakers
  static const int16_t Peak=Volume;               // Max height/peak  
  static const int16_t Trough=-Volume;            // Max low/trough
  
  static int16_t OutputValue=Peak;                // Value to send to each channel (left/right), start off at the peak
  static const uint16_t WaveLength=100;           // Bigger =longer wavelength and higher frequency
  static uint16_t TimeAtPeakOrTrough=WaveLength;  // Var to count down how long we hold at either peak or trough
                                                  // (high/low)
  uint32_t Value32Bit;                            // This holds the value we actually send to the I2S buffers, it basically
                                                  // will hold the "OutputValue" but combined for both left/right channels
                                                  
  size_t BytesWritten;                            // Returned by the I2S write routine, we are not interested in it but
                                                  // must supply it as a param to the routine.
                                                    
  if(TimeAtPeakOrTrough==0)                       // If 0 then we're ready to switch to a peak (high) or trough (low)
  {
    if(OutputValue==Peak)                         // If we were a peak (high), switch to trough (low) value
        OutputValue=Trough;
    else                                          // else we were a trough (low) switch to peak (high)
        OutputValue=Peak;
    TimeAtPeakOrTrough=WaveLength;                // Reset this counter back to max, ready to countdown again
  }
  TimeAtPeakOrTrough--;                           // Decrement this counter that controls wavelenght/ frequency
 
  // This next line just creates the 32 bit word we send to the I2S interface/buffers. 16 bits for the righ channel
  // and 16 bits for the left, as we are sending the same wave to both channels we just combine them using some
  // bit shifting and masking. If you're not sure what's happening here then look up bit shifting/ manipulation
  // on a C programming website.
  
  Value32Bit = (OutputValue<<16) | (OutputValue & 0xffff); // Output same value on both left and right channels


  // False print statements to "lock range" on serial plotter display
  // Change rangelimit value to adjust "sensitivity"
  int rangelimit = 3000;
  // Serial.print(rangelimit * -1);
  // Serial.print(" ");
  // Serial.print(rangelimit);
  // Serial.print(" ");
 
  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  size_t BytesOut = 0;     
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
      // Serial.println(mean);
    }
  }
  i2s_write(I2S_PORT,&sBuffer,bufferLen,&BytesOut,portMAX_DELAY ); 

}