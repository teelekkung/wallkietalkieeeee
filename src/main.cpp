//Libraries for NRF24L01+ module.
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <driver/i2s.h>

//RF24 object with two pins defined with arguments. CE: 4, CSN: 5
#if(CONFIG_IDF_TARGET_ESP32S3)
RF24 radio(9,10);
#define MIC_PIN 14
#define I2S_DOUT 15  
#define I2S_BCLK 16  
#define I2S_LRC  17
#define button1 4
#else
RF24 radio(21,5);
// cs = 5, ce = 21, clk = 18, miso = 19, mosi = 23
#define button1 22
#define MIC_PIN 14
#define I2S_DOUT 25  // ESP32 I2S Data Out (connect to DIN on MAX98357)
#define I2S_BCLK 26  // ESP32 I2S Bit Clock (connect to BCLK on MAX98357)
#define I2S_LRC  27  // ESP32 I2S Left-Right Clock (connect to LRC on MAX98357) 
#endif 

uint64_t address = 0x2306254820LL;
#define BUFFER_SIZE 32

volatile bool sendAudio = false;  // Flag to indicate whether to send audio
int roundloop = 0 ;
void IRAM_ATTR buttonInterrupt() {
  sendAudio = true;  // Set the flag when button is pressed
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  // Set up the button pin and attach the interrupt
  pinMode(button1, INPUT_PULLUP); // pull up data is con to vcc and r ,pull down data is con to gnd and r
  attachInterrupt(button1, buttonInterrupt, FALLING);
  radio.begin();
  radio.setChannel(23);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening();
  // Initialize I2S (for output to speaker)
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX) ,
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE*2,
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,     // Bit Clock pin
    .ws_io_num = I2S_LRC,       // Left-Right Clock pin
    .data_out_num = I2S_DOUT,   // Data Out pin
    .data_in_num = I2S_PIN_NO_CHANGE // No input pin used
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}
 
void loop() {
  if (sendAudio) {
    // Stop listening to send audio
    radio.stopListening();
    // Capture audio from microphone (assuming using ADC)
    int16_t audioBuffer[BUFFER_SIZE];
    for (int i = 0; i < BUFFER_SIZE; i++) {
      int raw = analogRead(MIC_PIN);
      audioBuffer[i] = (raw - 2048);  // Center around zero
      delayMicroseconds(22.7); // 62arrount 16khz 22.7 arr 48khz
    }
    // Send the captured audio data over the radio
    radio.write(audioBuffer, sizeof(audioBuffer));
  } 

  // If the button is released
  if (digitalRead(button1) == HIGH) {
    sendAudio = false; // Stop sending once the button is released
    radio.startListening();  // Start listening for incoming audio data
  }

  // Listen for incoming audio and play it to the I2S speaker
  if (!sendAudio && radio.available()) {
    int16_t audioBuffer[BUFFER_SIZE];
    radio.read(audioBuffer, sizeof(audioBuffer));
    // Send received audio data to I2S speaker
    size_t bytes_written;
    i2s_write(I2S_NUM_0, audioBuffer, sizeof(audioBuffer), &bytes_written, portMAX_DELAY);
    roundloop += 1;
    Serial.println(roundloop);
  }
}
