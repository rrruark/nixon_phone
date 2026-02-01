#include <driver/i2s.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

// PCM5102 DAC Pin Definitions
#define I2S_BCK_PIN    26  // IO26: DAC_BCK (Bit Clock)
#define I2S_LRCLK_PIN  25  // IO25: DAC_LRCLK (Left/Right Clock)
#define I2S_DIN_PIN    22  // IO22: DAC_DIN (Data Input)

// SLIC Control Pins
#define SHK_1_PIN      34  // IO34: SHK_1
#define SHK_2_PIN      35  // IO35: SHK_2
#define RM_1_PIN       32  // IO32: RM_1
#define RM_2_PIN       33  // IO33: RM_2
#define FR_1_PIN       27  // IO27: F/R_1
#define FR_2_PIN       12  // IO12: F/R_2

// LED Pins
#define LED_RED_PIN    2   // IO2: LED_RED
#define LED_GREEN_PIN  0   // IO0: LED_GREEN

// Button Pins
#define BUTTON_1_PIN   16  // IO16: BUTTON_1
#define BUTTON_2_PIN   17  // IO17: BUTTON_2

// SD Card Pins (SPI)
#define SD_SS_PIN      5   // IO5: SD_SS
#define SD_SCK_PIN     18  // IO18: SD_SCK
#define SD_MISO_PIN    19  // IO19: SD_MISO
#define SD_MOSI_PIN    23  // IO23: SD_MOSI

// I2C Pins
#define I2C_SDA_PIN    4   // IO4: OLED_SDA (shared with TCA9534)
#define I2C_SCL_PIN    15  // IO15: OLED_SCK (shared with TCA9534)

// TCA9534 GPIO Expander
#define TCA9534_ADDR   0x20  // I2C address (default, may vary based on A0-A2 pins)
#define TCA9534_REG_INPUT_PORT     0x00  // Input port register (read-only)
#define TCA9534_REG_OUTPUT_PORT    0x01  // Output port register
#define TCA9534_REG_POLARITY       0x02  // Polarity inversion register
#define TCA9534_REG_CONFIG         0x03  // Configuration register (0=output, 1=input)

// TCA9534 Pin definitions
#define TCA9534_MUXA0_1    0  // P0: MUXA0_1
#define TCA9534_MUXA1_1    1  // P1: MUXA1_1
#define TCA9534_MUXA0_2    2  // P2: MUXA0_2
#define TCA9534_MUXA1_2    3  // P3: MUXA1_2

// I2S Configuration
#define SAMPLE_RATE    44100
#define BITS_PER_SAMPLE 16
#define CHANNELS       2

// Buffer size for audio samples
#define BUFFER_SIZE    512

// I2S port
#define I2S_PORT       I2S_NUM_0

// WAV file folder
#define WAV_FOLDER_PATH  "/nixon"

// Maximum number of WAV files to support
#define MAX_WAV_FILES    20

// WAV file list
String wavFiles[MAX_WAV_FILES];
uint8_t wavFileCount = 0;
String currentWavFile = "";

// WAV file handle
File wavFile;
uint32_t wavDataStart = 0;  // Position where audio data starts in file
uint32_t wavDataSize = 0;    // Size of audio data chunk
uint32_t wavBytesRead = 0;   // Bytes read from data chunk
bool playbackStarted = false;  // Track if playback has started
uint32_t consecutiveReadFailures = 0;  // Track consecutive read failures
bool playbackActive = false;  // Track if playback should be active (controlled by SHK)

// Previous state of SHK pins for transition detection
static bool prev_shk_1_state = false;
static bool prev_shk_2_state = false;

// Rotary dial pulse detection
#define PULSE_TIMEOUT_MS    1000  // Timeout to detect end of dialing (1 second)
#define PULSE_COUNT_ZERO    10    // Number of pulses for digit '0'

struct PulseDetector {
  uint8_t pulseCount;
  unsigned long lastPulseTime;
  bool dialingActive;
};

PulseDetector pulseDetector_1 = {0, 0, false};
PulseDetector pulseDetector_2 = {0, 0, false};
bool muxConfiguredForDial = false;  // Track if mux is in dial configuration

// Previous state of buttons for debouncing
static bool prev_button_1_state = true;  // Pull-up, so HIGH when not pressed
static bool prev_button_2_state = true;

// Function to ring a phone
// phoneNum: 1 for phone 1, 2 for phone 2
// durationMs: duration in milliseconds
void ringPhone(uint8_t phoneNum, uint32_t durationMs) {
  uint8_t rm_pin, fr_pin;
  
  if (phoneNum == 1) {
    rm_pin = RM_1_PIN;
    fr_pin = FR_1_PIN;
    Serial.println("Ringing phone 1...");
  } else if (phoneNum == 2) {
    rm_pin = RM_2_PIN;
    fr_pin = FR_2_PIN;
    Serial.println("Ringing phone 2...");
  } else {
    Serial.println("Invalid phone number");
    return;
  }
  
  // Set RM high
  digitalWrite(rm_pin, HIGH);
  
  // Toggle F/R at ~20 Hz (25ms per half cycle = 50ms per full cycle)
  uint32_t startTime = millis();
  bool fr_state = false;
  
  while (millis() - startTime < durationMs) {
    digitalWrite(fr_pin, fr_state ? HIGH : LOW);
    fr_state = !fr_state;
    delay(25);  // 25ms delay = 20 Hz toggle frequency
  }
  
  // Restore pins to low
  digitalWrite(rm_pin, LOW);
  digitalWrite(fr_pin, LOW);
  
  Serial.printf("Finished ringing phone %d\n", phoneNum);
}

// Function to detect rotary dial pulses and handle dialing
// Takes current SHK states as parameters to avoid reading them again
void detectRotaryDialPulses(bool shk_1_current, bool shk_2_current) {
  unsigned long currentTime = millis();
  
  // Detect pulses on SHK_1 (phone 1) - LOW to HIGH transition
  if (shk_1_current != prev_shk_1_state) {
    if (shk_1_current == HIGH && prev_shk_1_state == LOW) {
      pulseDetector_1.pulseCount++;
      pulseDetector_1.lastPulseTime = currentTime;
      pulseDetector_1.dialingActive = true;
      Serial.printf("Phone 1 pulse detected: %d\n", pulseDetector_1.pulseCount);
    }
  }
  
  // Detect pulses on SHK_2 (phone 2) - LOW to HIGH transition
  if (shk_2_current != prev_shk_2_state) {
    if (shk_2_current == HIGH && prev_shk_2_state == LOW) {
      pulseDetector_2.pulseCount++;
      pulseDetector_2.lastPulseTime = currentTime;
      pulseDetector_2.dialingActive = true;
      Serial.printf("Phone 2 pulse detected: %d\n", pulseDetector_2.pulseCount);
    }
  }
  
  // Check for timeout on phone 1 (end of dialing)
  if (pulseDetector_1.dialingActive && 
      (currentTime - pulseDetector_1.lastPulseTime) > PULSE_TIMEOUT_MS) {
    if (pulseDetector_1.pulseCount == PULSE_COUNT_ZERO) {
      Serial.println("Phone 1 dialed '0' - ringing phone 2");
      setMuxDialConfig();
      ringPhone(2, 2000);  // Ring phone 2 for 2 seconds
    }
    pulseDetector_1.pulseCount = 0;
    pulseDetector_1.dialingActive = false;
  }
  
  // Check for timeout on phone 2 (end of dialing)
  if (pulseDetector_2.dialingActive && 
      (currentTime - pulseDetector_2.lastPulseTime) > PULSE_TIMEOUT_MS) {
    if (pulseDetector_2.pulseCount == PULSE_COUNT_ZERO) {
      Serial.println("Phone 2 dialed '0' - ringing phone 1");
      setMuxDialConfig();
      ringPhone(1, 2000);  // Ring phone 1 for 2 seconds
    }
    pulseDetector_2.pulseCount = 0;
    pulseDetector_2.dialingActive = false;
  }
  
  // Check if both phones hung up (both SHK LOW) and reset mux if needed
  if (!shk_1_current && !shk_2_current && muxConfiguredForDial) {
    Serial.println("Both phones hung up - resetting mux to boot configuration");
    setMuxBootConfig();
    // Reset pulse detectors
    pulseDetector_1.pulseCount = 0;
    pulseDetector_1.dialingActive = false;
    pulseDetector_2.pulseCount = 0;
    pulseDetector_2.dialingActive = false;
  }
}

// Function to initialize TCA9534 GPIO expander
bool initTCA9534() {
  Wire.beginTransmission(TCA9534_ADDR);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("TCA9534 not found at address 0x%02X (error: %d)\n", TCA9534_ADDR, error);
    return false;
  }
  
  // Configure P0-P3 as outputs (set config register bits 0-3 to 0)
  // P4-P7 remain as inputs (default)
  Wire.beginTransmission(TCA9534_ADDR);
  Wire.write(TCA9534_REG_CONFIG);
  Wire.write(0xF0);  // P0-P3 outputs (0), P4-P7 inputs (1)
  error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("Failed to configure TCA9534 (error: %d)\n", error);
    return false;
  }
  
  // Set output port register:
  // P0 (MUXA0_1) = LOW (0)
  // P1 (MUXA1_1) = HIGH (1)
  // P2 (MUXA0_2) = LOW (0)
  // P3 (MUXA1_2) = LOW (0)
  // P4-P7 = don't care (inputs)
  Wire.beginTransmission(TCA9534_ADDR);
  Wire.write(TCA9534_REG_OUTPUT_PORT);
  Wire.write(0x02);  // Bit 1 set (P1 HIGH), bits 0, 2-3 clear (P0, P2-P3 LOW)
  error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("Failed to set TCA9534 output port (error: %d)\n", error);
    return false;
  }
  
  Serial.println("TCA9534 initialized successfully");
  Serial.println("  MUXA0_1 (P0): LOW");
  Serial.println("  MUXA1_1 (P1): HIGH");
  Serial.println("  MUXA0_2 (P2): LOW");
  Serial.println("  MUXA1_2 (P3): LOW");
  return true;
}

// Function to set mux to dial configuration (when '0' is dialed)
bool setMuxDialConfig() {
  Wire.beginTransmission(TCA9534_ADDR);
  Wire.write(TCA9534_REG_OUTPUT_PORT);
  Wire.write(0x0D);  // P0=HIGH(1), P1=LOW(0), P2=HIGH(1), P3=HIGH(1) = 0b00001101
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("Failed to set mux dial config (error: %d)\n", error);
    return false;
  }
  
  Serial.println("Mux set to dial configuration:");
  Serial.println("  MUXA0_1 (P0): HIGH");
  Serial.println("  MUXA1_1 (P1): LOW");
  Serial.println("  MUXA0_2 (P2): HIGH");
  Serial.println("  MUXA1_2 (P3): HIGH");
  muxConfiguredForDial = true;
  return true;
}

// Function to set mux back to boot configuration
bool setMuxBootConfig() {
  Wire.beginTransmission(TCA9534_ADDR);
  Wire.write(TCA9534_REG_OUTPUT_PORT);
  Wire.write(0x02);  // P0=LOW(0), P1=HIGH(1), P2=LOW(0), P3=LOW(0) = 0b00000010
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("Failed to set mux boot config (error: %d)\n", error);
    return false;
  }
  
  Serial.println("Mux set to boot configuration:");
  Serial.println("  MUXA0_1 (P0): LOW");
  Serial.println("  MUXA1_1 (P1): HIGH");
  Serial.println("  MUXA0_2 (P2): LOW");
  Serial.println("  MUXA1_2 (P3): LOW");
  muxConfiguredForDial = false;
  return true;
}

// Function to scan for WAV files in the nixon folder
void scanWAVFiles() {
  wavFileCount = 0;
  File root = SD.open(WAV_FOLDER_PATH);
  if (!root) {
    Serial.println("Failed to open nixon folder");
    return;
  }
  
  if (!root.isDirectory()) {
    Serial.println("nixon is not a directory");
    root.close();
    return;
  }
  
  File file = root.openNextFile();
  while (file && wavFileCount < MAX_WAV_FILES) {
    if (!file.isDirectory()) {
      String fileName = file.name();
      if (fileName.endsWith(".wav") || fileName.endsWith(".WAV")) {
        // Store full path
        wavFiles[wavFileCount] = String(WAV_FOLDER_PATH) + "/" + fileName;
        wavFileCount++;
        Serial.printf("Found WAV file: %s\n", wavFiles[wavFileCount - 1].c_str());
      }
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();
  
  Serial.printf("Found %d WAV files\n", wavFileCount);
}

// Function to pick a random WAV file
String pickRandomWAVFile() {
  if (wavFileCount == 0) {
    Serial.println("No WAV files available");
    return "";
  }
  
  // Use ESP32's random number generator (seeded by noise)
  uint8_t index = random(0, wavFileCount);
  Serial.printf("Selected random file %d: %s\n", index, wavFiles[index].c_str());
  return wavFiles[index];
}

// Function to open and initialize a WAV file
bool openWAVFile(String filePath) {
  if (wavFile) {
    wavFile.close();
  }
  
  if (filePath.length() == 0) {
    Serial.println("Empty file path");
    return false;
  }
  
  Serial.printf("Opening WAV file: %s\n", filePath.c_str());
  wavFile = SD.open(filePath.c_str(), FILE_READ);
  if (!wavFile) {
    Serial.println("Failed to open WAV file");
    return false;
  }
  
  if (parseWAVHeader(wavFile)) {
    wavBytesRead = 0;
    consecutiveReadFailures = 0;
    currentWavFile = filePath;
    Serial.println("WAV file opened and initialized successfully");
    return true;
  } else {
    wavFile.close();
    Serial.println("Failed to parse WAV file header");
    return false;
  }
}

// Function to reopen and reinitialize the WAV file
bool reopenWAVFile() {
  return openWAVFile(currentWavFile);
}

// Function to parse WAV file header
bool parseWAVHeader(File file) {
  uint8_t header[44];
  if (file.read(header, 44) != 44) {
    Serial.println("Error reading WAV header");
    return false;
  }
  
  // Check for "RIFF" and "WAVE" identifiers
  if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F') {
    Serial.println("Not a valid RIFF file");
    return false;
  }
  if (header[8] != 'W' || header[9] != 'A' || header[10] != 'V' || header[11] != 'E') {
    Serial.println("Not a valid WAVE file");
    return false;
  }
  
  // Find "data" chunk
  uint32_t pos = 12;
  while (pos < file.size() - 8) {
    file.seek(pos);
    uint8_t chunkId[4];
    if (file.read(chunkId, 4) != 4) break;
    
    if (chunkId[0] == 'd' && chunkId[1] == 'a' && chunkId[2] == 't' && chunkId[3] == 'a') {
      // Found data chunk
      uint8_t sizeBytes[4];
      file.read(sizeBytes, 4);
      wavDataSize = sizeBytes[0] | (sizeBytes[1] << 8) | (sizeBytes[2] << 16) | (sizeBytes[3] << 24);
      wavDataStart = file.position();
      wavBytesRead = 0;
      
      Serial.printf("WAV file parsed: Data starts at %lu, size: %lu bytes\n", wavDataStart, wavDataSize);
      return true;
    }
    
    // Read chunk size and skip to next chunk
    uint8_t chunkSizeBytes[4];
    if (file.read(chunkSizeBytes, 4) != 4) break;
    uint32_t chunkSize = chunkSizeBytes[0] | (chunkSizeBytes[1] << 8) | 
                         (chunkSizeBytes[2] << 16) | (chunkSizeBytes[3] << 24);
    pos += 8 + chunkSize;
    if (chunkSize % 2) pos++; // Align to word boundary
  }
  
  Serial.println("Data chunk not found in WAV file");
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 PCM5102 Audio Player");
  
  // Initialize random number generator with noise from ADC
  randomSeed(analogRead(0));
  
  // Initialize I2C for TCA9534 and OLED
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("I2C initialized");
  
  // Initialize TCA9534 GPIO expander
  if (!initTCA9534()) {
    Serial.println("Warning: TCA9534 initialization failed");
  }
  
  // Configure I2S with improved settings for stability
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 16,  // Increased buffer count for smoother playback
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = true,  // Enable APLL for better clock stability
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  
  // Configure I2S pins
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_DIN_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  // Install and start I2S driver
  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed to install I2S driver: %s\n", esp_err_to_name(err));
    return;
  }
  
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed to set I2S pins: %s\n", esp_err_to_name(err));
    return;
  }
  
  Serial.println("I2S driver installed and started");
  
  // Configure SHK pins as inputs with pull-up resistors
  pinMode(SHK_1_PIN, INPUT_PULLUP);
  pinMode(SHK_2_PIN, INPUT_PULLUP);
  
  // Initialize previous state of SHK pins
  prev_shk_1_state = digitalRead(SHK_1_PIN);
  prev_shk_2_state = digitalRead(SHK_2_PIN);
  
  // Configure LED pins as outputs
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);  // Green LED on by default
  
  // Configure RM pins as outputs and set them low
  pinMode(RM_1_PIN, OUTPUT);
  pinMode(RM_2_PIN, OUTPUT);
  digitalWrite(RM_1_PIN, LOW);
  digitalWrite(RM_2_PIN, LOW);
  
  // Configure F/R pins as outputs and set them low
  pinMode(FR_1_PIN, OUTPUT);
  pinMode(FR_2_PIN, OUTPUT);
  digitalWrite(FR_1_PIN, LOW);
  digitalWrite(FR_2_PIN, LOW);
  
  // Configure buttons as inputs with pull-up resistors
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  
  // Initialize previous button states
  prev_button_1_state = digitalRead(BUTTON_1_PIN);
  prev_button_2_state = digitalRead(BUTTON_2_PIN);
  
  Serial.println("SHK monitoring initialized");
  
  // Initialize SD card
  Serial.println("Initializing SD card...");
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_SS_PIN);
  
  // Set SPI clock to 4 MHz for stable SD card operation
  if (!SD.begin(SD_SS_PIN, SPI, 4000000)) {
    Serial.println("SD card initialization failed!");
    Serial.println("Please check:");
    Serial.println("1. Is the SD card inserted?");
    Serial.println("2. Is the wiring correct?");
    Serial.println("3. Is the card formatted (FAT32)?");
  } else {
    Serial.println("SD card initialized successfully!");
    
    // Scan for WAV files in the nixon folder
    Serial.println("Scanning for WAV files...");
    scanWAVFiles();
    
    if (wavFileCount > 0) {
      Serial.printf("Found %d WAV files. Waiting for SHK signal to start playback...\n", wavFileCount);
    } else {
      Serial.println("No WAV files found in nixon folder!");
    }
    
    // Initialize playback state - playback will start when SHK goes HIGH
    playbackActive = false;
  }
}

void loop() {
  // Check for button presses
  bool button_1_state = digitalRead(BUTTON_1_PIN);
  bool button_2_state = digitalRead(BUTTON_2_PIN);
  
  // Detect button 1 press (LOW when pressed due to pull-up)
  if (button_1_state == LOW && prev_button_1_state == HIGH) {
    Serial.println("Button 1 pressed");
    ringPhone(1, 1000);  // Ring phone 1 for 1 second
  }
  prev_button_1_state = button_1_state;
  
  // Detect button 2 press (LOW when pressed due to pull-up)
  if (button_2_state == LOW && prev_button_2_state == HIGH) {
    Serial.println("Button 2 pressed");
    ringPhone(2, 1000);  // Ring phone 2 for 1 second
  }
  prev_button_2_state = button_2_state;
  
  // Monitor SHK pins and control LED_GREEN and playback
  // If SHK_1 OR SHK_2 is high, set LED_GREEN high; if both are low, set it low
  bool shk_1_state = digitalRead(SHK_1_PIN);
  bool shk_2_state = digitalRead(SHK_2_PIN);
  bool shk_any_high = shk_1_state || shk_2_state;
  
  // Detect rotary dial pulses (must be called before updating prev_shk states)
  detectRotaryDialPulses(shk_1_state, shk_2_state);
  
  // Detect state transitions and control playback
  static unsigned long lastShkPrint = 0;
  if (shk_1_state != prev_shk_1_state) {
    if (millis() - lastShkPrint > 100) {  // Throttle serial output
      Serial.printf("SHK_1: %s\n", shk_1_state ? "HIGH" : "LOW");
      lastShkPrint = millis();
    }
    prev_shk_1_state = shk_1_state;
  }
  
  if (shk_2_state != prev_shk_2_state) {
    if (millis() - lastShkPrint > 100) {  // Throttle serial output
      Serial.printf("SHK_2: %s\n", shk_2_state ? "HIGH" : "LOW");
      lastShkPrint = millis();
    }
    prev_shk_2_state = shk_2_state;
  }
  
  // Control LED based on SHK state (using RED LED)
  if (shk_any_high) {
    digitalWrite(LED_RED_PIN, HIGH);
  } else {
    digitalWrite(LED_RED_PIN, LOW);
  }
  
  // Control playback based on SHK state
  if (shk_any_high && !playbackActive) {
    // SHK went HIGH - start playback
    Serial.println("SHK HIGH detected - starting playback");
    playbackActive = true;
    playbackStarted = false;
    
    // Pick and open a random WAV file
    if (wavFileCount > 0) {
      String randomFile = pickRandomWAVFile();
      if (randomFile.length() > 0) {
        if (!openWAVFile(randomFile)) {
          Serial.println("Failed to open random file for playback");
          playbackActive = false;
        }
      } else {
        playbackActive = false;
      }
    } else {
      Serial.println("No WAV files available for playback");
      playbackActive = false;
    }
  } else if (!shk_any_high && playbackActive) {
    // Both SHK went LOW - stop playback
    Serial.println("SHK LOW detected - stopping playback");
    playbackActive = false;
    playbackStarted = false;
    
    // Close current file
    if (wavFile) {
      wavFile.close();
      wavFile = File();
    }
    wavBytesRead = 0;
    wavDataSize = 0;
    currentWavFile = "";
  }
  
  // Play WAV file only if playback is active and file is open
  if (playbackActive && wavFile) {
    // Check if we've reached the end of audio data
    if (wavBytesRead >= wavDataSize) {
      // Pick a new random file instead of looping the same one
      Serial.println("End of file reached, selecting new random file...");
      String randomFile = pickRandomWAVFile();
      if (randomFile.length() > 0) {
        if (!openWAVFile(randomFile)) {
          Serial.println("Failed to open new random file");
          playbackActive = false;  // Stop playback on error
          return;  // Failed to open, skip this iteration
        }
      } else {
        Serial.println("No WAV files available");
        playbackActive = false;  // Stop playback if no files
        return;
      }
    }
    
    // Calculate how many bytes we can read (limit to buffer size)
    uint32_t bytesToRead = wavDataSize - wavBytesRead;
    uint32_t maxBufferSize = BUFFER_SIZE * 2 * sizeof(int16_t);
    if (bytesToRead > maxBufferSize) {
      bytesToRead = maxBufferSize;
    }
    
    // Static buffer for audio samples (stereo, 16-bit)
    static uint8_t buffer[BUFFER_SIZE * 2 * sizeof(int16_t)];
    
    // Read audio data from file
    size_t bytesRead = wavFile.read(buffer, bytesToRead);
    if (bytesRead > 0) {
      if (!playbackStarted) {
        Serial.println("Audio playback started!");
        playbackStarted = true;
      }
      wavBytesRead += bytesRead;
      consecutiveReadFailures = 0;  // Reset failure counter on success
      
      // Write buffer to I2S
      size_t bytes_written;
      i2s_write(I2S_PORT, buffer, bytesRead, &bytes_written, portMAX_DELAY);
    } else {
      // If no bytes read, there might be an issue
      consecutiveReadFailures++;
      Serial.printf("Warning: No bytes read from file (failure count: %lu)\n", consecutiveReadFailures);
      
      // If we have multiple consecutive failures, try to recover by picking a new random file
      if (consecutiveReadFailures >= 3) {
        Serial.println("Multiple read failures detected, selecting new random file...");
        String randomFile = pickRandomWAVFile();
        if (randomFile.length() > 0) {
          if (openWAVFile(randomFile)) {
            return;  // Successfully opened new file, continue on next iteration
          }
        }
        // If we can't open a new file, try to seek to expected position as fallback
        uint32_t expectedPos = wavDataStart + wavBytesRead;
        Serial.printf("Fallback: Seeking to expected position: %lu\n", expectedPos);
        wavFile.seek(expectedPos);
        consecutiveReadFailures = 0;  // Reset counter after recovery attempt
      }
      delay(10);
    }
  } else if (playbackActive) {
    // If playback is active but file is not open, try to open a file
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 1000) {
      Serial.println("Playback active but file not open, selecting new random file...");
      if (wavFileCount > 0) {
        String randomFile = pickRandomWAVFile();
        if (randomFile.length() > 0) {
          if (!openWAVFile(randomFile)) {
            playbackActive = false;  // Stop playback if we can't open a file
          }
        } else {
          playbackActive = false;
        }
      } else {
        // Rescan for files if none found
        Serial.println("Rescanning for WAV files...");
        scanWAVFiles();
        if (wavFileCount > 0) {
          String randomFile = pickRandomWAVFile();
          if (randomFile.length() > 0) {
            if (!openWAVFile(randomFile)) {
              playbackActive = false;
            }
          } else {
            playbackActive = false;
          }
        } else {
          playbackActive = false;
        }
      }
      lastCheck = millis();
    }
    delay(10);
  }
}

