# Smart Quadruped Robot

*ESP32-S3 powered voice-controlled quadruped robot with AI interaction*

## Overview
This project implements a smart quadruped robot using the ESP32-S3 microcontroller. Combining voice recognition, motion control, and cloud-based AI interaction, it features:

- **Voice command recognition** for motion control and conversation
- **Quadruped trot gait locomotion** using inverse kinematics
- **AI-powered dialogue system** with DeepSeek API
- **Natural voice feedback** via Baidu TTS
- **Memory-optimized architecture** with PSRAM support

The system prioritizes low-latency audio processing and responsive motion control while maintaining a friendly interaction style. The robot can walk, turn, dance, and answer questions using natural language.

## Features
- **Voice Interaction**
  - Wake-word activation via physical button
  - Local preset command recognition ("forward", "turn left", "dance")
  - Cloud-based AI conversation handling
  - Natural speech synthesis with emotional tones
- **Motion System**
  - Real-time inverse kinematics calculations
  - Configurable trot gait parameters
  - Smooth servo motion transitions
  - Safety position reset
- **System Management**
  - Dual-level memory protection (heap + PSRAM)
  - Automatic storage cleanup
  - Hardware monitoring and emergency recovery
  - Over-the-air debugging via serial interface

## Hardware Requirements
- **Main Controller**: ESP32-S3 (with PSRAM)
- **Motion System**:
  - 8× SG90 servos (hip + knee joints)
  - PCA9685 servo driver module
- **Audio System**:
  - INMP441 microphone
  - MAX98357 I2S amplifier
  - 8Ω 1W speaker
- **Power System**:
  - XL4015 buck converter (5V/2A output)
  - 6× AA battery pack
  - Power switch
- **Connectivity**:
  - WiFi access (for cloud services)
- **Optional**: 1.3" display for debugging

**Pin Connections**:
| Module | Connection | ESP32-S3 Pin |
|--------|------------|--------------|
| INMP441 | WS | GPIO 41 |
| | SCK | GPIO 42 |
| | SD | GPIO 4 |
| PCA9685 | SDA | GPIO 17 |
| | SCL | GPIO 16 |
| MAX98357 | LRC | GPIO 15 |
| | BCLK | GPIO 14 |
| | DIN | GPIO 13 |
| Wake Button | Signal | GPIO 0 |

<img width="415" height="416" alt="image" src="https://github.com/user-attachments/assets/e7c271c3-877c-4a1c-93bf-65090ee36e4a" />


## Software Setup
1. **Development Environment**:
   - Arduino IDE 2.0+ or PlatformIO
   - ESP32 Arduino Core (supporting ESP32-S3)

2. **Required Libraries**:
<img width="522" height="393" alt="image" src="https://github.com/user-attachments/assets/dca71764-300a-44a7-9fa0-d9ef776c7922" />

3. **API Configuration**:
- Obtain DeepSeek API key from [DeepSeek Platform](https://platform.deepseek.com/)
- Create Baidu Cloud application for ASR/TTS at [Baidu AI Studio](https://ai.baidu.com/)
- Update credentials in code:
  <img width="742" height="164" alt="image" src="https://github.com/user-attachments/assets/5e622055-f578-45a1-b974-e094c4d4cc64" />

## Installation
1. **Hardware Assembly**:
- Mount servos on robot chassis
- Connect all electronic components as per pinout table
- Secure battery pack and power modules

2. **Firmware Upload**:
- Connect ESP32-S3 via USB
- Select board: `ESP32S3 Dev Module`
- Select appropriate USB port
- Compile and upload code

3. **Initial Setup**:
- Power on the robot
- System will automatically:
  - Initialize file system (SPIFFS)
  - Connect to WiFi
  - Obtain Baidu access token
  - Sync with NTP time server
- Successful initialization confirmed by test tone

## Usage
### Basic Operation
1. **Wake the robot**: Press the wake button (GPIO 0)
2. **Speak command**: Wait for beep, then speak within 3 seconds
3. **Receive response**: Robot will execute action or verbal response

### Voice Commands
| Command Type | Examples | Robot Response |
|--------------|----------|---------------|
| **Motion** | "前进" (Forward) | Executes forward gait |
|  | "左转" (Turn left) | Turns left |
|  | "跳舞" (Dance) | Performs dance sequence |
| **Information** | "现在几点" (What time is it?) | Speaks current time |
|  | "你叫什么名字" (What's your name?) | "我是您的语音助手" (I'm your voice assistant) |
| **Conversation** | "讲个笑话" (Tell a joke) | Uses DeepSeek API for response |

<img width="448" height="204" alt="image" src="https://github.com/user-attachments/assets/9b251093-0ff3-47fb-8728-26bf5a957789" />


### Serial Debugging
Access via 115200 baud serial monitor:

'''
void handleSerialCommands() {
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if(command == "restart") {
      addLog("Received restart command");
      ESP.restart();
    } 
    else if(command == "cache.list") {
      Serial.println("==== Cache List ====");
      for(const auto& entry : responseCache) {
        Serial.print(entry.first + " -> ");
        for(const String& response : entry.second) {
          Serial.print(response + " | ");
        }
        Serial.println();
      }
      Serial.println("====================");
    } 
    else if(command == "preset.list") {
      Serial.println("==== Preset Responses ====");
      for(const auto& entry : presetResponses) {
        Serial.println(entry.first + " -> " + entry.second);
      }
      Serial.println("========================");
    } 
    else if(command == "free") {
      size_t free_psram = 0;
      if(psramFound()) {
        free_psram = ESP.getFreePsram();
      }
      Serial.printf("Memory Status: Heap:%d, PSRAM:%d\n", 
                   esp_get_free_heap_size(), free_psram);
    }
    else if(command.startsWith("addpreset")) {
      // Format: addpreset|question|answer
      int firstPipe = command.indexOf('|');
      int secondPipe = command.indexOf('|', firstPipe + 1);
      
      if(firstPipe > 0 && secondPipe > firstPipe) {
        String question = command.substring(firstPipe + 1, secondPipe);
        String answer = command.substring(secondPipe + 1);
        
        // Update preset library
        const_cast<std::map<String, String>&>(presetResponses)[question] = answer;
        Serial.println("Added preset: " + question + " -> " + answer);
      } else {
        Serial.println("Error: Invalid format. Use: addpreset|question|answer");
      }
    }
    else {
      Serial.println("Unknown command");
    }
  }
}
void addLog(String message) {
  String timestamp = getFormattedTime();
  systemLog += "[" + timestamp + "] " + message + "\n";
  Serial.println("[" + timestamp + "] " + message);
}
'''
## Code Structure
Key components in `Main.ino`:
'''
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  while(!Serial) delay(10);
  i2sMutex = xSemaphoreCreateMutex();
  if(!SPIFFS.begin(true)) {
    addLog("SPIFFS initialization failed!");
    // Attempt repair
    SPIFFS.format();
    if(!SPIFFS.begin(true)) {
      addLog("SPIFFS repair failed, system halted");
      while(1);
    }
  }
  setupServos();
  initializeSystem();
  xTaskCreatePinnedToCore(memoryMonitorTask, "MemoryMonitor", 3000, nullptr, 1, nullptr, 0);
}

void loop() {
  handleSerialCommands();
  if(isReady && !isProcessing && digitalRead(wakeButtonPin) == LOW) {
    wakeUpAssistant();
    delay(300); // Debounce
  }
  if(WiFi.status() != WL_CONNECTED) {
    if(millis() - wifiConnectStartTime > 30000) {
      addLog("WiFi disconnected, attempting reconnect...");
      initializeSystem();
    }
  }
}

void trotGait(int direction, float speed) {
  direction = constrain(direction, 0, 3);
  speed = constrain(speed, 0.1, 1.0);
  unsigned long startTime = millis();
  const int phaseDuration = GAIT_CYCLE * speed;
  addLog("Starting gait: Direction=" + String(direction) + " Speed=" + String(speed, 2));
  while(millis() - startTime < phaseDuration) {
    float t = ((millis() - startTime) % GAIT_CYCLE) / (float)GAIT_CYCLE;
    moveLeg(RF_HIP, RF_KNEE, t, direction);
    moveLeg(LB_HIP, LB_KNEE, t, direction);
    moveLeg(LF_HIP, LF_KNEE, t + 0.5, direction);
    moveLeg(RB_HIP, RB_KNEE, t + 0.5, direction);
    delay(20); 
  }
  for(int i = 8; i <= 15; i++) {
    smoothMove(i, 90, 300); 
  }
  addLog("Gait completed");
}
'''

## Contributing
1. Fork the repository
2. Create feature branch (`git checkout -b feature/new-feature`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/new-feature`)
5. Submit pull request

**Testing requirements**:
- Verify memory usage with `free` command
- Check gait stability at different speeds
- Validate voice recognition accuracy

## License
Distributed under the MIT License. See `LICENSE` for full text.

## Acknowledgements
- [DeepSeek](https://deepseek.com) for conversational AI API
- Baidu Cloud for speech processing services
- Arduino and ESP32 communities for hardware libraries
- Contributors and beta testers
