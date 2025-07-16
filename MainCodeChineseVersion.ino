// SPDX-License-Identifier: MIT
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>
#include <SPIFFS.h>
#include <map>
#include <time.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_psram.h>
#include <math.h>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);

// 配置参数
const char* ssid = "输入您的WIFI名称";
const char* password = "输入您的WIFI密码";
const char* deepseekApiKey = "输入您的Deepseek API Key";
const char* baiduApiKey = "输入您的百度API Key";
const char* baiduSecretKey = "输入您的百度Secret Key";

// 全局变量
static uint8_t *recordBuffer = nullptr;   // 新增预分配录音缓冲区
static size_t recordBufferSize = 0;        // 缓冲区大小

String baiduAccessToken = "";
const char* baiduAsrApiUrl = "http://vop.baidu.com/server_api";
const char* baiduTtsUrl = "http://tsn.baidu.com/text2audio";
SemaphoreHandle_t i2sMutex = NULL;
volatile bool isRecording = false;

// 预设问答库
const std::map<String, String> presetResponses = {
  {"你好", "你好,请问有什么可以帮您?"},
  {"你好吗", "我很好，谢谢关心!"},
  {"你叫什么名字", "我是您的语音助手"},
  {"谢谢", "不客气!"},
  {"再见", "再见，期待下次为您服务!"},
  {"今天的天气怎么样？", "抱歉，我暂时无法获得天气信息!"},
  {"现在几点", ""},  // 动态内容
  {"时间", ""}       // 动态内容
};

// 动态响应缓存
std::map<String, std::vector<String>> responseCache;

// 最大缓存回复数
const int MAX_CACHE_RESPONSES = 5;

// 舵机当前角度跟踪
float currentAngles[16] = {0};

// 硬件引脚配置
const int wakeButtonPin = 0;     // 唤醒按钮
const int i2sSdPin = 2;          // INMP441 SD
const int i2sWsPin = 40;         // INMP441 WS
const int i2sSckPin = 39;        // INMP441 SCK
const int i2sOutLrcPin = 15;     // MAX98357 LRC
const int i2sOutBclkPin = 12;    // MAX98357 BCLK 
const int i2sOutDinPin = 13;     // MAX98357 DIN

// 对角步态参数
const int GAIT_CYCLE = 1200;      // 完整步态周期(ms)
const float STEP_HEIGHT = 3.0;    // 抬腿高度(cm)
const float STEP_LENGTH = 5.0;     // 步幅长度(cm)

// 音频设置
const int sampleRate = 16000;
const int bufferSize = 1024;
const int recordDuration = 3000;   // 3秒录音

// I2S配置
i2s_config_t i2s_input_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = sampleRate,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 8,
  .dma_buf_len = 256,
  .tx_desc_auto_clear = false
};

i2s_pin_config_t input_pin_config = {
  .mck_io_num = I2S_PIN_NO_CHANGE,
  .bck_io_num = i2sSckPin,
  .ws_io_num = i2sWsPin,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = i2sSdPin
};

i2s_config_t i2s_output_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = 16000,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 8,
  .dma_buf_len = 512,
  .tx_desc_auto_clear = true
};

i2s_pin_config_t output_pin_config = {
  .mck_io_num = I2S_PIN_NO_CHANGE,
  .bck_io_num = i2sOutBclkPin,
  .ws_io_num = i2sOutLrcPin,
  .data_out_num = i2sOutDinPin,
  .data_in_num = I2S_PIN_NO_CHANGE
};

// 舵机映射参数
const int SERVO_MIN_PULSE = 150;  // 最小脉冲(0度)
const int SERVO_MAX_PULSE = 600;  // 最大脉冲(180度)
const float LEG_L1 = 4.5;         // 大腿长度(cm)
const float LEG_L2 = 6.0;         // 小腿长度(cm)

// 四足舵机映射
enum ServoMap {
  LF_HIP = 8,
  LF_KNEE = 9,
  RF_HIP = 10,
  RF_KNEE = 11,
  LB_HIP = 12,
  LB_KNEE = 13,
  RB_HIP = 14,
  RB_KNEE = 15
};

// Point定义
struct Point {
  float x;
  float y;
};

// 系统状态
bool isReady = false;
bool isProcessing = false;
String systemLog = "";
unsigned long wifiConnectStartTime = 0;

// 函数声明
void addLog(String message);
String getFormattedTime();
String urlEncode(String str);
void setupServos();
void setServoAngle(unsigned char servoNum, float angle);
void resetAllServos();
Point calculateIK(float x, float y);
void trotGait(int direction, float speed);
void moveLeg(int hipServo, int kneeServo, float phase, int direction);
void smoothMove(uint8_t servo, float targetAngle, int duration);
void initializeSystem();
void wakeUpAssistant();
void processVoiceCommand(String audioFilePath);
bool isActionCommand(String command);
void executeAction(String command);
String baiduASR(String filePath);
String callDeepSeekAPI(String query);
void speakText(String text);
void httpTTS(String text);
void playStreamedAudio(Stream* stream, size_t contentLength);
String getBaiduToken();
String recordAudio();
void handleSerialCommands();
void playTestTone();
void playErrorTone();
String findBestMatch(const String& input);
float calculateSimilarity(const String& str1, const String& str2);

void setupMemoryMonitor() {
    xTaskCreatePinnedToCore([](void* param) {
        while(true) {
            // 添加内存不足保护
            size_t freeHeap = esp_get_free_heap_size();
            
            if(psramFound()) {
                size_t freePsram = ESP.getFreePsram();
                addLog("[内存]堆:" + String(freeHeap) + " PSRAM:" + String(freePsram));
                
                // 内存紧急恢复
                if(freeHeap < 8000 || freePsram < 8000) {
                    addLog("内存严重不足，执行紧急恢复");
                    resetAllServos();
                    
                    // 安全卸载I2S驱动
                    if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        i2s_driver_uninstall(I2S_NUM_0);
                        i2s_driver_uninstall(I2S_NUM_1);
                        xSemaphoreGive(i2sMutex);
                    }
                    
                    // 重启系统
                    ESP.restart();
                }
            } else {
                addLog("[内存]堆:" + String(freeHeap));
                if(freeHeap < 8000) {
                    addLog("内存严重不足，执行紧急恢复");
                    resetAllServos();
                    if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        i2s_driver_uninstall(I2S_NUM_0);
                        i2s_driver_uninstall(I2S_NUM_1);
                        xSemaphoreGive(i2sMutex);
                    }
                    ESP.restart();
                }
            }
            delay(15000); // 每15秒报告一次
        }
    }, "MemoryMonitor", 3000, nullptr, 1, nullptr, 0);
}

void maintainStorage() {
  const float CLEAN_THRESHOLD = 50.0; // 清理阈值降至50%
  
  // 健康检查
  if(!SPIFFS.exists("/")) {
    addLog("SPIFFS损坏，执行格式化");
    SPIFFS.format();
    return;
  }

  // 智能清理算法
  struct FileInfo { time_t timestamp; String path; };
  std::vector<FileInfo> files;
  
  File root = SPIFFS.open("/");
  while(File file = root.openNextFile()) {
    if(String(file.name()).indexOf("recording") != -1) {
      time_t t = file.getLastWrite(); // 直接获取修改时间
      files.push_back({t, file.name()});
    }
  }

  // 按时间排序
  std::sort(files.begin(), files.end(), [](const FileInfo& a, const FileInfo& b) {
    return a.timestamp < b.timestamp;
  });

  // 动态清理直到空间充足
  float usage = (SPIFFS.usedBytes() * 100.0f) / SPIFFS.totalBytes();
  for(auto& f : files) {
    if(usage > CLEAN_THRESHOLD) {
      SPIFFS.remove(f.path);
      usage = (SPIFFS.usedBytes() * 100.0f) / SPIFFS.totalBytes();
    } else break;
  }
}

void setup() {
  // 设置CPU频率
  setCpuFrequencyMhz(240);
  
  // 初始化串口
  Serial.begin(115200);
  while(!Serial) delay(10); // 等待
  
  // 创建I2S互斥锁
  i2sMutex = xSemaphoreCreateMutex();
  if(i2sMutex == NULL) {
    Serial.println("I2S互斥锁创建失败!");
  } else {
    Serial.println("I2S互斥锁初始化成功");
  }

  addLog("=====系统启动=====");

  // 1. 优先初始化文件系统
  if(!SPIFFS.begin(true)) {
    addLog("SPIFFS初始化失败!");
    // 尝试修复
    SPIFFS.format();
    if(!SPIFFS.begin(true)) {
      addLog("SPIFFS修复失败，系统挂起");
      while(1);
    } else {
      addLog("SPIFFS修复成功");
    }
  } else {
    addLog("SPIFFS初始化成功");
  }
  
  // 2. 初始化唤醒按钮引脚
  pinMode(wakeButtonPin, INPUT_PULLUP);
  
  // 3. 初始化 I2S输入
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_input_config, 0, NULL);
  if(err != ESP_OK) {
    addLog("I2S输入驱动安装失败:" + String(err));
    i2s_driver_uninstall(I2S_NUM_0);
    err = i2s_driver_install(I2S_NUM_0, &i2s_input_config, 0, NULL);
    if(err != ESP_OK) {
      addLog("I2S输入重试失败，系统挂起");
      while(1);
    } else {
      addLog("I2S输入重试成功");
    }
  }
  i2s_set_pin(I2S_NUM_0, &input_pin_config);
  addLog("I2S输入初始化成功");
  
  // 4. 初始化I2S输出
  err = i2s_driver_install(I2S_NUM_1, &i2s_output_config, 0, NULL);
  if(err != ESP_OK) {
    addLog("I2S输出驱动安装失败:" + String(err));
    i2s_driver_uninstall(I2S_NUM_1);
    err = i2s_driver_install(I2S_NUM_1, &i2s_output_config, 0, NULL);
    if(err != ESP_OK) {
      addLog("I2S输出重试失败，系统挂起");
      while(1);
    } else {
      addLog("I2S输出重试成功");
    }
  }
  i2s_set_pin(I2S_NUM_1, &output_pin_config);
  addLog("I2S输出初始化成功");
  
  // 5. 设置MAX98357时钟
  esp_err_t clkErr = i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  if(clkErr != ESP_OK) {
    addLog("MAX98357配置失败:" + String(clkErr));
  } else {
    addLog("MAX98357配置完成");
  }
  
  // 6. 初始化舵机控制器
  setupServos();
  
  // 7. 系统初始化
  initializeSystem();
  
  // 8. 内存信息
  if(psramFound()) {
    addLog("PSRAM可用:" + String(ESP.getPsramSize()) + "字节");
  }
  addLog("空闲内存:" + String(esp_get_free_heap_size()) + "字节");
  
  // 9. 启动内存监控任务
  xTaskCreatePinnedToCore([](void* param) {
    while(true) {
      // 添加内存不足保护
      size_t freeHeap = esp_get_free_heap_size();
      if(psramFound()) {
        size_t freePsram = ESP.getFreePsram();
        addLog("[内存]堆:" + String(freeHeap) + " PSRAM:" + String(freePsram) + 
               " 最大块:" + String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)));
        
        // 内存紧急恢复
        if(freeHeap < 8000 || freePsram < 8000) {
          addLog("内存严重不足，执行紧急恢复");
          resetAllServos();
          
          // 安全卸载I2S驱动
          if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            i2s_driver_uninstall(I2S_NUM_0);
            i2s_driver_uninstall(I2S_NUM_1);
            xSemaphoreGive(i2sMutex);
          }
          
          // 重启系统
          ESP.restart();
        }
      } else {
        addLog("[内存]堆:" + String(freeHeap) + " 最大块:" + String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)));
        if(freeHeap < 8000) {
          addLog("内存严重不足，执行紧急恢复");
          resetAllServos();
          if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            i2s_driver_uninstall(I2S_NUM_0);
            i2s_driver_uninstall(I2S_NUM_1);
            xSemaphoreGive(i2sMutex);
          }
          ESP.restart();
        }
      }
      delay(15000); // 每15秒报告一次
    }
  }, "MemoryMonitor", 3000, nullptr, 1, nullptr, 0);
  
  // 10. 预分配录音缓冲区（关键修改）
  recordBufferSize = sampleRate * sizeof(int16_t) * (recordDuration / 1000);
  
  if(psramFound()) {
    recordBuffer = (uint8_t*)ps_malloc(recordBufferSize);
    if(recordBuffer) {
      addLog("使用PSRAM预分配录音缓冲区:" + String(recordBufferSize) + "字节");
    } else {
      addLog("PSRAM分配失败! 尝试堆内存");
      recordBuffer = (uint8_t*)malloc(recordBufferSize);
    }
  } else {
    recordBuffer = (uint8_t*)malloc(recordBufferSize);
  }
  
  if(!recordBuffer) {
    addLog("致命错误:录音缓冲区分配失败!");
    ESP.restart();
  } else {
    // 初始化缓冲区为0
    memset(recordBuffer, 0, recordBufferSize);
    addLog("录音缓冲区预分配成功");
  }
  
  // 11. 播放测试音确认硬件工作正常
  playTestTone();
  addLog("=====系统初始化完成=====");
}

// ===== 修改后的loop函数 =====
void loop() {
    handleSerialCommands();
    
    if (isReady && !isProcessing && digitalRead(wakeButtonPin) == LOW) {
        wakeUpAssistant();
        delay(300);  // 按键防抖
    }
    
    // WiFi重连机制
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - wifiConnectStartTime > 30000) {  // 30秒超时
            addLog("WiFi断开，尝试重新连接...");
            initializeSystem();
        }
    }
    
    // 注意：移除了定期存储维护的代码
}


//=====舵机控制函数=====
void setupServos() {
  // 设置I2C引脚(根据您的实际连接修改)
  const int I2C_SDA = 18;  
  const int I2C_SCL = 19; 
  
  // 初始化I2C总线
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(50000);
  
  // 添加重试机制
  bool pwmInitialized = false;
  for (int i = 0; i < 5; i++) {
    if (pwm.begin()) {
      pwmInitialized = true;
      break;
    }
    addLog("PCA9685初始化失败，重试中...(" + String(i + 1) + "/5)");
    delay(200);
  }

  if (!pwmInitialized) {
    addLog("PCA9685初始化失败!请检查连接");
    return;
  }

  pwm.setPWMFreq(50);

  // 初始化舵机角度记录
  for (int i = 0; i < 16; i++) {
    currentAngles[i] = 90;
  }

  resetAllServos();
  addLog("PCA9685舵机初始化完成");
}

void setServoAngle(unsigned char servoNum, float angle) {
    // 验证舵机号范围
    if(servoNum < 8 || servoNum > 15) {
        addLog("错误:无效舵机号" + String(servoNum));
        return;
    }
    
    // 角度限幅
    angle = constrain(angle, 0, 180);
    
    // 精确脉冲计算
    float pulseRange = SERVO_MAX_PULSE - SERVO_MIN_PULSE;
    int pulse = SERVO_MIN_PULSE + (int)(angle / 180.0 * pulseRange);
    
    // 关键修复：添加脉冲安全约束
    pulse = constrain(pulse, 130, 620); // 保留10%安全余量
    
    pwm.setPWM(servoNum, 0, pulse);
    currentAngles[servoNum] = angle; // 更新记录
}


//====运动控制函数====
Point calculateIK(float targetX, float targetY) {
  // 计算目标点到关节的距离
  float legLength = sqrt(targetX * targetX + targetY * targetY);
  
  // 防止超出机械范围
  if (legLength > (LEG_L1 + LEG_L2) || legLength < abs(LEG_L1 - LEG_L2)) {
    addLog("IK错误:目标位置超出范围(" + String(targetX) + "," + String(targetY) + ")");
    Point error = {90, 90};  // 返回安全位置
    return error;
  }
  
  float alpha = atan2(targetY, targetX);
  float beta = acos((LEG_L1 * LEG_L1 + legLength * legLength - LEG_L2 * LEG_L2) / 
                   (2 * LEG_L1 * legLength));
  
  Point angles;
  angles.x = degrees(alpha + beta);  // 髋关节角度
  angles.y = degrees(acos((LEG_L1 * LEG_L1 + LEG_L2 * LEG_L2 - legLength * legLength) / 
                         (2 * LEG_L1 * LEG_L2)));  // 膝关节角度
  
  // 角度限幅
  angles.x = constrain(angles.x, 30, 150);
  angles.y = constrain(angles.y, 30, 150);
  
  return angles;
}

void moveLeg(int hipServo, int kneeServo, float phase, int direction) {
  // 确保相位在0-1范围内
  phase = fmod(phase, 1.0);
  if (phase < 0) phase += 1.0;
  
  float x, y;
  // 步态轨迹生成(椭圆路径)
  if (phase < 0.5) {  // 摆动相
    x = STEP_LENGTH * cos(2 * M_PI * phase);
    y = STEP_HEIGHT * sin(2 * M_PI * phase);
  } else {  // 支撑相
    x = STEP_LENGTH * cos(2 * M_PI * phase);
    y = 0;
  }
  
  // 方向修正
  switch (direction) {
    case 1:  // 后退
      x = -x;
      break;
    case 2:  // 左转
      // 交换x/y坐标实现转向
      y = x;
      x = 0;
      break;
    case 3:  // 右转
      y = -x;
      x = 0;
      break;
    default:  // 前进不做修改
      break;
  }
  
  Point angles = calculateIK(x, y);
  
  // 添加角度平滑处理
  float currentHip = currentAngles[hipServo];
  float targetHip = angles.x;
  float hipDiff = fabs(targetHip - currentHip);
  
  float currentKnee = currentAngles[kneeServo];
  float targetKnee = angles.y;
  float kneeDiff = fabs(targetKnee - currentKnee);
  
  // 如果角度变化过大，使用平滑过渡
  if (hipDiff > 30 || kneeDiff > 30) {
    smoothMove((unsigned char)hipServo, targetHip, 50);
    smoothMove((unsigned char)kneeServo, targetKnee, 50);
  } else {
    setServoAngle((unsigned char)hipServo, targetHip);
    setServoAngle((unsigned char)kneeServo, targetKnee);
  }
}

void trotGait(int direction, float speed) {
  // 验证输入参数
  if (direction < 0 || direction > 3) direction = 0;
  speed = constrain(speed, 0.1, 1.0);
  
  unsigned long startTime = millis();
  const int phaseDuration = GAIT_CYCLE * speed;
  
  addLog("启动步态:方向=" + String(direction) + "速度=" + String(speed, 2));
  
  while (millis() - startTime < phaseDuration) {
    float t = ((millis() - startTime) % GAIT_CYCLE) / (float)GAIT_CYCLE;
    
    // 对角腿组1(右前+左后)
    moveLeg(RF_HIP, RF_KNEE, t, direction);
    moveLeg(LB_HIP, LB_KNEE, t, direction);
    
    // 对角腿组2(左前+右后)
    moveLeg(LF_HIP, LF_KNEE, t + 0.5, direction);
    moveLeg(RB_HIP, RB_KNEE, t + 0.5, direction);
    
    delay(20);  // 控制刷新率
  }
  
  // 平滑过渡回中立位
  for (int i = 8; i <= 15; i++) {
    smoothMove(i, 90, 300);  // 300ms过渡
  }
  
  addLog("步态完成");
}

void smoothMove(uint8_t servo, float targetAngle, int duration) {
    if(servo < 8 || servo > 15) return;
    
    float startAngle = currentAngles[servo];
    unsigned long startTime = millis();
    const float MAX_ANGLE_CHANGE = 15.0f; // 最大15度/步
    
    addLog("平滑移动舵机" + String(servo) + 
          ":" + String(startAngle, 1) + "->" + String(targetAngle, 1));
    
    while(millis() - startTime < duration) {
        float progress = (millis() - startTime) / (float)duration;
        
        // 修复缓动函数计算
        float ease;
        if(progress < 0.5) {
            ease = 4 * progress * progress * progress;
        } else {
            ease = 1 - pow(-2 * progress + 2, 3) / 2;
        }
        
        float targetCurrent = startAngle + (targetAngle - startAngle) * ease;
        
        // 关键修复：增加加速度限制
        float current = currentAngles[servo];
        float delta = targetCurrent - current;
        if(fabs(delta) > MAX_ANGLE_CHANGE) {
            delta = (delta > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
            targetCurrent = current + delta;
        }
        
        // 确保角度在有效范围内
        targetCurrent = constrain(targetCurrent, 0, 180);
        setServoAngle(servo, targetCurrent);
        
        delay(10);
    }
    
    // 确保达到目标角度
    setServoAngle(servo, targetAngle);
}

void resetAllServos() {
  for (int i = 8; i <= 15; i++) {
    setServoAngle(i, 90);
  }
  addLog("舵机复位到中立位置");
}

//=====系统初始化=====
void initializeSystem() {
  addLog("===系统初始化开始===");
  addLog("连接WiFi...");
  wifiConnectStartTime = millis();
  WiFi.disconnect(true);
  WiFi.begin(ssid, password);
  
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    retryCount++;
    if (retryCount % 10 == 0) {  // 每5秒输出一次状态
      addLog("连接WiFi中..已尝试" + String(retryCount / 2) + "秒");
    }
    if (retryCount > 60) {  // 30秒后放弃
      addLog("WiFi连接超时，重启系统");
      ESP.restart();
    }
  }
  
  addLog("获取百度Token...");
  baiduAccessToken = getBaiduToken();
  if (baiduAccessToken == "") {
    addLog("百度Token获取失败,10秒后重试");
    delay(10000);
    initializeSystem();  // 重新尝试初始化
    return;
  }
  
  // 配置NTP获取时间
  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  addLog("等待NTP时间同步...");
  int timeRetry = 0;
  while (time(nullptr) < 1000000000 && timeRetry++ < 10) {
    delay(500);
  }
  addLog("当前时间:" + getFormattedTime());
  
  addLog("百度Token获取成功");
  addLog("系统准备就绪!");
  isReady = true;
  
  // 播放测试音确认硬件工作正常
  playTestTone();
}

//=====唤醒与语音处理=====
void wakeUpAssistant() {
  if(!isReady || isProcessing) return;
  isProcessing = true;
  addLog("唤醒按钮按下-开始语音捕获");
  
  // 使用预分配缓冲区
  uint8_t* audioData = recordBuffer;
  size_t dataSize = 0;
  
  // 调用录音函数
  if(!recordAudio(&audioData, &dataSize)) {
    addLog("音频录制失败");
    isProcessing = false;
    return;
  }
  
  // 处理语音命令
  String recognizedText = baiduASR(audioData, dataSize);
  
  if(recognizedText.isEmpty()) {
    addLog("ASR失败");
    isProcessing = false;
    return;
  }
  
  // 直接处理识别文本
  processVoiceCommand(recognizedText);
  
  // 内存监控
  addLog("空闲内存:" + String(esp_get_free_heap_size()));
  addLog("最大连续块:" + String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)));
}

//=====核心语音处理流程=====
float calculateSimilarity(const String& str1, const String& str2) {
  if (str1.isEmpty() || str2.isEmpty()) return 0.0f;
  
  // 计算公共字符数量
  int common = 0;
  int totalLen = max(str1.length(), str2.length());
  for (int i = 0; i < min(str1.length(), str2.length()); i++) {
    if (str1[i] == str2[i]) common++;
  }
  
  // 增加长度相似性权重
  int lenDiff = static_cast<int>(str1.length()) - static_cast<int>(str2.length());
  float lenSimilarity = 1.0f - (fabs(lenDiff) / float(totalLen));
  float charSimilarity = float(common) / float(totalLen);
  
  return (charSimilarity * 0.7 + lenSimilarity * 0.3);
}

void processVoiceCommand(String recognizedText) {
    addLog("处理语音命令: " + recognizedText);
    
    // 安全截断长度
    if (recognizedText.length() > 200) {
        recognizedText = recognizedText.substring(0, 200);
        addLog("截断过长识别文本");
    }
    
    // 预处理识别结果
    recognizedText.trim();
    recognizedText.replace("。", "");
    recognizedText.replace("?", "");
    recognizedText.replace("!", "");
    
    String response = "";
    
    // 1. 检查是否为动作命令
    if (isActionCommand(recognizedText)) {
        executeAction(recognizedText);
        response = "正在" + recognizedText + "啦~";
    }
    // 2. 处理动态内容
    else if (recognizedText.indexOf("时间") != -1 || recognizedText.indexOf("几点") != -1) {
        response = "现在是" + getFormattedTime() + "咯~";
    }
    // 3. 模糊匹配预设回答
    else {
        String matchedPreset = findBestMatch(recognizedText);
        if (!matchedPreset.isEmpty() && presetResponses.find(matchedPreset) != presetResponses.end()) {
            response = presetResponses.at(matchedPreset);
        }
        // 4. 调用 DeepSeek API
        else {
            response = callDeepSeekAPI(recognizedText);
        }
    }
    
    // 播放回答
    if (!response.isEmpty()) {
        speakText(response);
    } else {
        speakText("哎呀，刚才没听清楚呢~");
    }
    
    // 内存监控
    addLog("空闲内存: " + String(esp_get_free_heap_size()));
    isProcessing = false;
}


String findBestMatch(const String& input) {
  float bestScore = 0.0f;
  String bestMatch = "";
  
  for (const auto& preset : presetResponses) {
    const String& presetQuestion = preset.first;
    float similarity = calculateSimilarity(input, presetQuestion);
    
    // 相似度阈值+优先完全匹配
    if (similarity > bestScore && similarity > 0.65) {
      bestScore = similarity;
      bestMatch = presetQuestion;
    }
    // 完全匹配直接返回
    else if (input.equals(presetQuestion)) {
      return presetQuestion;
    }
  }
  
  if (!bestMatch.isEmpty()) {
    addLog("模糊匹配:" + input + "->" + bestMatch + "(相似度:" + String(bestScore) + ")");
  }
  return bestMatch;
}

String getFormattedTime() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char buf[20];
  strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
  return String(buf);
}

bool isActionCommand(String command) {
  String commands[] = {"前进", "后退", "左转", "右转", "停止", "跳舞"};
  for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
    if (command.indexOf(commands[i]) != -1)
      return true;
  }
  return false;
}

void executeAction(String command) {
  addLog("执行动作:" + command);
  
  if (command.indexOf("前进") != -1) {
    trotGait(0, 0.7);  // 70%速度前进
  } else if (command.indexOf("后退") != -1) {
    trotGait(1, 0.5);
  } else if (command.indexOf("左转") != -1) {
    trotGait(2, 0.6);
  } else if (command.indexOf("右转") != -1) {
    trotGait(3, 0.6);
  } else if (command.indexOf("停止") != -1) {
    resetAllServos();
  } else if (command.indexOf("跳舞") != -1) {
    // 自定义舞蹈动作序列
    for (int i = 0; i < 3; i++) {
      trotGait(0, 0.8);
      trotGait(3, 0.6);
      trotGait(2, 0.6);
      trotGait(1, 0.5);
    }
  }
}

void addLog(String message) {
  String timestamp = getFormattedTime();
  systemLog += "[" + timestamp + "]" + message + "\n";
  Serial.println("[" + timestamp + "]" + message);
}

//=====语音识别函数=====
String baiduASR(uint8_t* audioData, size_t dataSize) {
    if (dataSize == 0 || !audioData) {
        addLog("ASR错误: 音频数据无效");
        return "";
    }
    
    addLog("发送请求到百度ASR API... 数据大小: " + String(dataSize) + "字节");
    
    // 确保WiFi连接
    if (WiFi.status() != WL_CONNECTED) {
        addLog("ASR失败: WiFi断开");
        return "";
    }
    
    // 获取有效Token
    if (baiduAccessToken == "") {
        addLog("Token缺失，重新获取");
        baiduAccessToken = getBaiduToken();
        if (baiduAccessToken == "") return "";
    }
    
    String url = String(baiduAsrApiUrl) + "?cuid=" + WiFi.macAddress() + "&token=" + baiduAccessToken;
    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "audio/pcm;rate=16000");
    http.setTimeout(10000);  // 增加超时时间
    
    // 添加音频质量参数
    http.addHeader("X-Audio-Quality", "high");
    
    int httpResponseCode = http.POST(audioData, dataSize);
    
    if (httpResponseCode != 200) {
        String errorMsg = http.getString();
        addLog("ASR错误: " + String(httpResponseCode) + " 响应: " + errorMsg);
        
        // Token过期处理 (错误码110)
        if (errorMsg.indexOf("110") != -1) {
            addLog("检测到Token过期，刷新Token");
            baiduAccessToken = getBaiduToken();
        }
        
        http.end();
        return "";
    }
    
    String response = http.getString();
    http.end();
    
    // 安全截断日志长度
    addLog("ASR原始响应: " + response.substring(0, min((size_t)100, response.length())) + "...");
    
    // 解析JSON响应
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (error) {
        addLog("JSON解析错误: " + String(error.c_str()));
        return "";
    }
    
    if (doc.containsKey("result") && doc["result"].is<JsonArray>()) {
        JsonArray results = doc["result"];
        if (results.size() > 0) {
            String result = results[0].as<String>();
            addLog("识别结果: " + result);
            return result;
        }
    }
    
    // 错误信息提取
    if (doc.containsKey("err_msg")) {
        addLog("ASR错误: " + doc["err_msg"].as<String>());
    }
    
    addLog("未获取到有效识别结果");
    return "";
}

//=====DeepSeek API交互=====
String callDeepSeekAPI(String query){
  addLog("调用 DeepSeek API:" + query);
  
  //简化query防止过长
  if(query.length() > 100){
    query = query.substring(0, 100);
    addLog("Query过长已截断");
  }
  
  //检查WiFi连接
  if(WiFi.status() != WL_CONNECTED){
    addLog("DeepSeek失败:WiFi断开");
    return "哎呀，网络好像断开了呢~";
  }
  
  HTTPClient http;
  http.begin("https://api.deepseek.com/v1/chat/completions");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(deepseekApiKey));
  http.setTimeout(10000); //10秒超时
  
  //生成随机温度(0.6-0.85)增加回答多样性
  float temperature = random(60, 85) / 100.0;
  
  //创建请求体-添加随机温度参数和角色设定
  String requestBody = "{"
    "\"model\":\"deepseek-chat\","
    "\"temperature\":" + String(temperature, 1) + "," //随机温度
    "\"messages\":["
      "{\"role\":\"system\",\"content\":\"你是一个可爱的四足机器人助手，用朋友聊天语气回答。句子要简短自然(<15字)。多用语气词，结尾可以加~或表情符号。例如:'好呀~''明白啦!''要不要一起玩呢?'\"},"
      "{\"role\":\"user\",\"content\":\"" + query + "\"}"
    "]}";
  
  addLog("DeepSeek请求体:" + requestBody);
  
  //发送请求
  int httpCode = http.POST(requestBody);
  
  if(httpCode != 200){
    String error = http.getString();
    addLog("DeepSeek错误:" + String(httpCode) + "-" + error);
    http.end();
    return "抱歉，我暂时无法回答呢~";
  }
  
  String response = http.getString();
  http.end();
  addLog("DeepSeek原始响应:" + response);
  
  // 使用固定大小JSON文档（关键修改）
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, response);
  
  if(error){
    addLog("JSON解析错误:" + String(error.c_str()));
    doc.clear(); // 显式释放
    return "处理回答时出错了呢~";
  }
  
  String result = "";
  if(doc.containsKey("choices") && doc["choices"].is<JsonArray>()){
    JsonArray choices = doc["choices"];
    if(choices.size() > 0){
      JsonObject choice = choices[0];
      if(choice.containsKey("message") && choice["message"].is<JsonObject>()){
        JsonObject message = choice["message"];
        if(message.containsKey("content") && message["content"].is<String>()){
          result = message["content"].as<String>();
        }
      }
    }
  }
  
  //处理空响应
  if(result.isEmpty()){
    result = "嗯...这个问题有点难呢~";
  }
  
  //后处理:确保语气自然
  result.replace("。", "~"); //句号变波浪号
  result.replace("?", "嘛?"); //更口语化
  
  //简化响应长度
  if(result.length() > 200){
    result = result.substring(0, 200) + "...";
    addLog("响应截断至200字符");
  }
  
  //添加随机后缀增强语气(如果结尾没有波浪号)
  const char* randomSuffixes[] = {"~", "!", "哟", "呀"};
  if(!result.isEmpty() && result.charAt(result.length() - 1) != '~'){
    result += randomSuffixes[random(4)];
  }
  
  addLog("DeepSeek最终回答:" + result);
  
  doc.clear(); // 显式释放
  return result;
}

//=====TTS合成函数=====
void speakText(String text) {
  addLog("合成语音:" + text);
  
  // 增加截断长度到200字符
  if (text.length() > 200) {
    text = text.substring(0, 200);
    addLog("警告:文本过长，已截断前200字符");
  }
  
  httpTTS(text);
}

void httpTTS(String text) {
  addLog("使用百度TTS合成语音");
  
  // 更严格长度限制
  if (text.length() > 100) {
    text = text.substring(0, 100);
    addLog("TTS文本截断至100字符");
  }
  
  // 文本预处理:添加自然停顿
  text.replace("~", "，");  // 波浪号变逗号(停顿)
  text.replace("！", "，");  // 感叹号变逗号(更自然)
  
  // 情感语音参数配置
  String url = String(baiduTtsUrl) + "?tex=" + urlEncode(text) +
               "&lan=zh" +
               "&cuid=" + WiFi.macAddress() +
               "&ctp=1" +
               "&tok=" + baiduAccessToken +
               "&per=5118" +  // 情感女声
               "&spd=5" +     // 语速适中(4-6)
               "&pit=6" +     // 稍高音调(更活泼)
               "&vol=7" +     // 音量稍大
               "&aue=6";      // PCM格式
  
  HTTPClient http;
  http.begin(url);
  http.setTimeout(15000);  // 增加超时时间
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    // 流式播放
    WiFiClient* stream = http.getStreamPtr();
    size_t contentLength = http.getSize();
    playStreamedAudio(stream, contentLength);
  } else {
    String errorMsg = http.getString();
    addLog("TTS错误:" + String(httpCode) + "响应:" + errorMsg);
    playErrorTone();
  }
  http.end();
}

//=====流式音频播放====
void playStreamedAudio(Stream* stream, size_t contentLength){
  addLog("开始播放音频，长度:" + String(contentLength) + "字节");
  if(contentLength == 0){
    addLog("错误:音频内容长度为0");
    playErrorTone();
    return;
  }
  
  //设置I2S输出时钟(带互斥保护)
  if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE){
    esp_err_t clkErr = i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    xSemaphoreGive(i2sMutex);
    if(clkErr != ESP_OK){
      addLog("I2S时钟设置失败:" + String(clkErr));
      return;
    }
  }else{
    addLog("播放失败:无法获取I2S锁");
    return;
  }
  
  size_t bytesPlayed = 0;
  
  // 使用栈分配的小缓冲区（关键修改）
  uint8_t audioBuffer[512]; 
  
  //跳过WAV文件头(带安全检测)
  const size_t HEADER_SIZE = 44;
  if(contentLength > HEADER_SIZE){
    uint8_t header[HEADER_SIZE];
    size_t headerRead = stream->readBytes(header, HEADER_SIZE);
    if(headerRead == HEADER_SIZE && header[0] == 'R' && header[1] == 'I' && header[2] == 'F' && header[3] == 'F'){
      addLog("跳过WAV文件头");
      bytesPlayed = HEADER_SIZE;
    }else{
      //播放头数据(带互斥保护)
      if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE){
        size_t written = 0;
        esp_err_t err = i2s_write(I2S_NUM_1, header, headerRead, &written, portMAX_DELAY);
        xSemaphoreGive(i2sMutex);
        if(err != ESP_OK){
          addLog("I2S写入错误:" + String(err));
        } else if(written > 0){
          bytesPlayed += written;
        }
      }
    }
  }
  
  //播放剩余音频数据
  while(bytesPlayed < contentLength){
    //内存保护:定期检查内存状态
    if(esp_get_free_heap_size() < 8000){
      addLog("内存不足，停止播放");
      break;
    }
    
    size_t bytesToRead = min(sizeof(audioBuffer), (size_t)(contentLength - bytesPlayed));
    size_t bytesRead = stream->readBytes(audioBuffer, bytesToRead);
    
    if(bytesRead > 0){
      //带超时和互斥保护的写入
      if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE){
        size_t written = 0;
        esp_err_t err = i2s_write(I2S_NUM_1, audioBuffer, bytesRead, &written, portMAX_DELAY);
        xSemaphoreGive(i2sMutex);
        if(err != ESP_OK){
          addLog("I2S写入错误:" + String(err));
          break;
        }
        if(written > 0){
          bytesPlayed += written;
        }
      }else{
        addLog("写入超时,停止播放");
        break;
      }
    }else{
      addLog("读取音频数据失败");
      break;
    }
  }
  
  addLog("音频播放完成。共播放:" + String(bytesPlayed) + "/" + String(contentLength) + "字节");
  
  // 添加内存报告（关键修改）
  addLog("播放后连续内存块:" + String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)) + "字节");
}

//=====百度Token获取=====
String getBaiduToken() {
    addLog("获取百度Token...");
    const int MAX_RETRIES = 3;
    
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        HTTPClient http;
        http.setReuse(true);
        http.setTimeout(5000);
        
        String url = "https://aip.baidubce.com/oauth/2.0/token?";
        url += "grant_type=client_credentials";
        url += "&client_id=" + urlEncode(baiduApiKey);
        url += "&client_secret=" + urlEncode(baiduSecretKey);
        
        http.begin(url);
        int httpCode = http.GET();
        
        if (httpCode == 200) {
            String payload = http.getString();
            http.end();
            
            DynamicJsonDocument doc(512);
            DeserializationError error = deserializeJson(doc, payload);
            
            if (!error && doc.containsKey("access_token")) {
                String token = doc["access_token"].as<String>();
                addLog("Token获取成功");
                return token;
            }
        }
        
        String errorMsg = http.getString();
        http.end();
        addLog("Token获取失败(" + String(attempt) + "/3): " + String(httpCode) + " - " + errorMsg);
        delay(1000);
    }
    
    addLog("Token获取完全失败");
    return "";
}

//=====URL编码函数=====
String urlEncode(String str) {
  String encoded = "";
  char c;
  char hex[4];
  
  for (unsigned int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += c;
    } else if (c == ' ') {
      encoded += '+';
    } else {
      sprintf(hex, "%%%02X", (unsigned char)c);
      encoded += hex;
    }
  }
  return encoded;
}

//=====测试音播放====
void playTestTone() {
    addLog("播放测试音...");
    const int toneDuration = 500; // ms
    const int samples = 16000 * toneDuration / 1000;
    
    // 使用PSRAM优化音频缓冲区
    int16_t* sineWave = nullptr;
    if(psramFound()) {
        sineWave = (int16_t*)ps_malloc(samples * sizeof(int16_t));
        addLog("使用PSRAM生成测试音");
    } else {
        sineWave = (int16_t*)malloc(samples * sizeof(int16_t));
    }
    
    if(!sineWave) {
        addLog("内存分配失败");
        return;
    }
    
    // 生成正弦波
    for(int i = 0; i < samples; i++) {
        sineWave[i] = (int16_t)(32767 * sin(2 * M_PI * 440 * i / 16000.0));
    }
    
    // 通过I2S播放
    size_t bytesWritten = 0;
    size_t totalBytes = samples * sizeof(int16_t);
    
    while(bytesWritten < totalBytes) {
        size_t chunkSize = min((size_t)1024, totalBytes - bytesWritten);
        size_t written = 0;
        
        // 添加互斥锁保护I2S资源
        if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            i2s_write(I2S_NUM_1, 
                     (const char*)(sineWave + bytesWritten / sizeof(int16_t)),
                     chunkSize, &written, portMAX_DELAY);
            xSemaphoreGive(i2sMutex);
        }
        
        if(written > 0) {
            bytesWritten += written;
        }
    }
    
    // 关键修复：释放内存
    free(sineWave);
    addLog("测试音播放完成");
}

//=====错误提示音====
void playErrorTone() {
    addLog("播放错误提示音");
    const int toneDuration = 200; // ms
    const int samples = 16000 * toneDuration / 1000;
    
    int16_t* sineWave = (int16_t*)malloc(samples * sizeof(int16_t));
    if(!sineWave) return;
    
    for(int i = 0; i < samples; i++) {
        sineWave[i] = (int16_t)(32767 * sin(2 * M_PI * 880 * i / 16000.0));
    }
    
    size_t bytesWritten = 0;
    size_t totalBytes = samples * sizeof(int16_t);
    
    while(bytesWritten < totalBytes) {
        size_t chunkSize = min((size_t)512, totalBytes - bytesWritten);
        size_t written = 0;
        
        // 互斥锁保护
        if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            i2s_write(I2S_NUM_1, 
                     (const char*)(sineWave + bytesWritten / sizeof(int16_t)),
                     chunkSize, &written, portMAX_DELAY);
            xSemaphoreGive(i2sMutex);
        }
        
        if(written > 0) {
            bytesWritten += written;
        }
    }
    
    // 关键修复：释放内存
    free(sineWave);
}

//=====串口命令处理====
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "restart") {
      addLog("收到重启命令");
      ESP.restart();
    } else if (command == "cache.list") {
      Serial.println("====缓存列表===");
      for (const auto& entry : responseCache) {
        Serial.print(entry.first + " -> ");
        for (const String& response : entry.second) {
          Serial.print(response + " | ");
        }
        Serial.println();
      }
      Serial.println("===============");
    } else if (command == "preset.list") {
      Serial.println("====预设回答列表====");
      for (const auto& entry : presetResponses) {
        Serial.println(entry.first + " -> " + entry.second);
      }
      Serial.println("=================");
    } else if (command == "free") {
      size_t free_psram = 0;
      if (psramFound()) {
        free_psram = ESP.getFreePsram();
      }
      Serial.printf("内存状态: 堆:%d, PSRAM:%d\n",
                   esp_get_free_heap_size(), free_psram);
    } else if (command.startsWith("addpreset")) {
      // 格式: addpreset|问题|答案
      int firstPipe = command.indexOf('|');
      int secondPipe = command.indexOf('|', firstPipe + 1);
      
      if (firstPipe > 0 && secondPipe > firstPipe) {
        String question = command.substring(firstPipe + 1, secondPipe);
        String answer = command.substring(secondPipe + 1);
        
        // 更新预设库
        const_cast<std::map<String, String>&>(presetResponses)[question] = answer;
        Serial.println("已添加预设: " + question + " -> " + answer);
      } else {
        Serial.println("格式错误，应为: addpreset|问题|答案");
      }
    } else {
      Serial.println("未知命令");
    }
  }
}

//=====音频录制函数=====
bool recordAudio(uint8_t** buffer, size_t* size) {
  if(isRecording) {
    addLog("录音失败:已有录音在进行中");
    return false;
  }
  
  isRecording = true;
  
  // 使用预分配缓冲区
  *buffer = recordBuffer;
  *size = recordBufferSize;
  
  // 设置麦克风时钟
  esp_err_t err = ESP_FAIL;
  if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    err = i2s_set_clk(I2S_NUM_0, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    xSemaphoreGive(i2sMutex);
  }
  if(err != ESP_OK) {
    addLog("设置麦克风时钟失败:" + String(err));
    isRecording = false;
    return false;
  }
  
  size_t bytesWritten = 0;
  uint32_t startTime = millis();
  uint8_t* currentPos = *buffer;
  
  // 静音检测参数
  int silentCount = 0;
  const int maxSilentCycles = 50;
  const int silentThreshold = 500;  // 静音阈值
  
  // 音频质量监控
  int16_t minLevel = 32767;
  int16_t maxLevel = -32768;
  long totalEnergy = 0;
  int sampleCount = 0;
  
  addLog("开始录音...");
  
  while((millis() - startTime) < recordDuration) {
    // 添加内存连续块检查
    size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    if(largestBlock < 1024) { // 1KB安全阈值
      addLog("连续内存不足!最大块:" + String(largestBlock) + "字节");
      break;
    }
    
    // 计算本次要读取的字节数
    size_t bytesToRead = min((size_t)512, (size_t)(*size - bytesWritten));
    if(bytesToRead == 0) break;
    
    size_t bytesRead = 0;
    if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      i2s_read(I2S_NUM_0, currentPos, bytesToRead, &bytesRead, portMAX_DELAY);
      xSemaphoreGive(i2sMutex);
    } else {
      addLog("录音超时");
      break;
    }
    
    if(bytesRead > 0) {
      // 音频质量分析
      int16_t* samples = (int16_t*)currentPos;
      int samplesRead = bytesRead / sizeof(int16_t);
      long frameEnergy = 0;
      bool hasVoice = false;
      
      for(int i = 0; i < samplesRead; i++) {
        int16_t sample = samples[i];
        frameEnergy += abs(sample);
        if(sample < minLevel) minLevel = sample;
        if(sample > maxLevel) maxLevel = sample;
        if(abs(sample) > silentThreshold) {
          hasVoice = true;
        }
      }
      
      totalEnergy += frameEnergy;
      sampleCount += samplesRead;
      
      // 静音检测
      if(!hasVoice && frameEnergy/samplesRead < silentThreshold) {
        silentCount++;
      } else {
        silentCount = 0; // 重置静音计数器
      }
      
      // 连续静音检测
      if(silentCount > maxSilentCycles) {
        addLog("检测到持续静音，停止录音");
        break;
      }
      
      currentPos += bytesRead;
      bytesWritten += bytesRead;
    }
    
    // 内存状态检查
    if(esp_get_free_heap_size() < 8000) {
      addLog("内存不足，停止录音");
      break;
    }
  }
  
  // 音频质量报告
  if(sampleCount > 0) {
    float avgEnergy = totalEnergy / (float)sampleCount;
    float dynamicRange = 20 * log10((float)maxLevel / abs(minLevel));
    addLog("音频质量: 动态范围=" + String(dynamicRange, 1) + "dB 平均能量=" + String(avgEnergy, 0));
  }
  
  *size = bytesWritten; // 更新实际录制的长度
  addLog("录制完成:" + String(bytesWritten) + "字节");
  isRecording = false;
  return true;
}
