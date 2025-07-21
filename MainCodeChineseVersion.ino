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

#define CACHE_LIFETIME_MS 86400000 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
enum WakeState { WAKE_IDLE, WAKE_STARTING, WAKE_LISTENING, WAKE_PROCESSING };
WakeState wakeState = WAKE_IDLE;
bool isServoInitialized = false;
SemaphoreHandle_t i2cMutex = NULL;  // I2C专用互斥锁

const size_t PRE_RECORD_BUFFER_SIZE = 1024;  // 预录音缓冲区大小
unsigned long lastActivityTime = 0;          // 最后活动时间戳
const unsigned long WAKELOCK_TIMEOUT = 10000; // 唤醒超时10秒
const float VOICE_THRESHOLD_RATIO = 0.35f; // 35%音量
const int MIN_VOICE_FRAMES = 5; // 最低语音帧数

// 预设问答库
const std::map<String, String> presetResponses = {
    {"你好", "嘿!哈喽啊，今天想聊点什么呀?"},
    {"你好吗", "电量满格!随时准备陪你摸鱼呢~你呢?"},
    {"你叫什么", "他们都叫我铁爪!不过你想给我起新名字也可以哦(*^▽▽^*)"},
    {"谢谢", "嘿嘿不客气啦!需要我转个圈表示开心吗?"},  // 添加逗号
    {"拜拜", "要走了吗...记得下次带零食来看我呀!"},
    {"再见", "别忘了我在这等你回来玩~"},
    {"今天天气", "你抬头看看窗外嘛~我的摄像头分辨率太低看不清啦"},
    {"讲个笑话", "为什么机器人不去健身房?因为怕练出软件包!"},
    {"饿了", "我的充电桩在那边→不过你的话...建议打开冰箱看看?"},
    {"喜欢我吗", "这还用问!你可是我唯一的电源管理员呀!"},
    {"现在几点", "喵喵钟显示现在是:"},  // 动态内容
    {"时间", "喵了一眼我的电子骨头:"},  // 动态内容
    {"早安", "早上好呀!今天也要元气满满哦~"},
    {"晚安", "晚安...记得给我充电哦... Zzz"},
    {"你好棒", "被夸奖了好开心!要再来一段舞蹈吗?"},
    {"心情不好", "别难过,我陪你听会儿音乐吧~"},
    {"讲个故事", "从前有只机器狗...它最大的愿望是每天陪在你身边!"},
    {"你会什么", "我会摇尾巴,转圈圈,跳舞,还能陪你聊天!试试对我说'跳舞'?"}
};


struct CachedResponse {
    std::vector<String> responses;
    unsigned long timestamp = 0;
};
// 动态响应缓存
std::map<String, CachedResponse> responseCache;
// 最大缓存回复数
const int MAX_CACHE_RESPONSES = 20;

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

// Diagonal gait parameters
const int GAIT_CYCLE = 1200;      // Complete gait cycle (ms)
const float STEP_HEIGHT = 3.0;    // Leg lift height (cm)
const float STEP_LENGTH = 5.0;     // Stride length (cm)

// 音频设置
const int sampleRate = 16000;
const int bufferSize = 1024;
const int recordDuration = 3000;   // 3秒录音
const int SAMPLE_RATE_TOLERANCE = 50; // Hz

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


const int SERVO_MIN_PULSE = 150;  
const int SERVO_MAX_PULSE = 600;  
const float LEG_L1 = 4.5;         // thigh length（cm）
const float LEG_L2 = 6.0;         // calf length(cm)

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
int16_t sampleAmbientNoise();
int calculateSilenceThreshold(int16_t noiseLevel);

// 缓存初始化函数
void initializeCache() {
    // 检查缓存文件是否存在
    if (!SPIFFS.exists("/cache.json")) {
        addLog("缓存文件不存在，跳过初始化");
        return;
    }
    
    // 打开缓存文件
    File file = SPIFFS.open("/cache.json", "r");
    if (!file) {
        addLog("缓存文件打开失败");
        return;
    }
    
    // 解析 JSON
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        addLog("缓存解析错误:" + String(error.c_str()));
        return;
    }
    
    // 加载缓存数据
    int count = 0;
    for (JsonPair kv : doc.as<JsonObject>()) {
        String question = kv.key().c_str();
        JsonArray responses = kv.value().as<JsonArray>();
        
        CachedResponse entry;
        entry.timestamp = millis(); // 设置时间戳
        
        for (JsonObject resp : responses) {
            entry.responses.push_back(resp["r"].as<String>());
        }
        
        responseCache[question] = entry;
        count++;
    }
    addLog("缓存加载成功:" + String(count) + "条记录");
}
// 缓存保存函数
void saveCacheToStorage() {
    // 创建JSON文档
    StaticJsonDocument<4096> doc;
    
    // ==== 缓存过期检查 ====
    unsigned long currentTime = millis();
    int removedCount = 0;
    
    // 遍历缓存并移除过期条目
    for (auto it = responseCache.begin(); it != responseCache.end(); ) {
        // 检查是否过期
        if (it->second.timestamp != 0 && 
            currentTime - it->second.timestamp > CACHE_LIFETIME_MS) {
            it = responseCache.erase(it);
            removedCount++;
        } else {
            ++it;
        }
    }
    
    if (removedCount > 0) {
        addLog("缓存清理: 移除" + String(removedCount) + "个过期条目");
    }
    // ===================
    
    // 填充缓存数据
    for (auto& pair : responseCache) {
        JsonArray arr = doc.createNestedArray(pair.first);
        for (String resp : pair.second.responses) {
            JsonObject item = arr.createNestedObject();
            item["r"] = resp;
        }
    }
    
    // 安全写入(先写临时文件)
    File file = SPIFFS.open("/cache.json.tmp", "w");
    if (!file) {
        addLog("缓存保存失败:无法创建临时文件");
        return;
    }
    
    size_t bytes = serializeJson(doc, file);
    file.close();
    
    if (bytes == 0) {
        addLog("缓存保存失败:写入0字节");
        SPIFFS.remove("/cache.json.tmp");
        return;
    }
    
    // 重命名替换原文件
    if (SPIFFS.exists("/cache.json")) {
        SPIFFS.remove("/cache.json");
    }
    
    SPIFFS.rename("/cache.json.tmp", "/cache.json");
    
    addLog("缓存保存成功:" + String(bytes) + "字节。" + 
           String(responseCache.size()) + "条记录");
}
//储存维护
void setupMemoryMonitor() {
    xTaskCreatePinnedToCore(
        [](void* param) {
            while (true) {
                // 添加内存不足保护
                size_t freeHeap = esp_get_free_heap_size();
                
                if (psramFound()) {
                    size_t freePsram = ESP.getFreePsram();
                    
                    // 仅当内存低于阈值时才记录日志
                    if (freeHeap < 15000 || freePsram < 15000) {
                        addLog("[内存]堆: " + String(freeHeap) + " PSRAM: " + String(freePsram) + 
                               " 最大块: " + String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)));
                    }
                    
                    // 内存紧急恢复
                    if (freeHeap < 10000 || freePsram < 10000) {
                        addLog("内存严重不足，执行紧急恢复");
                        resetAllServos();
                        
                        // 安全卸载I2S驱动
                        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            i2s_driver_uninstall(I2S_NUM_0);
                            i2s_driver_uninstall(I2S_NUM_1);
                            xSemaphoreGive(i2sMutex);
                        }
                        
                        // 重启系统
                        ESP.restart();
                    }
                } else {
                    if (freeHeap < 15000) {
                        addLog("[内存]堆: " + String(freeHeap) + 
                               " 最大块: " + String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)));
                    }
                    
                    if (freeHeap < 10000) {
                        addLog("内存严重不足，执行紧急恢复");
                        resetAllServos();
                        
                        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            i2s_driver_uninstall(I2S_NUM_0);
                            i2s_driver_uninstall(I2S_NUM_1);
                            xSemaphoreGive(i2sMutex);
                        }
                        
                        ESP.restart();
                    }
                }
                
                delay(30000);  // 每30秒报告一次
            }
        },
        "MemoryMonitor", 4096, nullptr, 1, nullptr, 0  // 增加堆栈大小
    );
}
void maintainStorage() {
    const float CLEAN_THRESHOLD = 50.0; // 清理阈至50%
    
    // 健康检查
    if (!SPIFFS.exists("/")) {
        addLog("SPIFFS损坏，执行格式化");
        SPIFFS.format();
        return;
    }
    
    // === 新增缓存维护 ===
    if (SPIFFS.usedBytes() > SPIFFS.totalBytes() * 0.7) {
        int removeCount = responseCache.size() / 2;
        while (removeCount-- > 0 && !responseCache.empty()) {
            auto oldest = responseCache.begin();
            responseCache.erase(oldest);
        }
        saveCacheToStorage();
        addLog("存储告警: 清理" + String(removeCount) + "条缓存");
    }
    // === 结束新增部分 ===
    
    // 智能清理算法
    struct FileInfo {
        time_t timestamp;
        String path;
    };
    
    std::vector<FileInfo> files;
    File root = SPIFFS.open("/");
    while (File file = root.openNextFile()) {
        if (String(file.name()).indexOf("recording") != -1) {
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
    for (auto& f : files) {
        if (usage > CLEAN_THRESHOLD) {
            SPIFFS.remove(f.path);
            usage = (SPIFFS.usedBytes() * 100.0f) / SPIFFS.totalBytes();
        } else break;
    }
}
void safeI2SUninstall() {
    if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // 直接尝试卸载，不检查是否已安装
        i2s_driver_uninstall(I2S_NUM_0);
        i2s_driver_uninstall(I2S_NUM_1);
        addLog("I2S驱动已卸载");
        xSemaphoreGive(i2sMutex);
        addLog("I2S安全卸载完成");
    } else {
        addLog("警告：无法获取I2S互斥锁");
    }
}
//初始化
void setup() {
    // 设置CPU频率
    setCpuFrequencyMhz(240);
    
    // 初始化串口
    Serial.begin(115200);
    while (!Serial) delay(10);  // 等待串口连接
    
    // ==== 创建互斥锁 ====
    // I2S互斥锁
    i2sMutex = xSemaphoreCreateMutex();
    if (i2sMutex == NULL) {
        Serial.println("I2S互斥锁创建失败!");
    } else {
        Serial.println("I2S互斥锁初始化成功");
    }
    
    // I2C互斥锁
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("I2C互斥锁创建失败!");
    } else {
        Serial.println("I2C互斥锁初始化成功");
    }
    
    addLog("====系统启动====");
    
    // 1. 优先初始化文件系统
    if (!SPIFFS.begin(true)) {
        addLog("SPIFFS初始化失败!");
        // 尝试修复
        SPIFFS.format();
        if (!SPIFFS.begin(true)) {
            addLog("SPIFFS修复失败，系统挂起");
            while (1);
        } else {
            addLog("SPIFFS修复成功");
        }
    } else {
        addLog("SPIFFS初始化成功");
    }
    
    // 2. 初始化唤醒按钮引脚
    pinMode(wakeButtonPin, INPUT_PULLUP);
    
    // 3. 初始化I2S输入
    i2s_input_config.dma_buf_len = 512;    // 增加缓冲区长度
    i2s_input_config.dma_buf_count = 16;   // 增加缓冲区数量
    
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_input_config, 0, NULL);
    if (err != ESP_OK) {
        addLog("I2S输入驱动安装失败:" + String(err));
        i2s_driver_uninstall(I2S_NUM_0);
        err = i2s_driver_install(I2S_NUM_0, &i2s_input_config, 0, NULL);
        if (err != ESP_OK) {
            addLog("I2S输入重试失败，系统挂起");
            while (1);
        } else {
            addLog("I2S输入重试成功");
        }
    }
    
    i2s_set_pin(I2S_NUM_0, &input_pin_config);
    addLog("I2S输入初始化成功");
    
    // 4. 初始化I2S输出
    i2s_output_config.dma_buf_len = 1024;   // 增加缓冲区长度
    i2s_output_config.dma_buf_count = 8;    // 增加缓冲区数量
    
    err = i2s_driver_install(I2S_NUM_1, &i2s_output_config, 0, NULL);
    if (err != ESP_OK) {
        addLog("I2S输出驱动安装失败:" + String(err));
        i2s_driver_uninstall(I2S_NUM_1);
        err = i2s_driver_install(I2S_NUM_1, &i2s_output_config, 0, NULL);
        if (err != ESP_OK) {
            addLog("I2S输出重试失败，系统挂起");
            while (1);
        } else {
            addLog("I2S输出重试成功");
        }
    }
    
    i2s_set_pin(I2S_NUM_1, &output_pin_config);
    addLog("I2S输出初始化成功");
    
    // 5. 设置MAX98357时钟
    esp_err_t clkErr = i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (clkErr != ESP_OK) {
        addLog("MAX98357配置失败:" + String(clkErr));
    } else {
        addLog("MAX98357配置完成");
    }
    
    // 6. 初始化舵机控制器 - 使用静默初始化
    setupServos();  // 内部已修改为静默初始化
    
    // 确保舵机初始化完成
    if (!isServoInitialized) {
        addLog("舵机初始化失败。系统挂起");
        while (1);
    }
    
    // 7. 初始化缓存
    initializeCache();
    
    // 8. 系统初始化(网络连接等)
    initializeSystem();
    
    // 9. 内存信息
    if (psramFound()) {
        addLog("PSRAM可用:" + String(ESP.getPsramSize()) + "字节");
        addLog("空闲内存:" + String(esp_get_free_heap_size()) + "字节");
    }
    
    // 10. 启动内存监控任务
    xTaskCreatePinnedToCore(
        [](void* param) {
            while (true) {
                // 内存监控逻辑
                size_t freeHeap = esp_get_free_heap_size();
                if (psramFound()) {
                    size_t freePsram = ESP.getFreePsram();
                    // 仅当内存低于阈值时才记录日志
                    if (freeHeap < 15000 || freePsram < 15000) {
                        addLog("[内存]堆:" + String(freeHeap) + " PSRAM:" + String(freePsram) +
                            "最大块:" + String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)));
                    }
                    // 内存紧急恢复
                    if (freeHeap < 10000 || freePsram < 10000) {
                        addLog("内存严重不足，执行紧急恢复");
                        resetAllServos();
                        // 安全卸载I2S驱动
                        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            i2s_driver_uninstall(I2S_NUM_0);
                            i2s_driver_uninstall(I2S_NUM_1);
                            xSemaphoreGive(i2sMutex);
                        }
                        // 重启系统
                        ESP.restart();
                    }
                } else {
                    if (freeHeap < 15000) {
                        addLog("[内存]堆:" + String(freeHeap) + "最大块:" +
                            String(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT)));
                    }
                    if (freeHeap < 10000) {
                        addLog("内存严重不足，执行紧急恢复");
                        resetAllServos();
                        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            i2s_driver_uninstall(I2S_NUM_0);
                            i2s_driver_uninstall(I2S_NUM_1);
                            xSemaphoreGive(i2sMutex);
                        }
                        ESP.restart();
                    }
                }
                delay(30000);  // 每30秒报告
            }
        },
        "MemoryMonitor",
        4096,  // 增加堆栈大小
        nullptr,
        1,
        nullptr,
        0
    );
    
    // ===== 11. 录音缓冲区安全分配方案 =====
    recordBufferSize = sampleRate * sizeof(int16_t) * (recordDuration / 1000);
    bool bufferAllocated = false;
    
    // 首先尝试PSRAM
    if (psramFound()) {
        recordBuffer = (uint8_t*)ps_malloc(recordBufferSize);
        if (recordBuffer) {
            addLog("使用PSRAM预分配录音缓冲区:" + String(recordBufferSize) + "字节");
            bufferAllocated = true;
        }
    }
    
    // 如果PSRAM分配失败，尝试堆内存
    if (!bufferAllocated) {
        recordBuffer = (uint8_t*)malloc(recordBufferSize);
        if (recordBuffer) {
            addLog("使用堆内存预分配录音缓冲区:" + String(recordBufferSize) + "字节");
            bufferAllocated = true;
        }
    }
    
    // 安全降级方案
    if (!bufferAllocated) {
        addLog("主录音缓冲区分配失败，启用安全模式");
        
        // 第一级降级：2秒录音
        const int fallbackDuration = 2000;
        recordBufferSize = sampleRate * sizeof(int16_t) * (fallbackDuration / 1000);
        recordBuffer = (uint8_t*)malloc(recordBufferSize);
        
        if (recordBuffer) {
            addLog("安全模式录音缓冲区(2秒):" + String(recordBufferSize) + "字节");
            bufferAllocated = true;
        } 
        // 第二级降级：1.5秒录音
        else {
            const int safeDuration = 1500;
            recordBufferSize = sampleRate * sizeof(int16_t) * (safeDuration / 1000);
            recordBuffer = (uint8_t*)malloc(recordBufferSize);
            
            if (recordBuffer) {
                addLog("紧急安全模式录音缓冲区(1.5秒):" + String(recordBufferSize) + "字节");
                bufferAllocated = true;
            } 
            // 最终降级：禁用录音功能
            else {
                addLog("严重错误：录音缓冲区分配失败！禁用录音功能");
                recordBufferSize = 0; // 标记为禁用
            }
        }
    }
    
    // 初始化缓冲区
    if (bufferAllocated) {
        memset(recordBuffer, 0, recordBufferSize);
        addLog("录音缓冲区初始化完成");
    } else {
        addLog("录音功能完全禁用");
    }
    
    // 12. 播放测试音确认硬件工作正常
    playTestTone();
    addLog("====系统初始化完成====");
}
// loop函数 
void loop() {
    handleSerialCommands();
    
    // 检查唤醒按钮状态
    if (digitalRead(wakeButtonPin) == LOW) { // 开关开启
        // 检查超时
        if (lastActivityTime > 0 && millis() - lastActivityTime > WAKELOCK_TIMEOUT) {
            addLog("唤醒超时，重置状态");
            lastActivityTime = 0;
        }
        
        // 处理新请求
        if (!isProcessing) {
            wakeUpAssistant();
            // 添加防抖延迟
            delay(100);
        }
    } else { // 开关关闭
        // 重置唤醒状态
        lastActivityTime = 0;
    }
    
    // WiFi重连机制
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - wifiConnectStartTime > 30000) {
            addLog("WiFi断开，尝试重新连接...");
            initializeSystem();
        }
    }
}
//舵机控制函数
void setupServos() {
    // I2C引脚设置
    const int I2C_SDA = 18;
    const int I2C_SCL = 19;
    
    // 初始化I2C总线
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(50000); // 50kHz时钟
    
    // 添加重试机制
    bool pwmInitialized = false;
    for (int i = 0; i < 5; i++) {
        if (pwm.begin()) {
            pwmInitialized = true;
            break;
        }
        delay(200);
    }
    
    if (!pwmInitialized) {
        addLog("PCA9685初始化失败!请检查连接");
        isServoInitialized = false;
        return;
    }
    
    pwm.setPWMFreq(50); // 舵机PWM频率
    
    // ==== 关键修改: 静默初始化舵机 ====
    isServoInitialized = true;
    addLog("静默初始化舵机至中立位置...");
    
    for (int servo = 8; servo <= 15; servo++) {
        // 直接计算中立位置脉冲
        float pulseRange = SERVO_MAX_PULSE - SERVO_MIN_PULSE;
        int pulse = SERVO_MIN_PULSE + (int)(90.0 / 180.0 * pulseRange);
        pulse = constrain(pulse, 130, 620);  // 安全约束
        
        // 静默设置（不打印角度日志）
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            pwm.setPWM(servo, 0, pulse);
            xSemaphoreGive(i2cMutex);
        }
        
        // 更新角度跟踪
        currentAngles[servo] = 90;
        delay(50); // 添加延迟防止电源波动
    }
    addLog("舵机静默初始化完成");
}
void setServoAngle(unsigned char servoNum, float angle) {
    // 关键修复: 禁用不必要的错误日志
    if (!isServoInitialized) {
        return; // 不打印日志直接返回
    }
    
    // 严格验证舵机号范围
    if (servoNum < 8 || servoNum > 15) {
        addLog("错误:无效舵机号" + String(servoNum));
        return;
    }
    
    // 角度限幅
    angle = constrain(angle, 0, 180);
    
    // 精确脉冲计算
    float pulseRange = SERVO_MAX_PULSE - SERVO_MIN_PULSE;
    int pulse = SERVO_MIN_PULSE + (int)(angle / 180.0 * pulseRange);
    
    // 关键修复: 添加脉冲安全约束
    pulse = constrain(pulse, 130, 620);
    
    // 添加调试日志
    String debugMsg = "设置舵机" + String(servoNum) + "角度:" +
                     String(angle) + "°->脉冲:" + String(pulse);
    addLog(debugMsg);
    
    // 添加I2C互斥锁保护
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        pwm.setPWM(servoNum, 0, pulse);
        xSemaphoreGive(i2cMutex);
    } else {
        addLog("警告:获取I2C互斥锁超时");
    }
    
    // 更新当前角度
    currentAngles[servoNum] = angle;
}
//运动控制函数
Point calculateIK(float targetX, float targetY, bool isLeftLeg) {
    // 腿长参数
    const float LEG_L1 = 4.5; // 大腿长度(cm)
    const float LEG_L2 = 6.0; // 小腿长度(cm)
    
    // 计算距离
    float distance = sqrt(targetX * targetX + targetY * targetY);
    
    // 检查是否可达
    if(distance > LEG_L1 + LEG_L2 || distance < fabs(LEG_L1 - LEG_L2)) {
        // 不可达位置，返回中立角度
        return {90, 90};
    }
    
    // 计算膝关节角度
    float kneeAngle = acos((LEG_L1 * LEG_L1 + LEG_L2 * LEG_L2 - distance * distance) / 
                          (2 * LEG_L1 * LEG_L2));
    kneeAngle = degrees(kneeAngle);
    
    // 计算髋关节角度
    float alpha = atan2(targetY, targetX);
    float beta = acos((LEG_L1 * LEG_L1 + distance * distance - LEG_L2 * LEG_L2) / 
                     (2 * LEG_L1 * distance));
    float hipAngle = degrees(alpha + beta);
    
    // ==== 关键修改:确保角度范围正确 ====
    // 髋关节角度范围: 30-150度 (防止过度弯曲)
    hipAngle = constrain(hipAngle, 30, 150);
    
    // 膝关节角度范围: 30-150度
    kneeAngle = 180 - kneeAngle; // 转换为舵机角度
    kneeAngle = constrain(kneeAngle, 30, 150);
    
    // ==== 关键修改:根据腿部位置调整角度 ====
    // 左侧腿直接返回结果
    if(isLeftLeg) {
        return {hipAngle, kneeAngle};
    } 
    // 右侧腿髋关节角度反转
    else {
        return {180 - hipAngle, kneeAngle};
    }
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

    // ====关键修改:统一方向控制逻辑====
    // 根据运动方向调整轨迹
    switch (direction) {
        case 0:  // 前进
            x = -fabs(x);  // 前进方向为负
            break;
        case 1:  // 后退
            x = fabs(x);  // 后退方向为正
            break;
        case 2:  // 左转
            // 左侧腿后退，右侧腿前进
            if (hipServo == LF_HIP || hipServo == LB_HIP) {
                x = fabs(x);  // 左侧腿后退
            } else {
                x = -fabs(x);  // 右侧腿前进
            }
            break;
        case 3:  // 右转
            // 左侧腿前进，右侧腿后退
            if (hipServo == LF_HIP || hipServo == LB_HIP) {
                x = -fabs(x);  // 左侧腿前进
            } else {
                x = fabs(x);  // 右侧腿后退
            }
            break;
    }

    // ====关键修改:调用IK时传入腿部位置====
    bool isLeftLeg = (hipServo == LF_HIP || hipServo == LB_HIP);
    
    // 计算逆运动学角度
    Point angles = calculateIK(x, y, isLeftLeg);

    // ====左侧角度修正:确保左右对称运动====
    if (!isLeftLeg) {
        // 右侧髋关节镜像处理
        angles.x = 180 - angles.x;
    }

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
float parseDistanceCommand(String command) {
    // 关键词映射表(单位:厘米)
    std::map<String, float> distanceMap = {
        {"一点", 2.0}, {"微微", 2.0}, {"稍微", 2.0}, {"几步", 5.0},
        {"少量", 5.0}, {"一段", 5.0}, {"一些", 10.0}, {"中等", 10.0},
        {"适度", 10.0}, {"很多", 15.0}, {"大幅", 15.0}, {"长距", 15.0}
    };

    // 数字直接提取(如"前进5厘米")
    if (command.indexOf("厘米") != -1 || command.indexOf("cm") != -1) {
        int numStart = -1;
        for (int i = 0; i < command.length(); i++) {
            if (isdigit(command.charAt(i))) {
                numStart = i;
                break;
            }
        }
        if (numStart != -1) {
            String numStr = "";
            while (numStart < command.length() && 
                  (isdigit(command.charAt(numStart)) || command.charAt(numStart) == '.')) {
                numStr += command.charAt(numStart);
                numStart++;
            }
            return numStr.toFloat();
        }
    }

    // 模糊匹配
    for (const auto& pair : distanceMap) {
        if (command.indexOf(pair.first) != -1) {
            // 添加±30%的随机误差使移动更自然
            float error = 1.0 + (random(-300, 300) / 1000.0);
            return pair.second * error;
        }
    }
    
    // 默认距离带随机性
    return 5.0 + random(-200, 200) / 100.0;
}

void trotGait(int direction, float targetDistance) {
    // 关键修复:检查舵机是否初始化
    if (!isServoInitialized) {
        addLog("错误:舵机未初始化。无法执行步态");
        return;
    }

    // 验证输入参数
    if (direction < 0 || direction > 3) direction = 0;

    // 计算所需步数(每步约5cm)
    const float stepLength = 5.0;
    int steps = round(targetDistance / stepLength);

    // 添加人性化误差(±1步)
    steps += random(-1, 1);
    steps = max(1, steps);  // 至少1步

    addLog("启动步态:方向=" + String(direction) + "距离=" + String(targetDistance, 1) + "cm,步数=" + String(steps));

    // 关键修复:添加安全计数器
    unsigned long startTime = millis();
    int safetyCounter = 0;
    const int MAX_ITERATIONS = steps * 100;  // 基于步数

    // 执行步态
    for (int s = 0; s < steps; s++) {
        if (safetyCounter++ > MAX_ITERATIONS) {
            addLog("警告:步态安全计数器触发");
            break;
        }

        // 计算当前相位(基于步数)
        float t = fmod(s, 1.0);

        // ====关键修改:优化相位分配和运动同步====
        // 对角腿组1(右前+左后)
        moveLeg(RF_HIP, RF_KNEE, t, direction);
        moveLeg(LB_HIP, LB_KNEE, t, direction);  // 改为相同相位

        // 对角腿组2(左前+右后)
        moveLeg(LF_HIP, LF_KNEE, t + 0.5, direction);
        moveLeg(RB_HIP, RB_KNEE, t + 0.5, direction);

        // ====关键修改:增加步态同步延迟====
        delay(GAIT_CYCLE / 50);  // 控制刷新率
    }

    // 平滑过渡回中立位
    for (int i = 8; i <= 15; i++) {
        smoothMove(i, 90, 300);  // 300ms过渡
    }

    // ==== 重要优化：移除所有语音反馈 ====
    // 现在语音反馈仅在动作前播放确认信息
}

void smoothMove(uint8_t servo, float targetAngle, int duration) {
    // 严格验证舵机号范围
    if (servo < 8 || servo > 15) {
        addLog("错误:无效舵机号" + String(servo));
        return;
    }
    
    // 添加角度范围检查
    targetAngle = constrain(targetAngle, 0, 180);
    
    // ==== 关键修改:考虑舵机方向反转 ====
    bool isRightLeg = (servo == RF_HIP || servo == RF_KNEE || 
                      servo == RB_HIP || servo == RB_KNEE);
    
    // 使用原始角度进行计算
    float originalCurrent = currentAngles[servo];
    float originalTarget = isRightLeg ? (180 - targetAngle) : targetAngle;
    
    addLog("平滑移动舵机" + String(servo) + ":" + 
           String(originalCurrent, 1) + "->" + String(originalTarget, 1));
    
    float startAngle = originalCurrent;
    unsigned long startTime = millis();
    const float MAX_ANGLE_CHANGE = 15.0f; // 最大15度/步
    
    // 关键修复:添加安全计数器防止无限循环
    int safetyCounter = 0;
    const int MAX_ITERATIONS = duration / 10 + 100; // 基于持续时间计算最大迭代次数
    
    while (millis() - startTime < duration && safetyCounter++ < MAX_ITERATIONS) {
        float progress = (millis() - startTime) / (float)duration;
        
        // 修复缓动函数计算
        float ease;
        if (progress < 0.5) {
            ease = 4 * progress * progress * progress;
        } else {
            ease = 1 - pow(-2 * progress + 2, 3) / 2;
        }
        
        float targetCurrent = startAngle + (originalTarget - startAngle) * ease;
        
        // 关键修复:增加加速度限制
        float current = originalCurrent;
        float delta = targetCurrent - current;
        if (fabs(delta) > MAX_ANGLE_CHANGE) {
            delta = (delta > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
            targetCurrent = current + delta;
        }
        
        // 确保角度在有效范围内
        targetCurrent = constrain(targetCurrent, 0, 180);
        
        // 使用setServoAngle设置角度 (它会处理方向反转)
        setServoAngle(servo, targetCurrent);
        
        delay(10);
    }
    
    // 确保达到目标角度
    setServoAngle(servo, originalTarget);
    
    // 安全计数器检查
    if (safetyCounter >= MAX_ITERATIONS) {
        addLog("警告:平滑移动达到最大迭代次数");
    }
}

void resetAllServos() {
  for (int i = 8; i <= 15; i++) {
    setServoAngle(i, 90);
  }
  addLog("舵机复位到中立位置");
}

void danceSequence() {
    addLog("开始跳舞序列");
    
    // 跳舞动作1：原地摇摆
    for (int i = 0; i < 3; i++) {
        setServoAngle(LF_HIP, 110);
        setServoAngle(RF_HIP, 70);
        delay(300);
        setServoAngle(LF_HIP, 70);
        setServoAngle(RF_HIP, 110);
        delay(300);
    }
    
    // 跳舞动作2：旋转
    spinSequence();
    
    // 跳舞动作3：上下摆动
    for (int i = 0; i < 2; i++) {
        for (int j = 8; j <= 15; j++) {
            setServoAngle(j, 60);
        }
        delay(200);
        for (int j = 8; j <= 15; j++) {
            setServoAngle(j, 120);
        }
        delay(200);
    }
    
    // 回到中立位置
    resetAllServos();
    
    // 跳舞结束反馈
    const char* dancePhrases[] = {
        "跳得开心吗？",
        "我的舞技怎么样？",
        "再来一曲吗？"
    };
    speakText(dancePhrases[random(3)]);
}

void spinSequence() {
    addLog("开始转圈序列");
    
    // 顺时针转圈
    for (int i = 0; i < 4; i++) {
        moveLeg(LF_HIP, LF_KNEE, 0.0, 2);   // 左前腿前进
        moveLeg(RB_HIP, RB_KNEE, 0.5, 2);   // 右后腿前进
        moveLeg(RF_HIP, RF_KNEE, 0.0, 3);   // 右前腿后退
        moveLeg(LB_HIP, LB_KNEE, 0.5, 3);   // 左后腿后退
        delay(GAIT_CYCLE / 8);
    }
    
    // 回到中立位置
    resetAllServos();
    
    // 转圈结束反馈
    const char* spinPhrases[] = {
        "转得我有点晕~",
        "旋转跳跃我闭着眼",
        "这圈转得圆不圆？"
    };
    speakText(spinPhrases[random(3)]);
}

//系统初始化
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
        if (retryCount % 10 == 0) {
            addLog("连接WiFi中..已尝试" + String(retryCount / 2) + "秒");
        }
        if (retryCount > 60) {
            addLog("WiFi连接超时,重启系统");
            ESP.restart();
        }
    }
    
    addLog("获取百度Token...");
    baiduAccessToken = getBaiduToken();
    if (baiduAccessToken == "") {
        addLog("百度Token获取失败,10秒后重试");
        delay(10000);
        initializeSystem();
        return;
    }
    
    // ==== 增强的时间同步逻辑 ====
    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    addLog("等待NTP时间同步...");
    struct tm timeinfo;
    bool timeSynced = false;
    int timeRetry = 0;
    
    // 添加备用时间服务器
    const char* ntpServers[] = {
        "time1.google.com",
        "time2.google.com",
        "time3.google.com",
        "cn.pool.ntp.org"
    };
    
    while (!timeSynced && timeRetry < 30) {
        if (getLocalTime(&timeinfo, 5000)) {  // 5秒超时
            timeSynced = true;
        } else {
            addLog("NTP同步中...(" + String(timeRetry) + "/30)");
            
            // 每3次尝试切换时间服务器
            if (timeRetry % 3 == 0) {
                const char* server = ntpServers[timeRetry % 4];
                configTime(8 * 3600, 0, server);
                addLog("切换时间服务器: " + String(server));
            }
            
            delay(1000);
        }
        timeRetry++;
    }
    
    if (timeSynced) {
        char timeBuf[20];
        strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &timeinfo);
        addLog("当前时间:" + String(timeBuf));
    } else {
        addLog("NTP同步失败，使用RTC时间");
        // 设置合理的默认时间 (2024年1月1日 12:00:00)
        time_t now = 1704100800;  // 2024-01-01 12:00:00 UTC+8
        struct timeval tv = { .tv_sec = now };
        settimeofday(&tv, NULL);
        
        // 获取并记录设置的时间
        getLocalTime(&timeinfo);
        char timeBuf[20];
        strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &timeinfo);
        addLog("设置时间:" + String(timeBuf));
    }
    
    addLog("百度Token获取成功");
    addLog("系统准备就绪!");
    isReady = true;
    playTestTone();
}
//唤醒与语音处理
void wakeUpAssistant() {
    if (!isReady || isProcessing) return;
    
    // 重置唤醒超时
    lastActivityTime = millis();
    isProcessing = true;
    addLog("开始语音捕获");
    
    // 录音重试机制优化
    uint8_t* audioData = recordBuffer;
    size_t dataSize = 0;
    bool recordSuccess = false;
    
    for (int attempt = 0; attempt < 2; attempt++) {
        if (recordAudio(&audioData, &dataSize)) {
            recordSuccess = true;
            break;
        }
        addLog("录音失败，重试中...(" + String(attempt + 1) + "/2)");
        delay(300);
    }
    
    if (!recordSuccess) {
        addLog("多次录音失败");
        isProcessing = false;
        return;
    }
    
    // 语音识别优化
    String recognizedText = "";
    for (int attempt = 0; attempt < 2; attempt++) {
        recognizedText = baiduASR(audioData, dataSize);
        
        // ==== 关键修复：处理空结果 ====
        if (!recognizedText.isEmpty() && recognizedText != "[]") {
            break;
        }
        addLog("ASR返回空结果，重试中...(" + String(attempt + 1) + "/2)");
        delay(300);
    }
    
    // 处理空结果
    if (recognizedText.isEmpty() || recognizedText == "[]") {
        addLog("多次识别失败");
        // 播放提示音
        playErrorTone();
        delay(500);
        speakText("嗯？我没听清楚呢");
        isProcessing = false;
        return;
    }
    
    // 处理有效命令
    processVoiceCommand(recognizedText);
    isProcessing = false;
}

// 环境噪音采样
int16_t sampleAmbientNoise() {
    addLog("环境噪音采样中...");
    const int sampleCount = 16;  // 增加样本数量
    int32_t total = 0;
    int validSamples = 0;

    if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t noiseBuffer[512];  // 增大缓冲区
        size_t noiseRead = 0;
        
        for (int i = 0; i < sampleCount; i++) {
            i2s_read(I2S_NUM_0, noiseBuffer, sizeof(noiseBuffer), &noiseRead, 50);
            if (noiseRead > 0) {
                int16_t* samples = (int16_t*)noiseBuffer;
                int numSamples = noiseRead / sizeof(int16_t);
                // 计算RMS值而非绝对值
                for (int j = 50; j < numSamples - 50; j++) {
                    total += (int32_t)samples[j] * samples[j];
                    validSamples++;
                }
            }
            delay(30);
        }
        xSemaphoreGive(i2sMutex);
    }
    
    if (validSamples == 0) return 1;  // 防止除以0
    // 返回RMS值
    return sqrt(total / validSamples);
}
// 静音阈值计算
int calculateSilenceThreshold(int16_t noiseLevel) {
    // 动态基础阈值
    int baseThreshold = max(50, min(noiseLevel * 3, 500));
    addLog("计算阈值: 噪声RMS=" + String(noiseLevel) + " 基础阈值=" + String(baseThreshold));
    
    // 根据环境动态调整
    if (noiseLevel < 100) {
        return baseThreshold + 50;  // 安静环境
    } else if (noiseLevel < 300) {
        return baseThreshold + 100; // 普通环境
    } else {
        return baseThreshold + 200; // 嘈杂环境
    }
}

bool isSilentAudio(uint8_t* data, size_t size) {
    if(size < 100) return true; // 太短的录音直接认为是静音
    
    int16_t* samples = (int16_t*)data;
    int sampleCount = size / 2;
    int silentCount = 0;
    
    for(int i = 0; i < sampleCount; i++) {
        if(abs(samples[i]) < 500) {
            silentCount++;
        }
    }
    
    return silentCount > sampleCount * 0.95;
}
//核心语音处理流程
float calculateSimilarity(const String& str1, const String& str2) {
    if(str1.isEmpty() || str2.isEmpty()) return 0.0f;
    
    // 1. 标准化字符串（移除无关字符）
    String s1 = str1;
    String s2 = str2;
    
    // 修复：不能链式调用replace函数
    s1.replace("吗", "");
    s1.replace("呢", "");
    s1.replace("呀", "");
    s1.replace("什么", "");
    s1.replace(" ", "");
    
    s2.replace("吗", "");
    s2.replace("呢", "");
    s2.replace("呀", "");
    s2.replace("什么", "");
    s2.replace(" ", "");
    
    // 2. 计算编辑距离
    const int len1 = s1.length();
    const int len2 = s2.length();
    
    // 限制最大长度防止内存溢出
    const int MAX_LEN = 30;
    int n = min(len1, MAX_LEN);
    int m = min(len2, MAX_LEN);
    
    // 使用堆栈分配的小型数组
    int dp[MAX_LEN+1][MAX_LEN+1];
    
    for(int i=0; i<=n; i++) dp[i][0] = i;
    for(int j=0; j<=m; j++) dp[0][j] = j;
    
    for(int i=1; i<=n; i++) {
        for(int j=1; j<=m; j++) {
            int cost = (s1[i-1] == s2[j-1]) ? 0 : 1;
            dp[i][j] = min(min(dp[i-1][j] + 1, dp[i][j-1] + 1), dp[i-1][j-1] + cost);
        }
    }
    
    // 3. 计算相似度分数 (基于编辑距离)
    float maxLen = max(n, m);
    float similarity = 1.0f - (dp[n][m] / maxLen);
    
    // 4. 关键词权重加成
    const char* keywords[] = {"名字", "叫", "什么", "谁", "时间", "几点"};
    for(const char* keyword : keywords) {
        if(s1.indexOf(keyword) != -1 && s2.indexOf(keyword) != -1) {
            similarity = min(1.0f, similarity + 0.15f);
        }
    }
    
    return similarity;
}
void processVoiceCommand(String recognizedText) {
    addLog("处理语音命令:" + recognizedText);

    // ==== 声明所有局部变量 ====
    String response = "";          
    bool lowConfidence = false;    

    // ==== 增强时间命令检测 ====
    bool isTimeCommand = 
        recognizedText.indexOf("时间") != -1 || 
        recognizedText.indexOf("几点") != -1 || 
        recognizedText.indexOf("钟") != -1 ||
        recognizedText.indexOf("钟表") != -1;
    
    if (isTimeCommand) {
        // 动态生成完整时间响应
        String currentTime = getFormattedTime();
        const char* timeResponses[] = {
            "喵喵钟显示现在是：",
            "我的电子骨头显示：",
            "现在是："
        };
        int timeIdx = random(3);
        response = String(timeResponses[timeIdx]) + currentTime;
        
        // 添加语气词增强自然感
        if (timeIdx == 2) response += "啦";
        
        // 异常处理
        if (currentTime.length() < 8) {
            response = "哎呀，我的钟好像出问题了~";
        }
        
        addLog("生成时间响应：" + response);
        
        // 立即播放并跳过缓存
        speakText(response);
        isProcessing = false;
        return;
    }

    // ==== 原有预处理 ====
    String filteredText = recognizedText;
    filteredText.toLowerCase();
    filteredText.replace(" ", "");
    
    // 扩展过滤列表
    const char* invalidPatterns[] = {
        "idontknow", "idont'tknow", "idonotknow", "noinput", "nospeech",
        "silence", "noise", "nostatement", "background", "voicenotdetected",
        "我不知道", "没有听到", "未检测到", "无法识别", "请再说一次"
    };
    bool isInvalid = false;
    for (const char* pattern : invalidPatterns) {
        if (filteredText.indexOf(pattern) != -1) {
            isInvalid = true;
            break;
        }
    }
    
    // 检查长度过短或特殊无效文本
    if (recognizedText.isEmpty() || recognizedText.length() < 3 ||
        recognizedText == "我不知道" || recognizedText == "我不知道你在说什么" ||
        recognizedText == "请再说一次" || isInvalid) {
        addLog("忽略无效识别结果:" + recognizedText);
        return;
    }
    
    // 安全截断长度
    if (recognizedText.length() > 100) {
        recognizedText = recognizedText.substring(0, 100);
    }
    
    // 预处理识别结果
    recognizedText.trim();
    recognizedText.replace("。", "");
    recognizedText.replace("?", "");
    recognizedText.replace("!", "");
    recognizedText.replace(",", "");
    
    // ==== 缓存检查 ====
    if (responseCache.find(recognizedText) != responseCache.end()) {
        auto& cacheList = responseCache[recognizedText].responses;
        response = cacheList[0]; // 使用最新缓存
        addLog("缓存命中:" + recognizedText);
    }
    
    // ==== 动作命令处理 ====
    if (response.isEmpty() && isActionCommand(recognizedText)) {
        delay(100);
        // 立即播放确认语音
        const char* confirmations[] = {
            "好嘞嘞~", "收到!", "明白啦!", "好嘞嘞嘞嘞!", "收到命令!", "马上执行!"
        };
        String confirmation = confirmations[random(6)];
        speakText(confirmation);
        
        // 执行动作
        executeAction(recognizedText);
        
        addLog("动作命令执行中...");
        addLog("空闲内存:" + String(esp_get_free_heap_size()));
        isProcessing = false;
        return;
    }
    
    // ==== 预设回答匹配 ====
    if (response.isEmpty()) {
        String matchedPreset = findBestMatch(recognizedText);
        if (!matchedPreset.isEmpty()) {
            response = presetResponses.at(matchedPreset);
            addLog("模糊匹配成功:" + recognizedText + "->" + matchedPreset);
            
            // ==== 关键修复：动态时间处理 ====
            if (matchedPreset == "现在几点" || matchedPreset == "时间") {
                String currentTime = getFormattedTime();
                response += currentTime; // 直接追加时间
                
                // 特殊处理: 添加语气词
                if (random(10) > 7) { // 30%概率添加语气词
                    const char* suffixes[] = {"啦", "哟", "~"};
                    response += suffixes[random(3)];
                }
                addLog("动态生成完整时间响应:" + response);
            }
        }
    }
    
    // ==== 置信度检查 ====
    if (response.isEmpty()) {
        // 计算置信度: 基于命令长度
        float confidence = recognizedText.length() / 25.0;
        confidence = constrain(confidence, 0.0, 1.0);
        addLog("置信度评估:" + String(confidence * 100, 1) + "%");
        
        if (confidence < 0.4) {
            lowConfidence = true;
            response = "嗯? 你刚才说什么?";
            addLog("低置信度命令，不调用DeepSeek");
        } else {
            response = callDeepSeekAPI(recognizedText);
            addLog("使用DeepSeek处理命令");
        }
    }
    
    // ==== 缓存存储 ====
    if (!response.isEmpty() && !isActionCommand(recognizedText) && !lowConfidence) {
        // 创建LRU缓存队列
        if (responseCache.size() >= MAX_CACHE_RESPONSES) {
            auto oldest = responseCache.begin();
            addLog("缓存已满，移除旧记录:" + oldest->first);
            responseCache.erase(oldest);
        }
        
        // 更新缓存
        if (responseCache.find(recognizedText) == responseCache.end()) {
            // 新条目
            CachedResponse newEntry;
            newEntry.responses.push_back(response);
            newEntry.timestamp = millis();
            responseCache[recognizedText] = newEntry;
        } else {
            // 已有条目
            responseCache[recognizedText].responses[0] = response;
            responseCache[recognizedText].timestamp = millis();
        }
        addLog("缓存更新:" + recognizedText + "->" + response);
        
        // 异步保存
        static int saveCounter = 0;
        if (++saveCounter >= 5) {
            saveCacheToStorage();
            saveCounter = 0;
        }
    }
    
    // 播放回答
    if (!response.isEmpty()) {
        delay(200);
        speakText(response);
    }
    
    addLog("空闲内存:" + String(esp_get_free_heap_size()));
    isProcessing = false;
}

String findBestMatch(const String& input) {
    float bestScore = 0.72f;  // 降低阈值到0.72
    String bestMatch = "";
    
    for(const auto& preset : presetResponses) {
        const String& presetQuestion = preset.first;
        float similarity = calculateSimilarity(input, presetQuestion);
        
        if(similarity > bestScore) {
            bestScore = similarity;
            bestMatch = presetQuestion;
        }
    }
    
    if(!bestMatch.isEmpty()) {
        addLog("匹配成功 [" + String(bestScore, 2) + "]: " + input + " → " + bestMatch);
    } else {
        addLog("未匹配到预设问题: " + input);
    }
    
    return bestMatch;
}
String getFormattedTime() {
    time_t now = time(nullptr);
    struct tm timeinfo;
    
    // 确保获取到有效时间
    if (!localtime_r(&now, &timeinfo)) {
        addLog("错误：获取本地时间失败");
        return "00:00:00";
    }
    
    char buf[20];
    // 使用更可靠的格式
    strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
    
    // 添加时间有效性检查
    if (strlen(buf) == 0 || buf[0] == '\0') {
        addLog("错误：时间格式化失败");
        return "00:00:00";
    }
    
    // 检查时间是否合理
    int hours = timeinfo.tm_hour;
    int mins = timeinfo.tm_min;
    int secs = timeinfo.tm_sec;
    
    if (hours < 0 || hours > 23 || mins < 0 || mins > 59 || secs < 0 || secs > 59) {
        addLog("警告：无效时间值 - 时：" + String(hours) + " 分：" + String(mins) + " 秒：" + String(secs));
        return "时间异常";
    }
    
    return String(buf);
}

bool isActionCommand(String command) {
    // 扩展动作命令列表，包含更多拟人化表达
    String commands[] = {
        "前进", "向前", "往前走一点", "直走", "往前走", "动起来", "往前走几步", "走一些", "走一点", "走几步",
        "后退", "向后", "倒退", "退后一点", "回来", "往后走一点","往后走几步",
        "左转", "向左", "往左", "左边走", "左拐", "往左走几步","往左走一些","往左边走一些",
        "右转", "向右", "往右", "右边走", "右拐","往右走几步","往右走一些","往右边走一些",
        "停止", "停下", "别动", "站住", "立定", "暂停",
        "跳舞", "跳个舞", "扭一扭", "来段舞蹈", "蹦迪",
        "转圈", "转个圈", "原地转", "旋转"
    };
    
    // 计算命令与预设动作的相似度
    float bestScore = 0.0;
    for (int i = 0; i < sizeof(commands)/sizeof(commands[0]); i++) {
        float similarity = calculateSimilarity(command, commands[i]);
        if (similarity > bestScore) {
            bestScore = similarity;
        }
    }
    
    // 设置更宽松的阈值，接受模糊命令
    return bestScore >= 0.65; // 降低阈值以接受更多自然表达
}
void executeAction(String command) {
    addLog("执行动作:" + command);
    
    // 解析方向和距离 - 更自然的表达处理
    int direction = 0;
    float distance = 0.0;
    
    // 方向识别
    if (calculateSimilarity(command, "前进") >= 0.65 || 
        command.indexOf("向前") != -1 ||
        command.indexOf("往前走") != -1 ||
        command.indexOf("直走") != -1 ||
        command.indexOf("迈步") != -1 ||
        command.indexOf("走一些") != -1) {
        direction = 0;
    } 
    else if (calculateSimilarity(command, "后退") >= 0.65 || 
             command.indexOf("向后") != -1 ||
             command.indexOf("倒退") != -1) {
        direction = 1;
    }
    else if (calculateSimilarity(command, "左转") >= 0.65 || 
             command.indexOf("向左") != -1 ||
             command.indexOf("左拐") != -1) {
        direction = 2;
    }
    else if (calculateSimilarity(command, "右转") >= 0.65 || 
             command.indexOf("向右") != -1 ||
             command.indexOf("右拐") != -1) {
        direction = 3;
    }
    else if (calculateSimilarity(command, "跳舞") >= 0.65 || 
             command.indexOf("蹦迪") != -1 ||
             command.indexOf("舞蹈") != -1) {
        // 跳舞动作
        danceSequence();
        return;
    }
    else if (calculateSimilarity(command, "转圈") >= 0.65 || 
             command.indexOf("旋转") != -1) {
        // 转圈动作
        spinSequence();
        return;
    }
    
    // 距离识别 - 更自然的表达处理
    if (command.indexOf("一点点") != -1 || 
        command.indexOf("一小步") != -1 ||
        command.indexOf("微微") != -1) {
        distance = 2.0 + random(-5, 5)/10.0; // 增加随机性
    } 
    else if (command.indexOf("一点") != -1 || 
             command.indexOf("几步") != -1 ||
             command.indexOf("一小段") != -1 ||
             command.indexOf("一些") != -1) {  // 添加"一些"
        distance = 5.0 + random(-10, 10)/10.0;
    } 
    else if (command.indexOf("一段") != -1 || 
             command.indexOf("中距离") != -1) {
        distance = 10.0 + random(-15, 15)/10.0;
    } 
    else if (command.indexOf("很多") != -1 || 
             command.indexOf("大幅") != -1 ||
             command.indexOf("长距离") != -1) {
        distance = 15.0 + random(-20, 20)/10.0;
    } 
    else {
        // 默认距离带更多随机性
        distance = 5.0 + random(-20, 20)/10.0;
    }
    
    // 执行步态
    trotGait(direction, distance);
}
void addLog(String message) {
  String timestamp = getFormattedTime();
  systemLog += "[" + timestamp + "]" + message + "\n";
  Serial.println("[" + timestamp + "]" + message);
}
//语音识别函数
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
//DeepSeek API交互
String callDeepSeekAPI(String query) {
    addLog("调用 DeepSeek API: " + query);
    
    // 简化query防止过长
    if (query.length() > 100) {
        query = query.substring(0, 100);
        addLog("Query过长已截断");
    }
    
    // 检查WiFi连接
    if (WiFi.status() != WL_CONNECTED) {
        addLog("DeepSeek失败: WiFi断开");
        return "哎呀，网络好像断开了呢~";
    }
    
    HTTPClient http;
    http.begin("https://api.deepseek.com/v1/chat/completions");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + String(deepseekApiKey));
    http.setTimeout(10000);  // 10秒超时
    
    // 生成随机温度(0.6-0.85)增加回答多样性
    float temperature = random(60, 85) / 100.0;
    
    // 创建请求体 - 添加随机温度参数和角色设定
    String requestBody = "{"
        "\"model\":\"deepseek-chat\","
        "\"temperature\":" + String(temperature, 1) + ","
        "\"messages\":["
            "{\"role\":\"system\",\"content\":\"你是一个可爱的四足机器人铁爪,用朋友聊天语气回答。多用语气词,有时会说有一点点肉麻的话。回答最好不要太冗长，最好不超过25字左右\"},"
            "{\"role\":\"user\",\"content\":\"" + query + "\"}"
        "]}";
    
    addLog("DeepSeek请求体: " + requestBody);
    
    // 发送请求
    int httpCode = http.POST(requestBody);
    
    if (httpCode != 200) {
        String error = http.getString();
        addLog("DeepSeek错误: " + String(httpCode) + " - " + error);
        http.end();
        return "抱歉，我暂时无法回答呢~";
    }
    
    String response = http.getString();
    http.end();
    
    // 安全截断日志长度
    addLog("DeepSeek原始响应: " + response.substring(0, min((size_t)100, response.length())) + "...");
    
    // 使用固定大小JSON文档
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, response);
    
    if (error) {
        addLog("JSON解析错误: " + String(error.c_str()));
        return "处理回答时出错了呢~";
    }
    
    String result = "";
    
    if (doc.containsKey("choices") && doc["choices"].is<JsonArray>()) {
        JsonArray choices = doc["choices"];
        if (choices.size() > 0) {
            JsonObject choice = choices[0];
            if (choice.containsKey("message") && choice["message"].is<JsonObject>()) {
                JsonObject message = choice["message"];
                if (message.containsKey("content") && message["content"].is<String>()) {
                    result = message["content"].as<String>();
                }
            }
        }
    }
    
    // 处理空响应
    if (result.isEmpty()) {
        result = "嗯...这个问题有点难呢~";
    }
    
    // 后处理: 确保语气自然
    result.replace("。", "~");  // 句号变波浪号
    result.replace("?", "嘛?");  // 更口语化
    
    // 简化响应长度
    if (result.length() > 200) {
        result = result.substring(0, 200) + "...";
        addLog("响应截断至200字符");
    }
    
    // 添加随机后缀增强语气(如果结尾没有波浪号)
    const char* randomSuffixes[] = {"~", "!", "哟", "呀"};
    if (!result.isEmpty() && result.charAt(result.length() - 1) != '~') {
        result += randomSuffixes[random(4)];
    }
    
    addLog("DeepSeek最终回答: " + result);
    return result;
}
//TTS合成函数
void speakText(String text) {
    // 添加50ms延迟防止音频冲突
    delay(50);
    
    addLog("合成语音:" + text);
    
    // 增加截断长度到200字符
    if(text.length() > 200) {
        text = text.substring(0, 200);
        addLog("警告:文本过长，已截断前200字符");
    }
    
    httpTTS(text);
}
void httpTTS(String text) {
    addLog("合成语音:" + text);
    
    // 严格长度限制
    if(text.length() > 100) {
        text = text.substring(0, 100);
        addLog("TTS文本截断至100字符");
    }
    
    // 关键优化1：特殊字符处理
    text.replace(" ", "%20");
    text.replace("\"", "");
    
    // 关键优化2：精确参数控制
    String url = String(baiduTtsUrl) + 
        "?tex=" + text + 
        "&lan=zh" + 
        "&cuid=" + WiFi.macAddress() + 
        "&ctp=1" + 
        "&tok=" + baiduAccessToken + 
        "&per=4194" +  // 情感女声
        "&spd=4" +     // 语速7 (1-9)
        "&pit=5" +     // 音调8 (1-9)
        "&vol=4" +     // 音量8 (1-9)
        "&aue=6";      // PCM格式

    HTTPClient http;
    http.begin(url);
    http.setTimeout(10000);  // 10秒超时
    
    // 关键优化3：添加音频质量控制参数
    http.addHeader("X-Audio-Quality", "high");
    http.addHeader("X-Speech-Rate", "normal");

    int httpCode = http.GET();
    if(httpCode == HTTP_CODE_OK) {
        // 获取内容类型
        String contentType = http.header("Content-Type");
        addLog("内容类型:" + contentType);
        
        // 流式播放
        WiFiClient* stream = http.getStreamPtr();
        size_t contentLength = http.getSize();
        playStreamedAudio(stream, contentLength);
    } else {
        String errorMsg = http.getString();
        addLog("TTS错误:" + String(httpCode) + " 响应:" + errorMsg);
        
        // 详细错误诊断
        if(httpCode == 400) {
            addLog("错误详情:请求参数无效");
        } else if(httpCode == 401) {
            addLog("错误详情:Token过期或无效");
        }
        playErrorTone();
    }
    http.end();
}
//流式音频播放
void playStreamedAudio(Stream* stream, size_t contentLength) {
    addLog("开始播放音频,长度:" + String(contentLength) + "字节");
    
    // 关键修复1:精确采样率同步
    if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
        xSemaphoreGive(i2sMutex);
    }
    
    size_t bytesPlayed = 0;
    uint8_t audioBuffer[512]; // 使用栈分配的小缓冲区
    
    // 关键修复2:精确处理WAV头
    const size_t WAV_HEADER_SIZE = 44;
    if(contentLength > WAV_HEADER_SIZE) {
        // 读取并验证WAV头
        uint8_t header[WAV_HEADER_SIZE];
        size_t headerRead = stream->readBytes(header, WAV_HEADER_SIZE);
        
        // 验证RIFF头
        if(headerRead == WAV_HEADER_SIZE && 
           header[0] == 'R' && header[1] == 'I' && header[2] == 'F' && header[3] == 'F') {
            
            // 从头部提取实际采样率(位于字节24-27)
            uint32_t fileSampleRate = 
                ((uint32_t)header[24]) |
                ((uint32_t)header[25] << 8) |
                ((uint32_t)header[26] << 16) |
                ((uint32_t)header[27] << 24);
            
            addLog("WAV采样率:" + String(fileSampleRate) + "Hz");
            
            // 动态调整I2S采样率
            int32_t rateDiff = (int32_t)fileSampleRate - 16000;
            if(abs(rateDiff) > SAMPLE_RATE_TOLERANCE) {
                addLog("采样率不匹配，动态调整I2S");
                if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    i2s_set_clk(I2S_NUM_1, fileSampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
                    xSemaphoreGive(i2sMutex);
                }
            }
            
            bytesPlayed += headerRead;
            addLog("跳过44字节WAV文件头");
        } else {
            // 无效头，回退播放
            addLog("无效WAV头，直接播放");
            if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                size_t written = 0;
                i2s_write(I2S_NUM_1, header, headerRead, &written, portMAX_DELAY);
                xSemaphoreGive(i2sMutex);
                bytesPlayed += written;
            }
        }
    }
    
    // 新增音频淡入处理
    const int FADE_IN_SAMPLES = 800; // 50ms @16kHz
    int fadeCounter = 0;
    bool fadeCompleted = false;
    
    // 修复3:实时流处理优化
    while(bytesPlayed < contentLength) {
        // 计算本次要读取的字节数
        size_t bytesToRead = min(sizeof(audioBuffer), contentLength - bytesPlayed);
        size_t bytesRead = stream->readBytes(audioBuffer, bytesToRead);
        
        if(bytesRead == 0) {
            addLog("音频流结束");
            break;
        }
        
        // 应用淡入效果解决炸音问题
        if(!fadeCompleted) {
            int16_t* samples = (int16_t*)audioBuffer;
            int sampleCount = bytesRead / sizeof(int16_t);
            
            for(int i = 0; i < sampleCount; i++) {
                if(fadeCounter < FADE_IN_SAMPLES) {
                    float gain = (float)fadeCounter / FADE_IN_SAMPLES;
                    samples[i] = (int16_t)(samples[i] * gain);
                    fadeCounter++;
                } else {
                    fadeCompleted = true;
                    break;
                }
            }
        }
        
        // 通过I2S播放
        if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            size_t written = 0;
            esp_err_t err = i2s_write(I2S_NUM_1, audioBuffer, bytesRead, &written, portMAX_DELAY);
            xSemaphoreGive(i2sMutex);
            
            if(err != ESP_OK) {
                addLog("I2S写入错误:" + String(err));
                break;
            }
            
            if(written > 0) {
                bytesPlayed += written;
            }
        } else {
            addLog("I2S写入超时");
            break;
        }
        
        // 内存保护
        if(esp_get_free_heap_size() < 8000) {
            addLog("内存不足，停止播放");
            break;
        }
    }
    
    addLog("播放完成:" + String(bytesPlayed) + "/" + String(contentLength) + "字节");
    
    // 新增I2S缓冲清理
    if(xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t silence[128] = {0};
        size_t written = 0;
        i2s_write(I2S_NUM_1, silence, sizeof(silence), &written, 0);
        xSemaphoreGive(i2sMutex);
    }
}
//百度Token获取
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
    if (psramFound()) {
        sineWave = (int16_t*)ps_malloc(samples * sizeof(int16_t));
        addLog("使用PSRAM生成测试音");
    } else {
        sineWave = (int16_t*)malloc(samples * sizeof(int16_t));
    }
    
    if (!sineWave) {
        addLog("内存分配失败");
        return;
    }

    // 生成正弦波
    for (int i = 0; i < samples; i++) {
        sineWave[i] = (int16_t)(32767 * sin(2 * M_PI * 440 * i / 16000.0));
    }

    // 通过I2S播放
    size_t bytesWritten = 0;
    size_t totalBytes = samples * sizeof(int16_t);
    while (bytesWritten < totalBytes) {
        size_t chunkSize = min((size_t)1024, totalBytes - bytesWritten);
        size_t written = 0;
        
        // 添加互斥锁保护I2S资源
        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            i2s_write(I2S_NUM_1, 
                     (const char*)(sineWave + bytesWritten / sizeof(int16_t)),
                     chunkSize, &written, portMAX_DELAY);
            xSemaphoreGive(i2sMutex);
        }
        
        if (written > 0) {
            bytesWritten += written;
        }
    }
    
    // ==== 修复1：添加内存释放 ====
    if (sineWave) {
        free(sineWave); // 安全释放内存
        sineWave = nullptr;
        addLog("测试音内存已释放");
    }
    // ==========================
    
    addLog("测试音播放完成");
}
//=====错误提示音====
void playErrorTone() {
    addLog("播放错误提示音");
    const int toneDuration = 200; // ms
    const int samples = 16000 * toneDuration / 1000;
    int16_t* sineWave = (int16_t*)malloc(samples * sizeof(int16_t));
    
    if (!sineWave) return;
    
    for (int i = 0; i < samples; i++) {
        sineWave[i] = (int16_t)(32767 * sin(2 * M_PI * 880 * i / 16000.0));
    }
    
    size_t bytesWritten = 0;
    size_t totalBytes = samples * sizeof(int16_t);
    while (bytesWritten < totalBytes) {
        size_t chunkSize = min((size_t)512, totalBytes - bytesWritten);
        size_t written = 0;
        
        // 互斥锁保护
        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            i2s_write(I2S_NUM_1, 
                     (const char*)(sineWave + bytesWritten / sizeof(int16_t)),
                     chunkSize, &written, portMAX_DELAY);
            xSemaphoreGive(i2sMutex);
        }
        
        if (written > 0) {
            bytesWritten += written;
        }
    }
    
    // ==== 修复1：添加内存释放 ====
    if (sineWave) {
        free(sineWave); // 安全释放内存
        sineWave = nullptr;
        addLog("错误音内存已释放");
    }
    // ==========================
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
            // 修复遍历方式
            for (const auto& entry : responseCache) {
                Serial.print(entry.first + "->");
                // 遍历responses向量
                for (const String& response : entry.second.responses) {
                    Serial.print(response + "|");
                }
                Serial.println();
            }
            Serial.println("=============");
        } else if (command == "preset.list") {
            Serial.println("====预设回答列表====");
            for (const auto& entry : presetResponses) {
                Serial.println(entry.first + "->" + entry.second);
            }
            Serial.println("==================");
        } else if (command == "free") {
            size_t free_psram = 0;
            if (psramFound()) {
                free_psram = ESP.getFreePsram();
            }
            Serial.printf("内存状态:堆:%d，PSRAM:%d\n", esp_get_free_heap_size(), free_psram);
        } else if (command.startsWith("addpreset")) {
            // 格式:addpreset|问题|答案
            int firstPipe = command.indexOf('|');
            int secondPipe = command.indexOf('|', firstPipe + 1);
            if (firstPipe > 0 && secondPipe > firstPipe) {
                String question = command.substring(firstPipe + 1, secondPipe);
                String answer = command.substring(secondPipe + 1);
                // 更新预设库
                const_cast<std::map<String, String>&>(presetResponses)[question] = answer;
                Serial.println("已添加预设:" + question + "->" + answer);
            } else {
                Serial.println("格式错误，应为:addpreset|问题|答案");
            }
        } else {
            Serial.println("未知命令");
        }
    }
}
//=====音频录制函数=====
bool recordAudio(uint8_t** buffer, size_t* size) {
    if (isRecording) return false;
    isRecording = true;
    *buffer = recordBuffer;
    *size = recordBufferSize;

    // 1. 环境噪音采样 - 增加采样精度
    int16_t noiseLevel = sampleAmbientNoise();
    
    // ==== 优化阈值算法 ====
    // 基础阈值 = 噪声RMS * 2.5 + 150 (更宽松)
    int baseThreshold = (int)(noiseLevel * 2.5) + 150;
    // 动态环境适配
    int silenceThreshold;
    if (noiseLevel < 100) {
        silenceThreshold = baseThreshold + 30;  // 安静环境
    } else if (noiseLevel < 300) {
        silenceThreshold = baseThreshold + 60;  // 普通环境
    } else {
        silenceThreshold = baseThreshold + 100;  // 嘈杂环境
    }
    addLog("动态阈值: 噪音=" + String(noiseLevel) + " | 阈值=" + String(silenceThreshold));
    
    // 设置麦克风
    if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        i2s_set_clk(I2S_NUM_0, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
        xSemaphoreGive(i2sMutex);
    }
    
    // 预热麦克风
    uint8_t dummyBuffer[256];
    size_t dummyRead = 0;
    for (int i = 0; i < 3; i++) {  // 增加预热次数
        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            i2s_read(I2S_NUM_0, dummyBuffer, sizeof(dummyBuffer), &dummyRead, 10);
            xSemaphoreGive(i2sMutex);
        }
        delay(10);
    }
    
    // 录音参数
    size_t bytesWritten = 0;
    uint8_t* currentPos = *buffer;
    unsigned long startTime = millis();
    const unsigned long maxDuration = recordDuration;
    
    // VAD 参数优化
    bool speechStarted = false;
    unsigned long lastVoiceTime = 0;
    const unsigned long silenceTimeout = 1500;  // 增加静音超时到1.5秒
    int consecutiveVoiceFrames = 0;
    int validVoiceFrames = 0;
    int totalFrames = 0;
    const int minSpeechFrames = 3;  // 降低最低语音帧要求
    
    // ==== 关键优化：添加预录音缓冲 ====
    const size_t PRE_RECORD_BUFFER_SIZE = 1024;  // 0.5秒缓冲
    uint8_t preRecordBuffer[PRE_RECORD_BUFFER_SIZE];
    size_t preRecordIndex = 0;
    
    while ((millis() - startTime) < maxDuration) {
        size_t bytesToRead = min((size_t)512, (size_t)(*size - bytesWritten));
        if (bytesToRead == 0) break;
        
        size_t bytesRead = 0;
        if (xSemaphoreTake(i2sMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            i2s_read(I2S_NUM_0, currentPos, bytesToRead, &bytesRead, 10);
            xSemaphoreGive(i2sMutex);
        }
        
        if (bytesRead > 0) {
            // ==== 预录音缓冲逻辑 ====
            if (!speechStarted && preRecordIndex < PRE_RECORD_BUFFER_SIZE) {
                // 修复点1: 确保使用相同类型(size_t)
                size_t copySize = min(bytesRead, PRE_RECORD_BUFFER_SIZE - preRecordIndex);
                memcpy(preRecordBuffer + preRecordIndex, currentPos, copySize);
                preRecordIndex += copySize;
            }
            
            // 分析音频帧
            int16_t* samples = (int16_t*)currentPos;
            int samplesRead = bytesRead / sizeof(int16_t);
            int voiceSamples = 0;
            
            for (int i = 0; i < samplesRead; i++) {
                if (abs(samples[i]) > silenceThreshold) voiceSamples++;
            }
            
            bool isVoiceFrame = (voiceSamples > samplesRead * 0.25);  // 降低阈值到25%
            totalFrames++;
            
            if (isVoiceFrame) validVoiceFrames++;
            
            // VAD 检测优化
            if (!speechStarted) {
                if (isVoiceFrame) {
                    consecutiveVoiceFrames++;
                    if (consecutiveVoiceFrames >= 2) {  // 降低连续帧要求
                        speechStarted = true;
                        lastVoiceTime = millis();
                        addLog("检测到语音开始");
                        
                        // ==== 关键：插入预录音缓冲 ====
                        if (preRecordIndex > 0) {
                            // 修复点2: 确保使用相同类型(size_t)
                            size_t copySize = min(preRecordIndex, PRE_RECORD_BUFFER_SIZE);
                            memcpy(*buffer, preRecordBuffer, copySize);
                            bytesWritten += copySize;
                            currentPos += copySize;
                            addLog("插入预录音缓冲:" + String(copySize) + "字节");
                        }
                    }
                } else {
                    consecutiveVoiceFrames = 0;
                }
            } else {
                if (isVoiceFrame) {
                    lastVoiceTime = millis();
                }
                // 静音超时检测
                if ((millis() - lastVoiceTime) > silenceTimeout) {
                    addLog("静音超时，结束录音");
                    break;
                }
            }
            
            currentPos += bytesRead;
            bytesWritten += bytesRead;
        }
        
        // 内存保护
        if (esp_get_free_heap_size() < 4000) {
            addLog("内存不足，停止录音");
            break;
        }
        delay(5);
    }
    
    // 最终处理
    *size = bytesWritten;
    
    // 有效性检查优化
    if (!speechStarted || validVoiceFrames < minSpeechFrames) {
        addLog("未检测到有效语音");
        isRecording = false;
        return false;
    }
    
    if (bytesWritten < sampleRate * sizeof(int16_t) * 0.3) {
        addLog("录音过短");
        isRecording = false;
        return false;
    }
    
    addLog("录音完成: " + String(bytesWritten) + "字节 | 时长:" +
           String(millis() - startTime) + "ms | 有效帧:" +
           String(validVoiceFrames) + "/" + String(totalFrames));
           
    isRecording = false;
    return true;
}
