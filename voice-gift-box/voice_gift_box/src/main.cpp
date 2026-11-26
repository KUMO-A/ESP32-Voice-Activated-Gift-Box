#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <algorithm>
#include <cmath>

// ESP_S3_AUDIO RE1.1: wiring confirmed on the physical board.
constexpr int BTN_RESET = 12;       // top: erase password / close box
constexpr int BTN_RECORD = 11;      // middle: record new password
constexpr int BTN_COMPARE = 13;     // bottom: compare and unlock
constexpr int SERVO_PIN = 40;
constexpr int AMP_ENABLE = 16;
constexpr int I2C_SDA = 14;
constexpr int I2C_SCL = 47;
constexpr int I2S_MCLK = 8;
constexpr int I2S_BCLK = 3;
constexpr int I2S_LRCK = 9;
constexpr int I2S_ADC_DATA = 10;
constexpr int I2S_DAC_DATA = 46;
constexpr uint8_t ES8388_ADDR = 0x10;

constexpr int SAMPLE_RATE = 16000;
constexpr int RECORD_SECONDS = 5;
constexpr size_t MAX_SAMPLES = SAMPLE_RATE * RECORD_SECONDS;
constexpr int FRAME_LEN = 400;       // 25 ms
constexpr int FRAME_HOP = 200;       // 12.5 ms
constexpr int FFT_N = 512;
constexpr int MEL_BANDS = 24;
constexpr int CEP_COUNT = 12;
constexpr int MAX_FEATURE_FRAMES = 180;
constexpr float MATCH_THRESHOLD = 1.25f;
constexpr uint32_t TEMPLATE_MAGIC = 0x47424632;  // GBF2

struct VoiceTemplate {
  uint32_t magic;
  uint16_t frames;
  uint16_t dimensions;
  float feature[MAX_FEATURE_FRAMES][CEP_COUNT];
};

static int16_t *audioBuffer = nullptr;
static float fftReal[FFT_N];
static float fftImag[FFT_N];
static float frameEnergy[(MAX_SAMPLES - FRAME_LEN) / FRAME_HOP + 1];
static VoiceTemplate referenceVoice = {};
static VoiceTemplate workingVoice = {};
static Preferences preferences;
static bool hasReference = false;
static bool filesystemReady = false;
static bool audioReady = false;
static bool busy = false;

bool codecWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ES8388_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool initCodec() {
  // ES8388 slave, 16-bit standard I2S. ADC PGA is 18 dB/channel: enough for
  // speech at 10-30 cm without the large hiss produced by the old 33 dB setup.
  const uint8_t setup[][2] = {
      {0x19,0x04},{0x1A,0x00},{0x1B,0x00},
      {0x01,0x50},{0x02,0x00},{0x35,0xA0},{0x37,0xD0},{0x39,0xD0},
      {0x08,0x00},{0x04,0xC0},{0x00,0x12},
      {0x17,0x18},{0x18,0x02},
      {0x26,0x00},{0x27,0x90},{0x2A,0x90},{0x2B,0x80},{0x2D,0x00},
      {0x2E,0x1E},{0x2F,0x1E},{0x30,0x1E},{0x31,0x1E},
      {0x04,0x3C},
      {0x03,0xFF},{0x09,0x66},{0x0A,0xF0},{0x0B,0x02},
      {0x0C,0x0C},{0x0D,0x02},{0x10,0x00},{0x11,0x00},{0x03,0x00}
  };
  bool ok = true;
  for (const auto &item : setup) ok &= codecWrite(item[0], item[1]);
  return ok;
}

bool initI2S() {
  i2s_config_t config = {};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX);
  config.sample_rate = SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = true;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_MCLK;
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num = I2S_LRCK;
  pins.data_out_num = I2S_DAC_DATA;
  pins.data_in_num = I2S_ADC_DATA;
  return i2s_driver_install(I2S_NUM_0, &config, 0, nullptr) == ESP_OK &&
         i2s_set_pin(I2S_NUM_0, &pins) == ESP_OK &&
         i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT,
                     I2S_CHANNEL_STEREO) == ESP_OK;
}

void setSpeaker(bool enabled) {
  if (enabled) {
    codecWrite(0x19, 0x00);
    digitalWrite(AMP_ENABLE, HIGH);
  } else {
    digitalWrite(AMP_ENABLE, LOW);
    codecWrite(0x19, 0x04);
  }
}

void writeSilence(size_t frames = 512) {
  int16_t zero[256 * 2] = {};
  while (frames > 0) {
    const size_t count = std::min<size_t>(frames, 256);
    size_t written = 0;
    i2s_write(I2S_NUM_0, zero, count * 2 * sizeof(int16_t),
              &written, portMAX_DELAY);
    frames -= count;
  }
}

bool playPrompt(const char *path, float gain = 1.35f) {
  if (!filesystemReady || !audioReady) return false;
  File file = LittleFS.open(path, "r");
  if (!file || file.size() < 2) {
    Serial.printf("提示音不存在: %s\n", path);
    return false;
  }

  writeSilence();
  setSpeaker(true);
  delay(90);
  int16_t mono[256];
  int16_t stereo[256 * 2];
  size_t totalFrames = 0;
  while (file.available()) {
    const size_t bytes = file.read(reinterpret_cast<uint8_t *>(mono), sizeof(mono));
    const size_t frames = bytes / sizeof(int16_t);
    for (size_t i = 0; i < frames; ++i) {
      const int32_t value = constrain(static_cast<int32_t>(mono[i] * gain),
                                      -29000, 29000);
      stereo[i * 2] = static_cast<int16_t>(value);
      stereo[i * 2 + 1] = static_cast<int16_t>(value);
    }
    size_t written = 0;
    if (i2s_write(I2S_NUM_0, stereo, frames * 2 * sizeof(int16_t),
                  &written, portMAX_DELAY) != ESP_OK) {
      file.close();
      setSpeaker(false);
      return false;
    }
    totalFrames += frames;
  }
  file.close();
  writeSilence();
  delay(35);
  setSpeaker(false);
  delay(100);
  Serial.printf("提示音 %s: %.2f秒\n", path,
                static_cast<double>(totalFrames) / SAMPLE_RATE);
  return true;
}

void setServoAngle(int degrees) {
  // This exact board/servo combination was verified with direct 50 Hz pulses;
  // LEDC did not move the servo. 0° uses 1000 us, 90° uses 2000 us.
  degrees = constrain(degrees, 0, 90);
  const uint32_t pulseUs = 1000 + (1000UL * degrees) / 90;
  pinMode(SERVO_PIN, OUTPUT);
  for (int cycle = 0; cycle < 40; ++cycle) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(20000 - pulseUs);
  }
  digitalWrite(SERVO_PIN, LOW);
  Serial.printf("舵机角度: %d°\n", degrees);
}

size_t captureSpeech() {
  if (!audioBuffer) return 0;
  setSpeaker(false);
  i2s_zero_dma_buffer(I2S_NUM_0);
  int16_t stereo[256 * 2];
  size_t bytes = 0;
  for (int i = 0; i < 4; ++i)
    i2s_read(I2S_NUM_0, stereo, sizeof(stereo), &bytes, pdMS_TO_TICKS(100));

  Serial.println("开始录音，最长5秒，请说出口令...");
  size_t stored = 0;
  float previousInput = 0.0f;
  float previousHigh = 0.0f;
  float previousLow = 0.0f;
  while (stored < MAX_SAMPLES) {
    bytes = 0;
    if (i2s_read(I2S_NUM_0, stereo, sizeof(stereo), &bytes,
                 pdMS_TO_TICKS(200)) != ESP_OK) continue;
    const size_t frames = bytes / (2 * sizeof(int16_t));
    for (size_t i = 0; i < frames && stored < MAX_SAMPLES; ++i) {
      // The two ADC channels on this physical board have opposite polarity.
      const float input = 0.5f * (stereo[i * 2] - stereo[i * 2 + 1]);
      const float high = 0.955f * (previousHigh + input - previousInput);
      previousInput = input;
      previousHigh = high;
      const float low = previousLow + 0.586f * (high - previousLow);
      previousLow = low;
      audioBuffer[stored++] = constrain(static_cast<int32_t>(low), -30000, 30000);
    }
  }
  Serial.printf("录音结束: %.2f秒\n", static_cast<double>(stored) / SAMPLE_RATE);
  return stored;
}

void fft(float *real, float *imag) {
  for (int i = 1, j = 0; i < FFT_N; ++i) {
    int bit = FFT_N >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      std::swap(real[i], real[j]);
      std::swap(imag[i], imag[j]);
    }
  }
  for (int length = 2; length <= FFT_N; length <<= 1) {
    const float angle = -2.0f * PI / length;
    const float wLenReal = cosf(angle);
    const float wLenImag = sinf(angle);
    for (int start = 0; start < FFT_N; start += length) {
      float wReal = 1.0f;
      float wImag = 0.0f;
      for (int j = 0; j < length / 2; ++j) {
        const int even = start + j;
        const int odd = even + length / 2;
        const float oddReal = real[odd] * wReal - imag[odd] * wImag;
        const float oddImag = real[odd] * wImag + imag[odd] * wReal;
        const float evenReal = real[even];
        const float evenImag = imag[even];
        real[even] = evenReal + oddReal;
        imag[even] = evenImag + oddImag;
        real[odd] = evenReal - oddReal;
        imag[odd] = evenImag - oddImag;
        const float nextReal = wReal * wLenReal - wImag * wLenImag;
        wImag = wReal * wLenImag + wImag * wLenReal;
        wReal = nextReal;
      }
    }
  }
}

float hzToMel(float hz) { return 2595.0f * log10f(1.0f + hz / 700.0f); }
float melToHz(float mel) { return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f); }

void computeMfcc(size_t sampleStart, float *output) {
  for (int i = 0; i < FFT_N; ++i) {
    float sample = 0.0f;
    if (i < FRAME_LEN) {
      const float current = audioBuffer[sampleStart + i];
      const float previous = i == 0 ? current : audioBuffer[sampleStart + i - 1];
      const float emphasized = current - 0.97f * previous;
      const float window = 0.54f - 0.46f * cosf(2.0f * PI * i / (FRAME_LEN - 1));
      sample = emphasized * window;
    }
    fftReal[i] = sample;
    fftImag[i] = 0.0f;
  }
  fft(fftReal, fftImag);

  const float minMel = hzToMel(80.0f);
  const float maxMel = hzToMel(7600.0f);
  int bins[MEL_BANDS + 2];
  for (int i = 0; i < MEL_BANDS + 2; ++i) {
    const float mel = minMel + (maxMel - minMel) * i / (MEL_BANDS + 1);
    bins[i] = constrain(static_cast<int>((FFT_N + 1) * melToHz(mel) / SAMPLE_RATE),
                        0, FFT_N / 2);
  }
  float logMel[MEL_BANDS];
  for (int band = 0; band < MEL_BANDS; ++band) {
    float energy = 0.0f;
    const int left = bins[band];
    const int center = std::max(left + 1, bins[band + 1]);
    const int right = std::max(center + 1, bins[band + 2]);
    for (int k = left; k < center && k <= FFT_N / 2; ++k) {
      const float weight = static_cast<float>(k - left) / (center - left);
      energy += weight * (fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k]);
    }
    for (int k = center; k < right && k <= FFT_N / 2; ++k) {
      const float weight = static_cast<float>(right - k) / (right - center);
      energy += weight * (fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k]);
    }
    logMel[band] = logf(std::max(energy, 1.0f));
  }
  for (int cep = 0; cep < CEP_COUNT; ++cep) {
    float value = 0.0f;
    for (int band = 0; band < MEL_BANDS; ++band)
      value += logMel[band] * cosf(PI * cep * (band + 0.5f) / MEL_BANDS);
    output[cep] = value;
  }
}

bool extractFeatures(size_t samples, VoiceTemplate &result) {
  if (samples < FRAME_LEN) return false;
  const int totalFrames = 1 + (samples - FRAME_LEN) / FRAME_HOP;
  for (int frame = 0; frame < totalFrames; ++frame) {
    uint64_t energy = 0;
    const size_t start = frame * FRAME_HOP;
    for (int i = 0; i < FRAME_LEN; ++i) {
      const int32_t value = audioBuffer[start + i];
      energy += static_cast<int64_t>(value) * value;
    }
    frameEnergy[frame] = sqrt(static_cast<double>(energy) / FRAME_LEN);
  }

  // Use the quieter 20th percentile as this recording's ambient noise estimate.
  static float sortedEnergy[(MAX_SAMPLES - FRAME_LEN) / FRAME_HOP + 1];
  memcpy(sortedEnergy, frameEnergy, totalFrames * sizeof(float));
  std::sort(sortedEnergy, sortedEnergy + totalFrames);
  const float noise = sortedEnergy[totalFrames / 5];
  const float activityThreshold = std::max(70.0f, noise * 2.4f);
  int first = -1;
  int last = -1;
  for (int frame = 0; frame < totalFrames; ++frame) {
    if (frameEnergy[frame] >= activityThreshold) {
      if (first < 0) first = frame;
      last = frame;
    }
  }
  if (first < 0 || last - first < 6) {
    Serial.printf("未检测到有效语音: noise=%.1f threshold=%.1f\n",
                  noise, activityThreshold);
    return false;
  }
  first = std::max(0, first - 3);
  last = std::min(totalFrames - 1, last + 3);
  const int activeFrames = last - first + 1;
  const int stride = std::max(1, (activeFrames + MAX_FEATURE_FRAMES - 1) /
                                  MAX_FEATURE_FRAMES);

  result.magic = TEMPLATE_MAGIC;
  result.dimensions = CEP_COUNT;
  result.frames = 0;
  for (int frame = first; frame <= last && result.frames < MAX_FEATURE_FRAMES;
       frame += stride) {
    computeMfcc(static_cast<size_t>(frame) * FRAME_HOP,
                result.feature[result.frames]);
    ++result.frames;
  }
  if (result.frames < 7) return false;

  // Cepstral mean/variance normalization improves robustness across speakers,
  // microphone distance and playback volume.
  for (int dimension = 0; dimension < CEP_COUNT; ++dimension) {
    float mean = 0.0f;
    for (int frame = 0; frame < result.frames; ++frame)
      mean += result.feature[frame][dimension];
    mean /= result.frames;
    float variance = 0.0f;
    for (int frame = 0; frame < result.frames; ++frame) {
      const float centered = result.feature[frame][dimension] - mean;
      variance += centered * centered;
    }
    const float scale = sqrtf(variance / result.frames + 1e-4f);
    for (int frame = 0; frame < result.frames; ++frame)
      result.feature[frame][dimension] =
          (result.feature[frame][dimension] - mean) / scale;
  }
  Serial.printf("有效语音帧: %u, noise=%.1f threshold=%.1f\n",
                result.frames, noise, activityThreshold);
  return true;
}

float frameDistance(const float *a, const float *b) {
  float sum = 0.0f;
  for (int i = 0; i < CEP_COUNT; ++i) {
    const float difference = a[i] - b[i];
    sum += difference * difference;
  }
  return sqrtf(sum / CEP_COUNT);
}

float compareTemplates(const VoiceTemplate &a, const VoiceTemplate &b) {
  static float previous[MAX_FEATURE_FRAMES + 1];
  static float current[MAX_FEATURE_FRAMES + 1];
  constexpr float INF = 1e20f;
  std::fill(previous, previous + b.frames + 1, INF);
  previous[0] = 0.0f;
  const int band = std::max(18, std::max(a.frames, b.frames) / 3);
  for (int i = 1; i <= a.frames; ++i) {
    std::fill(current, current + b.frames + 1, INF);
    const int center = static_cast<int>(static_cast<int64_t>(i) * b.frames / a.frames);
    const int start = std::max(1, center - band);
    const int end = std::min<int>(b.frames, center + band);
    for (int j = start; j <= end; ++j) {
      const float best = std::min(previous[j],
                         std::min(current[j - 1], previous[j - 1]));
      current[j] = best + frameDistance(a.feature[i - 1], b.feature[j - 1]);
    }
    std::swap(previous, current);
  }
  return previous[b.frames] / (a.frames + b.frames);
}

bool saveReference() {
  const size_t bytes = preferences.putBytes("template", &referenceVoice,
                                            sizeof(referenceVoice));
  return bytes == sizeof(referenceVoice);
}

void loadReference() {
  hasReference = false;
  if (preferences.getBytesLength("template") != sizeof(referenceVoice)) return;
  if (preferences.getBytes("template", &referenceVoice,
                           sizeof(referenceVoice)) != sizeof(referenceVoice)) return;
  hasReference = referenceVoice.magic == TEMPLATE_MAGIC &&
                 referenceVoice.frames >= 7 &&
                 referenceVoice.frames <= MAX_FEATURE_FRAMES &&
                 referenceVoice.dimensions == CEP_COUNT;
}

void resetPassword() {
  busy = true;
  playPrompt("/Reset.pcm");
  preferences.remove("template");
  memset(&referenceVoice, 0, sizeof(referenceVoice));
  hasReference = false;
  setServoAngle(0);
  Serial.println("旧口令已清除，礼盒已关闭");
  busy = false;
}

void recordPassword() {
  busy = true;
  playPrompt("/Record.pcm");
  const size_t samples = captureSpeech();
  if (extractFeatures(samples, workingVoice)) {
    referenceVoice = workingVoice;
    hasReference = saveReference();
    Serial.println(hasReference ? "新口令已保存" : "口令保存失败");
  } else {
    Serial.println("录制失败：请靠近麦克风并清楚说话");
  }
  busy = false;
}

void comparePassword() {
  busy = true;
  // Always acknowledge the compare button audibly. Previously this prompt was
  // skipped when no reference existed, which looked like a dead button.
  playPrompt("/Unlocking.pcm");
  if (!hasReference) {
    Serial.println("尚未录制口令，请先按中间按键");
    busy = false;
    return;
  }
  const size_t samples = captureSpeech();
  if (!extractFeatures(samples, workingVoice)) {
    Serial.println("验证失败：未检测到有效语音");
    busy = false;
    return;
  }
  const float score = compareTemplates(referenceVoice, workingVoice);
  const float durationRatio = static_cast<float>(std::max(referenceVoice.frames,
                                                           workingVoice.frames)) /
                              std::min(referenceVoice.frames, workingVoice.frames);
  const bool matched = score <= MATCH_THRESHOLD && durationRatio <= 1.85f;
  Serial.printf("匹配分数=%.3f (门限 %.2f), 时长比=%.2f => %s\n",
                score, MATCH_THRESHOLD, durationRatio,
                matched ? "匹配成功" : "匹配失败");
  if (matched) setServoAngle(90);
  busy = false;
}

struct DebouncedButton {
  int pin;
  bool stable = HIGH;
  bool lastReading = HIGH;
  uint32_t changedAt = 0;

  explicit DebouncedButton(int buttonPin) : pin(buttonPin) {}

  bool pressed() {
    const bool reading = digitalRead(pin);
    if (reading != lastReading) {
      lastReading = reading;
      changedAt = millis();
    }
    if (millis() - changedAt >= 35 && reading != stable) {
      stable = reading;
      return stable == LOW;
    }
    return false;
  }
};

DebouncedButton resetButton{BTN_RESET};
DebouncedButton recordButton{BTN_RECORD};
DebouncedButton compareButton{BTN_COMPARE};

void setup() {
  Serial.begin(115200);
  delay(700);
  Serial.println("BOOT stage 1: GPIO");
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_RECORD, INPUT_PULLUP);
  pinMode(BTN_COMPARE, INPUT_PULLUP);
  pinMode(AMP_ENABLE, OUTPUT);
  digitalWrite(AMP_ENABLE, LOW);

  // Do not drive the servo at boot. This avoids a USB-5V brownout if SG90 is
  // powered from the board. PWM starts only on reset or successful matching.
  pinMode(SERVO_PIN, INPUT);
  audioBuffer = static_cast<int16_t *>(ps_malloc(MAX_SAMPLES * sizeof(int16_t)));
  Serial.printf("PSRAM=%s, size=%u, audioBuffer=%s\n",
                psramFound() ? "OK" : "FAIL",
                static_cast<unsigned>(ESP.getPsramSize()),
                audioBuffer ? "OK" : "FAIL");
  Serial.println("BOOT stage 2: codec");
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  const bool codecOk = initCodec();
  Serial.printf("BOOT codec=%d, stage 3: I2S\n", codecOk);
  const bool i2sOk = initI2S();
  Serial.printf("BOOT I2S=%d, stage 4: filesystem\n", i2sOk);
  filesystemReady = LittleFS.begin(false);
  audioReady = codecOk && i2sOk;
  Serial.printf("BOOT FS=%d, stage 5: preferences\n", filesystemReady);
  preferences.begin("giftbox", false);
  loadReference();
  Serial.printf("READY: ES8388=%s I2S=%s FS=%s password=%s\n",
                codecOk ? "OK" : "FAIL", i2sOk ? "OK" : "FAIL",
                filesystemReady ? "OK" : "FAIL",
                hasReference ? "已保存" : "未录制");
  Serial.println("上键=重置，中键=录制口令，下键=验证口令");
}

void loop() {
  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    Serial.printf("RUN heap=%u buttons=%d,%d,%d busy=%d\n",
                  ESP.getFreeHeap(), digitalRead(BTN_RESET),
                  digitalRead(BTN_RECORD), digitalRead(BTN_COMPARE), busy);
  }
  if (!busy) {
    if (resetButton.pressed()) resetPassword();
    else if (recordButton.pressed()) recordPassword();
    else if (compareButton.pressed()) comparePassword();
  }
  delay(5);
}
