// M5Stack Core2 オーディオスペクトラムアナライザー
//
// 内蔵PDMマイク(SPM1423) を M5Unified の Mic クラス(I2S PDM RX)で取り込み、
// arduinoFFT で周波数解析して LCD にバーグラフ表示する。

#include <M5Unified.h>
#include <arduinoFFT.h>

#include <cmath>

namespace {

// ---- 取り込み / FFT パラメータ ----
constexpr uint32_t kSampleRate = 16000;          // Hz
constexpr uint16_t kFftSize    = 512;            // 2のべき乗
constexpr uint16_t kBinCount   = kFftSize / 2;   // 256ビン (31.25Hz刻み)
constexpr float    kBinHz      = static_cast<float>(kSampleRate) / kFftSize;

// ---- 表示パラメータ ----
constexpr int   kBandCount = 32;        // 表示バンド数
constexpr float kBandLowHz = 62.5f;     // 最低表示周波数 (= 2ビン目)
constexpr float kBandHighHz = 8000.0f;  // 最高表示周波数 (= ナイキスト)
constexpr float kMinDbfs = -75.0f;      // 表示下限 (dBFS)
constexpr float kMaxDbfs = 0.0f;        // 表示上限 (dBFS)
constexpr float kReleaseRate = 0.85f;   // バー減衰係数 (1フレームあたり)

constexpr int kScreenW = 320;
constexpr int kScreenH = 240;
constexpr int kHeaderH = 18;
constexpr int kFooterH = 14;
constexpr int kGraphX = 4;
constexpr int kGraphY = kHeaderH;
constexpr int kGraphW = kScreenW - kGraphX * 2;
constexpr int kGraphH = kScreenH - kHeaderH - kFooterH;
constexpr int kBarGap = 1;

int16_t rec_buf[kFftSize];
float   vReal[kFftSize];
float   vImag[kFftSize];
ArduinoFFT<float> fft(vReal, vImag, kFftSize, static_cast<float>(kSampleRate));

M5Canvas canvas(&M5.Display);

// band_edges[i] .. band_edges[i+1]-1 がバンド i に属するビン番号
uint16_t band_edges[kBandCount + 1];
float    band_level[kBandCount];  // 0.0 .. 1.0 (減衰処理後)

// 対数周波数軸上の位置 (0.0 .. 1.0) を返す
float logPosition(float hz) {
  return std::log(hz / kBandLowHz) / std::log(kBandHighHz / kBandLowHz);
}

// 周波数軸を対数で等分割し、各バンドに属するビン範囲を決める
void setupBandEdges() {
  const float ratio = kBandHighHz / kBandLowHz;
  for (int i = 0; i <= kBandCount; ++i) {
    float hz = kBandLowHz * std::pow(ratio, static_cast<float>(i) / kBandCount);
    int bin = static_cast<int>(std::lround(hz / kBinHz));
    if (bin > kBinCount) bin = kBinCount;
    if (i > 0 && bin <= band_edges[i - 1]) bin = band_edges[i - 1] + 1;  // 空バンドを作らない
    band_edges[i] = static_cast<uint16_t>(bin);
  }
}

// 高さ比 (0..1) からバーの色を求める (緑 -> 黄 -> 赤)
uint16_t barColor(float ratio) {
  uint8_t r, g;
  if (ratio < 0.5f) {
    r = static_cast<uint8_t>(ratio * 2.0f * 255.0f);
    g = 255;
  } else {
    r = 255;
    g = static_cast<uint8_t>((1.0f - ratio) * 2.0f * 255.0f);
  }
  return M5.Display.color565(r, g, 0);
}

// FFT 結果 (vReal: 振幅スペクトル) を各バンドの表示レベルに変換する
void updateBandLevels() {
  // Hamming窓 + N点FFT での正弦波フルスケール時のピーク値
  const float full_scale = 32768.0f * (kFftSize / 2.0f) * 0.54f;
  for (int b = 0; b < kBandCount; ++b) {
    float peak = 0.0f;
    for (int bin = band_edges[b]; bin < band_edges[b + 1]; ++bin) {
      if (vReal[bin] > peak) peak = vReal[bin];
    }
    float dbfs = (peak > 0.0f) ? 20.0f * std::log10(peak / full_scale) : kMinDbfs;
    float level = (dbfs - kMinDbfs) / (kMaxDbfs - kMinDbfs);
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;

    // アタックは即時、リリースはゆっくり
    float decayed = band_level[b] * kReleaseRate;
    band_level[b] = (level > decayed) ? level : decayed;
  }
}

void drawHeader() {
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.drawString("Audio Spectrum", kGraphX, 4);
  char buf[48];
  snprintf(buf, sizeof(buf), "%lu Hz / %u pt / %.1f Hz",
           static_cast<unsigned long>(kSampleRate), kFftSize, kBinHz);
  canvas.setTextDatum(top_right);
  canvas.drawString(buf, kScreenW - kGraphX, 4);
}

void drawFooter() {
  struct Tick { float hz; const char* label; };
  static const Tick ticks[] = {
    {100.0f, "100"}, {200.0f, "200"}, {500.0f, "500"},
    {1000.0f, "1k"}, {2000.0f, "2k"}, {4000.0f, "4k"}, {8000.0f, "8k"},
  };
  canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
  canvas.setTextDatum(top_center);
  const int y = kGraphY + kGraphH;
  for (const auto& t : ticks) {
    int x = kGraphX + static_cast<int>(logPosition(t.hz) * kGraphW);
    canvas.drawFastVLine(x, y, 3, TFT_DARKGREY);
    // 右端ラベルは画面外に出ないよう寄せる
    int lx = (x > kScreenW - 10) ? kScreenW - 10 : x;
    canvas.drawString(t.label, lx, y + 4);
  }
}

void drawBars() {
  const float band_w = static_cast<float>(kGraphW) / kBandCount;
  for (int b = 0; b < kBandCount; ++b) {
    int x0 = kGraphX + static_cast<int>(b * band_w);
    int x1 = kGraphX + static_cast<int>((b + 1) * band_w) - kBarGap;
    int w = x1 - x0;
    if (w < 1) w = 1;
    int h = static_cast<int>(band_level[b] * kGraphH);
    if (h <= 0) continue;
    // 下から上へ色を変えながら積む
    const int y_bottom = kGraphY + kGraphH;
    for (int y = 0; y < h; ++y) {
      float ratio = static_cast<float>(y) / kGraphH;
      canvas.drawFastHLine(x0, y_bottom - 1 - y, w, barColor(ratio));
    }
  }
}

void render() {
  canvas.fillScreen(TFT_BLACK);
  drawHeader();
  canvas.drawFastHLine(0, kGraphY + kGraphH, kScreenW, TFT_DARKGREY);
  drawBars();
  drawFooter();
  canvas.pushSprite(0, 0);
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  cfg.internal_mic = true;
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.setBrightness(128);
  M5.Display.fillScreen(TFT_BLACK);

  canvas.setColorDepth(8);
  if (!canvas.createSprite(kScreenW, kScreenH)) {
    M5.Display.setTextColor(TFT_RED);
    M5.Display.drawString("createSprite failed", 4, 4);
    while (true) M5.delay(1000);
  }

  setupBandEdges();
  for (auto& l : band_level) l = 0.0f;

  // Core2 ではマイクとスピーカーが同じ I2S ポートを使うため、スピーカーを止めてからマイクを開始する
  M5.Speaker.end();
  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = kSampleRate;
  M5.Mic.config(mic_cfg);
  if (!M5.Mic.begin()) {
    M5.Display.setTextColor(TFT_RED);
    M5.Display.drawString("Mic.begin failed", 4, 4);
    while (true) M5.delay(1000);
  }
}

void loop() {
  M5.update();

  if (!M5.Mic.record(rec_buf, kFftSize, kSampleRate)) {
    M5.delay(10);
    return;
  }
  // 録音完了(バッファ解放)まで待つ
  while (M5.Mic.isRecording()) {
    M5.delay(1);
  }

  for (int i = 0; i < kFftSize; ++i) {
    vReal[i] = static_cast<float>(rec_buf[i]);
    vImag[i] = 0.0f;
  }
  fft.dcRemoval();
  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  updateBandLevels();
  render();
}
