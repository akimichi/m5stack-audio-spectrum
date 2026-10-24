# m5stack-audio-spectrum

このプロジェクトは、M5Stackを利用したオーディオスペクトラムアナライザーである。

M5Stack Core2にはPDMマイク（SPM1423）が内蔵されているので、I2S入力とFFT（arduinoFFTなど）だけでオーディオスペクトラムアナライザーを作りたい。
言語は、MicroPython ではなく、Arduino + M5Unified を採用する。

## 構成

- `platformio.ini` : PlatformIO 設定（board: `m5stack-core2`, M5Unified + arduinoFFT）
- `src/main.cpp` : 本体
  - 内蔵PDMマイクを `M5.Mic`（I2S PDM RX）で 16kHz 取り込み
  - 512点 FFT（Hamming窓、分解能 31.25Hz）
  - 62.5Hz〜8kHz を対数軸で32バンドに集約し、dBFS でバーグラフ表示

## ビルドと書き込み

```sh
pio run                    # ビルド
pio run -t upload          # Core2 を USB 接続して書き込み
pio device monitor         # シリアルモニタ (115200bps)
```
