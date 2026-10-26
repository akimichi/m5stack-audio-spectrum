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

## 注意: マイクはバッテリーボトム側にある

Core2（v1.1 含む）の内蔵マイク SPM1423 と IMU（MPU6886）は本体ではなく、背面に装着する標準のバッテリーボトム基板に載っており、
M-BUS 経由（GPIO0 = PDM CLK, GPIO34 = PDM DATA）で接続される。ボトムを外した状態ではマイクが存在せず、サンプル値が定数（-8）になる。
本プログラムは起動時に IMU の有無を確認し、見つからない場合はシリアルと画面に `NO IMU: bottom (mic) not attached?` と警告を出す。

## ビルドと書き込み

```sh
pio run                    # ビルド
pio run -t upload          # Core2 を USB 接続して書き込み
pio device monitor         # シリアルモニタ (115200bps)
```
