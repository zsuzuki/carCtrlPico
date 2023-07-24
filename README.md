# RaspberryPi PICO Wを使って古いラジコンを動かしたい

頂いた古いラジコンが壊れてしまったので、折角だからマイコン使ってブルートゥースで操作出来るようにしようと思い立った。

## 構成

- RaspberryPi PICO W
- [モーターサーボドライバ基板](https://kitronik.co.uk/products/5348-kitronik-simply-robotics-for-raspberry-pi-pico)

### 2023/07/24現在

かなり古くて、ステアリングにはサーボを使ってすらいない。何時かサーボに差し替えたいけど、とりあえずそのままモーターを使用する。

## プログラム

pico-examplesに入っているpico_w/のサンプル、spp_counterを拝借して、モーター出力部分だけ書き換える形にしている。
