# LYNCS-rover

[![Build Status](https://travis-ci.com/LYNCS-Keio/LYNCS-rover.svg?branch=develop)](https://travis-ci.com/LYNCS-Keio/LYNCS-rover)  
2019種子島宇宙ロケットコンテストヘ向けたローバーのプログラム

## How to Contribute

See [Let's-get-start-with-git!](https://github.com/LYNCS-Keio/LYNCS-rover/wiki/Let's-get-start-with-git!)

## How to Start

Ubuntu等で`git clone`した後ターミナルで以下を実行するとこのプロジェクトに使われている
Arduino環境を自動で構築してくれます. 自分でArduino環境を構築したい場合は注意してください. 

```sh
bash bin/install_dependencies.sh
```

## Requirement for Arduino

- Arduino IDE > 1.8.7
- i2cdevlib (https://github.com/jrowberg/i2cdevlib)

## Requirement for Raspberry pi

- OpenCV 2
- pybind11
- CMake > 3.0
- micropyGPS
