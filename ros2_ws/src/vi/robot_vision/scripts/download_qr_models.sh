#!/bin/bash
set -e

MODEL_DIR="$HOME/.local/share/robot_vision/wechat_qrcode"

mkdir -p "$MODEL_DIR"
cd "$MODEL_DIR"

echo "Downloading WeChatQRCode models..."

BASE="https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/a8b69ccc738421293254aec5ddb38bd523503252"

wget -O detect.prototxt "$BASE/detect.prototxt"
wget -O detect.caffemodel "$BASE/detect.caffemodel"
wget -O sr.prototxt "$BASE/sr.prototxt"
wget -O sr.caffemodel "$BASE/sr.caffemodel"

echo "Models installed to:"
echo "$MODEL_DIR"
