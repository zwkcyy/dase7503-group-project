#!/bin/bash
set -euo pipefail

MODEL_DIR="$HOME/.local/share/robot_vision/wechat_qrcode"

mkdir -p "$MODEL_DIR"
cd "$MODEL_DIR"

echo "Downloading WeChatQRCode models..."

BASE="https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/a8b69ccc738421293254aec5ddb38bd523503252"

for model in detect.prototxt detect.caffemodel sr.prototxt sr.caffemodel; do
  wget -q --timeout=30 --tries=3 -O "$model.part" "$BASE/$model"
  test -s "$model.part"
  mv -- "$model.part" "$model"
done

echo "Models installed to:"
echo "$MODEL_DIR"
