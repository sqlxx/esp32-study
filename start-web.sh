#!/usr/bin/env bash
# 启动 ESP32 Web Bluetooth 学习页（HTTPS，供 Android Chrome 使用）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
WEB_DIR="$ROOT/web"

if [[ ! -f "$WEB_DIR/serve.py" ]]; then
  echo "找不到 $WEB_DIR/serve.py" >&2
  exit 1
fi

if ! command -v openssl >/dev/null 2>&1; then
  echo "需要 openssl 才能生成自签名证书" >&2
  exit 1
fi

cd "$WEB_DIR"
exec python3 serve.py
