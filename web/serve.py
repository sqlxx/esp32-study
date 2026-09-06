#!/usr/bin/env python3
"""用自签名 HTTPS 提供 web 页，方便 Android Chrome 使用 Web Bluetooth。"""

from __future__ import annotations

import http.server
import ipaddress
import os
import socket
import ssl
import subprocess
import tempfile
from pathlib import Path

PORT = 8443
DIR = Path(__file__).resolve().parent


def local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def make_cert(cert_file: Path, key_file: Path, host: str) -> None:
    san = f"DNS:localhost,IP:127.0.0.1,IP:{host}"
    # 若 host 本身不是合法 IP，只保留 localhost
    try:
        ipaddress.ip_address(host)
    except ValueError:
        san = "DNS:localhost,IP:127.0.0.1"

    subprocess.run(
        [
            "openssl",
            "req",
            "-x509",
            "-newkey",
            "rsa:2048",
            "-keyout",
            str(key_file),
            "-out",
            str(cert_file),
            "-days",
            "3",
            "-nodes",
            "-subj",
            "/CN=ESP32-WebBLE",
            "-addext",
            f"subjectAltName={san}",
        ],
        check=True,
        capture_output=True,
    )


def main() -> None:
    os.chdir(DIR)
    host = local_ip()

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        cert = tmp_path / "cert.pem"
        key = tmp_path / "key.pem"
        make_cert(cert, key, host)

        handler = http.server.SimpleHTTPRequestHandler
        httpd = http.server.HTTPServer(("0.0.0.0", PORT), handler)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(certfile=cert, keyfile=key)
        httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

        print("ESP32 Web Bluetooth 页面已启动（自签名 HTTPS）")
        print(f"  本机:   https://localhost:{PORT}")
        print(f"  手机:   https://{host}:{PORT}")
        print("手机浏览器若提示证书不安全，选「继续访问」即可。")
        print("Ctrl+C 结束。")
        httpd.serve_forever()


if __name__ == "__main__":
    main()
