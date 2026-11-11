"""Local QR entry and an allowlist of public CA files. Never serve private keys."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import ipaddress
from pathlib import Path
from urllib.parse import urlsplit

from control_launcher import detect_ipv4

ROOT=Path(__file__).resolve().parent
PUBLIC_FILES={
    '/voice-rootCA.crt':('voice-rootCA.crt','application/x-x509-ca-cert'),
    '/voice-rootCA.cer':('voice-rootCA.cer','application/pkix-cert'),
    '/iphone.mobileconfig':('iphone.mobileconfig','application/x-apple-aspen-config'),
}


def index(address):
    address=str(ipaddress.IPv4Address(address))
    return '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>平衡车控制台</title>
<style>body{font:17px system-ui;max-width:600px;margin:35px auto;padding:20px;line-height:1.8;background:#f3f7f6;color:#18332d}main{padding:25px;background:white;border-radius:22px;text-align:center}h1{margin:0}img{display:block;margin:12px auto;width:250px;max-width:100%}a{display:block;margin:14px 0}.open{background:#176b58;color:white;padding:12px;border-radius:12px;text-decoration:none}details{text-align:left;margin-top:24px}small{color:#536b64}</style>
<main><h1>手机扫码，直接控制</h1><p>手机与电脑连接同一热点或 Wi-Fi。</p>
<img src="/access.svg" alt="手机控制页面二维码">
<a class="open" href="https://__IP__:8080/">打开控制页面</a>
<small>当前地址：https://__IP__:8080/</small>
<p>已信任过证书的手机直接打开即可。连接后先确认角色和急停状态。</p>
<details><summary>首次使用手机：安装证书</summary>
<a href="/voice-rootCA.crt">安卓：下载 CA 证书</a>
<a href="/iphone.mobileconfig">iPhone：下载证书描述文件</a>
<p>安卓在系统设置中安装 CA 证书。iPhone 安装描述文件后，在“设置 → 通用 → 关于本机 → 证书信任设置”开启该证书的完全信任。</p>
<p>后续换 IP 时由启动程序自动更新服务端证书，手机保留同一 CA。</p></details></main></html>'''.replace('__IP__',address)


def qr_svg(address):
    import qrcode
    from qrcode.image.svg import SvgPathImage
    address=str(ipaddress.IPv4Address(address))
    result=io.BytesIO()
    qrcode.make('https://'+address+':8080/',image_factory=SvgPathImage,border=4).save(result)
    return result.getvalue()


def handler(root=ROOT,ip_provider=detect_ipv4):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route=urlsplit(self.path).path
            try:
                if route=='/':
                    data,mime=index(ip_provider()).encode(),'text/html; charset=utf-8'
                elif route=='/access.svg':
                    data,mime=qr_svg(ip_provider()),'image/svg+xml'
                elif route in PUBLIC_FILES:
                    name,mime=PUBLIC_FILES[route]
                    data=(root/'certs/public'/name).read_bytes()
                else:
                    self.send_error(404);return
            except (OSError,RuntimeError,ValueError,ImportError):
                self.send_error(503,'Entry page not ready');return
            self.send_response(200)
            self.send_header('Content-Type',mime)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data)
    return Handler


if __name__=='__main__':
    ThreadingHTTPServer(('0.0.0.0',8081),handler()).serve_forever()
