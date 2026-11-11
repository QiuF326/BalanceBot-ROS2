"""Network address choice, process ownership and public-only phone entry."""
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

from control_launcher import certificate_covers,detect_ipv4,ensure_certificate,stop_owned
from serve_certificates import handler


class AccessTests(unittest.TestCase):
    def test_address_comes_from_current_default_network_not_docker(self):
        calls=[]
        def command(args):
            calls.append(args)
            if args[-1]=='default':return json.dumps([{'dev':'ens33'}]).encode()
            return json.dumps([{'addr_info':[{'scope':'global','local':'192.168.154.46'}]}]).encode()
        self.assertEqual(detect_ipv4(command),'192.168.154.46')
        self.assertEqual(calls[-1][-1],'ens33')

    def test_no_default_network_or_link_local_does_not_produce_a_phone_url(self):
        with self.assertRaises(RuntimeError):detect_ipv4(lambda args:b'[]')
        def command(args):
            if args[-1]=='default':return b'[{"dev":"ens33"}]'
            return b'[{"addr_info":[{"scope":"global","local":"169.254.1.2"}]}]'
        with self.assertRaises(RuntimeError):detect_ipv4(command)

    def test_certificate_ip_check_is_exact(self):
        san='DNS:localhost, IP Address:192.168.154.46, IP Address:127.0.0.1'
        self.assertTrue(certificate_covers(san,'192.168.154.46'))
        self.assertFalse(certificate_covers(san,'192.168.154.4'))
        self.assertFalse(certificate_covers(san,'192.168.106.46'))

    def test_unknown_port_owner_is_never_stopped(self):
        with patch('control_launcher.owned_process',return_value=None),patch('control_launcher.os.kill') as kill:
            with self.assertRaises(RuntimeError):stop_owned(Path('/unused'),'dual_control_voice_server.py',8080)
            kill.assert_not_called()

    @unittest.skipUnless(shutil.which('mkcert') and shutil.which('openssl'),'certificate tools required')
    def test_ip_change_renews_server_certificate_and_keeps_the_same_ca(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for folder in ('run','certs/ca','certs/public'):(root/folder).mkdir(parents=True,exist_ok=True)
            env=os.environ.copy();env['CAROOT']=str(root/'certs/ca')
            subprocess.run(['mkcert','-cert-file',str(root/'certs/phone-cert.pem'),
                            '-key-file',str(root/'certs/phone-key.pem'),'localhost','127.0.0.1','192.0.2.1'],
                           env=env,check=True,capture_output=True)
            ca=(root/'certs/ca/rootCA.pem').read_bytes()
            (root/'certs/public/voice-rootCA.crt').write_bytes(ca)
            self.assertTrue(ensure_certificate(root,'192.0.2.2'))
            self.assertEqual((root/'certs/ca/rootCA.pem').read_bytes(),ca)
            self.assertFalse(ensure_certificate(root,'192.0.2.2'),'Repeated startup does not reissue a valid certificate')

    def test_qr_entry_only_exposes_public_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'certs/public').mkdir(parents=True)
            (root/'certs/phone-key.pem').write_text('PRIVATE KEY MUST NOT BE SERVED')
            (root/'certs/public/voice-rootCA.crt').write_text('PUBLIC CA')
            server=ThreadingHTTPServer(('127.0.0.1',0),handler(root,lambda:'192.168.154.46'))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            base='http://127.0.0.1:'+str(server.server_port)
            try:
                with urllib.request.urlopen(base+'/') as res:
                    page=res.read().decode()
                    self.assertIn('https://192.168.154.46:8080/',page)
                    self.assertIn('/access.svg',page)
                    self.assertNotIn('balancecar.local',page)
                with urllib.request.urlopen(base+'/access.svg') as res:
                    self.assertEqual(res.headers['Content-Type'],'image/svg+xml')
                    self.assertTrue(ET.fromstring(res.read()).tag.endswith('svg'))
                with urllib.request.urlopen(base+'/voice-rootCA.crt') as res:
                    self.assertEqual(res.read(),b'PUBLIC CA')
                for route in ('/phone-key.pem','/certs/phone-key.pem','/../certs/phone-key.pem','/rootCA-key.pem'):
                    with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(base+route)
                    self.assertEqual(error.exception.code,404)
            finally:
                server.shutdown();server.server_close();thread.join(timeout=2)


if __name__=='__main__':unittest.main()
