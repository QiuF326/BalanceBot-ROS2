"""Idempotent Ubuntu launcher: current IP, existing CA, one Agent and one server."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import signal
import ssl
import subprocess
import tempfile
import time
import urllib.request

ROOT = Path(__file__).resolve().parent


def detect_ipv4(check_output=subprocess.check_output):
    routes = json.loads(check_output(['ip', '-j', '-4', 'route', 'show', 'default']))
    for route in routes:
        dev = route.get('dev')
        if not dev:
            continue
        interfaces = json.loads(check_output(['ip', '-j', '-4', 'addr', 'show', 'dev', dev]))
        for interface in interfaces:
            for item in interface.get('addr_info', []):
                address = ipaddress.IPv4Address(item.get('local', '0.0.0.0'))
                if item.get('scope') == 'global' and not (address.is_loopback or address.is_link_local or address.is_unspecified):
                    return str(address)
    raise RuntimeError('虚拟机没有可用的局域网 IP，请先连接手机热点或 Wi-Fi')


def certificate_covers(text, address):
    return address in re.findall(r'IP Address:([^,\s]+)', text)


def ensure_certificate(root, address):
    certs = root/'certs'
    ca = certs/'ca/rootCA.pem'
    if not ca.is_file() or not (certs/'ca/rootCA-key.pem').is_file():
        raise RuntimeError('缺少原 CA，未创建新的手机信任；请先完成首次证书配置')
    if ca.read_bytes() != (certs/'public/voice-rootCA.crt').read_bytes():
        raise RuntimeError('公开 CA 与签发 CA 不一致，已停止启动')
    cert, key = certs/'phone-cert.pem', certs/'phone-key.pem'
    valid = False
    if cert.exists() and key.exists():
        info = subprocess.run(['openssl','x509','-in',str(cert),'-noout','-ext','subjectAltName'],capture_output=True,text=True)
        expiry = subprocess.run(['openssl','x509','-in',str(cert),'-noout','-checkend','604800'],capture_output=True)
        chain = subprocess.run(['openssl','verify','-CAfile',str(ca),str(cert)],capture_output=True)
        valid = info.returncode == expiry.returncode == chain.returncode == 0 and certificate_covers(info.stdout,address)
    if valid:
        return False
    with tempfile.TemporaryDirectory(prefix='certificate-',dir=root/'run') as folder:
        temporary = Path(folder)
        env = os.environ.copy();env['CAROOT'] = str(certs/'ca')
        subprocess.run(['mkcert','-cert-file',str(temporary/'cert.pem'),'-key-file',str(temporary/'key.pem'),
                        'localhost','127.0.0.1',address,'balancecar.local'],env=env,check=True,capture_output=True)
        subprocess.run(['openssl','verify','-CAfile',str(ca),str(temporary/'cert.pem')],check=True,capture_output=True)
        backup = root/'setup-cache'/('certificate-'+time.strftime('%Y%m%d-%H%M%S'))
        backup.mkdir(mode=0o700,parents=True,exist_ok=True)
        for source in (cert,key):
            if source.exists():
                shutil.copy2(source,backup/source.name)
                os.chmod(backup/source.name,0o600)
        os.chmod(temporary/'key.pem',0o600)
        os.replace(temporary/'key.pem',key)
        os.replace(temporary/'cert.pem',cert)
    return True


def listener(port, udp=False):
    result = subprocess.run(['ss','-H','-lunp' if udp else '-ltnp','sport = :'+str(port)],capture_output=True,text=True,check=True)
    return result.stdout.strip()


def owned_process(root, script):
    found = []
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            args = (item/'cmdline').read_bytes().split(b'\0')
            if item.stat().st_uid == os.getuid() and (item/'cwd').resolve() == root.resolve() and any(
                    arg == script.encode() or arg.endswith(b'/'+script.encode()) for arg in args):
                found.append((int(item.name),(item/'stat').read_text().split()[21]))
        except OSError:
            continue
    if len(found) > 1:
        raise RuntimeError(script+' 存在多个实例，未自动停止任何进程')
    return found[0] if found else None


def stop_owned(root, script, port):
    identity = owned_process(root,script)
    if identity is None:
        raise RuntimeError(str(port)+' 由其他程序占用，未停止它')
    pid, started = identity
    if (Path('/proc')/str(pid)/'stat').read_text().split()[21] != started:
        raise RuntimeError('进程身份变化，未停止它')
    os.kill(pid,signal.SIGTERM)
    deadline = time.monotonic()+7
    while listener(port) and time.monotonic()<deadline:
        time.sleep(.1)
    if listener(port):
        raise RuntimeError('原服务未正常退出，未启动重复实例')


def launch(root, name, args):
    log = root/'logs'/(name+'-'+time.strftime('%Y%m%d-%H%M%S')+'.log')
    with log.open('ab') as out:
        process = subprocess.Popen(args,cwd=root,stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,
                                   start_new_session=True,close_fds=True)
    record = dict(pid=process.pid,started=(Path('/proc')/str(process.pid)/'stat').read_text().split()[21],log=str(log))
    if name=='voice':
        record.update(mode='ros',args=['--start-estopped'])
    (root/'run'/(name+'.json')).write_text(json.dumps(record))
    return process


def agent_command(root=ROOT):
    """Use the cached vendor runtime; never pull images during vehicle control."""
    if shutil.which('docker'):
        try:
            image=subprocess.run(['docker','image','inspect','microros/micro-ros-agent:humble',
                                  '--format','{{.Id}}'],capture_output=True,text=True,timeout=6)
        except subprocess.TimeoutExpired:
            raise RuntimeError('Docker 未响应，未启动其他 Agent；请检查 Docker 服务')
        if image.returncode==0 and image.stdout.strip().startswith('sha256:'):
            return ['docker','run','--rm','--name','balancecar-control-agent','--label','balancecar.control=1',
                    '--network','host','--ipc','host','--user',str(os.getuid())+':'+str(os.getgid()),
                    '-e','ROS_DOMAIN_ID=20',image.stdout.strip(),'udp4','--port','8899','-v','4'],'docker'
    setup=Path.home()/'uros_ws/install/setup.bash'
    if not setup.is_file():raise RuntimeError('缺少 Agent 镜像或本机 Agent 工作区')
    command='source /opt/ros/humble/setup.bash && source "$1" && export ROS_DOMAIN_ID=20 && exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 8899 -v 4'
    return ['bash','-c',command,'agent',str(setup)],'native'


def start(root=ROOT, open_page=False):
    import fcntl
    for folder in ('run','logs','setup-cache'):
        (root/folder).mkdir(exist_ok=True)
    with (root/'run/launcher.lock').open('a') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print('控制台正在启动，请稍候。')
            return
        address = detect_ipv4()
        changed = ensure_certificate(root,address)
        python = root/'.venv/bin/python'
        if not python.is_file():
            raise RuntimeError('缺少 .venv，请先按 README 安装依赖')
        agent = listener(8899,udp=True)
        if agent and 'micro_ros_agent' not in agent:
            raise RuntimeError('UDP 8899 已被其他程序占用，未启动第二个 Agent')
        if not agent and owned_process(root,'agent_runner.py') is None:
            launch(root,'agent',['/usr/bin/python3','-u','agent_runner.py'])
        voice = listener(8080)
        if voice and owned_process(root,'dual_control_voice_server.py') is None:
            raise RuntimeError('TCP 8080 已被其他程序占用，未停止它')
        if voice and changed:
            stop_owned(root,'dual_control_voice_server.py',8080)
            voice = ''
        if not voice:
            command = 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 && exec "$@"'
            launch(root,'voice',['bash','-c',command,'voice',str(python),'-u','dual_control_voice_server.py',
                                 '--model','vosk-model-small-cn-0.22','--cert','certs/phone-cert.pem',
                                 '--key','certs/phone-key.pem','--start-estopped'])
        if listener(8081):
            if owned_process(root,'serve_certificates.py') is None:
                raise RuntimeError('TCP 8081 已被其他程序占用，未停止它')
        else:
            launch(root,'certificate',[str(python),'-u','serve_certificates.py'])
        context = ssl.create_default_context(cafile=str(root/'certs/public/voice-rootCA.crt'))
        deadline = time.monotonic()+25
        while True:
            try:
                with urllib.request.urlopen('https://'+address+':8080/health',context=context,timeout=2) as response:
                    healthy = json.load(response).get('ok')
                with urllib.request.urlopen('http://127.0.0.1:8081/access.svg',timeout=2) as response:
                    qr_ready = b'<svg' in response.read()
                if healthy and qr_ready and 'micro_ros_agent' in listener(8899,udp=True):
                    break
            except (OSError,ValueError):
                pass
            if time.monotonic()>=deadline:
                raise RuntimeError('控制服务或 Agent 尚未就绪，请检查 logs/voice-*.log、logs/agent-*.log 和 run/agent-status.json')
            time.sleep(.3)
        print('控制服务已就绪： https://'+address+':8080/')
        print('二维码入口： http://127.0.0.1:8081/ （在 Ubuntu 浏览器打开，手机扫码）')
        if open_page:
            env=os.environ.copy()
            for name in ('LD_LIBRARY_PATH','PYTHONPATH','GTK_PATH','GTK_MODULES','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH'):
                env.pop(name,None)
            with (root/'logs/access-window.log').open('ab') as log:
                subprocess.Popen(['/usr/bin/python3',str(root/'access_window.py')],env=env,stdout=log,stderr=log)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--open',action='store_true',help='打开本机二维码入口')
    options=parser.parse_args()
    try:
        start(open_page=options.open)
    except (RuntimeError,OSError,subprocess.SubprocessError) as error:
        if options.open and shutil.which('zenity'):
            subprocess.Popen(['zenity','--error','--title=平衡车控制台','--text='+str(error)])
        parser.exit(1,str(error)+'\n')
