"""Bounded Agent crash recovery. Reconnection never resets the controller estop."""
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from control_launcher import ROOT,agent_command


def main():
    import fcntl
    (ROOT/'run').mkdir(exist_ok=True)
    with (ROOT/'run/agent.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return
        command,backend=agent_command(ROOT)
        stopped=[False];child=[None];failures=deque()
        def stop(signum,frame):
            stopped[0]=True
            if child[0] and child[0].poll() is None:child[0].terminate()
        signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
        def status(value,code=None):
            data=dict(status=value,backend=backend,last_exit=code,recent_failures=len(failures),at=time.time())
            temporary=ROOT/'run/agent-status.tmp'
            temporary.write_text(json.dumps(data));os.replace(temporary,ROOT/'run/agent-status.json')
            print('AGENT',data,flush=True)
        while not stopped[0]:
            status('STARTING')
            try:
                child[0]=subprocess.Popen(command)
            except OSError as error:
                print('Agent 启动失败:',error,flush=True)
                status('FAILED');return
            running=False
            while child[0].poll() is None and not stopped[0]:
                if not running:
                    try:
                        entries=Path('/proc/net/udp').read_text().splitlines()[1:]
                        if any(row.split()[1].endswith(':22C3') for row in entries if len(row.split())>2):
                            status('RUNNING');running=True
                    except OSError:pass
                time.sleep(.2)
            if stopped[0]:
                if backend=='docker':
                    try:subprocess.run(['docker','stop','--time','2','balancecar-control-agent'],capture_output=True,timeout=6)
                    except (OSError,subprocess.TimeoutExpired):pass
                try:child[0].wait(timeout=5)
                except subprocess.TimeoutExpired:pass
                status('STOPPED');return
            code=child[0].returncode
            if code==0:status('STOPPED',code);return
            now=time.monotonic()
            while failures and now-failures[0]>60:failures.popleft()
            failures.append(now)
            if len(failures)>=3:status('FAILED',code);return
            status('RETRYING',code)
            until=time.monotonic()+min(8,2**len(failures))
            while not stopped[0] and time.monotonic()<until:time.sleep(.2)


if __name__=='__main__':main()
