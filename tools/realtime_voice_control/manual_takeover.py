"""Held manual lease; continuous while renewed, never a latched radar bypass."""
from collections import deque
import time

DIRECTIONS={'forward':(.20,0.),'backward':(-.20,0.),'left':(0.,.35),'right':(0.,-.35)}


class ManualTakeover:
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.current=None;self.cancelled=deque(maxlen=128)
        self.message='按住持续低速脱困，松手即停'

    def begin(self,owner,token,role,direction):
        if not isinstance(token,str) or not 8<=len(token)<=80:
            raise ValueError('脱困操作标识无效')
        if (owner,token) in self.cancelled:
            raise ValueError('该按住操作已结束，请松开后重新按住')
        if direction not in DIRECTIONS:
            raise ValueError('脱困方向无效')
        linear,angular=DIRECTIONS[direction]
        if role not in ('FULL','LINEAR','ANGULAR') or (role=='LINEAR' and angular) or (role=='ANGULAR' and linear):
            raise ValueError('脱困方向超出本端角色权限')
        if self.current:
            raise ValueError('另一按住操作正在脱困，请先松手')
        now=self.clock()
        self.current=dict(owner=owner,token=token,role=role,direction=direction,linear=linear,angular=angular,
                          until=now+.25,started=now)
        self.message='人工低速脱困：雷达限制临时旁路，须目视确认路径；松手即停'

    def renew(self,owner,token):
        item=self.current
        now=self.clock()
        if not item or item['owner']!=owner or item['token']!=token:
            raise ValueError('脱困操作已停止，请重新按住')
        if now>=item['until']:
            raise ValueError('脱困续期已超时，请松开后重新按住')
        item['until']=now+.25

    def cancel(self,owner=None,token=None,reason='人工脱困已停止，旧指令不会恢复'):
        if owner is not None and token is not None:
            self.cancelled.append((owner,token))
        item=self.current
        if item and (owner is None or (item['owner']==owner and (token is None or item['token']==token))):
            self.cancelled.append((item['owner'],item['token']))
            self.current=None;self.message=reason;return True
        return False

    def expired(self):
        return self.current is not None and self.clock()>=self.current['until']

    def snapshot(self):
        item=self.current
        return dict(active=item is not None,owner=item['owner'] if item else None,
                    direction=item['direction'] if item else None,message=self.message,
                    remaining_ms=None,held_seconds=round(self.clock()-item['started'],1) if item else 0,
                    lease_ms=250,maximum_seconds=None,linear_limit=.20,angular_limit=.35)
