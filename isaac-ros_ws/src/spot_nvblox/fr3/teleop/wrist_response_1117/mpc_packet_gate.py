"""Require a fresh feasible native MPC packet matching the exact preview joints."""
from collections import deque
import json,math,socket,time
class PacketGate:
    def __init__(self):self.rows=deque(maxlen=128)
    def add(self,p,received_ns):
        try:
            if p['version']!=1 or len(p['names'])!=len(p['q']) or not all(math.isfinite(x) for x in p['q']):return
            int(p['stamp_ns'])
        except (KeyError,TypeError,ValueError):return
        self.rows.append((p,received_ns))
    def accept(self,names,q,now_ns):
        if not self.rows or len(names)!=len(q) or not all(math.isfinite(x) for x in q):return False
        latest,received=self.rows[-1]
        if not latest.get('feasible') or not 0<=now_ns-int(latest['stamp_ns'])<300_000_000 or not 0<=now_ns-received<300_000_000:return False
        for p,r in reversed(self.rows):
            if not 0<=now_ns-int(p['stamp_ns'])<300_000_000 or not 0<=now_ns-r<300_000_000:continue
            if list(names)==p['names'] and len(q)==len(p['q']) and max((abs(a-b) for a,b in zip(q,p['q'])),default=0)<=1e-7:return bool(p.get('feasible'))
        return False
class SocketGate(PacketGate):
    def __init__(self,port=47927):
        super().__init__();self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',port));self.sock.setblocking(False)
    def accept(self,names,q,now_ns=None):
        for _ in range(128):
            try:
                raw,_=self.sock.recvfrom(8192);p=json.loads(raw);self.add(p,time.time_ns())
            except BlockingIOError:break
            except (ValueError,TypeError,UnicodeError):continue
        return super().accept(names,q,time.time_ns() if now_ns is None else now_ns)
