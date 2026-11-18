from mpc_packet_gate import PacketGate
n=['j1','j2'];q=[.1,.2];t=1_000_000_000
p=dict(version=1,names=n,q=q,feasible=True,stamp_ns=t)
g=PacketGate();assert not g.accept(n,q,t)
g.add(p,t);assert g.accept(n,q,t+1)
assert not g.accept(n,[.1,.3],t+1)
assert not g.accept(list(reversed(n)),q,t+1)
assert not g.accept(n,q,t+300_000_000)
assert not g.accept(n,q,t-1)
g.add(dict(p,feasible=False,stamp_ns=t+2),t+2);assert not g.accept(n,q,t+3)
g.add(dict(p,stamp_ns=t+4),t+4);assert g.accept(n,q,t+5)
assert not g.accept(n,[float('nan'),.2],t+5)
print('PASS feasible match; missing, stale, future, nonfinite, mismatched and newer-infeasible refusals')
