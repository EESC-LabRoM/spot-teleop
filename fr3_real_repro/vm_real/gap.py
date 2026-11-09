import csv,sys
rows=[r for r in csv.DictReader(open('log/mpc/mpc_node.csv')) if r.get('step') and r.get('cmd_pos')]
out=[]
for r in rows:
    c=[float(x) for x in r['cmd_pos'].split('|')]; q=[float(x) for x in r['curr_pos'].split('|')]
    out.append((r['timestamp'][11:19],int(r['step']),1000*max(abs(a-b) for a,b in zip(c,q)),r['pose_error'][:6]))
for o in out[::max(1,len(out)//12)]: print(*o)
print('max gap mrad %.1f'%max(o[2] for o in out))
