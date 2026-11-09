import csv,sys,datetime as D,numpy as np
rows=[]
for r in csv.reader(open(sys.argv[1])):
    try: c=np.array([float(x) for x in r[-3].split('|')]); m=np.array([float(x) for x in r[-2].split('|')])
    except: continue
    rows.append((D.datetime.fromisoformat(r[0]).timestamp(),c,m,r[0][11:23],r[-1][:80]))
for a,b in [("21:16:31","21:16:37"),("21:36:00","21:36:06")]:
    W=[x for x in rows if a<=x[3][:8]<=b]; print("window",a,b)
    for p,q in zip(W[:-1],W[1:]):
        v=(q[2]-p[2])/(q[0]-p[0]); print("  ",q[3],"v",np.round(v,2),"err %.3f"%np.abs(q[1]-q[2]).max())
