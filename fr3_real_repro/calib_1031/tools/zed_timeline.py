import re, datetime, collections, subprocess
def episodes(path, label):
    ts=[]; rec=[]
    pat=re.compile(r"\[(\d{10})\.\d+\].*Connection issue detected: CAMERA REBOOTING")
    rp=re.compile(r"\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d) UTC\].*Automatically recover from image capture failure")
    for line in open(path,errors="ignore"):
        if "CAMERA REBOOTING" in line:
            m=pat.search(line)
            if m: ts.append(int(m.group(1)))
        elif "Automatically recover" in line:
            m=rp.search(line)
            if m: rec.append(m.group(1))
    # group reboot lines into episodes (gap > 60 s)
    eps=[]; 
    for t in ts:
        if eps and t-eps[-1][1]<=60: eps[-1][1]=t; eps[-1][2]+=1
        else: eps.append([t,t,1])
    print(f"== {label}: {len(ts)} REBOOTING lines in {len(eps)} episodes, {len(rec)} recoveries logged")
    for a,b,n in eps[-14:]:
        print("   ", datetime.datetime.utcfromtimestamp(a).strftime("%m-%d %H:%M:%S"), "->", datetime.datetime.utcfromtimestamp(b).strftime("%H:%M:%S"), f"({b-a+1:4d} s, {n} lines)")
    byh=collections.Counter(datetime.datetime.utcfromtimestamp(a).strftime("%m-%d %H") for a,_,_ in eps); print("   episodes per hour (UTC):", dict(sorted(byh.items())[-12:]))
    print("   last recoveries:", rec[-4:])
episodes("/tmp/zed1.log","zed1"); episodes("/tmp/zed2.log","zed2")
