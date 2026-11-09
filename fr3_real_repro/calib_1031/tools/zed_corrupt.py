import re, datetime, collections
def scan(path,label):
    cor=collections.Counter(); reb=[]; first=None; last=None
    for l in open(path,errors="ignore"):
        if "Corrupted frame detected" in l:
            m=re.search(r"\[(\d{10})\.",l)
            if m:
                t=int(m.group(1)); cor[datetime.datetime.utcfromtimestamp(t).strftime("%m-%d %H")]+=1; first=first or t; last=t
        elif "CAMERA REBOOTING" in l:
            m=re.search(r"\[(\d{10})\.",l)
            if m: reb.append(int(m.group(1)))
    eps=0; p=-999
    for t in reb:
        if t-p>60: eps+=1
        p=t
    tot=sum(cor.values()); print(f"== {label}: corrupted-frame warnings {tot}, reboot episodes {eps}")
    if first: print("   span", datetime.datetime.utcfromtimestamp(first).strftime("%m-%d %H:%M"), "->", datetime.datetime.utcfromtimestamp(last).strftime("%m-%d %H:%M"))
    top=cor.most_common(6); print("   hours with the most corrupted frames:", top)
scan("/tmp/zed1.log","zed1 (current log)"); scan("/tmp/zed2.log.bak_before_restart_20261030","zed2 (log before the restart this morning)"); scan("/tmp/zed2.log","zed2 (since the restart)")
