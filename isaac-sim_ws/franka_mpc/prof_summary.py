"""Summarise a py-spy raw profile (py-spy record -f raw -t): samples per thread, and the hottest frames (leaf = where the time is spent, inclusive = in the stack).
   python3 prof_summary.py runs/<run>/ep00/prof.raw [--top 25]"""
import sys, collections
f = sys.argv[1]; top = int(sys.argv[sys.argv.index("--top") + 1]) if "--top" in sys.argv else 25
thr = collections.Counter(); leaf = collections.Counter(); incl = collections.Counter(); tot = 0
for line in open(f):
    line = line.rstrip("\n")
    if not line: continue
    stack, _, n = line.rpartition(" "); n = int(n); frames = stack.split(";"); tot += n
    thr[frames[0]] += n; leaf[frames[-1]] += n
    for fr in set(frames[1:]): incl[fr] += n
print(f"{tot} samples")
print("threads:"); [print(f"  {c:6d} {100 * c / tot:5.1f}%  {t}") for t, c in thr.most_common(12)]
print("leaf frames:"); [print(f"  {c:6d} {100 * c / tot:5.1f}%  {t[:150]}") for t, c in leaf.most_common(top)]
print("inclusive frames:"); [print(f"  {c:6d} {100 * c / tot:5.1f}%  {t[:150]}") for t, c in incl.most_common(top)]
