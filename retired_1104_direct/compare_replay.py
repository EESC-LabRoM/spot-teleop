#!/usr/bin/env python3
"""Direct-vs-bridged replay comparison (joint space + velocity)."""
import csv
import math

ARM = 7


def load_new(path):
    t, pq, mq = [], [], []
    with open(path) as f:
        for d in csv.DictReader(f):
            try:
                p = [float(d['pq%d' % i]) for i in range(ARM)]
                m = [float(d['mq%d' % i]) for i in range(ARM)]
            except ValueError:
                continue  # dropped sample (empty fields)
            t.append(float(d['t']))
            pq.append(p)
            mq.append(m)
    return t, pq, mq


def load_old(path):
    t, cmd, cur, pe = [], [], [], []
    with open(path) as f:
        for d in csv.DictReader(f):
            try:
                c = [float(x) for x in d['cmd_pos'].split('|')]
                m = [float(x) for x in d['curr_pos'].split('|')]
                if len(c) != ARM or len(m) != ARM:
                    continue
            except (ValueError, AttributeError):
                continue
            t.append(d['t'])
            cmd.append(c)
            cur.append(m)
            try:
                pe.append(float(d['pose_error']))
            except ValueError:
                pass
    return t, cmd, cur, pe


def vel(series, dt):
    return [[(series[k + 1][j] - series[k][j]) / dt for j in range(ARM)]
            for k in range(len(series) - 1)]


def rmse(a, b):
    n = len(a)
    per = [math.sqrt(sum((a[k][j] - b[k][j]) ** 2 for k in range(n)) / n) for j in range(ARM)]
    return per, math.sqrt(sum(x * x for x in per) / ARM)


def lag(ref, meas, dt, max_lag=30):
    # delay (s) of meas behind ref minimizing squared error, joint-averaged
    best, bl = None, 0
    for lag in range(max_lag + 1):
        n = len(ref) - lag
        if n < 10:
            break
        e = sum((ref[k][j] - meas[k + lag][j]) ** 2 for k in range(n) for j in range(ARM)) / n
        if best is None or e < best:
            best, bl = e, lag
    return bl * dt


def smoothness(series, dt):
    v = vel(series, dt)
    acc = [[(v[k + 1][j] - v[k][j]) / dt for j in range(ARM)] for k in range(len(v) - 1)]
    return max(abs(x) for row in acc for x in row)


def gaps(t, thresh):
    dts = [t[k + 1] - t[k] for k in range(len(t) - 1)]
    big = [d for d in dts if d > thresh]
    return (sum(dts) / len(dts), max(dts), len(big))


nt, npq, nmq = load_new('/tmp/trace_new.csv')
ot, ocmd, ocur, ope = load_old('/tmp/orig_2005.csv')
ndt = (nt[-1] - nt[0]) / (len(nt) - 1)

print('NEW (direct, %d samples, %.1f s @ %.1f Hz)' % (len(nt), nt[-1] - nt[0], 1 / ndt))
p_rmse, p_all = rmse(npq, nmq)
nv_cmd, nv_meas = vel(npq, ndt), vel(nmq, ndt)
v_rmse, v_all = rmse(nv_cmd, nv_meas)
print('  pos RMSE per joint (rad): %s' % ' '.join('%.4f' % x for x in p_rmse))
print('  pos RMSE overall: %.4f rad (%.2f deg)' % (p_all, math.degrees(p_all)))
print('  vel RMSE overall: %.4f rad/s' % v_all)
print('  lag: %.3f s  maxaccel: %.2f rad/s^2' % (lag(npq, nmq, ndt), smoothness(nmq, ndt)))
mn, mx, nbig = gaps(nt, 0.1)
print('  sample dt mean/max: %.1f/%.1f ms, gaps>100ms: %d' % (mn * 1e3, mx * 1e3, nbig))
nmax = max(abs(npq[k][j] - nmq[k][j]) for k in range(len(nt)) for j in range(ARM))
print('  max abs pos err: %.4f rad (%.2f deg)' % (nmax, math.degrees(nmax)))

print('OLD (bridged 20:05, %d samples)' % len(ot))
o_rmse, o_all = rmse(ocmd, ocur)
ots = list(range(len(ot)))
odt = 60.0 / len(ot)
ov_cmd, ov_meas = vel(ocmd, odt), vel(ocur, odt)
ov_rmse, ov_all = rmse(ov_cmd, ov_meas)
print('  pos RMSE per joint (rad): %s' % ' '.join('%.4f' % x for x in o_rmse))
print('  pos RMSE overall: %.4f rad (%.2f deg)' % (o_all, math.degrees(o_all)))
print('  vel RMSE overall: %.4f rad/s' % ov_all)
print('  lag: %.3f s  maxaccel: %.2f rad/s^2' % (lag(ocmd, ocur, odt), smoothness(ocur, odt)))
print('  MPC pose_error mean/max: %.4f / %.4f m' % (sum(ope) / len(ope), max(ope)))
omax = max(abs(ocmd[k][j] - ocur[k][j]) for k in range(len(ot)) for j in range(ARM))
print('  max abs pos err: %.4f rad (%.2f deg)' % (omax, math.degrees(omax)))
print('RATIO new/old: pos %.2f  vel %.2f' % (p_all / o_all, v_all / ov_all))
