# -*- coding: utf-8 -*-
"""exp0.5 回放诊断：抬脚/腾空/步态时序，对照 exp0.3 拖步基线"""
import csv, os

def load(path):
    with open(path, newline='') as f:
        rdr = csv.reader(f)
        header = next(rdr)
        rows = [[float(x) for x in r] for r in rdr if r]
    return {name: i for i, name in enumerate(header)}, rows

def col(rows, idx, name):
    return [r[idx[name]] for r in rows]

def pct(v, q):
    s = sorted(v); n = len(s)
    k = (n - 1) * q; f = int(k); c = min(f + 1, n - 1)
    return s[f] + (s[c] - s[f]) * (k - f)

SEGS = [("S0 站立", 0, 10), ("S1 cmd0.4", 10, 20), ("S2 cmd0.6", 20, 30), ("S3 停止", 30, 40)]
CONTACT_TH = 5.0

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
idx, rows = load(os.path.join(ROOT, 'czy/data/exp0.5/isaac_diag.csv'))
print("[exp0.5 model_3200] rows={}".format(len(rows)))

# 全程最低身高（摔倒检查）
h = col(rows, idx, 'base_height')
print("全程 base_height: min={:.3f} p01={:.3f} max={:.3f}".format(min(h), pct(h, .01), max(h)))

for name, t0, t1 in SEGS:
    seg = [r for r in rows if t0 <= r[idx['time_s']] < t1]
    if not seg:
        continue
    n = len(seg)
    vx = col(seg, idx, 'base_vel_x')
    cmdx = col(seg, idx, 'cmd_linear_x')
    dpx = seg[-1][idx['base_pos_x']] - seg[0][idx['base_pos_x']]
    fz_l, fz_r = col(seg, idx, 'foot_z_l'), col(seg, idx, 'foot_z_r')
    ff_l, ff_r = col(seg, idx, 'foot_force_l'), col(seg, idx, 'foot_force_r')
    sin_ = col(seg, idx, 'phase_sin')

    def base(fz, ff):
        loaded = [z for z, f in zip(fz, ff) if f > 100]
        s = sorted(loaded if loaded else fz)
        return s[len(s) // 2]
    bl, br = base(fz_l, ff_l), base(fz_r, ff_r)
    lift_l = [z - bl for z in fz_l]
    lift_r = [z - br for z in fz_r]

    def windows(sign):
        wins, cur = [], None
        for i, s in enumerate(sin_):
            if s * sign < -0.02:
                cur = [i] if cur is None else cur + [i]
            elif cur is not None:
                wins.append(cur); cur = None
        if cur:
            wins.append(cur)
        return [w for w in wins if len(w) >= 5]
    lw, rw = windows(+1), windows(-1)

    def wstats(wins, lift, ff):
        if not wins:
            return None
        maxl = [max(lift[i] for i in w) for w in wins]
        airt = [sum(1 for i in w if ff[i] < CONTACT_TH) * 0.02 for w in wins]
        band = [sum(1 for i in w if 0.03 <= lift[i] <= 0.06) / len(w) for w in wins]
        return (max(maxl), pct(maxl, .5), max(airt), sum(airt) / len(airt), 100 * sum(band) / len(band))
    ls, rs = wstats(lw, lift_l, ff_l), wstats(rw, lift_r, ff_r)

    print("\n-- {} n={} cmd={:.2f} vx均值={:.3f} Δx={:+.3f}m".format(name, n, sum(cmdx) / n, sum(vx) / n, dpx))
    print("   全段峰值离地 {:+.1f}/{:+.1f}cm | p95离地 {:+.1f}/{:+.1f}cm".format(
        max(lift_l) * 100, max(lift_r) * 100, pct(lift_l, .95) * 100, pct(lift_r, .95) * 100))
    if ls:
        print("   左摆窗{}: 峰值 max/中位 {:.1f}/{:.1f}cm | 腾空 max/均值 {}/{}ms | 带占比 {:.0f}%".format(
            len(lw), ls[0] * 100, ls[1] * 100, ls[2] * 1000, ls[3] * 1000, ls[4]))
    if rs:
        print("   右摆窗{}: 峰值 max/中位 {:.1f}/{:.1f}cm | 腾空 max/均值 {}/{}ms | 带占比 {:.0f}%".format(
            len(rw), rs[0] * 100, rs[1] * 100, rs[2] * 1000, rs[3] * 1000, rs[4]))
    for jn in ['hip_pitch', 'knee_pitch']:
        line = "   "
        for sd in ['left', 'right']:
            p = pct(col(seg, idx, 'pos_{}_{}_joint'.format(sd, jn)), .95) - pct(col(seg, idx, 'pos_{}_{}_joint'.format(sd, jn)), .05)
            line += "{} {} pos幅{:.2f} ".format(sd[0].upper(), jn[:4], p)
        print(line)
    # 摆臂
    sp_l = pct(col(seg, idx, 'pos_left_shoulder_pitch_joint'), .95) - pct(col(seg, idx, 'pos_left_shoulder_pitch_joint'), .05)
    sp_r = pct(col(seg, idx, 'pos_right_shoulder_pitch_joint'), .95) - pct(col(seg, idx, 'pos_right_shoulder_pitch_joint'), .05)
    print("   摆臂 shoulder_pitch 幅度 L/R {:.2f}/{:.2f} rad".format(sp_l, sp_r))
