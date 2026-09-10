# -*- coding: utf-8 -*-
"""exp0.3 回放诊断：脚步离地/关节幅度/步态奖励 earning 估算，对照 exp0.2 与 mocap 参考"""
import csv, os

def load(path):
    with open(path, newline='') as f:
        rdr = csv.reader(f)
        header = next(rdr)
        rows = [[float(x) for x in r] for r in rdr if r]
    return {name: i for i, name in enumerate(header)}, rows

def col(rows, idx, name):
    i = idx[name]
    return [r[i] for r in rows]

def pct(v, q):
    s = sorted(v); n = len(s)
    if n == 0: return float('nan')
    k = (n - 1) * q; f = int(k); c = min(f + 1, n - 1)
    return s[f] + (s[c] - s[f]) * (k - f)

SEGS = [("S0 站立", 0, 10), ("S1 cmd0.4", 10, 20), ("S2 cmd0.6", 20, 30), ("S3 停止", 30, 40)]
CONTACT_TH = 5.0

def analyze(path, label):
    idx, rows = load(path)
    print("=" * 86)
    print("[{}] rows={}".format(label, len(rows)))
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
            src = loaded if loaded else fz
            s = sorted(src)
            return s[len(s) // 2]
        bl, br = base(fz_l, ff_l), base(fz_r, ff_r)
        lift_l = [z - bl for z in fz_l]
        lift_r = [z - br for z in fz_r]

        def windows(sign):
            wins, cur = [], None
            for i, s in enumerate(sin_):
                inwin = (s * sign < -0.02)
                if inwin:
                    if cur is None:
                        cur = [i]
                    else:
                        cur.append(i)
                elif cur is not None:
                    wins.append(cur); cur = None
            if cur:
                wins.append(cur)
            return [w for w in wins if len(w) >= 5]
        lw, rw = windows(+1), windows(-1)  # sin<0 左摆 / sin>0 右摆

        def wstats(wins, lift, ff):
            if not wins:
                return None
            maxl = [max(lift[i] for i in w) for w in wins]
            airt = [sum(1 for i in w if ff[i] < CONTACT_TH) * 0.02 for w in wins]
            band = [sum(1 for i in w if 0.03 <= lift[i] <= 0.06) / len(w) for w in wins]
            return (max(maxl), pct(maxl, .5), max(airt), sum(airt) / len(airt),
                    100 * sum(band) / len(band))
        ls, rs = wstats(lw, lift_l, ff_l), wstats(rw, lift_r, ff_r)

        # contact_number 估算 raw（contact==stance_mask 得 1，否则 -0.3）
        tot = cnt = 0
        for s, cl, cr in zip(sin_, ff_l, ff_r):
            dbl = abs(s) < 0.1
            lst = dbl or s >= 0
            rst = dbl or s < 0
            for c, st in ((cl > CONTACT_TH, lst), (cr > CONTACT_TH, rst)):
                tot += 1 if c == st else -0.3
                cnt += 1

        print("\n-- {} n={} cmd={:.2f} vx均值={:.3f} Δx={:+.3f}m contact_number估raw={:.2f}".format(
            name, n, sum(cmdx) / n, sum(vx) / n, dpx, tot / cnt))
        print("   脚z基线 L/R {:+.3f}/{:+.3f}m | 全段峰值离地 {:+.1f}/{:+.1f}cm | p95离地 {:+.1f}/{:+.1f}cm".format(
            bl, br, max(lift_l) * 100, max(lift_r) * 100, pct(lift_l, .95) * 100, pct(lift_r, .95) * 100))
        if ls:
            print("   左摆窗{}个: 抬脚峰值 max/中位 {:.1f}/{:.1f}cm | 窗内腾空 max/均值 {}/{}ms | 0.03-0.06带占比 {:.0f}%".format(
                len(lw), ls[0] * 100, ls[1] * 100, ls[2] * 1000, ls[3] * 1000, ls[4]))
        if rs:
            print("   右摆窗{}个: 抬脚峰值 max/中位 {:.1f}/{:.1f}cm | 窗内腾空 max/均值 {}/{}ms | 0.03-0.06带占比 {:.0f}%".format(
                len(rw), rs[0] * 100, rs[1] * 100, rs[2] * 1000, rs[3] * 1000, rs[4]))
        for jn in ['hip_pitch', 'knee_pitch', 'ankle_pitch']:
            line = "   "
            for sd in ['left', 'right']:
                p = pct(col(seg, idx, 'pos_{}_{}_joint'.format(sd, jn)), .95) - pct(col(seg, idx, 'pos_{}_{}_joint'.format(sd, jn)), .05)
                d = pct(col(seg, idx, 'pos_des_raw_{}_{}_joint'.format(sd, jn)), .95) - pct(col(seg, idx, 'pos_des_raw_{}_{}_joint'.format(sd, jn)), .05)
                line += "{} {} pos幅{:.2f}/des幅{:.2f}(比{:.2f}) ".format(sd[0].upper(), jn[:4], p, d, p / d if d > 1e-6 else 0)
            print(line)

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
analyze(os.path.join(ROOT, 'czy/data/exp0.3/isaac_diag.csv'), 'exp0.3 压幅度+mocap ref2.4（原地踏步指控）')
analyze(os.path.join(ROOT, 'czy/data/exp0.2/isaac_diag.csv'), 'exp0.2 mocap查表（会走 bang-bang）')

try:
    import torch
    lib = torch.load(os.path.join(ROOT, 'resources/motions/processed/ref_lib.pt'), map_location='cpu')
    for segname in ['walk_norm', 'walk_slow']:
        dof = list(lib[segname]['dof_names'])
        q = lib[segname]['dof_pos']
        print("=" * 86)
        print("[ref_lib {}] 关节角幅度 p95-p5 (rad)".format(segname))
        for jn in ['hip_pitch', 'knee_pitch', 'ankle_pitch']:
            for sd in ['left', 'right']:
                i = dof.index('{}_{}_joint'.format(sd, jn))
                v = q[:, i]
                print("   {} {:11s}: {:.3f}".format(sd, jn,
                      torch.quantile(v.float(), .95).item() - torch.quantile(v.float(), .05).item()))
except Exception as e:
    print("(ref_lib.pt 跳过: {})".format(e))
