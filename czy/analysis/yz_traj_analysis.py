# -*- coding: utf-8 -*-
"""分析 czy/data/yz 新参考轨迹：速度/周期/幅度/对称性，并给出接入 ref_lib 管线的评估"""
import csv, math, os

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'yz',
                    '07_03_walk_yup_recwalk_base_lowerbody_smooth_p8_120_180_groundfit_minima_safe.csv')

with open(PATH, newline='') as f:
    rdr = csv.reader(f)
    header = next(rdr)
    rows = [[float(x) for x in r] for r in rdr if r]

n = len(rows)
ts = [r[0] for r in rows]
dt = (ts[-1] - ts[0]) / (n - 1)
fps = 1.0 / dt
col = {name: i for i, name in enumerate(header)}

print("=== 基础 ===")
print("帧数={} 时长={:.2f}s 帧率={:.1f}Hz dt={:.4f}s".format(n, ts[-1] - ts[0], fps, dt))

# root 平移速度
vx, vy = [], []
vz_list = []
for i in range(1, n):
    dx = rows[i][col['root_pos_x']] - rows[i-1][col['root_pos_x']]
    dy = rows[i][col['root_pos_y']] - rows[i-1][col['root_pos_y']]
    dz = rows[i][col['root_pos_z']] - rows[i-1][col['root_pos_z']]
    v = math.hypot(dx, dy) / dt
    vx.append(dx / dt); vy.append(dy / dt)
    vz_list.append(dz / dt)
speed = [math.hypot(a, b) for a, b in zip(vx, vy)]
sp_sorted = sorted(speed)
print("\n=== root 平移 ===")
print("速度 mean={:.3f} p05={:.3f} p50={:.3f} p95={:.3f} max={:.3f} m/s".format(
    sum(speed)/len(speed), sp_sorted[int(.05*len(sp_sorted))], sp_sorted[len(sp_sorted)//2],
    sp_sorted[int(.95*len(sp_sorted))], max(speed)))
z = [r[col['root_pos_z']] for r in rows]
zs = sorted(z)
print("root_z mean={:.3f} p01={:.3f} p99={:.3f} 波动={:.3f}m".format(
    sum(z)/len(z), zs[int(.01*n)], zs[int(.99*n)], zs[int(.99*n)]-zs[int(.01*n)]))
# 净位移与方向
total = math.hypot(rows[-1][col['root_pos_x']]-rows[0][col['root_pos_x']],
                   rows[-1][col['root_pos_y']]-rows[0][col['root_pos_y']])
print("净位移={:.2f}m（起始({:.2f},{:.2f})→结束({:.2f},{:.2f})）".format(
    total, rows[0][col['root_pos_x']], rows[0][col['root_pos_y']],
    rows[-1][col['root_pos_x']], rows[-1][col['root_pos_y']]))
# 直线度：逐帧速度方向一致性
import statistics
hd = [math.atan2(b, a) for a, b in zip(vx, vy)]
mean_hd = math.atan2(sum(math.sin(h) for h in hd), sum(math.cos(h) for h in hd))
dev = [abs((h - mean_hd + math.pi) % (2*math.pi) - math.pi) for h in hd]
print("航向偏差 mean={:.1f}° max={:.1f}°（越小越直）".format(
    57.3*sum(dev)/len(dev), 57.3*max(dev)))

def pct(v, q):
    s = sorted(v); k = (len(s)-1)*q; f = int(k); c = min(f+1, len(s)-1)
    return s[f] + (s[c]-s[f])*(k-f)

print("\n=== 关节幅度 p95-p5 (rad) ===")
joints = [c for c in header if c.endswith('_joint')]
for j in joints:
    v = [r[col[j]] for r in rows]
    amp = pct(v, .95) - pct(v, .05)
    print("  {:34s} {:.3f}".format(j, amp))

# 步态周期：左髋 pitch 自相关
sig = [r[col['left_hip_pitch_joint']] for r in rows]
m = sum(sig)/len(sig)
sig = [x - m for x in sig]
best, bestv = None, -2
maxlag = int(2.0 * fps)  # 搜索 0.2~2s
for lag in range(int(0.2*fps), maxlag):
    num = sum(sig[i]*sig[i+lag] for i in range(n-lag))
    den = sum(s*s for s in sig)
    c = num/den if den > 0 else 0
    if c > bestv:
        bestv, best = c, lag
print("\n=== 步态周期（左髋pitch自相关） ===")
print("周期={:.3f}s（{}帧） 相关={:.3f} 频率={:.2f}Hz".format(best*dt, best, bestv, 1.0/(best*dt)))

# 半周期对称检查：右髋与左髋相位差
sigr = [r[col['right_hip_pitch_joint']] for r in rows]
mr = sum(sigr)/len(sigr)
sigr = [x - mr for x in sigr]
for lag_frac, label in [(0.5, "半周期"), (0.45, "0.45周期"), (0.55, "0.55周期")]:
    lag = int(best*lag_frac)
    num = sum(sigr[i]*sig[i+lag] for i in range(n-lag))
    den = math.sqrt(sum(s*s for s in sigr)*sum(s*s for s in sig))
    print("右vs左 @{}: corr={:.3f}（负值≈左右交替正常）".format(label, num/den if den > 0 else 0))

# 平滑度：二阶差分 RMS（与 ref_lib 对比需同帧率，仅绝对参考）
print("\n=== 平滑度（二阶差分 RMS，越小越平滑） ===")
for j in ['left_hip_pitch_joint', 'left_knee_pitch_joint', 'left_shoulder_pitch_joint']:
    v = [r[col[j]] for r in rows]
    d2 = [(v[i+1]-2*v[i]+v[i-1]) for i in range(1, n-1)]
    rms = math.sqrt(sum(x*x for x in d2)/len(d2))
    print("  {:34s} {:.5f}".format(j, rms))

print("\n=== 接入 ref_lib 管线的差异清单 ===")
print("1. 帧率 30Hz vs ref_lib 50Hz → prep 需重采样（现有逻辑 120→50 改 30→50）")
print("2. 关节序为 gmr 序（腰/左臂/右臂/颈头/左腿/右腿）→ 复用现有 FLIP+重排规则")
print("   注意新增 neck_motor_base_pitch/head_face_bracket_pitch 2 关节（ref_lib 29 关节里没有头颈，需裁剪或扩展）")
print("3. root_quat 为 xyzw；ref_lib 存 wxyz（root_rot_wxyz）→ 转换")
print("4. 有 root_pos 平移 → 真实行走段（对比跑步机原地段，速度参考更真实）")
