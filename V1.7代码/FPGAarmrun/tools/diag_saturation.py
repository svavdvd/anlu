import math

L1, L2, L3, H0 = 104.0, 147.0, 185.0, 113.0
TOOL, ELBOW_SIGN = -90.0, 1.0
DEG = math.pi / 180.0
A, B, C = 0.0, 0.0, 5.0      # 当前肩
D, E = 0.0, 5.0               # 当前肘
BASE0, WRIST0 = 0.0, -10.0    # 当前底座/腕

data = [
    (180, 0, 30, 143, 0, 4),
    (140, 0, 30, 132, 0, 23),
    (150, 80, 30, 115, 52, 3),
    (150, -80, 30, 114, -60, 3),
    (120, 120, 30, 90, 90, 8),
    (120, -120, 30, 90, -95, 10),
    (120, 0, 80, 134, 0, 55),
    (160, 0, 80, 173, 0, 55),
]

def clamp(x):
    return 0.0 if x < 0.0 else (180.0 if x > 180.0 else x)

def ik(x, y, z):
    r = math.hypot(x, y)
    th1 = math.atan2(y, x) if r > 1e-3 else 0.0
    th4 = TOOL * DEG
    r_w = r - L3 * math.cos(th4)
    z_w = z - L3 * math.sin(th4)
    px, py = r_w, z_w - H0
    D = min(max(math.hypot(px, py), abs(L1 - L2)), L1 + L2)
    phi = math.atan2(py, px)
    delta = math.acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D))
    th2 = phi + ELBOW_SIGN * delta
    th3 = math.atan2(py - L1 * math.sin(th2), px - L1 * math.cos(th2))
    return th1, th2, th3, th4

def fk(t1, t2, t3, t4):
    r = L1 * math.cos(t2) + L2 * math.cos(t3) + L3 * math.cos(t4)
    return (r * math.cos(t1), r * math.sin(t1),
            H0 + L1 * math.sin(t2) + L2 * math.sin(t3) + L3 * math.sin(t4))

print("各关节物理角(命令值, 加当前偏移后) 及是否限幅:")
print(f"{'目标':>16} | {'底座':>6} {'肩':>6} {'肘':>6} {'腕':>6} | 限幅标记")
for xt, yt, zt, xm, ym, zm in data:
    t1c, t2c, t3c, t4c = ik(xt, yt, zt)
    pb = t1c / DEG + 90 + BASE0
    ps = t2c / DEG + (A * math.cos(t2c) + B * math.cos(t3c) + C)
    pe = (t3c - t2c) / DEG + 90 + (D * math.cos(t3c) + E)
    pw = 90 - (t4c - t3c) / DEG + WRIST0
    flags = []
    for name, v in [('底座', pb), ('肩', ps), ('肘', pe), ('腕', pw)]:
        if v < 0 or v > 180:
            flags.append(f"{name}{v:+.1f}")
    print(f"({xt:4},{yt:4},{zt:3}) | {pb:6.1f} {ps:6.1f} {pe:6.1f} {pw:6.1f} | {' '.join(flags) if flags else 'OK'}")

print()
print("限幅模型预测 vs 实测:")
print(f"{'目标':>16} | {'预测':>16} | {'实测':>16} | 误差")
tot = 0.0
for xt, yt, zt, xm, ym, zm in data:
    t1c, t2c, t3c, t4c = ik(xt, yt, zt)
    pb = clamp(t1c / DEG + 90 + BASE0)
    ps = clamp(t2c / DEG + (A * math.cos(t2c) + B * math.cos(t3c) + C))
    pe = clamp((t3c - t2c) / DEG + 90 + (D * math.cos(t3c) + E))
    pw = clamp(90 - (t4c - t3c) / DEG + WRIST0)
    t1a = (pb - 90) * DEG
    t2a = ps * DEG
    t3a = t2a + (pe - 90) * DEG
    t4a = t3a + (90 - pw) * DEG
    xp, yp, zp = fk(t1a, t2a, t3a, t4a)
    e = math.hypot(xp - xm, yp - ym, zp - zm)
    tot += e
    print(f"({xt:4},{yt:4},{zt:3}) | ({xp:6.1f},{yp:6.1f},{zp:5.1f}) | ({xm:4},{ym:4},{zm:3}) | {e:5.1f}mm")
print(f"平均误差 = {tot/len(data):.1f} mm")
