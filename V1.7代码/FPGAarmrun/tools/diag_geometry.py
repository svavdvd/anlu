import math

DEG = math.pi / 180.0
# 代码里写死的长度(用于 IK 解算命令角)
CL1, CL2, CL3, CH0 = 104.0, 147.0, 185.0, 113.0
TOOL, ELBOW_SIGN = -90.0, 1.0

# 当前偏移
A0, B0, C0 = 0.0, 0.0, 5.0
D0, E0 = 0.0, 5.0
BASE0, WRIST0 = 0.0, -10.0

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

def ik(x, y, z, L1, L2, L3, H0):
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

def fk(t1, t2, t3, t4, L1, L2, L3, H0):
    r = L1 * math.cos(t2) + L2 * math.cos(t3) + L3 * math.cos(t4)
    return (r * math.cos(t1), r * math.sin(t1),
            H0 + L1 * math.sin(t2) + L2 * math.sin(t3) + L3 * math.sin(t4))

def predict(p):
    L1, L2, L3, H0 = p
    out = []
    for xt, yt, zt, _xm, _ym, _zm in data:
        # 命令角用代码长度
        t1c, t2c, t3c, t4c = ik(xt, yt, zt, CL1, CL2, CL3, CH0)
        pb = clamp(t1c / DEG + 90 + BASE0)
        ps = clamp(t2c / DEG + (A0 * math.cos(t2c) + B0 * math.cos(t3c) + C0))
        pe = clamp((t3c - t2c) / DEG + 90 + (D0 * math.cos(t3c) + E0))
        pw = clamp(90 - (t4c - t3c) / DEG + WRIST0)
        t1a = (pb - 90) * DEG
        t2a = ps * DEG
        t3a = t2a + (pe - 90) * DEG
        t4a = t3a + (90 - pw) * DEG
        # 实际正向用拟合长度
        out.extend(fk(t1a, t2a, t3a, t4a, L1, L2, L3, H0))
    return out

def residual(p):
    meas = []
    for d in data:
        meas.extend(d[3:6])
    pred = predict(p)
    return [pred[i] - meas[i] for i in range(len(meas))]

def solve_linear(A, b):
    n = len(b)
    M = [A[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        piv = col
        for r in range(col + 1, n):
            if abs(M[r][col]) > abs(M[piv][col]):
                piv = r
        M[col], M[piv] = M[piv], M[col]
        if abs(M[col][col]) < 1e-12:
            continue
        for r in range(col + 1, n):
            f = M[r][col] / M[col][col]
            for c in range(col, n + 1):
                M[r][c] -= f * M[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        s = M[r][n]
        for c in range(r + 1, n):
            s -= M[r][c] * x[c]
        x[r] = s / M[r][r] if abs(M[r][r]) > 1e-12 else 0.0
    return x

def gauss_newton(p0, lo, hi):
    p = list(p0)
    lam = 1e-3
    for _ in range(400):
        r = residual(p)
        n, m = len(r), len(p)
        J = [[0.0] * m for _ in range(n)]
        for j in range(m):
            h = 1e-4 * max(1.0, abs(p[j]))
            pp = p[:]; pp[j] += h
            rp = residual(pp)
            for i in range(n):
                J[i][j] = (rp[i] - r[i]) / h
        JTJ = [[0.0] * m for _ in range(m)]
        JTr = [0.0] * m
        for i in range(n):
            for a in range(m):
                JTr[a] += J[i][a] * r[i]
                for b in range(m):
                    JTJ[a][b] += J[i][a] * J[i][b]
        for a in range(m):
            JTJ[a][a] += lam
        dp = solve_linear(JTJ, JTr)
        p2 = [p[a] + dp[a] for a in range(m)]
        # clamp to bounds
        p2 = [max(lo[a], min(hi[a], p2[a])) for a in range(m)]
        r2 = residual(p2)
        if sum(x * x for x in r2) < sum(x * x for x in r):
            p = p2
            lam = max(lam * 0.5, 1e-9)
        else:
            lam = min(lam * 10.0, 1e6)
        if max(abs(x) for x in dp) < 1e-6:
            break
    return p

def rms(r):
    return math.sqrt(sum(x * x for x in r) / len(r))

# 假设无误差(长度=代码值): 应该接近目标
p = [CL1, CL2, CL3, CH0]
print(f"假设长度=代码值(应=目标): RMS={rms(residual(p)):.1f} mm")

# 拟合几何
pfit = gauss_newton([CL1, CL2, CL3, CH0],
                    [60, 90, 120, 60], [160, 200, 260, 160])
print(f"\n拟合后几何: L1={pfit[0]:.1f}  L2={pfit[1]:.1f}  L3={pfit[2]:.1f}  H0={pfit[3]:.1f}")
print(f"  (代码值: L1=104  L2=147  L3=185  H0=113)")
print(f"  RMS = {rms(residual(pfit)):.1f} mm")
r = residual(pfit)
for i, d in enumerate(data):
    print(f"  点{i+1} ({d[0]},{d[1]},{d[2]}) 残差 ({r[3*i]:+6.1f},{r[3*i+1]:+6.1f},{r[3*i+2]:+6.1f})")
