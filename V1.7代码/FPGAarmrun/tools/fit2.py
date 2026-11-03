import math

DEG = math.pi / 180.0
TOOL = -90.0
ELBOW_SIGN = 1.0

# ---- 数据: (目标x, 目标y, 目标z, 实测x, 实测y, 实测z) ----
# 点7 的实测 y 按对称性修正为 -95(原 95 疑为符号笔误)
data = [
    (180, 0, 30, 143, 0, 4),
    (140, 0, 30, 132, 0, 23),
    (100, 0, 30, 140, 0, 70),   # 原被当离群点
    (150, 80, 30, 115, 52, 3),
    (150, -80, 30, 114, -60, 3),
    (120, 120, 30, 90, 90, 8),
    (120, -120, 30, 90, -95, 10),
    (120, 0, 80, 134, 0, 55),
    (160, 0, 80, 173, 0, 55),
]

def clamp(x):
    return 0.0 if x < 0.0 else (180.0 if x > 180.0 else x)

def make_ik(L1, L2, L3, H0):
    def ik(x, y, z):
        r = math.hypot(x, y)
        th1 = math.atan2(y, x) if r > 1e-3 else 0.0
        th4 = TOOL * DEG
        r_w = r - L3 * math.cos(th4)
        z_w = z - L3 * math.sin(th4)
        px, py = r_w, z_w - H0
        D_raw = math.hypot(px, py)
        D = min(max(D_raw, abs(L1 - L2)), L1 + L2)
        phi = math.atan2(py, px)
        delta = math.acos((L1 * L1 + D * D - L2 * L2) / (2.0 * L1 * D))
        th2 = phi + ELBOW_SIGN * delta
        th3 = math.atan2(py - L1 * math.sin(th2), px - L1 * math.cos(th2))
        return th1, th2, th3, th4
    return ik

def make_fk(L1, L2, L3, H0):
    def fk(t1, t2, t3, t4):
        r = L1 * math.cos(t2) + L2 * math.cos(t3) + L3 * math.cos(t4)
        return (r * math.cos(t1), r * math.sin(t1),
                H0 + L1 * math.sin(t2) + L2 * math.sin(t3) + L3 * math.sin(t4))
    return fk

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

def rms(r):
    return math.sqrt(sum(x * x for x in r) / len(r))

def build(L1, L2, L3, H0):
    ik = make_ik(L1, L2, L3, H0)
    fk = make_fk(L1, L2, L3, H0)

    def predict(p):
        A, B, C, D, E, BASE, WRIST = p
        out = []
        for xt, yt, zt, _xm, _ym, _zm in data:
            t1c, t2c, t3c, t4c = ik(xt, yt, zt)
            c2, c3 = math.cos(t2c), math.cos(t3c)
            off_sh = A * c2 + B * c3 + C
            off_el = D * c3 + E
            # 舵机物理角(加补偿后), 限幅
            pb = clamp(t1c / DEG + 90 + BASE)
            ps = clamp(t2c / DEG + off_sh)
            pe = clamp((t3c - t2c) / DEG + 90 + off_el)
            pw = clamp(90 - (t4c - t3c) / DEG + WRIST)
            # 实际绝对角
            t1a = (pb - 90) * DEG
            t2a = ps * DEG
            t3a = t2a + (pe - 90) * DEG
            t4a = t3a + (90 - pw) * DEG
            out.extend(fk(t1a, t2a, t3a, t4a))
        return out

    def residual(p):
        meas = [v for d in data for v in d[3:6]]
        pred = predict(p)
        return [pred[i] - meas[i] for i in range(len(meas))]

    return predict, residual

def gauss_newton(residual, p0, lo=None, hi=None):
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
        if lo is not None:
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

def show(residual, p, label):
    r = residual(p)
    print(f"=== {label} ===")
    print(f"  GRAV_A={p[0]:+.3f}  GRAV_B={p[1]:+.3f}  GRAV_C={p[2]:+.3f}")
    print(f"  GRAV_D={p[3]:+.3f}  GRAV_E={p[4]:+.3f}")
    print(f"  BASE={p[5]:+.3f}  WRIST={p[6]:+.3f}")
    print(f"  RMS = {rms(r):.2f} mm")
    print("  每点残差(dx,dy,dz) mm:")
    for i, d in enumerate(data):
        dx, dy, dz = r[3*i], r[3*i+1], r[3*i+2]
        print(f"    点{i+1} 目标({d[0]:4},{d[1]:4},{d[2]:3}) -> ({dx:+6.1f},{dy:+6.1f},{dz:+6.1f})")
    print()

# 当前代码里的几何
for (L1, L2, L3, H0, tag) in [(104, 147, 180, 113, "L3=180(当前)"),
                               (104, 147, 185, 113, "L3=185(旧)")]:
    print("=" * 70)
    print(f"几何: L1={L1} L2={L2} L3={L3} H0={H0}  [{tag}]")
    print("=" * 70)
    predict, residual = build(L1, L2, L3, H0)

    # 当前固件里的系数(用户测量时用的)
    cur = [4.981, 2.849, -0.062, -2.924, 0.237, 0.0, 0.0]
    show(residual, cur, "当前系数(测量时)的残差")

    # 全 7 参数拟合
    fit = gauss_newton(residual, cur)
    show(residual, fit, "7参数拟合结果")
