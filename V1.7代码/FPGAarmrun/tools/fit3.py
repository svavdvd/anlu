import math

DEG = math.pi / 180.0
RAD = 180.0 / math.pi
TOOL = -90.0
ELBOW_SIGN = 1.0
L1, L2, L3, H0 = 104.0, 147.0, 180.0, 113.0

# 目标(x,y,z) -> 实测尖端(x,y,z)
data = [
    (180, 0, 0,   164, 0, 13),
    (210, 0, 0,   202, 0, 29),
    (203, 0, 0,   227, 0, 45),
    (180, 0, 50,  182, 0, 65),
    (200, 0, 50,  217, 0, 72),
    (215, 0, 0,   252, 0, 90),
]

def clamp(x):
    return 0.0 if x < 0.0 else (180.0 if x > 180.0 else x)

def ik(x, y, z):
    r = math.hypot(x, y)
    t1 = math.atan2(y, x) if r > 1e-3 else 0.0
    t4 = TOOL * DEG
    rw = r - L3 * math.cos(t4)
    zw = z - L3 * math.sin(t4)
    px, py = rw, zw - H0
    Draw = math.hypot(px, py)
    Dmin = math.sqrt(L1 * L1 + L2 * L2)
    D = min(max(Draw, Dmin), L1 + L2)
    phi = math.atan2(py, px)
    delta = math.acos((L1 * L1 + D * D - L2 * L2) / (2.0 * L1 * D))
    t2 = phi + ELBOW_SIGN * delta
    t3 = math.atan2(py - L1 * math.sin(t2), px - L1 * math.cos(t2))
    return t1, t2, t3, t4

def fk(t1, t2, t3, t4):
    r = L1 * math.cos(t2) + L2 * math.cos(t3) + L3 * math.cos(t4)
    return (r * math.cos(t1), r * math.sin(t1),
            H0 + L1 * math.sin(t2) + L2 * math.sin(t3) + L3 * math.sin(t4))

def predict(p):
    A, B, C, D, E = p
    out = []
    for xt, yt, zt, _xm, _ym, _zm in data:
        t1c, t2c, t3c, t4c = ik(xt, yt, zt)
        c2, c3 = math.cos(t2c), math.cos(t3c)
        off_sh = A * c2 + B * c3 + C
        off_el = D * c3 + E
        pb = clamp(t1c * RAD + 90 + 0.0)                 # BASE = 0
        ps = clamp(t2c * RAD + off_sh)                   # 肩
        pe = clamp((t3c - t2c) * RAD + 90 + off_el)      # 肘
        pw = clamp(90 - (t4c - t3c) * RAD + 0.0)         # 腕 WRIST=0
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

def gauss_newton(p0, lo=None, hi=None):
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

def rms(r):
    return math.sqrt(sum(x * x for x in r) / len(r))

def show(p, label):
    r = residual(p)
    print(f"=== {label} ===")
    print(f"  GRAV_A={p[0]:+.3f}  GRAV_B={p[1]:+.3f}  GRAV_C={p[2]:+.3f}")
    print(f"  GRAV_D={p[3]:+.3f}  GRAV_E={p[4]:+.3f}")
    print(f"  RMS = {rms(r):.2f} mm")
    print("  每点残差(预测-实测, dx,dy,dz) mm:")
    for i, d in enumerate(data):
        dx, dy, dz = r[3*i], r[3*i+1], r[3*i+2]
        print(f"    点{i+1} 目标({d[0]:4},{d[1]:4},{d[2]:3}) 实测({d[3]:4},{d[4]:4},{d[5]:3}) -> ({dx:+6.1f},{dy:+6.1f},{dz:+6.1f})")
    print()

# 当前系数(用户手动输入的)
cur = [10.7, 13.3, 0.0, 0.0, -1.85]
show(cur, "当前系数(10.7,13.3,0,0,-1.85)")

# 5 参数拟合
fit = gauss_newton(cur)
show(fit, "5参数拟合结果")

# 打印建议写入的值
print("建议 servo.h 写入:")
print(f"  GRAV_A {fit[0]:.2f}f  GRAV_B {fit[1]:.2f}f  GRAV_C {fit[2]:.2f}f")
print(f"  GRAV_D {fit[3]:.2f}f  GRAV_E {fit[4]:.2f}f")
