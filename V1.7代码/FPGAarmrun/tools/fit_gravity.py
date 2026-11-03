import math

L1, L2, L3, H0 = 104.0, 147.0, 185.0, 113.0
TOOL = -90.0
ELBOW_SIGN = 1.0
DEG = math.pi / 180.0

# 当前代码里已填的值
CUR = {'A': 0.0, 'B': 0.0, 'C': 5.0, 'D': 0.0, 'E': 5.0,
       'BASE': 0.0, 'WRIST': -10.0}

# 每行: (目标x, 目标y, 目标z, 实测x, 实测y, 实测z)
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

def fk(t1, t2, t3, t4):
    r = L1 * math.cos(t2) + L2 * math.cos(t3) + L3 * math.cos(t4)
    return (r * math.cos(t1), r * math.sin(t1),
            H0 + L1 * math.sin(t2) + L2 * math.sin(t3) + L3 * math.sin(t4))

def predict(p):
    A, B, C, D, E, BASE, WRIST = p
    out = []
    for xt, yt, zt, _xm, _ym, _zm in data:
        t1c, t2c, t3c, t4c = ik(xt, yt, zt)
        c2, c3 = math.cos(t2c), math.cos(t3c)
        off_sh_cur = CUR['A'] * c2 + CUR['B'] * c3 + CUR['C']
        off_el_cur = CUR['D'] * c3 + CUR['E']
        sag_sh = A * c2 + B * c3 + C
        sag_el = D * c3 + E
        t1a = t1c + CUR['BASE'] - BASE
        t2a = t2c + off_sh_cur - sag_sh
        t3a = t3c + (off_sh_cur - sag_sh) + (off_el_cur - sag_el)
        t4a = t4c + (off_sh_cur - sag_sh) + (off_el_cur - sag_el) - CUR['WRIST'] + WRIST
        out.extend(fk(t1a, t2a, t3a, t4a))
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

def gauss_newton(p0, fixed=None):
    """fixed: dict index->value 表示固定不拟合的参数"""
    free = [i for i in range(7) if i not in (fixed or {})]
    p = list(p0)
    lam = 1e-3
    for _ in range(300):
        r = residual(p)
        n, m = len(r), len(free)
        J = [[0.0] * m for _ in range(n)]
        for jj, j in enumerate(free):
            h = 1e-4 * max(1.0, abs(p[j]))
            pp = p[:]
            pp[j] += h
            rp = residual(pp)
            for i in range(n):
                J[i][jj] = (rp[i] - r[i]) / h
        JTJ = [[0.0] * m for _ in range(m)]
        JTr = [0.0] * m
        for i in range(n):
            for a in range(m):
                JTr[a] += J[i][a] * r[i]
                for b in range(m):
                    JTJ[a][b] += J[i][a] * J[i][b]
        for a in range(m):
            JTJ[a][a] += lam
        dp_free = solve_linear(JTJ, JTr)
        p2 = p[:]
        for jj, j in enumerate(free):
            p2[j] += dp_free[jj]
        r2 = residual(p2)
        if sum(x * x for x in r2) < sum(x * x for x in r):
            p = p2
            lam = max(lam * 0.5, 1e-9)
        else:
            lam = min(lam * 10.0, 1e6)
        if max(abs(x) for x in dp_free) < 1e-6:
            break
    return p

def rms(r):
    return math.sqrt(sum(x * x for x in r) / len(r))

def show(p, label):
    r = residual(p)
    print(f"=== {label} ===")
    print(f"GRAV_A={p[0]:+.3f}  GRAV_B={p[1]:+.3f}  GRAV_C={p[2]:+.3f}")
    print(f"GRAV_D={p[3]:+.3f}  GRAV_E={p[4]:+.3f}")
    print(f"SERVO_OFFSET_BASE={p[5]:+.3f}  SERVO_OFFSET_WRIST={p[6]:+.3f}")
    print(f"残余 RMS = {rms(r):.2f} mm")
    print("每点残差 (dx, dy, dz) mm:")
    for i, d in enumerate(data):
        dx, dy, dz = r[3*i], r[3*i+1], r[3*i+2]
        print(f"  点{i+1} 目标({d[0]},{d[1]},{d[2]}) -> ({dx:+6.1f},{dy:+6.1f},{dz:+6.1f})")
    print()

# 当前参数(拟合前)的残差
p_cur = [CUR[k] for k in ('A', 'B', 'C', 'D', 'E', 'BASE', 'WRIST')]
show(p_cur, "拟合前(当前参数)")

# 全 7 参数拟合
p7 = gauss_newton(p_cur)
show(p7, "7参数拟合 A,B,C,D,E,BASE,WRIST")

# 只拟合 5 个重力参数, BASE/WRIST 固定
p5 = gauss_newton(p_cur, fixed={5: CUR['BASE'], 6: CUR['WRIST']})
show(p5, "5参数拟合 A,B,C,D,E (BASE/WRIST固定)")
