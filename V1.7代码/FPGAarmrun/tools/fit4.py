import math

DEG = math.pi / 180.0
RAD = 180.0 / math.pi
L1, L2, L3, H0 = 104.0, 147.0, 180.0, 113.0
TOOL = -90.0
ELBOW_SIGN = 1.0

# 目标(x,y,z) -> 实测尖端(x,y,z)
data = [
    (180, 0, 0,   164, 0, 13),
    (210, 0, 0,   202, 0, 29),
    (203, 0, 0,   227, 0, 45),
    (180, 0, 50,  182, 0, 65),
    (200, 0, 50,  217, 0, 72),
    (215, 0, 0,   252, 0, 90),
]

def ik(x, y, z):
    r = math.hypot(x, y)
    t1 = math.atan2(y, x) if r > 1e-3 else 0.0
    t4 = TOOL * DEG
    rw = r - L3 * math.cos(t4)
    zw = z - L3 * math.sin(t4)
    px, py = rw, zw - H0
    Draw = math.hypot(px, py)
    D = min(max(Draw, math.sqrt(L1 * L1 + L2 * L2)), L1 + L2)
    phi = math.atan2(py, px)
    delta = math.acos((L1 * L1 + D * D - L2 * L2) / (2.0 * L1 * D))
    t2 = phi + ELBOW_SIGN * delta
    t3 = math.atan2(py - L1 * math.sin(t2), px - L1 * math.cos(t2))
    return t1, t2, t3, t4

def solve23_from_wrist(rw, zw):
    px, py = rw, zw - H0
    D = math.hypot(px, py)
    Dmin = math.sqrt(L1 * L1 + L2 * L2)
    if D > L1 + L2 + 0.5 or D < Dmin - 0.5:
        return None, None, D
    phi = math.atan2(py, px)
    delta = math.acos((L1 * L1 + D * D - L2 * L2) / (2.0 * L1 * D))
    t2 = phi + ELBOW_SIGN * delta
    t3 = math.atan2(py - L1 * math.sin(t2), px - L1 * math.cos(t2))
    return t2, t3, D

def gauss(A, b):
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

# 当前固件系数
A0, B0, C0 = 10.7, 13.3, 0.0
D0, E0 = 0.0, -1.85

rows_sh = []
rows_el = []
print("每点反解 (theta2a, theta3a, S_sh, S_el):")
for i, (xt, yt, zt, xm, ym, zm) in enumerate(data):
    t1c, t2c, t3c, t4c = ik(xt, yt, zt)
    rw_a = math.hypot(xm, ym)
    zw_a = zm + L3                      # 腕在尖端正上方 L3 处
    t2a, t3a, D = solve23_from_wrist(rw_a, zw_a)
    t2c_d, t3c_d = t2c * RAD, t3c * RAD
    off_sh = A0 * math.cos(t2c) + B0 * math.cos(t3c) + C0
    off_el = D0 * math.cos(t3c) + E0
    if t2a is None:
        print(f"  点{i+1} 目标({xt},{zt}) 实测({xm},{zm}): 腕D={D:.1f} 超出可达 -> 跳过")
        continue
    t2a_d, t3a_d = t2a * RAD, t3a * RAD
    S_sh = (t2c_d + off_sh) - t2a_d
    S_el = ((t3c_d - t2c_d) + off_el) - (t3a_d - t2a_d)
    print(f"  点{i+1} 目标({xt},{zt}) 实测({xm},{zm}): "
          f"theta2a={t2a_d:6.1f} theta3a={t3a_d:6.1f} S_sh={S_sh:+6.2f} S_el={S_el:+6.2f}")
    rows_sh.append((math.cos(t2a), math.cos(t3a), S_sh))
    rows_el.append((math.cos(t3a), S_el))

print()
print(f"用于肩拟合的点数: {len(rows_sh)}")

# 肩: S_sh = A*cos2 + B*cos3 + C  (线性最小二乘)
N = len(rows_sh)
M = [[0.0]*3 for _ in range(3)]
b = [0.0]*3
for c2, c3, s in rows_sh:
    M[0][0] += c2*c2; M[0][1] += c2*c3; M[0][2] += c2
    M[1][0] += c3*c2; M[1][1] += c3*c3; M[1][2] += c3
    M[2][0] += c2;    M[2][1] += c3;    M[2][2] += 1.0
    b[0] += c2*s; b[1] += c3*s; b[2] += s
A, B, C = gauss(M, b)

# 肘: S_el = D*cos3 + E
M2 = [[0.0]*2 for _ in range(2)]
b2 = [0.0]*2
for c3, s in rows_el:
    M2[0][0] += c3*c3; M2[0][1] += c3
    M2[1][0] += c3;    M2[1][1] += 1.0
    b2[0] += c3*s; b2[1] += s
D, E = gauss(M2, b2)

print()
print("拟合结果:")
print(f"  GRAV_A={A:+.2f}  GRAV_B={B:+.2f}  GRAV_C={C:+.2f}")
print(f"  GRAV_D={D:+.2f}  GRAV_E={E:+.2f}")

# 残差: 用拟合系数预测真实下垂, 反推应落点 vs 实测
print()
print("用拟合系数预测尖端落点, 与实测对比:")
for i, (xt, yt, zt, xm, ym, zm) in enumerate(data):
    t1c, t2c, t3c, t4c = ik(xt, yt, zt)
    t2c_d, t3c_d = t2c * RAD, t3c * RAD
    off_sh_new = A * math.cos(t2c) + B * math.cos(t3c) + C
    off_el_new = D * math.cos(t3c) + E
    # 预测实际角 = 命令物理 - 真实下垂(=新系数)
    t2p = t2c_d + off_sh_new - (A * math.cos(t2c) + B * math.cos(t3c) + C)  # 理想下=命令角
    # 精确: 实际角 = 命令角 + 新off - 真实下垂; 真实下垂用新系数在实际角处, 近似用命令角
    t2p = t2c_d  # 若补偿=真实下垂, 应落在命令角
    t3p = t3c_d
    t4p = -90.0
    # 正运动学
    r = L1*math.cos(t2p*DEG) + L2*math.cos(t3p*DEG) + L3*math.cos(t4p*DEG)
    z = H0 + L1*math.sin(t2p*DEG) + L2*math.sin(t3p*DEG) + L3*math.sin(t4p*DEG)
    print(f"  点{i+1} 目标({xt},{zt}) 实测({xm},{zm}) -> 若系数准应落在({r:.0f},{z:.0f})")
