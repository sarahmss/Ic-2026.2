"""
Simulação do DDMR com Controle Fuzzy
Reproduz a lógica do firmware Arduino e anima as trajetórias com orientação theta.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, PillowWriter
import os, warnings
warnings.filterwarnings("ignore")

# ============================================================
# PARÂMETROS DO ROBÔ (do Arduino)
# ============================================================
R  = 0.032   # raio da roda (m)
L  = 0.124   # distância entre rodas (m)
DT = 0.07    # passo de tempo (s) = 70 ms
ERRO_STOP = 0.03
MAX_STEPS = 3000

ErroAgParametros = [0.0126, 0.2452, 0.2405, 0.4736, 1.3946]

# ============================================================
# FUNÇÕES DE PERTINÊNCIA (espelham o C++ do Arduino)
# ============================================================

def gauss_mf(x, a, b):
    return np.exp(-((x - b)**2) / (2.0 * a**2))

def lins_mf(x, a, b):
    if x <= a: return 0.0
    if x >= b: return 1.0
    return (x - a) / (b - a)

def linz_mf(x, a, b):
    if x <= a: return 1.0
    if x >= b: return 0.0
    return (b - x) / (b - a)

def tri_mf(x, a, b, c):
    if x <= a or x >= c: return 0.0
    if x <= b: return (x - a) / (b - a)
    return (c - x) / (c - b)

def pi_mf_vect(x_arr, a, b, c, d):
    """Função pi vetorizada (numpy)."""
    y = np.zeros_like(x_arr)
    ab_mid = (a + b) / 2.0
    cd_mid = (c + d) / 2.0
    mask_full  = (x_arr > b) & (x_arr < c)
    mask_r1    = (x_arr > a) & (x_arr <= ab_mid)
    mask_r2    = (x_arr > ab_mid) & (x_arr <= b)
    mask_r3    = (x_arr >= c) & (x_arr <= cd_mid)
    mask_r4    = (x_arr > cd_mid) & (x_arr < d)
    y[mask_full] = 1.0
    y[mask_r1]   = 2 * ((x_arr[mask_r1] - a) / (b - a))**2
    y[mask_r2]   = 1 - 2 * ((x_arr[mask_r2] - b) / (b - a))**2
    y[mask_r3]   = 1 - 2 * ((x_arr[mask_r3] - c) / (d - c))**2
    y[mask_r4]   = 2 * ((x_arr[mask_r4] - d) / (d - c))**2
    return y

# ============================================================
# CONTROLE FUZZY (tradução fiel do FuzzyControl() do Arduino)
# ============================================================
_N = 101
_x_lin = np.linspace(-0.15,  0.15, _N)
_x_ang = np.linspace(-0.5,   0.5,  _N)

# Pré-computa as funções de saída (invariantes ao setpoint)
_y_lin = [
    pi_mf_vect(_x_lin, -0.16, -0.15, -0.10, -0.05),
    pi_mf_vect(_x_lin, -0.10, -0.05, -0.05,  0.00),
    pi_mf_vect(_x_lin, -0.05,  0.00,  0.00,  0.05),
    pi_mf_vect(_x_lin,  0.00,  0.05,  0.05,  0.10),
    pi_mf_vect(_x_lin,  0.05,  0.10,  0.15,  0.16),
]
_y_ang_z = np.array([linz_mf(xi, -0.5, 0.0) for xi in _x_ang])
_y_ang_t = np.array([tri_mf(xi, -0.5, 0.0, 0.5) for xi in _x_ang])
_y_ang_s = np.array([lins_mf(xi, 0.0, 0.5) for xi in _x_ang])

def fuzzy_control(phi, ErroL):
    p = ErroAgParametros
    PI = np.pi

    # ---- Fuzzificação ----
    ae = np.array([
        gauss_mf(phi, p[0], -PI),
        gauss_mf(phi, p[1], p[3] - PI),
        gauss_mf(phi, p[2], p[4] - PI),
        gauss_mf(phi, p[2], -p[4]),
        gauss_mf(phi, p[1], -p[3]),
        gauss_mf(phi, p[0],  0.0),
        gauss_mf(phi, p[1],  p[3]),
        gauss_mf(phi, p[2],  p[4]),
        gauss_mf(phi, p[2],  PI - p[4]),
        gauss_mf(phi, p[1],  PI - p[3]),
        gauss_mf(phi, p[0],  PI),
    ])

    # No Arduino 'l' (distância entre rodas=0.124) é usado aqui por engano,
    # fazendo dist[0]=0 e dist[1]=1 sempre. Reproduzimos esse comportamento.
    d0 = linz_mf(L, 0.01, 0.02)   # → 0.0
    d1 = lins_mf(L, 0.01, 0.02)   # → 1.0

    # ---- Inferência ----
    fls = np.array([
        min(d1, max(ae[0], ae[10])),
        min(d1, max(ae[1], ae[9])),
        min(1,  max(ae[2], ae[3], ae[7], ae[8], d0)),
        min(d1, max(ae[4], ae[6])),
        min(d1, ae[5]),
    ])
    fas = np.array([
        min(d1, max(ae[3], ae[4], ae[8], ae[9])),
        min(1,  max(ae[0], ae[10], ae[5], d0)),
        min(d1, max(ae[1], ae[2], ae[6], ae[7])),
    ])

    # ---- Agregação ----
    agg_lin = np.maximum.reduce([np.minimum(fls[i], _y_lin[i]) for i in range(5)])
    agg_ang = np.maximum.reduce([
        np.minimum(fas[0], _y_ang_z),
        np.minimum(fas[1], _y_ang_t),
        np.minimum(fas[2], _y_ang_s),
    ])

    # ---- Defuzzificação (centroide) ----
    sl = np.sum(agg_lin)
    sa = np.sum(agg_ang)
    v_lin = np.dot(_x_lin, agg_lin) / sl if sl > 1e-12 else 0.0
    v_ang = np.dot(_x_ang, agg_ang) / sa if sa > 1e-12 else 0.0

    # ---- Saída em rad/s para cada roda ----
    velD = (1.0/R) * (v_lin + v_ang * L/2.0)
    velE = (1.0/R) * (v_lin - v_ang * L/2.0)
    return velD, velE

# ============================================================
# ODOMETRIA (idêntica ao Arduino)
# ============================================================
def odometry_step(x0, y0, th0, velD, velE):
    x     = x0 + R/2 * DT * np.cos(th0) * (velD + velE)
    y     = y0 + R/2 * DT * np.sin(th0) * (velD + velE)
    theta = th0 + R/L  * DT * (velD - velE)
    return x, y, theta

# ============================================================
# SIMULAÇÃO COMPLETA PARA UMA TRAJETÓRIA
# ============================================================
def simulate(waypoints):
    x, y, theta = 0.0, float(waypoints[0, 1]), 0.0
    hist = [(x, y, theta)]

    for wp in waypoints:
        ox, oy = float(wp[0]), float(wp[1])
        for _ in range(MAX_STEPS):
            ex, ey = ox - x, oy - y
            ErroL  = np.hypot(ex, ey)
            if ErroL <= ERRO_STOP:
                break
            phi      = np.arctan2(ey, ex) - theta
            phi_norm = 2.0 * np.arctan(np.tan(phi / 2.0))
            velD, velE = fuzzy_control(phi_norm, ErroL)
            x, y, theta = odometry_step(x, y, theta, velD, velE)
            hist.append((x, y, theta))

    return np.array(hist)

# ============================================================
# TRAJETÓRIAS
# ============================================================
trajetorias = {
    "Reta": np.array([[0.5,0],[1,0]]),

    "Zigzag Reto": np.array([
        [0.25,0],[0.5,0],[0.75,0],[1,0],
        [0.75,0.175],[0.5,0.35],[0.25,0.525],[0,0.7],
        [0.25,0.7],[0.5,0.7],[0.75,0.7],[1,0.7],
        [0.75,0.525],[0.5,0.35],[0.25,0.175],[0,0],[0.25,0],
    ]),

    "ZZX 1": np.array([
        [0.0,0.7],[1.0,0.7],[0.8889,0.5829],[0.7778,0.478],
        [0.6667,0.3842],[0.5556,0.3002],[0.4445,0.2251],[0.3333,0.1579],
        [0.2222,0.0978],[0.1111,0.0439],[0.0,-0.0042],[1.0,-0.0042],
        [0.8889,0.0439],[0.7778,0.0978],[0.6667,0.1579],[0.5556,0.2251],
        [0.4445,0.3002],[0.3333,0.3842],[0.2222,0.478],[0.1111,0.5829],
        [0.0,0.7],
    ]),

    "ZZY 1": np.array([
        [0.0,0.7],[0.0,-0.3],[0.1171,-0.1889],[0.222,-0.0778],
        [0.3158,0.0333],[0.3998,0.1444],[0.4749,0.2555],[0.5421,0.3667],
        [0.6022,0.4778],[0.6561,0.5889],[0.7042,0.7],
        [0.7042,-0.3],[0.6561,-0.1889],[0.6022,-0.0778],[0.5421,0.0333],
        [0.4749,0.1444],[0.3998,0.2555],[0.3158,0.3667],[0.222,0.4778],
        [0.1171,0.5889],[0.0,0.7],
    ]),

    "ZZX 1 INV": np.array([
        [0.0,0.0],[1.05,0.0],[0.9333,0.1052],[0.8167,0.1993],
        [0.7,0.2835],[0.5833,0.3588],[0.4667,0.4262],[0.35,0.4866],
        [0.2333,0.5406],[0.1167,0.5889],[0.0,0.6321],[1.05,0.6321],
        [0.9333,0.5889],[0.8167,0.5406],[0.7,0.4866],[0.5833,0.4262],
        [0.4667,0.3588],[0.35,0.2835],[0.2333,0.1993],[0.1167,0.1052],
        [0.0,0.0],[0.0,0.0],
    ]),

    "ZZX 2": np.array([
        [0.7324,0.0],[0.6103,-0.502],[0.4883,-0.9199],
        [0.3662,-1.2679],[0.2441,-1.5578],[0.1221,-1.7991],
        [0.0,-2.0],[0.7324,-2.0],[0.6103,-1.7991],
        [0.4883,-1.5578],[0.3662,-1.2679],[0.2441,-0.9199],
        [0.1221,-0.502],[0.0,0.0],
    ]),

    "ZZY 2": np.array([
        [0.0,0.7324],[-0.502,0.6103],[-0.9199,0.4883],[-1.2679,0.3662],
        [-1.5578,0.2441],[-1.7991,0.1221],[-2.0,0.0],[-2.0,0.7324],
        [-1.7991,0.6103],[-1.5578,0.4883],[-1.2679,0.3662],[-0.9199,0.2441],
        [-0.502,0.1221],[0.0,0.0],[0.0,0.7324],
    ]),

    "ZZX 2 INV": np.array([
        [0.7324,0.0],[0.6103,0.502],[0.4883,0.9199],
        [0.3662,1.2679],[0.2441,1.5578],[0.1221,1.7991],
        [0.0,2.0],[0.7324,2.0],[0.6103,1.7991],
        [0.4883,1.5578],[0.3662,1.2679],[0.2441,0.9199],
        [0.1221,0.502],[0.0,0.0],[0.7324,0.0],
    ]),

    "Losango 1": np.array([
        [0.0,0.0],[0.25,0.5],[0.5,1],[0.75,0.5],
        [1,0],[0.75,-0.5],[0.5,-1],[0.25,-0.5],[0,0],
    ]),

    "Losango 2": np.array([
        [0.5,0.25],[1,0.5],[1.5,0.25],[2,0],
        [1.5,-0.25],[1,-0.5],[0.5,-0.25],[0,0],[0.5,0.25],
    ]),

    "Infinito": np.array([
        [0.1818,0.5406],[0.3636,0.9096],[0.5455,0.9898],
        [0.7273,0.7557],[0.9091,0.2817],[1.0991,-0.2817],
        [1.2727,-0.7557],[1.4545,-0.9898],[1.6364,-0.9096],
        [1.8182,-0.5406],[2,0],[1.8182,0.5406],
        [1.6364,0.9096],[1.4545,0.9898],[1.2727,0.7557],
        [1.0909,0.2817],[0.9091,-0.2817],[0.7273,-0.7557],
        [0.5455,-0.9898],[0.3636,-0.9096],[0.1818,-0.5406],
        [0,0],[0.1818,0.5406],
    ]),
}

# Trajetória circular
_theta_c = np.linspace(0, 2*np.pi, 20)
trajetorias["Circular"] = np.column_stack((1.0 + 1.0*np.cos(_theta_c),
                                           0.0 + 1.0*np.sin(_theta_c)))

# ============================================================
# GERAÇÃO DOS GIFs
# ============================================================
ROBOT_RADIUS = 0.04  # tamanho visual do robô (m)
ARROW_LEN    = 0.10  # comprimento do eixo vermelho

def make_gif(nome, waypoints, hist, out_path):
    xs, ys, ths = hist[:,0], hist[:,1], hist[:,2]
    wps_x, wps_y = waypoints[:,0], waypoints[:,1]

    # Limites do plot com margem
    margin = 0.25
    all_x = np.concatenate([xs, wps_x])
    all_y = np.concatenate([ys, wps_y])
    xlim = (all_x.min() - margin, all_x.max() + margin)
    ylim = (all_y.min() - margin, all_y.max() + margin)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(*xlim); ax.set_ylim(*ylim)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_title(f"DDMR – {nome}", fontsize=13, fontweight='bold')
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

    # Waypoints de referência
    ax.plot(wps_x, wps_y, 'k--', lw=1, alpha=0.45, zorder=1)
    ax.scatter(wps_x, wps_y, s=30, c='gray', zorder=2)
    ax.scatter(wps_x[0],  wps_y[0],  s=80, c='green', marker='s',
               label='Início ref', zorder=3)
    ax.scatter(wps_x[-1], wps_y[-1], s=80, c='purple', marker='x',
               lw=2, label='Fim ref', zorder=3)

    # Trilha do robô (cresce)
    trail,  = ax.plot([], [], 'b-', lw=1.2, alpha=0.7, zorder=4)

    # Corpo do robô
    body = plt.Circle((xs[0], ys[0]), ROBOT_RADIUS,
                       fc='steelblue', ec='navy', lw=1.5, zorder=5)
    ax.add_patch(body)

    # Eixo de orientação (vermelho)
    orient, = ax.plot([], [], 'r-', lw=2.5, zorder=6)

    # Seta de orientação
    arrow = ax.annotate("", xy=(xs[0], ys[0]),
                        xytext=(xs[0], ys[0]),
                        arrowprops=dict(arrowstyle="-|>", color='red', lw=2),
                        zorder=7)

    step_txt = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                       fontsize=8, va='top', color='gray')
    ax.legend(loc='lower right', fontsize=8)

    # Decimação para GIF razoável
    SKIP = max(1, len(hist) // 300)
    frames = list(range(0, len(hist), SKIP))
    if frames[-1] != len(hist)-1:
        frames.append(len(hist)-1)

    def init():
        trail.set_data([], [])
        orient.set_data([], [])
        return trail, body, orient

    def update(fi):
        i = frames[fi]
        xi, yi, thi = xs[i], ys[i], ths[i]

        trail.set_data(xs[:i+1], ys[:i+1])
        body.set_center((xi, yi))

        # Eixo vermelho: linha do centro ao nariz
        x_tip = xi + ARROW_LEN * np.cos(thi)
        y_tip = yi + ARROW_LEN * np.sin(thi)
        orient.set_data([xi, x_tip], [yi, y_tip])

        # Atualiza seta
        arrow.set_position((xi, yi))
        arrow.xy = (x_tip, y_tip)

        step_txt.set_text(f"passo {i}/{len(hist)-1}  θ={np.degrees(thi):.1f}°")
        return trail, body, orient, step_txt

    ani = FuncAnimation(fig, update, frames=len(frames),
                        init_func=init, blit=True, interval=40)
    writer = PillowWriter(fps=25)
    ani.save(out_path, writer=writer)
    plt.close(fig)
    print(f"  → Salvo: {out_path}")

# ============================================================
# MAIN
# ============================================================
OUT_DIR = "./Trajectory"
os.makedirs(OUT_DIR, exist_ok=True)

saved = []
total = len(trajetorias)

for idx, (nome, wps) in enumerate(trajetorias.items(), 1):
    print(f"[{idx}/{total}] Simulando: {nome} ({len(wps)} waypoints)...")
    hist = simulate(wps)
    print(f"         {len(hist)} passos simulados")
    slug = nome.lower().replace(" ", "_").replace("/", "")
    out = os.path.join(OUT_DIR, f"ddmr_{slug}.gif")
    make_gif(nome, wps, hist, out)
    saved.append(out)

print("\nTodos os GIFs gerados:")
for p in saved:
    print(" ", p)