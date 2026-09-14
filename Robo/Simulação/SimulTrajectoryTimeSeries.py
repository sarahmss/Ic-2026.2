"""
Simulação do DDMR com Controle Fuzzy
Agora armazenando:
- PWM direito/esquerdo
- velocidades das rodas
- estados x, y, theta

O GIF mostra:
ESQUERDA:
    y(x) com orientação do robô

DIREITA:
    séries temporais:
    PWMd
    PWMe
    wd
    we
    x
    y
    theta
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.gridspec import GridSpec
import os
import warnings

warnings.filterwarnings("ignore")

# ============================================================
# PARÂMETROS
# ============================================================

R  = 0.032
L  = 0.124
DT = 0.07

ERRO_STOP = 0.03
MAX_STEPS = 3000

ErroAgParametros = [0.0126, 0.2452, 0.2405, 0.4736, 1.3946]

ROBOT_RADIUS = 0.04
ARROW_LEN    = 0.10

# ============================================================
# FUNÇÕES FUZZY
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
    y = np.zeros_like(x_arr)

    ab_mid = (a + b) / 2.0
    cd_mid = (c + d) / 2.0

    mask_full = (x_arr > b) & (x_arr < c)
    mask_r1   = (x_arr > a) & (x_arr <= ab_mid)
    mask_r2   = (x_arr > ab_mid) & (x_arr <= b)
    mask_r3   = (x_arr >= c) & (x_arr <= cd_mid)
    mask_r4   = (x_arr > cd_mid) & (x_arr < d)

    y[mask_full] = 1.0
    y[mask_r1] = 2*((x_arr[mask_r1]-a)/(b-a))**2
    y[mask_r2] = 1 - 2*((x_arr[mask_r2]-b)/(b-a))**2
    y[mask_r3] = 1 - 2*((x_arr[mask_r3]-c)/(d-c))**2
    y[mask_r4] = 2*((x_arr[mask_r4]-d)/(d-c))**2

    return y

# ============================================================
# FUZZY
# ============================================================

_N = 101

_x_lin = np.linspace(-0.15, 0.15, _N)
_x_ang = np.linspace(-0.5, 0.5, _N)

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

    ae = np.array([
        gauss_mf(phi, p[0], -PI),
        gauss_mf(phi, p[1], p[3]-PI),
        gauss_mf(phi, p[2], p[4]-PI),
        gauss_mf(phi, p[2], -p[4]),
        gauss_mf(phi, p[1], -p[3]),
        gauss_mf(phi, p[0], 0.0),
        gauss_mf(phi, p[1], p[3]),
        gauss_mf(phi, p[2], p[4]),
        gauss_mf(phi, p[2], PI-p[4]),
        gauss_mf(phi, p[1], PI-p[3]),
        gauss_mf(phi, p[0], PI),
    ])

    d0 = linz_mf(L, 0.01, 0.02)
    d1 = lins_mf(L, 0.01, 0.02)

    fls = np.array([
        min(d1, max(ae[0], ae[10])),
        min(d1, max(ae[1], ae[9])),
        min(1, max(ae[2], ae[3], ae[7], ae[8], d0)),
        min(d1, max(ae[4], ae[6])),
        min(d1, ae[5]),
    ])

    fas = np.array([
        min(d1, max(ae[3], ae[4], ae[8], ae[9])),
        min(1, max(ae[0], ae[10], ae[5], d0)),
        min(d1, max(ae[1], ae[2], ae[6], ae[7])),
    ])

    agg_lin = np.maximum.reduce([
        np.minimum(fls[i], _y_lin[i]) for i in range(5)
    ])

    agg_ang = np.maximum.reduce([
        np.minimum(fas[0], _y_ang_z),
        np.minimum(fas[1], _y_ang_t),
        np.minimum(fas[2], _y_ang_s),
    ])

    sl = np.sum(agg_lin)
    sa = np.sum(agg_ang)

    v_lin = np.dot(_x_lin, agg_lin) / sl if sl > 1e-12 else 0.0
    v_ang = np.dot(_x_ang, agg_ang) / sa if sa > 1e-12 else 0.0

    velD = (1.0/R) * (v_lin + v_ang*L/2.0)
    velE = (1.0/R) * (v_lin - v_ang*L/2.0)

    return velD, velE

# ============================================================
# PWM
# ============================================================

def velocity_to_pwm(w):

    PWM_MAX = 255
    W_MAX   = 8.0

    pwm = np.clip(np.abs(w)/W_MAX * PWM_MAX, 0, PWM_MAX)

    return pwm

# ============================================================
# ODOMETRIA
# ============================================================

def odometry_step(x0, y0, th0, velD, velE):

    x = x0 + R/2 * DT * np.cos(th0) * (velD + velE)
    y = y0 + R/2 * DT * np.sin(th0) * (velD + velE)

    theta = th0 + R/L * DT * (velD - velE)

    return x, y, theta

# ============================================================
# SIMULAÇÃO
# ============================================================

def simulate(waypoints):

    x = 0.0
    y = float(waypoints[0,1])
    theta = 0.0

    hist = []

    t = 0.0

    for wp in waypoints:

        ox, oy = wp

        for _ in range(MAX_STEPS):

            ex = ox - x
            ey = oy - y

            ErroL = np.hypot(ex, ey)

            if ErroL <= ERRO_STOP:
                break

            phi = np.arctan2(ey, ex) - theta
            phi_norm = 2*np.arctan(np.tan(phi/2))

            velD, velE = fuzzy_control(phi_norm, ErroL)

            pwmD = velocity_to_pwm(velD)
            pwmE = velocity_to_pwm(velE)

            hist.append([
                t,
                x,
                y,
                theta,
                velD,
                velE,
                pwmD,
                pwmE
            ])

            x, y, theta = odometry_step(
                x, y, theta,
                velD, velE
            )

            t += DT

    return np.array(hist)

# ============================================================
# TRAJETÓRIAS
# ============================================================
trajetorias = {
     "SuperZZ1": np.array([
        # --- ZZX 1 ---
        [0.0000, 0.7000], [1.0000, 0.7000],
        [0.8889, 0.5829], [0.7778, 0.4780],
        [0.6667, 0.3842], [0.5556, 0.3002],
        [0.4445, 0.2251], [0.3333, 0.1579],
        [0.2222, 0.0978], [0.1111, 0.0439],
        [0.0000, -0.0042], [1.0000, -0.0042],
        [0.8889, 0.0439], [0.7778, 0.0978],
        [0.6667, 0.1579], [0.5556, 0.2251],
        [0.4445, 0.3002], [0.3333, 0.3842],
        [0.2222, 0.4780], [0.1111, 0.5829],
        [0.0000, 0.7000],
        # --- ZZY 1 (deslocada Δx=-0.7000, Δy=+0.7000) ---
        [0.0000, 1.7000], [-0.1171, 1.5889],
        [-0.2220, 1.4778], [-0.3158, 1.3667],
        [-0.3998, 1.2556], [-0.4749, 1.1445],
        [-0.5421, 1.0333], [-0.6022, 0.9222],
        [-0.6561, 0.8111], [-0.7042, 0.7000],
        [-0.7042, 1.7000], [-0.6561, 1.5889],
        [-0.6022, 1.4778], [-0.5421, 1.3667],
        [-0.4749, 1.2556], [-0.3998, 1.1445],
        [-0.3158, 1.0333], [-0.2220, 0.9222],
        [-0.1171, 0.8111], [0.0000, 0.7000],
    ])
#  "SuperZZ2": np.array([
#         # --- ZZX 2 ---
#         [0.7324, -0.0000], [0.6103, -0.5020],
#         [0.4883, -0.9199], [0.3662, -1.2679],
#         [0.2441, -1.5578], [0.1221, -1.7991],
#         [0.0000, -2.0000], [0.7324, -2.0000],
#         [0.6103, -1.7991], [0.4883, -1.5578],
#         [0.3662, -1.2679], [0.2441, -0.9199],
#         [0.1221, -0.5020], [0.0000, -0.0000],
#         # --- ZZY 2 (deslocada para iniciar no fim da ZZX2) ---
#         [-0.5020, -0.1221], [-0.9199, -0.2441],
#         [-1.2679, -0.3662], [-1.5578, -0.4883],
#         [-1.7991, -0.6103], [-2.0000, -0.7324],
#         [-2.0000, 0.0000],  [-1.7991, -0.1221],
#         [-1.5578, -0.2441], [-1.2679, -0.3662],
#         [-0.9199, -0.4883], [-0.5020, -0.6103],
#         [-0.0000, -0.7324], [-0.0000, 0.0000],
#     ])
    #     "SCurve": np.array([
    #     [0.00, 0.00], [0.10, 0.05], [0.20, 0.09], [0.30, 0.09],
    #     [0.40, 0.05], [0.50, 0.00], [0.60, -0.05], [0.70, -0.09],
    #     [0.80, -0.09], [0.90, -0.05], [1.00, 0.00],
    # ]),
    
    #"Reta": np.array([[0.5,0],[1,0]]),

    # "Zigzag Reto": np.array([
    #     [0.25,0],[0.5,0],[0.75,0],[1,0],
    #     [0.75,0.175],[0.5,0.35],[0.25,0.525],[0,0.7],
    #     [0.25,0.7],[0.5,0.7],[0.75,0.7],[1,0.7],
    #     [0.75,0.525],[0.5,0.35],[0.25,0.175],[0,0],[0.25,0],
    # ]),

    # "ZZY 1": np.array([
    #     [0.7,0.0],[0.7,1.0],[0.5829,0.8889],[0.478,0.7778],
    #     [0.3842,0.6667],[0.3002,0.5556],[0.2251,0.4445],[0.1579,0.3333],
    #     [0.0978,0.2222],[0.0439,0.1111],[-0.0042,0.0],[-0.0042,1.0],
    #     [0.0439,0.8889],[0.0978,0.7778],[0.1579,0.6667],[0.2251,0.5556],
    #     [0.3002,0.4445],[0.3842,0.3333],[0.478,0.2222],[0.5829,0.1111],
    #     [0.7,0.0],
    # ]),
    
    # "ZZY 1": np.array([
    #     [0.0,0.7],[0.0,-0.3],[0.1171,-0.1889],[0.222,-0.0778],
    #     [0.3158,0.0333],[0.3998,0.1444],[0.4749,0.2555],[0.5421,0.3667],
    #     [0.6022,0.4778],[0.6561,0.5889],[0.7042,0.7],
    #     [0.7042,-0.3],[0.6561,-0.1889],[0.6022,-0.0778],[0.5421,0.0333],
    #     [0.4749,0.1444],[0.3998,0.2555],[0.3158,0.3667],[0.222,0.4778],
    #     [0.1171,0.5889],[0.0,0.7],
    # ]),

    # "ZZX 1 INV": np.array([
    #     [0.0,0.0],[1.05,0.0],[0.9333,0.1052],[0.8167,0.1993],
    #     [0.7,0.2835],[0.5833,0.3588],[0.4667,0.4262],[0.35,0.4866],
    #     [0.2333,0.5406],[0.1167,0.5889],[0.0,0.6321],[1.05,0.6321],
    #     [0.9333,0.5889],[0.8167,0.5406],[0.7,0.4866],[0.5833,0.4262],
    #     [0.4667,0.3588],[0.35,0.2835],[0.2333,0.1993],[0.1167,0.1052],
    #     [0.0,0.0],[0.0,0.0],
    # ]),

    # "ZZX 2": np.array([
    #     [0.7324,0.0],[0.6103,-0.502],[0.4883,-0.9199],
    #     [0.3662,-1.2679],[0.2441,-1.5578],[0.1221,-1.7991],
    #     [0.0,-2.0],[0.7324,-2.0],[0.6103,-1.7991],
    #     [0.4883,-1.5578],[0.3662,-1.2679],[0.2441,-0.9199],
    #     [0.1221,-0.502],[0.0,0.0],
    # ]),

    # "ZZY 2": np.array([
    #     [0.0,0.7324],[-0.502,0.6103],[-0.9199,0.4883],[-1.2679,0.3662],
    #     [-1.5578,0.2441],[-1.7991,0.1221],[-2.0,0.0],[-2.0,0.7324],
    #     [-1.7991,0.6103],[-1.5578,0.4883],[-1.2679,0.3662],[-0.9199,0.2441],
    #     [-0.502,0.1221],[0.0,0.0],[0.0,0.7324],
    # ]),

    # "ZZX 2 INV": np.array([
    #     [0.7324,0.0],[0.6103,0.502],[0.4883,0.9199],
    #     [0.3662,1.2679],[0.2441,1.5578],[0.1221,1.7991],
    #     [0.0,2.0],[0.7324,2.0],[0.6103,1.7991],
    #     [0.4883,1.5578],[0.3662,1.2679],[0.2441,0.9199],
    #     [0.1221,0.502],[0.0,0.0],[0.7324,0.0],
    # ]),

    # "Losango 1": np.array([
    #     [0.0,0.0],[0.25,0.5],[0.5,1],[0.75,0.5],
    #     [1,0],[0.75,-0.5],[0.5,-1],[0.25,-0.5],[0,0],
    # ]),

    # "Losango 2": np.array([
    #     [0.5,0.25],[1,0.5],[1.5,0.25],[2,0],
    #     [1.5,-0.25],[1,-0.5],[0.5,-0.25],[0,0],[0.5,0.25],
    # ]),

    # "Infinito": np.array([
    #     [0.1818,0.5406],[0.3636,0.9096],[0.5455,0.9898],
    #     [0.7273,0.7557],[0.9091,0.2817],[1.0991,-0.2817],
    #     [1.2727,-0.7557],[1.4545,-0.9898],[1.6364,-0.9096],
    #     [1.8182,-0.5406],[2,0],[1.8182,0.5406],
    #     [1.6364,0.9096],[1.4545,0.9898],[1.2727,0.7557],
    #     [1.0909,0.2817],[0.9091,-0.2817],[0.7273,-0.7557],
    #     [0.5455,-0.9898],[0.3636,-0.9096],[0.1818,-0.5406],
    #     [0,0],[0.1818,0.5406],
    # ]),
}

# _tc = np.linspace(0, 2*np.pi, 20)

# trajetorias["Circular"] = np.column_stack((
#     1 - np.cos(_tc),
#     np.sin(_tc)
# ))

# ============================================================
# GIF
# ============================================================

def make_gif(nome, waypoints, hist, out_path):

    t     = hist[:,0]
    xs    = hist[:,1]
    ys    = hist[:,2]
    ths   = hist[:,3]

    wd    = hist[:,4]
    we    = hist[:,5]

    pwmD  = hist[:,6]
    pwmE  = hist[:,7]

    # --------------------------------------------------------
    # FIGURA
    # --------------------------------------------------------

    fig = plt.figure(figsize=(16,8))

    gs = GridSpec(
        7,
        2,
        width_ratios=[1.4,1],
        hspace=0.35,
        wspace=0.25
    )

    # --------------------------------------------------------
    # TRAJETÓRIA
    # --------------------------------------------------------

    ax_traj = fig.add_subplot(gs[:,0])

    margin = 0.25

    all_x = np.concatenate([xs, waypoints[:,0]])
    all_y = np.concatenate([ys, waypoints[:,1]])

    ax_traj.set_xlim(all_x.min()-margin, all_x.max()+margin)
    ax_traj.set_ylim(all_y.min()-margin, all_y.max()+margin)

    ax_traj.set_aspect('equal')

    ax_traj.grid(True, alpha=0.3)

    ax_traj.set_xlabel("x (m)")
    ax_traj.set_ylabel("y (m)")

    ax_traj.set_title(f"Trajetória - {nome}")

    ax_traj.plot(
        waypoints[:,0],
        waypoints[:,1],
        'k--',
        alpha=0.4
    )

    trail, = ax_traj.plot([], [], lw=2)

    body = plt.Circle(
        (xs[0], ys[0]),
        ROBOT_RADIUS,
        fc='steelblue',
        ec='navy'
    )

    ax_traj.add_patch(body)

    orient, = ax_traj.plot([], [], 'r-', lw=3)

    # --------------------------------------------------------
    # SUBPLOTS DIREITA
    # --------------------------------------------------------

    labels = [
        ("PWMd", pwmD),
        ("PWMe", pwmE),
        ("wd", wd),
        ("we", we),
        ("x", xs),
        ("y", ys),
        ("theta", np.degrees(ths)),
    ]

    axes = []
    lines = []

    for i, (lab, sig) in enumerate(labels):

        ax = fig.add_subplot(gs[i,1])

        ax.set_xlim(t.min(), t.max())

        ymin = np.min(sig)
        ymax = np.max(sig)

        if ymin == ymax:
            ymin -= 1
            ymax += 1

        pad = 0.1*(ymax-ymin)

        ax.set_ylim(ymin-pad, ymax+pad)

        ax.grid(True, alpha=0.3)

        ax.set_ylabel(lab)

        if i < len(labels)-1:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel("t (s)")

        line, = ax.plot([], [], lw=2)

        axes.append(ax)
        lines.append(line)

    # --------------------------------------------------------
    # FRAMES
    # --------------------------------------------------------

    SKIP = max(1, len(hist)//300)

    frames = list(range(0, len(hist), SKIP))

    if frames[-1] != len(hist)-1:
        frames.append(len(hist)-1)

    # --------------------------------------------------------
    # INIT
    # --------------------------------------------------------

    def init():

        trail.set_data([], [])

        orient.set_data([], [])

        for line in lines:
            line.set_data([], [])

        return [trail, orient, body] + lines

    # --------------------------------------------------------
    # UPDATE
    # --------------------------------------------------------

    def update(fi):

        i = frames[fi]

        xi = xs[i]
        yi = ys[i]
        thi = ths[i]

        # trajetória
        trail.set_data(xs[:i+1], ys[:i+1])

        body.set_center((xi, yi))

        x_tip = xi + ARROW_LEN*np.cos(thi)
        y_tip = yi + ARROW_LEN*np.sin(thi)

        orient.set_data(
            [xi, x_tip],
            [yi, y_tip]
        )

        # séries temporais
        sigs = [
            pwmD,
            pwmE,
            wd,
            we,
            xs,
            ys,
            np.degrees(ths)
        ]

        for line, sig in zip(lines, sigs):

            line.set_data(
                t[:i+1],
                sig[:i+1]
            )

        return [trail, orient, body] + lines

    # --------------------------------------------------------
    # ANIMAÇÃO
    # --------------------------------------------------------

    ani = FuncAnimation(
        fig,
        update,
        frames=len(frames),
        init_func=init,
        interval=40,
        blit=True
    )

    writer = PillowWriter(fps=25)

    ani.save(out_path, writer=writer)

    plt.close(fig)

    print(f"GIF salvo em: {out_path}")

# ============================================================
# MAIN
# ============================================================

OUT_DIR = "./outputs"
os.makedirs(OUT_DIR, exist_ok=True)

for nome, wps in trajetorias.items():

    print(f"Simulando {nome}...")

    hist = simulate(wps)

    out = os.path.join(
        OUT_DIR,
        f"ddmr_{nome.lower()}.gif"
    )

    make_gif(nome, wps, hist, out)

print("Finalizado.")