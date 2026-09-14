"""
Simulação DDMR – Controle Fuzzy + PID
GIF com layout:  [ x(t)    ]  [ y(x) trajetória ]  [ pwmd(t) ]
                 [ y(t)    ]  [                 ]  [ pwme(t) ]
                 [ theta(t)]  [                 ]  [ wd(t)   ]
                 [         ]  [                 ]  [ we(t)   ]
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation, PillowWriter
import os, warnings
warnings.filterwarnings("ignore")

# ============================================================
# PARÂMETROS (do Arduino)
# ============================================================
R  = 0.032    # raio da roda (m)
L  = 0.124    # distância entre rodas (m)
DT = 0.07     # passo de tempo (s)
ERRO_STOP = 0.03
MAX_STEPS = 2000

ErroAgP = [0.0126, 0.2452, 0.2405, 0.4736, 1.3946]

# Ganhos PID (do Arduino)
KPD, KID, KDD = 9.0, 60.0, 0.0
KPE, KIE, KDE = 9.0, 60.0, 0.0

# ============================================================
# FUNÇÕES DE PERTINÊNCIA
# ============================================================
def gauss_mf(x, a, b): return np.exp(-((x-b)**2)/(2*a**2))
def lins_mf(x, a, b):
    if x<=a: return 0.
    if x>=b: return 1.
    return (x-a)/(b-a)
def linz_mf(x, a, b):
    if x<=a: return 1.
    if x>=b: return 0.
    return (b-x)/(b-a)
def tri_mf(x, a, b, c):
    if x<=a or x>=c: return 0.
    return (x-a)/(b-a) if x<=b else (c-x)/(c-b)

def pi_mf_vect(xa, a, b, c, d):
    y = np.zeros_like(xa)
    ab, cd = (a+b)/2, (c+d)/2
    y[(xa>b)&(xa<c)] = 1.0
    m=(xa>a)&(xa<=ab);  y[m]=2*((xa[m]-a)/(b-a))**2
    m=(xa>ab)&(xa<=b);  y[m]=1-2*((xa[m]-b)/(b-a))**2
    m=(xa>=c)&(xa<=cd); y[m]=1-2*((xa[m]-c)/(d-c))**2
    m=(xa>cd)&(xa<d);   y[m]=2*((xa[m]-d)/(d-c))**2
    return y

# ============================================================
# PRÉ-COMPUTA SAÍDAS FUZZY (invariantes)
# ============================================================
_N  = 51
_xl = np.linspace(-0.15, 0.15, _N)
_xa = np.linspace(-0.5,  0.5,  _N)
_yl = [pi_mf_vect(_xl,-0.16,-0.15,-0.10,-0.05),
       pi_mf_vect(_xl,-0.10,-0.05,-0.05, 0.00),
       pi_mf_vect(_xl,-0.05, 0.00, 0.00, 0.05),
       pi_mf_vect(_xl, 0.00, 0.05, 0.05, 0.10),
       pi_mf_vect(_xl, 0.05, 0.10, 0.15, 0.16)]
_yaz = np.array([linz_mf(x,-0.5, 0.0) for x in _xa])
_yat = np.array([tri_mf(x, -0.5, 0.0, 0.5) for x in _xa])
_yas = np.array([lins_mf(x, 0.0, 0.5) for x in _xa])

def fuzzy_control(phi, ErroL):
    p = ErroAgP
    ae = np.array([
        gauss_mf(phi,p[0],-np.pi), gauss_mf(phi,p[1],p[3]-np.pi),
        gauss_mf(phi,p[2],p[4]-np.pi), gauss_mf(phi,p[2],-p[4]),
        gauss_mf(phi,p[1],-p[3]),  gauss_mf(phi,p[0], 0.0),
        gauss_mf(phi,p[1], p[3]),  gauss_mf(phi,p[2], p[4]),
        gauss_mf(phi,p[2],np.pi-p[4]), gauss_mf(phi,p[1],np.pi-p[3]),
        gauss_mf(phi,p[0], np.pi)])
    fls = np.array([min(1,max(ae[0],ae[10])), min(1,max(ae[1],ae[9])),
                    min(1,max(ae[2],ae[3],ae[7],ae[8],0.)),
                    min(1,max(ae[4],ae[6])),  min(1,ae[5])])
    fas = np.array([min(1,max(ae[3],ae[4],ae[8],ae[9])),
                    min(1,max(ae[0],ae[10],ae[5],0.)),
                    min(1,max(ae[1],ae[2],ae[6],ae[7]))])
    al = np.maximum.reduce([np.minimum(fls[i],_yl[i]) for i in range(5)])
    aa = np.maximum.reduce([np.minimum(fas[0],_yaz),
                            np.minimum(fas[1],_yat),
                            np.minimum(fas[2],_yas)])
    sl, sa = al.sum(), aa.sum()
    vl = _xl@al/sl if sl>1e-12 else 0.
    va = _xa@aa/sa if sa>1e-12 else 0.
    setD = (1/R)*(vl + va*L/2)
    setE = (1/R)*(vl - va*L/2)
    return setD, setE

# ============================================================
# PID DISCRETO (replica PID_v1 do Arduino)
#   output = Kp*e + Ki*ITerm*dt - Kd*(input - last_input)/dt
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, out_min=-255, out_max=255):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.iterm    = 0.
        self.last_inp = 0.

    def compute(self, setpoint, measured):
        err    = setpoint - measured
        dinput = measured - self.last_inp
        self.iterm += self.ki * err * DT
        self.iterm  = np.clip(self.iterm, self.out_min, self.out_max)
        out = self.kp*err + self.iterm - self.kd*dinput/DT
        out = np.clip(out, self.out_min, self.out_max)
        self.last_inp = measured
        return out

    def reset(self):
        self.iterm = 0.; self.last_inp = 0.

# ============================================================
# SIMULAÇÃO
# ============================================================
def simulate(waypoints):
    x, y, th = 0., float(waypoints[0,1]), 0.
    velD, velE = 0., 0.
    pid_d = PID(KPD, KID, KDD)
    pid_e = PID(KPE, KIE, KDE)
    pid_d.reset(); pid_e.reset()

    # colunas: t, x, y, theta, velD, velE, setD, setE, pwmD, pwmE
    hist = []
    t = 0.

    for wp in waypoints:
        ox, oy = float(wp[0]), float(wp[1])
        for _ in range(MAX_STEPS):
            ex, ey = ox-x, oy-y
            ErroL  = np.hypot(ex, ey)
            if ErroL <= ERRO_STOP:
                break

            phi    = np.arctan2(ey, ex) - th
            phi_n  = 2.*np.arctan(np.tan(phi/2.))
            setD, setE = fuzzy_control(phi_n, ErroL)

            pwmD = pid_d.compute(setD, velD)
            pwmE = pid_e.compute(setE, velE)

            # Modelo simplificado de motor: velocidade proporcional ao PWM
            # wd = (pwm/255) * V_max  onde V_max ~ 8 rad/s (empirico)
            W_MAX = 8.0
            velD = (pwmD / 255.) * W_MAX
            velE = (pwmE / 255.) * W_MAX

            hist.append([t, x, y, th, velD, velE, setD, setE, pwmD, pwmE])

            # Odometria
            x  += R/2 * DT * np.cos(th) * (velD + velE)
            y  += R/2 * DT * np.sin(th) * (velD + velE)
            th += R/L  * DT * (velD - velE)
            t  += DT

    hist.append([t, x, y, th, velD, velE, 0., 0., 0., 0.])
    return np.array(hist)
# índices das colunas
iT,iX,iY,iTH,iWD,iWE,iSD,iSE,iPD,iPE = range(10)

# ============================================================
# TRAJETÓRIAS
# ============================================================
trajetorias = {
    
    
    "SCurve": np.array([
        [0.00, 0.00], [0.10, 0.05], [0.20, 0.09], [0.30, 0.09],
        [0.40, 0.05], [0.50, 0.00], [0.60, -0.05], [0.70, -0.09],
        [0.80, -0.09], [0.90, -0.05], [1.00, 0.00],
    ]),
    
    # "Reta":         np.array([[0.5,0],[1,0]]),
    # "Zigzag Reto":  np.array([[0.25,0],[0.5,0],[0.75,0],[1,0],[0.75,0.175],[0.5,0.35],
    #                            [0.25,0.525],[0,0.7],[0.25,0.7],[0.5,0.7],[0.75,0.7],[1,0.7],
    #                            [0.75,0.525],[0.5,0.35],[0.25,0.175],[0,0],[0.25,0]]),
    # "ZZX 1":        np.array([[0,0.7],[1,0.7],[0.8889,0.5829],[0.7778,0.478],[0.6667,0.3842],
    #                            [0.5556,0.3002],[0.4445,0.2251],[0.3333,0.1579],[0.2222,0.0978],
    #                            [0.1111,0.0439],[0,-0.0042],[1,-0.0042],[0.8889,0.0439],
    #                            [0.7778,0.0978],[0.6667,0.1579],[0.5556,0.2251],[0.4445,0.3002],
    #                            [0.3333,0.3842],[0.2222,0.478],[0.1111,0.5829],[0,0.7]]),
    # "ZZY 1":        np.array([[0,0.7],[0,-0.3],[0.1171,-0.1889],[0.222,-0.0778],[0.3158,0.0333],
    #                            [0.3998,0.1444],[0.4749,0.2555],[0.5421,0.3667],[0.6022,0.4778],
    #                            [0.6561,0.5889],[0.7042,0.7],[0.7042,-0.3],[0.6561,-0.1889],
    #                            [0.6022,-0.0778],[0.5421,0.0333],[0.4749,0.1444],[0.3998,0.2555],
    #                            [0.3158,0.3667],[0.222,0.4778],[0.1171,0.5889],[0,0.7]]),
    # "ZZX 1 INV":    np.array([[0,0],[1.05,0],[0.9333,0.1052],[0.8167,0.1993],[0.7,0.2835],
    #                            [0.5833,0.3588],[0.4667,0.4262],[0.35,0.4866],[0.2333,0.5406],
    #                            [0.1167,0.5889],[0,0.6321],[1.05,0.6321],[0.9333,0.5889],
    #                            [0.8167,0.5406],[0.7,0.4866],[0.5833,0.4262],[0.4667,0.3588],
    #                            [0.35,0.2835],[0.2333,0.1993],[0.1167,0.1052],[0,0],[0,0]]),
    # "ZZX 2":        np.array([[0.7324,0],[0.6103,-0.502],[0.4883,-0.9199],[0.3662,-1.2679],
    #                            [0.2441,-1.5578],[0.1221,-1.7991],[0,-2],[0.7324,-2],
    #                            [0.6103,-1.7991],[0.4883,-1.5578],[0.3662,-1.2679],
    #                            [0.2441,-0.9199],[0.1221,-0.502],[0,0]]),
    # "ZZY 2":        np.array([[0,0.7324],[-0.502,0.6103],[-0.9199,0.4883],[-1.2679,0.3662],
    #                            [-1.5578,0.2441],[-1.7991,0.1221],[-2,0],[-2,0.7324],
    #                            [-1.7991,0.6103],[-1.5578,0.4883],[-1.2679,0.3662],
    #                            [-0.9199,0.2441],[-0.502,0.1221],[0,0],[0,0.7324]]),
    # "ZZX 2 INV":    np.array([[0.7324,0],[0.6103,0.502],[0.4883,0.9199],[0.3662,1.2679],
    #                            [0.2441,1.5578],[0.1221,1.7991],[0,2],[0.7324,2],
    #                            [0.6103,1.7991],[0.4883,1.5578],[0.3662,1.2679],
    #                            [0.2441,0.9199],[0.1221,0.502],[0,0],[0.7324,0]]),
    # "Losango 1":    np.array([[0,0],[0.25,0.5],[0.5,1],[0.75,0.5],[1,0],
    #                            [0.75,-0.5],[0.5,-1],[0.25,-0.5],[0,0]]),
    
    # "Losango 2":    np.array([[0.0,  0.0 ], [0.5,  0.25], [1.0,  0.0 ], [1.5, -0.25],
    # [1.0, -0.5 ], [0.5, -0.75], [0.0, -0.5 ], [-0.5, -0.25], [0.0,  0.0]]),
    
    # "Infinito":     np.array([[0.1818,0.5406],[0.3636,0.9096],[0.5455,0.9898],[0.7273,0.7557],
    #                            [0.9091,0.2817],[1.0991,-0.2817],[1.2727,-0.7557],[1.4545,-0.9898],
    #                            [1.6364,-0.9096],[1.8182,-0.5406],[2,0],[1.8182,0.5406],
    #                            [1.6364,0.9096],[1.4545,0.9898],[1.2727,0.7557],[1.0909,0.2817],
    #                            [0.9091,-0.2817],[0.7273,-0.7557],[0.5455,-0.9898],
    #                            [0.3636,-0.9096],[0.1818,-0.5406],[0,0],[0.1818,0.5406]]),
}
# _tc = np.linspace(0, 2*np.pi, 20)

# trajetorias["Circular"] = np.column_stack((
#     1 - np.cos(_tc),
#     np.sin(_tc)
# ))

# ============================================================
# GIF COM LAYOUT COMPLETO
# ============================================================
ROBOT_R  = 0.04
ARROW_L  = 0.09
OUT_DIR  = "./OutputsTS"
os.makedirs(OUT_DIR, exist_ok=True)

def make_gif(nome, waypoints, hist):
    t   = hist[:,iT]
    xs  = hist[:,iX];  ys  = hist[:,iY];  ths = hist[:,iTH]
    wd  = hist[:,iWD]; we  = hist[:,iWE]
    pwd = hist[:,iPD]; pwe = hist[:,iPE]
    wps_x, wps_y = waypoints[:,0], waypoints[:,1]

    # ---- Limites eixos ----
    mg = 0.25
    xl = (min(xs.min(),wps_x.min())-mg, max(xs.max(),wps_x.max())+mg)
    yl = (min(ys.min(),wps_y.min())-mg, max(ys.max(),wps_y.max())+mg)
    t_max = t[-1]

    def ylim_sym(arr, pad=0.15):
        lo, hi = arr.min(), arr.max()
        rng = max(abs(lo), abs(hi), 0.1)
        return (-rng-pad, rng+pad)

    # ---- Figura e GridSpec ----
    fig = plt.figure(figsize=(14, 8), dpi=90)
    fig.suptitle(f"DDMR – {nome}", fontsize=13, fontweight='bold', y=1.01)
    gs = GridSpec(4, 3, figure=fig, hspace=0.55, wspace=0.38,
                  left=0.06, right=0.97, top=0.95, bottom=0.06)

    # Coluna esquerda: x, y, theta
    ax_x  = fig.add_subplot(gs[0, 0])
    ax_y  = fig.add_subplot(gs[1, 0])
    ax_th = fig.add_subplot(gs[2, 0])
    # linha 3 col 0 fica vazia (equilibra visualmente)

    # Centro: trajetória y(x)
    ax_tr = fig.add_subplot(gs[:, 1])

    # Coluna direita: pwmD, pwmE, ωd, ωe
    ax_pd = fig.add_subplot(gs[0, 2])
    ax_pe = fig.add_subplot(gs[1, 2])
    ax_wd = fig.add_subplot(gs[2, 2])
    ax_we = fig.add_subplot(gs[3, 2])

    # ---- Configuração estática dos eixos ----
    for ax, lbl, ylims, color in [
        (ax_x,  'x (m)',         (xs.min()-mg,  xs.max()+mg),  'tab:blue'),
        (ax_y,  'y (m)',         (ys.min()-mg,  ys.max()+mg),  'tab:green'),
        (ax_th, 'θ (rad)',       ylim_sym(ths),                 'tab:orange'),
        (ax_pd, 'PWM D',         (-270, 270),                   'crimson'),
        (ax_pe, 'PWM E',         (-270, 270),                   'darkorchid'),
        (ax_wd, 'ωd (rad/s)',    ylim_sym(wd),                  'saddlebrown'),
        (ax_we, 'ωe (rad/s)',    ylim_sym(we),                  'teal'),
    ]:
        ax.set_xlim(0, t_max)
        ax.set_ylim(*ylims)
        ax.set_ylabel(lbl, fontsize=7, color=color)
        ax.tick_params(axis='both', labelsize=6)
        ax.grid(True, alpha=0.25)
        ax.axhline(0, color='gray', lw=0.5, ls='--')
        ax.yaxis.label.set_color(color)
        ax.spines['left'].set_color(color)
        ax.tick_params(axis='y', colors=color)

    ax_we.set_xlabel('t (s)', fontsize=7)
    ax_th.set_xlabel('t (s)', fontsize=7)

    # Trajetória central
    ax_tr.set_xlim(*xl); ax_tr.set_ylim(*yl)
    ax_tr.set_aspect('equal')
    ax_tr.set_xlabel('x (m)', fontsize=8); ax_tr.set_ylabel('y (m)', fontsize=8)
    ax_tr.set_title('y(x)', fontsize=9)
    ax_tr.grid(True, alpha=0.25)
    ax_tr.plot(wps_x, wps_y, 'k--', lw=1, alpha=0.4)
    ax_tr.scatter(wps_x, wps_y, s=18, c='gray', zorder=2)
    ax_tr.scatter(wps_x[0],  wps_y[0],  s=70, c='green',  marker='s', zorder=4, label='início')
    ax_tr.scatter(wps_x[-1], wps_y[-1], s=70, c='purple', marker='x', lw=2, zorder=4, label='fim')
    ax_tr.legend(fontsize=7, loc='lower right')

    # ---- Elementos animados ----
    # Séries temporais (linhas crescentes)
    ln_x,  = ax_x.plot( [], [], 'tab:blue',    lw=1.2)
    ln_y,  = ax_y.plot( [], [], 'tab:green',   lw=1.2)
    ln_th, = ax_th.plot([], [], 'tab:orange',  lw=1.2)
    ln_pd, = ax_pd.plot([], [], 'crimson',     lw=1.2)
    ln_pe, = ax_pe.plot([], [], 'darkorchid',  lw=1.2)
    ln_wd, = ax_wd.plot([], [], 'saddlebrown', lw=1.2)
    ln_we, = ax_we.plot([], [], 'teal',        lw=1.2)
    # Marcador de tempo (linha vertical)
    vlines = []
    for ax in (ax_x, ax_y, ax_th, ax_pd, ax_pe, ax_wd, ax_we):
        vl = ax.axvline(0, color='gray', lw=0.8, ls=':')
        vlines.append(vl)

    # Trajetória + robô
    trail,  = ax_tr.plot([], [], 'b-', lw=1.2, alpha=0.7, zorder=3)
    body    = plt.Circle((xs[0],ys[0]), ROBOT_R, fc='steelblue', ec='navy', lw=1.5, zorder=5)
    ax_tr.add_patch(body)
    orient, = ax_tr.plot([], [], 'r-', lw=2.5, zorder=6)
    theta_lbl = ax_tr.text(0.02, 0.97, '', transform=ax_tr.transAxes,
                           fontsize=8, va='top', color='red')

    # ---- Decimação ----
    SKIP   = max(1, len(hist)//220)
    frames = list(range(0, len(hist), SKIP))
    if frames[-1] != len(hist)-1:
        frames.append(len(hist)-1)

    lines_ts = [ln_x, ln_y, ln_th, ln_pd, ln_pe, ln_wd, ln_we]
    data_ts  = [xs, ys, ths, pwd, pwe, wd, we]

    def init():
        for ln in lines_ts: ln.set_data([], [])
        trail.set_data([], [])
        orient.set_data([], [])
        return lines_ts + [trail, body, orient, theta_lbl] + vlines

    def update(fi):
        i = frames[fi]
        ti = t[i]

        # Séries temporais
        for ln, data in zip(lines_ts, data_ts):
            ln.set_data(t[:i+1], data[:i+1])

        # Linhas verticais de tempo
        for vl in vlines:
            vl.set_xdata([ti, ti])

        # Trajetória + robô
        xi, yi, thi = xs[i], ys[i], ths[i]
        trail.set_data(xs[:i+1], ys[:i+1])
        body.set_center((xi, yi))
        tip_x = xi + ARROW_L*np.cos(thi)
        tip_y = yi + ARROW_L*np.sin(thi)
        orient.set_data([xi, tip_x], [yi, tip_y])
        theta_lbl.set_text(f'θ = {np.degrees(thi):.1f}°')

        return lines_ts + [trail, body, orient, theta_lbl] + vlines

    ani = FuncAnimation(fig, update, frames=len(frames),
                        init_func=init, blit=True, interval=45)

    slug = nome.lower().replace(' ','_').replace('/','')
    out  = f"{OUT_DIR}/ddmr_v2_{slug}.gif"
    ani.save(out, writer=PillowWriter(fps=20))
    plt.close(fig)
    print(f"  → {out}  ({len(hist)} passos, {len(frames)} frames)")
    return out

# ============================================================
# MAIN
# ============================================================
saved = []
total = len(trajetorias)
for idx, (nome, wps) in enumerate(trajetorias.items(), 1):
    print(f"[{idx}/{total}] {nome}...", flush=True)
    hist = simulate(wps)
    out  = make_gif(nome, wps, hist)
    saved.append(out)

print("\n=== CONCLUÍDO ===")
for p in saved:
    print(" ", p)