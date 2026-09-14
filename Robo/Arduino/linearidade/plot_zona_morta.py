import pandas as pd
import matplotlib.pyplot as plt

# ============================================================
# Carregar dados
# ============================================================
arquivo = "./Robo/Arduino/linearidade/results-600-800.csv"

df = pd.read_csv(arquivo)

# Garantir ordenação pelo tempo
df = df.sort_values(["motor", "pwm", "t_ms"])

# ============================================================
# Criar figura com os dois motores lado a lado
# ============================================================
fig, axes = plt.subplots(
    1, 2,
    figsize=(12, 5),
    sharey=True
)

# ============================================================
# Plotar Motor D e Motor E
# ============================================================
for ax, motor in zip(axes, ["D", "E"]):

    dados_motor = df[df["motor"].astype(str).str.upper() == motor]

    # Uma resposta para cada PWM
    for pwm, dados_pwm in dados_motor.groupby("pwm"):

        ax.plot(
            dados_pwm["t_ms"],  # ms -> s
            dados_pwm["vel_rad_s"],
            label=f"PWM = {pwm}"
        )

    ax.set_title(f"Motor {motor}")
    ax.set_xlabel("Tempo (s)")
    ax.grid(True, alpha=0.3)
    ax.legend()

# ============================================================
# Rótulos e título
# ============================================================
axes[0].set_ylabel(r"Velocidade angular (rad/s)")

fig.suptitle(
    "Respostas ao degrau para diferentes valores de PWM",
    fontsize=14
)

plt.tight_layout()

# ============================================================
# Salvar figura
# ============================================================
plt.savefig(
    "./Robo/Arduino/linearidade/respostas_degrau.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()