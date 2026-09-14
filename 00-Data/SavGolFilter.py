from scipy.signal import savgol_filter

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import math


# ============================================================
# CONFIGURAÇÃO
# ============================================================

TITLES = [
    "LSG-1",
    "LSG-2",
    "ZZx1-inv",
    "ZZx1",
    "ZZx2-inv2",
    "ZZx2",
    "ZZxReto",
    "ZZy1",
    "ZZy2",
    "semiCirc",
]

TS = 0.07

PLOT = True


# ============================================================
# PARÂMETROS DO FILTRO SAVITZKY-GOLAY
# ============================================================
#
# A janela precisa ser ÍMPAR.
# Foi utilizado 51 em vez de 50.
#
# (window, poly)
# ============================================================

parametros_savgol = {

    # Pose
    "x": (51, 3),
    "y": (51, 3),
    "theta": (51, 3),

    # Velocidade angular das rodas
    "phi_d": (51, 3),
    "phi_e": (51, 3),

    # Referências
    "phi_d_ref": (51, 3),
    "phi_e_ref": (51, 3),

    # Tensao
    "e_a_d": (51, 3),
    "e_a_e": (51, 3),
}


# ============================================================
# CAMINHOS
# ============================================================

os.chdir("./Final/Data")

input_path = "./Datasets.xlsx"
output_path = "./SavgolDatasets.xlsx"


# ============================================================
# CRIA EXCEL DE SAÍDA
# ============================================================

writer = pd.ExcelWriter(
    output_path,
    engine="openpyxl"
)


# ============================================================
# LOOP SOBRE AS TRAJETÓRIAS
# ============================================================

for title in TITLES:

    print(f"\nProcessando: {title}")

    # --------------------------------------------------------
    # Lê a sheet
    # --------------------------------------------------------

    df = pd.read_excel(
        input_path,
        sheet_name=title
    )

    # --------------------------------------------------------
    # Tempo
    # --------------------------------------------------------

    tempo = np.arange(len(df)) * TS

    # --------------------------------------------------------
    # DataFrame que receberá os dados filtrados
    # --------------------------------------------------------

    data_filt = pd.DataFrame()

    data_filt["Tempo"] = tempo

    # --------------------------------------------------------
    # Verifica quais colunas estão presentes
    # --------------------------------------------------------

    cols_validas = [
        col
        for col in parametros_savgol.keys()
        if col in df.columns
    ]

    print("Colunas encontradas:")
    print(cols_validas)

    # ========================================================
    # FIGURA
    # ========================================================

    n = len(cols_validas)

    n_cols = 3

    n_rows = math.ceil(n / n_cols)

    if PLOT:

        fig, axs = plt.subplots(
            n_rows,
            n_cols,
            figsize=(
                6 * n_cols,
                4 * n_rows
            )
        )

        # Garante que axs seja sempre um vetor
        axs = np.atleast_1d(axs).flatten()

    # ========================================================
    # LOOP DAS COLUNAS
    # ========================================================

    for i, col in enumerate(cols_validas):

        window, poly = parametros_savgol[col]

        # ----------------------------------------------------
        # Dados originais
        # ----------------------------------------------------

        y = df[col].to_numpy(dtype=float)

        # ----------------------------------------------------
        # Verificação do tamanho da janela
        # ----------------------------------------------------

        if window >= len(y):

            # maior janela ímpar possível
            window = len(y)

            if window % 2 == 0:
                window -= 1

        if window <= poly:

            raise ValueError(
                f"Janela inválida para {title}/{col}: "
                f"window={window}, poly={poly}"
            )

        # ----------------------------------------------------
        # Tratamento especial para theta
        # ----------------------------------------------------
        #
        # Remove descontinuidades em ±pi antes do filtro.
        # ----------------------------------------------------

        if col == "theta":

            y_unwrapped = np.unwrap(y)

        else:

            y_unwrapped = y

        # ====================================================
        # FILTRO
        # ====================================================

        y_f = savgol_filter(
            y_unwrapped,
            window_length=window,
            polyorder=poly
        )

        # ====================================================
        # DERIVADAS
        # ====================================================
        #
        # Somente para x, y e theta.
        #
        # dx/dt
        # dy/dt
        # dtheta/dt
        # ====================================================

        if col in ["theta", "x", "y"]:

            dy_f = savgol_filter(
                y_unwrapped,
                window_length=window,
                polyorder=poly,
                deriv=1,
                delta=TS
            )

            # Nome da derivada
            data_filt[f"d{col}"] = dy_f

        # ----------------------------------------------------
        # Retorna theta para [-pi, pi]
        # ----------------------------------------------------

        if col == "theta":

            y_f = (
                (y_f + np.pi)
                % (2 * np.pi)
            ) - np.pi

        # ====================================================
        # SALVA SINAL FILTRADO
        # ====================================================

        data_filt[col] = y_f

        # ====================================================
        # PLOT
        # ====================================================

        if PLOT:

            ax = axs[i]

            # Dados originais
            ax.scatter(
                tempo,
                df[col],
                s=10,
                alpha=0.4,
                label="Original"
            )

            # Dados filtrados
            ax.plot(
                tempo,
                y_f,
                linewidth=2,
                label="Filtrado"
            )

            ax.set_title(col)

            ax.set_xlabel(
                "Tempo [s]"
            )

            ax.grid(True)

            ax.legend()

    # ========================================================
    # REMOVE SUBPLOTS VAZIOS
    # ========================================================

    if PLOT:

        for j in range(
            len(cols_validas),
            len(axs)
        ):

            fig.delaxes(axs[j])

        fig.suptitle(
            f"{title} - Savitzky-Golay",
            fontsize=16
        )

        plt.tight_layout()

        plt.show()

    # ========================================================
    # SALVA SHEET
    # ========================================================

    data_filt.to_excel(
        writer,
        sheet_name=title,
        index=False
    )

    print(
        f"Concluído: {title}"
    )


# ============================================================
# FECHA EXCEL
# ============================================================

writer.close()

print("\n========================================")
print("Processamento concluído!")
print(f"Arquivo salvo em: {output_path}")
print("========================================")