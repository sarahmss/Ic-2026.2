#!/bin/bash
#
# zip_models.sh
#
# Procura todas as pastas chamadas "models" dentro do repositório (ou de um
# diretório específico passado como argumento), compacta o conteúdo de cada
# uma em um "models.zip" ao lado da pasta e garante que a pasta "models/"
# fique ignorada pelo git (via .gitignore). No final, roda "git add -A":
# tudo o que estiver em Theta/ (Data, scalers, resultados-*.xlsx, Train.ipynb
# etc.) sobe normalmente — só o conteúdo bruto de "models/" fica de fora,
# substituído pelo models.zip.
#
# Uso:
#   ./zip_models.sh                # varre a raiz do repositório git atual
#   ./zip_models.sh 00-TrainRnns   # varre só essa subpasta
#
set -euo pipefail

# ---------------------------------------------------------------------------
# 0. Dependências
# ---------------------------------------------------------------------------
if ! command -v zip >/dev/null 2>&1; then
    echo "Erro: o comando 'zip' não está instalado. Instale com 'sudo apt install zip'." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# 1. Diretórios base
# ---------------------------------------------------------------------------
ROOT_DIR="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
SEARCH_DIR="${1:-$ROOT_DIR}"

if [ ! -d "$SEARCH_DIR" ]; then
    echo "Erro: diretório '$SEARCH_DIR' não existe." >&2
    exit 1
fi

echo "Repositório: $ROOT_DIR"
echo "Buscando pastas 'models' em: $SEARCH_DIR"
echo

# ---------------------------------------------------------------------------
# 2. Garante que a pasta "models" (em qualquer nível) seja ignorada pelo git,
#    mas que o .zip ao lado dela continue rastreável.
# ---------------------------------------------------------------------------
GITIGNORE="$ROOT_DIR/.gitignore"
IGNORE_LINE="**/models/"

touch "$GITIGNORE"
if ! grep -qxF "$IGNORE_LINE" "$GITIGNORE"; then
    {
        echo ""
        echo "# Pastas de modelos treinados: versionar só o .zip, não os arquivos soltos"
        echo "$IGNORE_LINE"
    } >> "$GITIGNORE"
    echo "Adicionada a regra '$IGNORE_LINE' ao .gitignore"
fi

# ---------------------------------------------------------------------------
# 3. Encontra cada pasta "models" e compacta
# ---------------------------------------------------------------------------
found_any=false

while IFS= read -r -d '' models_dir; do
    found_any=true

    parent_dir="$(dirname "$models_dir")"
    zip_path="$parent_dir/models.zip"
    zip_name="models.zip"

    echo "-> Encontrado: $models_dir"

    # Se já existir um models.zip mais novo que o conteúdo da pasta, pula
    if [ -f "$zip_path" ] && [ -z "$(find "$models_dir" -newer "$zip_path" -print -quit)" ]; then
        echo "   models.zip já está atualizado, pulando compactação."
    else
        echo "   Compactando -> $zip_path"
        # Entra no diretório pai para que o zip guarde caminhos relativos
        # (ex.: "models/arquivo.pt") em vez de caminhos absolutos.
        (cd "$parent_dir" && zip -r -q -X "$zip_name" "models")
    fi

    # Se a pasta "models" (ou arquivos dentro dela) já estava sendo
    # rastreada pelo git de commits anteriores, remove do índice (mantendo
    # os arquivos em disco) para que ela pare de ser versionada individualmente.
    if git -C "$ROOT_DIR" ls-files --error-unmatch "$models_dir" >/dev/null 2>&1 \
        || [ -n "$(git -C "$ROOT_DIR" ls-files "$models_dir" 2>/dev/null)" ]; then
        echo "   Removendo conteúdo antigo de '$models_dir' do índice do git (mantido em disco)"
        git -C "$ROOT_DIR" rm -r --cached --ignore-unmatch "$models_dir" >/dev/null
    fi

    echo

done < <(find "$SEARCH_DIR" -type d -name "models" -print0)

if [ "$found_any" = false ]; then
    echo "Nenhuma pasta 'models' encontrada em '$SEARCH_DIR'."
fi

# ---------------------------------------------------------------------------
# 4. git add em tudo, exceto as pastas "models" (que o .gitignore exclui).
#    Isso pega o(s) models.zip recém-criado(s) e também qualquer outro
#    arquivo novo/alterado no repositório (Data, scalers, xlsx, ipynb, etc.).
# ---------------------------------------------------------------------------
echo "Rodando 'git add -A' (a pasta 'models/' fica de fora por causa do .gitignore)..."
git -C "$ROOT_DIR" add -A

echo
echo "Concluído. Revise com 'git status' e depois rode 'git commit'."
