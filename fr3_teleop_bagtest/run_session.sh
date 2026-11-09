#!/usr/bin/env bash
# Run the full 12-stage bag-as-camera session on the lab machine.
# Usage: bash run_session.sh takes/20261103 [--execute]
# Without --execute every take is a dry run (recognition pipeline only).
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:?usage: run_session.sh takes/DIR [--execute]}
MODE=${2:-}
STAGES="00_neutro 01_giro_plano_imagem 02_inclinar_dedos 03_virar_palma 04_frente_tras 05_direita_esquerda 05_direita_esquerda_2 06_cima_baixo 07_abrir_fechar 08_cruz 09_gesto_em_movimento 10_perda_retorno 11_distancias"
mkdir -p "$OUT"
python3 "$HERE/run_take.py" --chain up
trap 'python3 "$HERE/run_take.py" --chain down' EXIT
pass=0; fail=0; failed=""
for s in $STAGES; do
  echo "=== $s ==="
  if python3 "$HERE/run_take.py" --stage "$s" --out "$OUT/$s" --persistent-tracker $MODE; then
    pass=$((pass+1))
  else
    fail=$((fail+1)); failed="$failed $s"
  fi
done
trap - EXIT
python3 "$HERE/run_take.py" --chain down
echo "SESSION: pass=$pass fail=$fail failed=[$failed ]"
echo "CHECK lines:"
grep -h "^CHECK\|^RECOGNITION\|^\[run_take\] CHECK" "$OUT"/*/summary.json 2>/dev/null || true
for s in $STAGES; do
  python3 -c "import json;d=json.load(open('$OUT/$s/summary.json'));c=d.get('check');print('$s',('PASS' if c and c['ok'] else ('RECOG-%s'%d.get('recognition_ok') if 'recognition_ok' in d else 'FAIL')))" 2>/dev/null || echo "$s NO-SUMMARY"
done
