#!/usr/bin/env bash
# FR3 bag-as-camera teleop test — the only script you need.
#   bash bagtest.sh 04_frente_tras   # one take (~2-3 min), foreground output
#   bash bagtest.sh all              # remaining 11 takes in order (~25 min), pauses once for grasp
#   bash bagtest.sh down             # stop the test chain at end of session
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
CMD=${1:?usage: bagtest.sh STAGE|all|down}
if [ "$CMD" = down ]; then
  python3 "$HERE/run_take.py" --chain down
  exit $?
fi
chain_up() {
  if ! pgrep -f "[o]perator_hand_tracker.*fr3_bagtest" >/dev/null; then
    echo "--- starting test chain ---"
    python3 "$HERE/run_take.py" --chain up --stage "$1" || exit 3
  fi
}
if [ "$CMD" = all ]; then
  STAGES="05_direita_esquerda 05_direita_esquerda_2 06_cima_baixo 01_giro_plano_imagem 02_inclinar_dedos 03_virar_palma 07_abrir_fechar 08_cruz 09_gesto_em_movimento 10_perda_retorno 11_distancias"
  BASE="takes/exec_all_$(date +%Y%m%d_%H%M%S)"
  chain_up 05_direita_esquerda
  pass=0; fail=0; failed=""
  for s in $STAGES; do
    echo "=== $s ==="
    if python3 "$HERE/run_take.py" --stage "$s" --out "$BASE/$s" --persistent-tracker --execute; then
      pass=$((pass+1))
    else
      fail=$((fail+1)); failed="$failed $s"
    fi
    if [ "$s" = 05_direita_esquerda ]; then
      echo "=== grasp window: the grasp lane runs their ~2 min final attempt now."
      echo "=== Press Enter here AFTER they post done, to continue with the rest. ==="
      read -r _ || true
    fi
  done
  python3 "$HERE/run_take.py" --chain down
  echo "SESSION: pass=$pass fail=$fail failed=[$failed ]"
  for s in $STAGES; do
    python3 - "$BASE/$s/summary.json" "$s" <<'EOF' 2>/dev/null || echo "$s NO-SUMMARY"
import json, sys
d = json.load(open(sys.argv[1])); c = d.get('check') or {}; p = d.get('post_stop') or {}
span = c.get('span_cm') or [0, 0, 0]
post = ('drift=%scm cmds=%s' % (p.get('post_stop_drift_cm'), p.get('commands_after_stop'))
        if p.get('ok') else 'poststop n/a')
print('%s %s span=[%.1f,%.1f,%.1f]cm oriexc=%.1fdeg %s' % (
    sys.argv[2], 'PASS' if c.get('ok') else 'FAIL',
    span[0], span[1], span[2], c.get('oriexc_deg') or 0, post))
EOF
  done
  [ "$fail" = 0 ]
  exit $?
fi
OUT=${2:-takes/exec_$(date +%Y%m%d_%H%M%S)_$CMD}
chain_up "$CMD"
python3 "$HERE/run_take.py" --stage "$CMD" --out "$OUT" --persistent-tracker --execute
rc=$?
echo "--- take exit: $rc ($OUT) ---"
exit $rc
