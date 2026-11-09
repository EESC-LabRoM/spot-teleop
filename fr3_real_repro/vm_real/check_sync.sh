#!/usr/bin/env bash
# Read-only drift check: this repo (vm_real/) is the definitive version. Compares it with the lab PC's copies (hub-curobo/) and with the cuRobo library.
# Exit 0 = identical, 1 = something differs. Changes nothing anywhere.
H=$(cd "$(dirname "$0")" && pwd); LAB="ssh -i $HOME/.ssh/id_ed25519_lab_claude -o ConnectTimeout=8 murilo@100.88.134.127"; L=/home/murilo/franka_ws/hub-curobo; rc=0
chk() { a=$(md5sum < "$H/$1" | cut -c1-32); b=$($LAB "md5sum < $L/$2" 2>/dev/null | cut -c1-32); [ "$a" = "$b" ] && echo "same   $1" || { echo "DIFFER $1 (repo $a, lab ${b:-missing})"; rc=1; }; }
chk node/curobo_mpc_node.py node/curobo_mpc_node.py; chk lab/curobo_to_jtc.py lab/curobo_to_jtc.py; chk lab/fr3_arm_jtc.yaml lab/fr3_arm_jtc.yaml
for f in "$H"/node_config/*.yml; do chk "node_config/$(basename "$f")" "node_config/$(basename "$f")"; done
LIB='cd src && find . -name __pycache__ -prune -o -type f \( -name "*.py" -o -name "*.yml" -o -name "*.yaml" -o -name "*.cu" -o -name "*.cpp" -o -name "*.h" \) -print0 | sort -z | xargs -0 md5sum | md5sum | cut -c1-32'
a=$(cd "$H/../../isaac-sim_ws/curobo-isaac6" && bash -c "$LIB"); b=$($LAB "cd $L/curobo-isaac6 && $LIB")
[ "$a" = "$b" ] && echo "same   cuRobo library src ($a)" || { echo "DIFFER cuRobo library src (VM $a, lab $b)"; rc=1; }
exit $rc
