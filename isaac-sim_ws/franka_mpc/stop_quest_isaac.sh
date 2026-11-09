#!/usr/bin/env bash
# Stops everything start_quest_isaac.sh started (only containers of this stack; the Quest lane's quest-wrist-teleop is never touched).
docker stop -t 20 quest-isaac-quest franka-mpc quest-isaac-sim >/dev/null 2>&1; docker rm -f quest-isaac-quest >/dev/null 2>&1; sleep 2
docker ps --format '{{.Names}}' | grep -E "^(quest-isaac-quest|franka-mpc|quest-isaac-sim)$" || echo "stopped"
