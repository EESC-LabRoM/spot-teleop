#!/bin/bash
# FR3 collision thresholds as the default on EVERY fr3-ros2 container start (2026-11-05, Murilo go).
# murilo's crontab runs this every minute. When the container has a new StartedAt (new container or restart) and is
# real hardware, it runs /lab/collision_on_start.sh inside it (waits for the driver service, applies
# fr3_ros2.sh real-collision values, logs to runs/collision_apply.log). Marked done per StartedAt only on success,
# so a failed apply is retried the next minute. Never moves the arm; only the service call.
# Disable: touch /home/murilo/franka_ws/runs/collision_watch.disable   (or remove the crontab line).
NAME=fr3-ros2; R=/home/murilo/franka_ws/runs; ST=$R/.collision_applied_for
[ -f $R/collision_watch.disable ] && exit 0
s=$(docker inspect -f '{{.State.Running}} {{.State.StartedAt}}' $NAME 2>/dev/null) || exit 0
set -- $s; [ "$1" = true ] || exit 0
[ "$(cat $ST 2>/dev/null)" = "$2" ] && exit 0
docker inspect -f '{{.Args}}' $NAME | grep -q 'use_fake_hardware:=true' && { echo "$2" > $ST; exit 0; }
exec 9>/tmp/collision_watch.lock; flock -n 9 || exit 0
docker exec -e COLL_TAG="watch, container started $2" $NAME bash /lab/collision_on_start.sh && echo "$2" > $ST
