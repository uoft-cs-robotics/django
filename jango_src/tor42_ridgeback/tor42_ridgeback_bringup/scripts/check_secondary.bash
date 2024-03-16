#!/bin/bash
# Make sure to add the secondary PC to /etc/hosts
export SECONDARY_PC="tor42-jetson"

printf "%s\n" "[Zed2-Remote] Waiting for Secondary PC ($SECONDARY_PC)"

while ! ssh "$SECONDARY_PC" bash -c '/etc/ros/setup-remote.bash rostopic list'
do
  printf "%s\n" "[Zed2-Remote] Could not ping Secondary PC ($SECONDARY_PC), trying again in 10 seconds..."
  sleep 10
done

printf "%s\n" "[Zed2-Remote] Secondary PC ($SECONDARY_PC) is connected, starting Zed2 node."
exec "$@"

