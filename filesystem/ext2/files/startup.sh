#!/bin/sh
cd /mnt/kiss

./run_qt_app.sh ./cbcui -qws -display VNC:LinuxFb &

if [ ! -e /tmp/cbc_run_startup ]; then
  touch /tmp/cbc_run_startup
  for codedir in /mnt/user/code/*; do
    script="$codedir/chumby_startup.sh"
    if [ -e "$script" ]; then
      sh $script
    fi
  done
fi

