#!/bin/sh
# Deploy the AppDaemon apps: copy the deployable files into appdaemon/,
# which the AppDaemon container watches and auto-reloads.
set -eu
cd "$(dirname "$0")"

FILES="actrl.py statctrl.py control.py"

cp $FILES appdaemon/
echo "Deployed: $FILES"
echo "Watch the reload: docker logs -f hass-appdaemon-1"
