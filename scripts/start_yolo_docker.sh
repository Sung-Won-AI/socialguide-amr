#!/usr/bin/env bash
set -Eeo pipefail
docker run --rm \
  --name socialguide-amr-yolo \
  --runtime=nvidia \
  --network=host \
  --ipc=host \
  -v /dev/bus/usb:/dev/bus/usb \
  socialguide-amr-oak-yolo:jp6 "$@"
