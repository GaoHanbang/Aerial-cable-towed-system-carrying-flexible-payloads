#!/bin/bash
if ! grep -Fxq "setserial /dev/ttyS0 low_latency" ~/.bashrc
then
  sed -i '1isetserial /dev/ttyS0 low_latency' ~/.bashrc
fi