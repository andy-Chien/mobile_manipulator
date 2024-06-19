#!/bin/sh
echo $PASSWORD | sudo -S sysctl net.ipv4.ipfrag_time=3
echo $PASSWORD | sudo -S sysctl net.ipv4.ipfrag_high_thresh=134217728
echo $PASSWORD | sudo -S sysctl -w net.core.rmem_max=2147483647
# clearup
PASSWORD=
USER_NAME=
unset PASSWORD
unset USER_NAME

# Keep the container alive
tail -f /dev/null