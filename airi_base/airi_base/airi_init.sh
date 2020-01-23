#!/bin/bash

CONFUSB="auto eth1
iface eth1 inet static
address 10.10.10.1
netmask 255.255.255.0"
echo "$CONFUSB" > /etc/network/interfaces
ifup -a
echo "ETH1 has been configured with IP 10.10.10.1/24 at $(date +%F) $(date +%T)" >> /home/airi/Documents/airi_scripts/log/airi_logs
