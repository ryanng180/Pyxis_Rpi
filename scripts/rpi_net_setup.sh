#!/bin/bash

echo "Setting static Ethernet IP on Raspberry Pi..."

sudo ip addr flush dev eth0
sudo ip addr add 10.42.0.2/24 dev eth0
sudo ip link set eth0 up

echo "Raspberry Pi Ethernet configured."
ip addr show eth0
