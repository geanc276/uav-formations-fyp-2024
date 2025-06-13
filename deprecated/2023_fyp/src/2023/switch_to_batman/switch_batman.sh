#!/bin/bash

sudo modprobe batman-adv
sleep 1
sudo service network-manager stop
sleep 1
sudo ip link set wlan0 down
sleep 1
sudo iwconfig wlan0 mode ad-hoc
sleep 1
sudo iwconfig wlan0 channel 4
sleep 1
sudo iwconfig wlan0 essid 'test'
sleep 1
sudo iwconfig wlan0 key 1234567890
sleep 1
sudo ip link set wlan0 up
sleep 1
sudo batctl if add wlan0
sleep 1
sudo ip link set bat0 up
sleep 1
sudo ip addr add 192.168.0.12/24 dev bat0
