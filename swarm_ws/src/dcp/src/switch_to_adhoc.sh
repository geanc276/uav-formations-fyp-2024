sudo service network-manager stop

sudo ip link set wlan0 down

sudo wpa_cli terminate

sudo iwconfig wlan0 mode ad-hoc

sudo iwconfig wlan0 channel 10

sudo iwconfig wlan0 essid 'test'

sudo iwconfig wlan0 key 1234567890

sudo ip link set wlan0 up

sudo dhclient wlan0