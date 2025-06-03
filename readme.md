Ap Alink
How the script works

Alink works by pinging the gs to 192.168.0.1, it takes the highest value, if this is > 50ms, the bitrate drops by interval (5) if smaller increases the bitrate, if the max bitrate is reached, no increase, if the min bitrate is reached no decrease.

in rc.local

/etc/ap_alink.sh &
exit0
