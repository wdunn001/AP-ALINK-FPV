Ap Alink
How install

copy ap_alink to /usr/bin
do in putty chmod +x /usr/bin/ap_alink

in rc.local

ap_alink &
exit 0

only tested with af1 and eu2 card on ssc338q soc

progress and new coming feature

adaptive bitrate DONE

ACS script on kwad side SOON

wifi card profile ( in progress )

msp osd Vlq indicator DONE ( in progress )

range / penetration test of wifililnk 2 : https://www.youtube.com/watch?v=SV-0z6YXHIM

range / penetrationt test of 8812 af1 ( emax wyverne link ) : will be made once wifi card profile is done, value in the script will not work very well on af1 card, need lower dbm treshold at -48 and compile if you want give a try

include ap_alink.conf

you can set bitratemax, bitratemin and dbm_treshold
copy it in /etc/
the stok setting is good for eu2
for af1 card use this

bitrate_max=22
bitrate_min=2
dbm_threshold=-47
