#!/bin/sh

# CONFIGURATION
ip_gs="192.168.0.10"       # Adresse IP fixe du GS            
bitrate=40               # Bitrate initial (en Mbps)
bitratemax=60              # Bitrate maximum (en Mbps)
bitratemin=0               # Bitrate minimum (en Mbps)

fail_count=0               # Compteur de pannes GS

# FONCTION : ping le GS et retourne le RTT max
get_max_rtt() {
    output=$(ping -c 5 -W 1 "$ip_gs" 2>/dev/null)

    if [ $? -ne 0 ] || [ -z "$output" ]; then
        return 1
    fi

    times=$(echo "$output" | grep -oE 'time=[0-9]+\.[0-9]+' | cut -d= -f2)

    max_rtt=0
    for t in $times; do
        int_t=${t%.*}
        if [ "$int_t" -gt "$max_rtt" ]; then
            max_rtt=$int_t
        fi
    done

    echo "$max_rtt" 
    return 0
}
get_dbm() {
    iw dev wlan0 station dump 2>/dev/null | grep 'signal:' | awk '{print $2}'
    
}


get_dynamic_interval() {
    dbm=$(get_dbm)
    echo $(awk -v d="$dbm" 'BEGIN {
        if (d > -40)      print 8;
        else if (d > -60) print 6;
        else if (d > -75) print 4;
        else if (d > -85) print 2;
        else              print 1;
    }')
}

get_dynamic_max_bitrate() {
    dbm=$(get_dbm)
    echo $(awk -v d="$dbm" 'BEGIN {
        if (d > -75)      print 60;  
        else if (d > -65) print 30;  
        else if (d > -75) print 8;   
        else if (d > -85) print 4;   
        else              print 1;   
    }')
}

# BOUCLE PRINCIPALE
while true; do
    sleep 1
    dbm=$(get_dbm)
    rtt=$(get_max_rtt)
    interval=$(get_dynamic_intervals)
    bitratemax=$(get_dynamic_max_bitrate)
    if [ $? -ne 0 ]; then
        fail_count=$((fail_count + 1))
        echo "[$fail_count] Le GS $ip_gs est injoignable"
        echo " en attente de connexion"
        continue
    fi

    echo " Réponse GS — RTT max mesuré : $rtt ms"

    if [ "$dbm" -lt -80 ]; then
        bitrate=$((bitrate - 2))
        if [ "$bitrate" -lt "$bitratemin" ]; then
            bitrate=$bitratemin
        fi
        curl -s "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
        echo "Bitrate réduit à $bitrate Mbps (RTT > 100ms)"
    else
        if [ "$bitrate" -ge "$bitratemax" ]; then
            echo "Bitrate max dynamique atteint : $bitrate Mbps (limite : $bitratemax Mbps)"
        else
            bitrate=$((bitrate + interval))
            [ "$bitrate" -gt "$bitratemax" ] && bitrate=$bitratemax
            curl -s "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
            echo "Bitrate augmenté à $bitrate Mbps (+$interval Mbps)"
        fi
        
    fi
done
