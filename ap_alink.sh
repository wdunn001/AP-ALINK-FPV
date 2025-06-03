#!/bin/sh

# CONFIGURATION
ip_gs="192.168.0.10"       # Adresse IP fixe du GS
interval=4                 # Variation du bitrate en Mbps
bitrate=40               # Bitrate initial (en Mbps)
bitratemax=50              # Bitrate maximum (en Mbps)
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

# BOUCLE PRINCIPALE
while true; do
    sleep 1
    dbm=$(get_dbm)
    rtt=$(get_max_rtt)
    if [ $? -ne 0 ]; then
        fail_count=$((fail_count + 1))
        echo "[$fail_count] Le GS $ip_gs est injoignable — baisse du bitrate"
        bitrate=$((bitrate - interval))
        if [ "$bitrate" -lt "$bitratemin" ]; then
            bitrate=$bitratemin
        fi
        cli -s .video0.bitrate $((bitrate * 1000))
        echo "  Bitrate réduit à $bitrate Mbps (en attente de connexion)"
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
            echo "Bitrate max déjà atteint ($bitrate Mbps)"
        else
            bitrate=$((bitrate + interval))
            if [ "$bitrate" -gt "$bitratemax" ]; then
                bitrate=$bitratemax
            fi
            curl -s "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
            echo "Bitrate augmenté à $bitrate Mbps"
        fi
    fi
done
