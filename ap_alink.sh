#!/bin/sh

# CONFIGURATION
ip_gs="192.168.0.10"       # Adresse IP fixe du GS
interval=5                 # Variation du bitrate en Mbps
bitrate=40               # Bitrate initial (en Mbps)
bitratemax=50              # Bitrate maximum (en Mbps)
bitratemin=0               # Bitrate minimum (en Mbps)

fail_count=0               # Compteur de pannes GS

# FONCTION : ping le GS et retourne le RTT min
get_min_rtt() {
    output=$(ping -c 5 -W 1 "$ip_gs" 2>/dev/null)

    if [ $? -ne 0 ] || [ -z "$output" ]; then
        return 1
    fi

    times=$(echo "$output" | grep -oE 'time=[0-9]+\.[0-9]+' | cut -d= -f2)

    min_rtt=99999
    for t in $times; do
        int_t=${t%.*}
        if [ "$int_t" -lt "$min_rtt" ]; then
            min_rtt=$int_t
        fi
    done

    echo "$min_rtt"
    return 0
}

get_dbm() {
    iw dev wlan0 station dump 2>/dev/null | grep 'signal:' | awk '{print $2}'
    
}

# BOUCLE PRINCIPALE
while true; do
    sleep 1

    rtt=$(get_min_rtt)
    dbm=$(get_dbm)
    if [ $? -ne 0 ]; then
        fail_count=$((fail_count + 1))
        echo "[$fail_count] Le GS $ip_gs est injoignable — baisse du bitrate"
        bitrate=$((bitrate - 1))
        if [ "$bitrate" -lt "$bitratemin" ]; then
            bitrate=$bitratemin
        fi
        cli -s .video0.bitrate $((bitrate * 1000))
        echo "  Bitrate réduit à $bitrate Mbps (en attente de connexion)"
        continue
    fi
    fail_count=0  # reset fail count
    echo " Réponse GS — RTT max mesuré : $rtt ms"

    if [ "$rtt" -lt 50 ] && [ "$dbm" -lt -50 ]; then
        bitrate=$((bitrate + 10))
        if [ "$bitrate" -lt "$bitratemin" ]; then
            bitrate=$bitratemin
        fi
        wget -s "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
        echo "signal tres bon"
    else

        if [ "$rtt" -gt 50 ] && [ "$dbm" -lt -70 ]; then
            
            bitrate=$((bitrate - 2))
            if [ "$bitrate" -lt "$bitratemin" ]; then
                bitrate=$bitratemin
            fi
            wget -s "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
            echo "bitrate down, low rssi, high ping"
    else

        if [ "$rtt" -gt 60 ] && [ "$dbm" -lt -80 ]; then
            #fallback mode
            bitrate=1
            if [ "$bitrate" -lt "$bitratemin" ]; then
                bitrate=$bitratemin
            fi
            wget -s "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
            echo "fallback"

    else
        if [ "$bitrate" -ge "$bitratemax" ]; then
            echo "Bitrate max déjà atteint ($bitrate Mbps)"
        else
            bitrate=$((bitrate + 10))
            if [ "$bitrate" -gt "$bitratemax" ]; then
                bitrate=$bitratemax
            fi
            wget -s "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
            echo "Bitrate augmenté à $bitrate Mbps"
        fi
    fi
done

#3 profile, low signal, very low signal high signal

#high signal: up bitrate 10mbps
#low signal signal : down bitrate of 5mbps
#very low signal : down bitrate 10mbps
#critics : fall back 1mbps passe tout ce script en async 
