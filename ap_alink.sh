#!/bin/sh

# CONFIGURATION
ip_gs="192.168.0.10"       # Adresse IP fixe du GS            
bitrate=10                 # Bitrate initial (en Mbps)
bitratemax=25              # Bitrate maximum (en Mbps)
bitratemin=2               # Bitrate minimum (en Mbps)

fail_count=0               # Compteur de pannes GS

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
    iw dev wlan0 station dump 2>/dev/null | grep 'signal:' | awk '{print $2}' | sort -nr | head -n 1
}

# BOUCLE PRINCIPALE
while true; do
    sleep 0.1

    dbm=$(get_dbm)
    rtt=$(get_max_rtt)

    if [ $? -ne 0 ]; then
        fail_count=$((fail_count + 1))
        echo "[$fail_count] Le GS $ip_gs est injoignable"
        echo "En attente de connexion..."
        continue
    fi

    if [ "$dbm" -lt -50 ]; then
        bitrate=$bitratemin
        echo "Low signal fallback (dbm=$dbm), setting bitrate to $bitrate Mbps"
        
    else 
        bitrate=$bitratemax
        echo "Signal OK (dbm=$dbm), setting bitrate to $bitrate Mbps"
    fi

    # Set bitrate (convert Mbps to Kbps)
    curl -q "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1024))"
done
