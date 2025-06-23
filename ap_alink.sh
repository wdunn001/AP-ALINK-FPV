#!/bin/sh

# CONFIGURATION
ip_gs="192.168.0.10"       # Adresse IP fixe du GS            
bitrate=10               # Bitrate initial (en Mbps)
bitratemax=14              # Bitrate maximum (en Mbps)
bitratemin=0               # Bitrate minimum (en Mbps)

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
    iw dev wlan0 station dump 2>/dev/null | grep 'signal:' | awk '{print $2}'
    
}



get_dynamic_txpower_interval() {
    powertx=$(get_tx_power)
    echo $(awk -v d="$powertx" 'BEGIN {
        if (d > -40)      print 100;
        else if (d > -60) print 200;
        else if (d > -75) print 300;
        else if (d > -85) print 500;
        else              print 700;
    }')
}



get_dynamic_txpower_decrease() {
    powertx=$(get_tx_power)
    echo $(awk -v d="$powertx" 'BEGIN {
        if (d > -40)      print 700;
        else if (d > -60) print 400;
        else if (d > -75) print 300;
        else if (d > -85) print 200;
        else              print 100;
    }')
}


get_dynamic_interval() {
    dbm=$(get_dbm)
    echo $(awk -v d="$dbm" 'BEGIN {
        if (d > -40)      print 8;
        else if (d > -65) print 6;
        else if (d > -75) print 4;
        else if (d > -85) print 2;
        else              print 1;
    }')
}


get_dynamic_decrease() {
    dbm=$(get_dbm)
    echo $(awk -v d="$dbm" 'BEGIN {
        if (d > -60)      print 2;    
        else if (d > -75) print 5;  
        else if (d > -85) print 15;    
        else              print 20;    
    }')
}

# BOUCLE PRINCIPALE
while true; do
    sleep 0.1
    

    dbm=$(get_dbm)
    rtt=$(get_max_rtt)
    interval=$(get_dynamic_interval)
    decrease=$(get_dynamic_decrease)
    

    if [ $? -ne 0 ]; then
        fail_count=$((fail_count + 1))
        echo "[$fail_count] Le GS $ip_gs est injoignable"
        echo " en attente de connexion"
        continue
    fi

    if [ "$dbm" -lt -62 ]; then

        bitrate=$((bitrate - decrease))
        if [ "$bitrate" -lt "$bitratemin" ]; then
            bitrate=$bitratemin
            echo "Bitrate down to $bitrate Mbps (+$interval Mbps), current lq is $dbm"
        fi
        wget -q "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1024))"
        
        echo "Bitrate réduit à $bitrate Mbps, up txpower"
    else
        if [ "$bitrate" -ge "$bitratemax" ]; then
            echo "Bitrate max reach : $bitrate Mbps (max : $bitratemax Mbps), current lq is $dbm"
        else

            bitrate=$((bitrate + interval))
            

            [ "$bitrate" -gt "$bitratemax" ] && bitrate=$bitratemax
            wget -q "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1024))"
            echo "Bitrate up to $bitrate Mbps (+$interval Mbps), current lq is $dbm"
        fi
        
    fi
done
