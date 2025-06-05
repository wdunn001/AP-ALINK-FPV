#!/bin/sh

# CONFIGURATION
ip_gs="192.168.0.10"       # Adresse IP fixe du GS            
bitrate=30               # Bitrate initial (en Mbps)
bitratemax=40              # Bitrate maximum (en Mbps)
bitratemin=0               # Bitrate minimum (en Mbps)

fail_count=0               # Compteur de pannes GS
#auto=True 

#txpowermax=2000 #for the moment is only for stock af1 without custom pa
txpower_index=200

#soft tx power alg (sta)
#will increase tx power at few db to get the link rock solid, is different to auto which will increase tx power at the maximum, here is just few db less or few db more around a base value 15db will become at max 17db for exemple and come to 15db after
#for exemple it will increase of index 200 more and 200 down so max 17db is not exponantial



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

get_tx_power() {
    iw dev wlan0 info 2>/dev/null | grep 'tx power' | awk 'print{3}'
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

get_dynamic_max_bitrate() {
    dbm=$(get_dbm)
    echo $(awk -v d="$dbm" 'BEGIN {
        if (d > -40)      print 40;  
        else if (d > -65) print 30;  
        else if (d > -75) print 8;   
        else if (d > -85) print 4;   
        else              print 1;   
    }')
}


get_dynamic_decrease() {
    dbm=$(get_dbm)
    echo $(awk -v d="$dbm" 'BEGIN {
        if (d > -60)      print 2;    
        else if (d > -75) print 4;  
        else if (d > -85) print 10;    
        else              print 20;    
    }')
}

txpower_get=$(get_tx_power)
# BOUCLE PRINCIPALE
while true; do
    sleep 1
    
    #init variable

    dbm=$(get_dbm)
    rtt=$(get_max_rtt)
    interval=$(get_dynamic_intervals)
    decrease=$(get_dynamic_decrease)
    bitratemax=$(get_dynamic_max_bitrate)
    #txpower_decrease=$(get_dynamic_txpower_decrease)
    #txpower_increase=$(get_dynamic_txpower_interval)
    

    if [ $? -ne 0 ]; then
        fail_count=$((fail_count + 1))
        echo "[$fail_count] Le GS $ip_gs est injoignable"
        echo " en attente de connexion"
        continue
    fi

    echo " Réponse GS — RTT max mesuré : $rtt ms"

    if [ "$dbm" -lt -80 ]; then
        bitrate=$((bitrate - decrease))
        if [ "$bitrate" -lt "$bitratemin" ]; then
            bitrate=$bitratemin
        fi
        wget -q "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
        iw wlan0 set txpower fixed $((txpower_get * 100 + txpower_index))
        echo "Bitrate réduit à $bitrate Mbps, up txpower"
    else
        if [ "$bitrate" -ge "$bitratemax" ]; then
            echo "Bitrate max dynamique atteint : $bitrate Mbps (limite : $bitratemax Mbps)"
        else
            bitrate=$((bitrate + interval))
            iw wlan0 set txpower fixed $((txpower_get * 100 - txpower_index))

            [ "$bitrate" -gt "$bitratemax" ] && bitrate=$bitratemax
            wget -q "http://localhost/api/v1/set?video0.bitrate=$((bitrate * 1000))"
            echo "Bitrate augmenté à $bitrate Mbps (+$interval Mbps)"
        fi
        
    fi
done
