#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#define IP_GS "192.168.0.10"
#define BITRATE_MAX 23
#define BITRATE_MIN 2
#define DBM_THRESHOLD -52
#define PING_CMD "ping -c 5 -W 1 " IP_GS
#define IW_CMD "iw dev wlan0 station dump"

int fail_count = 0;

// DBM VALUE READING
int get_dbm() {
    FILE *fp;
    char line[256];
    int dbm = -100;

    fp = popen(IW_CMD, "r");
    if (fp == NULL) {
        printf("IW ERROR\n");
        return dbm;
    }

    while (fgets(line, sizeof(line), fp)) {
        printf("read line: %s", line);

        if (strstr(line, "signal:")) {
            char *ptr = strstr(line, "signal:");
            int temp_dbm;
            if (sscanf(ptr, "signal: %d", &temp_dbm) == 1) {
                dbm = temp_dbm;
                printf("RSSI: %d dBm\n", dbm);
                break;
            } else {
                printf("parse error\n");
            }
        }
    }

    pclose(fp);
    return dbm;
}


// GS PING FOR SEE IF VTX TALK TO GS
int get_max_rtt() {
    FILE *fp;
    char line[256];
    float max_rtt = 0.0;
    fp = popen(PING_CMD, "r");
    if (fp == NULL) {
        perror("error ping");
        return -1;
    }

    while (fgets(line, sizeof(line), fp)) {
        float rtt;
        if (strstr(line, "time=")) {
            if (sscanf(line, "%*[^=]=%f", &rtt) == 1) {
                if (rtt > max_rtt) {
                    max_rtt = rtt;
                }
            }
        }
    }

    pclose(fp);
    return (int)max_rtt;
}


void mspLQ(int LQ) {
    char command[128];

    // Crée la commande avec la valeur LQ dans le texte
    snprintf(command, sizeof(command),
             "echo \"LQ (%d) &F60 &L30\" > /tmp/MSPOSD.msg",
             LQ);

    system(command);
}

void mspSIGNAL() {
    char command[128];

    // Crée la commande avec la valeur LQ dans le texte
    snprintf(command, sizeof(command),
             "echo \"LOW SIGNAL &F60 &L46\" > /tmp/MSPOSD.msg");

    system(command);
}

void *set_bitrate_thread(void *arg) {
    int bitrate_kbps = *((int *)arg);
    char command[256];
    snprintf(command, sizeof(command),
             "wget -qO- \"http://localhost/api/v1/set?video0.bitrate=%d\" > /dev/null 2>&1",
             bitrate_kbps);

    free(arg); 
    system(command);
    return NULL;
}

void set_bitrate_async(int bitrate_mbps) {
    pthread_t thread_id;
    int *bitrate_kbps = malloc(sizeof(int));
    if (bitrate_kbps == NULL) {
        perror("malloc");
        return;
    }
    *bitrate_kbps = bitrate_mbps * 1024;

    if (pthread_create(&thread_id, NULL, set_bitrate_thread, bitrate_kbps) == 0) {
        pthread_detach(thread_id); 
    } else {
        free(bitrate_kbps); 
        perror("pthread_create");
    }
}

int main() {
    int bitrate = 4;

    while (1) {
        usleep(100000); 

        int dbm = get_dbm();
        int rtt = get_max_rtt();

        if (rtt == -1) {
            fail_count++;
            printf("[%d] GS %s unreachable\n", fail_count, IP_GS);
            printf("wait for connect\n");
            continue;
        }

        if (dbm < DBM_THRESHOLD) {
            bitrate = BITRATE_MIN;
            printf("Low signal fallback (dbm=%d), setting bitrate to %d Mbps\n", dbm, bitrate);
        } else {
            bitrate = BITRATE_MAX;
            printf("Signal OK (dbm=%d), setting bitrate to %d Mbps\n", dbm, bitrate);
        }
        
        set_bitrate_async(bitrate);
        mspLQ(dbm);
        if (dbm < -65) {
            mspSIGNAL();
        }
    }

    return 0;
}