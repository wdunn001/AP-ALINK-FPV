#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>



void autopower() {
    system("iw wlan0 set tx power auto");
}




typedef struct {
    int bitrateMcs;
    char mcspath[256];
} mcs_arg_t;

void *set_mcs_thread(void *arg) {
    mcs_arg_t *data = (mcs_arg_t *)arg;
    int bitrateMcs = data->bitrateMcs;
    char *mcspath = data->mcspath;
    char cmd[512];
    if (bitrateMcs >= 1 && bitrateMcs < 3) {
        system("wget -qO- \"http://localhost/api/v1/set?fpv.roiQp=30,0,0,30\" > /dev/null 2>&1");
        snprintf(cmd, sizeof(cmd), "echo 0x0c > %s/rate_ctl", mcspath);
        system(cmd);
    }
    else  {
        system("wget -qO- \"http://localhost/api/v1/set?fpv.roiQp=0,0,0,0\" > /dev/null 2>&1");
        snprintf(cmd, sizeof(cmd), "echo 0xFF > %s/rate_ctl", mcspath);
        system(cmd);
    }

    free(arg);
    return NULL;
}



void set_mcs_async(int bitrateMcs, const char *mcspath) {
    pthread_t thread_id;
    mcs_arg_t *arg = malloc(sizeof(mcs_arg_t));
    if (!arg) return;

    arg->bitrateMcs = bitrateMcs;
    snprintf(arg->mcspath, sizeof(arg->mcspath), "%s", mcspath);

    if (pthread_create(&thread_id, NULL, set_mcs_thread, arg) == 0) {
        pthread_detach(thread_id);
    } else {
        free(arg);
    }
}






void config(const char *filename, int *BITRATE_MAX, int *BITRATE_MIN, int *DBM_MAX, int *DBM_MIN, int *auto_power, char *WIFICARD, int *ACS, int *MCS, int *RACE) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("No config file found\n");
        exit(1);
    }

    char line[128];
    char wificard[64] = {0};

    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "bitrate_max=", 12) == 0) {
            sscanf(line + 12, "%d", BITRATE_MAX);
        } else if (strncmp(line, "bitrate_min=", 12) == 0) {
            sscanf(line + 12, "%d", BITRATE_MIN);
        } else if (strncmp(line, "dbmMax=", 7) == 0) {
            sscanf(line + 7, "%d", DBM_MAX);
        } else if (strncmp(line, "dbmMin=", 7) == 0) {
            sscanf(line + 7, "%d", DBM_MIN);
        } else if (strncmp(line, "autoPower=", 10) == 0) {
            sscanf(line + 10, "%d", auto_power);
        }
        else if (strncmp(line, "wificard=", 9) == 0) {
            sscanf(line + 9, "%63s", WIFICARD);
        }
        else if (strncmp(line, "acs=", 4) == 0) {
            sscanf(line + 4, "%d", ACS);
        }
        else if (strncmp(line, "mcs=", 4) == 0) {
            sscanf(line + 4, "%d", MCS);
        }
        else if (strncmp(line, "LowLatency=", 11) == 0) {
            sscanf(line + 11, "%d", RACE);
        }
    }

    fclose(fp);
}


int get_dbm() {
    int pipefd[2];
    pid_t pid;
    char buffer[256];
    int dbm = -100;

    if (pipe(pipefd) == -1) {
        perror("pipe");
        return dbm;
    }

    if ((pid = fork()) == -1) {
        perror("fork");
        return dbm;
    }

    if (pid == 0) { // Child
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        close(pipefd[1]);
        execlp("iw", "iw", "dev", "wlan0", "station", "dump", NULL);
        perror("execlp failed");
        exit(1);
    } else { // Parent
        close(pipefd[1]);
        FILE *fp = fdopen(pipefd[0], "r");
        if (!fp) {
            perror("fdopen failed");
            close(pipefd[0]);
            waitpid(pid, NULL, 0);
            return dbm;
        }
        while (fgets(buffer, sizeof(buffer), fp)) {
            // Exemple typique: "signal: -42 dBm"
            if (strstr(buffer, "signal:")) {
                int temp;
                if (sscanf(buffer, " signal: %d dBm", &temp) == 1) {
                    dbm = temp;
                    break;
                }
            }
        }
        fclose(fp);
        waitpid(pid, NULL, 0);
    }
    return dbm;
}

void get_acs(const char *acsdriverpath) {
    FILE *fp;
    char buffer[256];
    char cmd[256];
    char path[512];
    int channel = 0;
    int freq = 0;

    // Liste des fréquences valides
    int valid_channels[] = {
        5180,5200,5220,5240,5260,5280,5300,5320,
        5500,5520,5540,5560,5580,5600,5620,5640,
        5660,5680,5700,5720,5745,5765,5785,5805,5825
    };
    int n_valid = sizeof(valid_channels)/sizeof(valid_channels[0]);

    // Lance le scan avec timeout
    system("timeout 20 iw wlan0 scan");

    // Ouvre le fichier ACS généré par le driver
    snprintf(path, sizeof(path), "%s/acs", acsdriverpath);

    fp = fopen(path, "r");
    if (!fp) {
        perror("fopen");
        return;
    }

    // Cherche le meilleur canal 5G
    while (fgets(buffer, sizeof(buffer), fp)) {
        char *pos = strstr(buffer, "Best 5G Channel");
        if (pos) {
            if (sscanf(pos, "Best 5G Channel : %d %%", &channel) != 1) {
                sscanf(pos, "Best 5G Channel: %d %%", &channel);
            }
            break;
        }
    }
    fclose(fp);

    // Convertit canal en fréquence
    freq = 5000 + 5 * channel;

    // Trouve la fréquence valide la plus proche
    int closest_freq = valid_channels[0];
    int min_diff = abs(freq - closest_freq);
    for(int i = 1; i < n_valid; i++) {
        int diff = abs(freq - valid_channels[i]);
        if(diff < min_diff) {
            min_diff = diff;
            closest_freq = valid_channels[i];
        }
    }
    freq = closest_freq;

    // Remplace la ligne frequency=... dans wpa_supplicant.conf
    snprintf(cmd, sizeof(cmd),
        "sed -i '/^\\s*frequency=/c\\frequency=%d' /tmp/wpa_supplicant.conf",
        freq);

    if (system(cmd) != 0) {
        perror("system");
        return;
    }

    printf("Channel updated to %d (frequency=%d MHz)\n", channel, freq);

    // Applique les changements
    system("killall -HUP wpa_supplicant");
}




int get_rssi(const char *readcmd) {
    FILE *fp;
    char buffer[256];
    int rssi_percent = 0;
    char path[512];

    // Construit le chemin complet vers sta_tp_info
    snprintf(path, sizeof(path), "%s/sta_tp_info", readcmd);

    fp = fopen(path, "r");
    if (!fp) {
        perror("fopen");
        return rssi_percent;
    }
    while (fgets(buffer, sizeof(buffer), fp)) {
        char *pos = strstr(buffer, "rssi");
        if (pos) {
            // Tolère espaces optionnels autour du ':'
            if (sscanf(pos, "rssi : %d %%", &rssi_percent) != 1) {
                sscanf(pos, "rssi: %d %%", &rssi_percent);
            }
            break;
        }
    }

    fclose(fp);
    return rssi_percent;
}




void mspLQ(int rssi_osd) {
    char command[128];
    snprintf(command, sizeof(command),
             "echo \"VLQ %d &B &F60 &L30\" > /tmp/MSPOSD.msg",
              rssi_osd);
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
    if (!bitrate_kbps) {
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
    int bitrate_min = 0;
    int bitrate_max = 0;
    int dbm_Max = 0;
    int dbm_Min = 0;
    int autopw = 0;
    int fail_count = 0;
    int rssi = 0;
    char NIC[10] = {0};
    int Acs = 0;
    char driverpath[256] = {0};
    int MCS = 0;
    int RaceMode = 0;
    int histeris = 3;
    int minushisteris = -3;
    int aDb = 0;
    int currentDb = 0;
    int dbm = -100;
    config("/etc/ap_alink.conf", &bitrate_max, &bitrate_min, &dbm_Max, &dbm_Min, &autopw, NIC, &Acs, &MCS, &RaceMode);
    

    if (strcmp(NIC, "8812eu2") == 0) {
        strcpy(driverpath, "/proc/net/rtl88x2eu/wlan0");
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "sed -i 's/MCS=.*/MCS=1/' /etc/ap_alink.conf");
        system(cmd);
    }
    else if (strcmp(NIC, "8812au") == 0) {
        strcpy(driverpath, "/proc/net/rtl88xxau/wlan0");
    }

    if (autopw != 1 && autopw != 0) {
        printf("invalid value for autopower\n");
    } else if (autopw == 1) {
        autopower();
    } else {
        printf("tx power manual\n");
    }

    if (Acs != 1 && Acs != 0) {
        printf("invalid value for acs\n");
    } else if (Acs == 1) {
        get_acs(driverpath);
        printf("ACS ENABLE");
    } else {
        printf("acs disable\n");
    }
    
    if (RaceMode != 1 && RaceMode != 0) {
        printf("invalid value for racemode\n");
    } else if (RaceMode == 1) {
       
        printf("RACEMODE ENABLE");
        char cmd1[512];
        snprintf(cmd1, sizeof(cmd1), "echo 20 > %s/ack_timeout", driverpath);

    } else {
        printf("racemode disable\n");

        
    }
    
    while (1) {
        currentDb = dbm - aDb;
        int dbm = get_dbm();
        aDb = dbm; 
        int rssi = get_rssi(driverpath);
        

        double vlq = ((double)((dbm) - (dbm_Min)) / (double)((dbm_Max) - (dbm_Min))) * 100.0;
      
        printf("vlq = %.2f%%\n", vlq);
        printf("rssi = %d\n", rssi);
        printf("adb= %d\n", aDb);
        printf("dbm= %d\n", dbm);
        printf("current %d\n", currentDb);
        mspLQ(rssi);
        
        // RSSI fallback
            // Clamp VLQ between 0 and 100
            if ( currentDb > histeris || currentDb < minushisteris ) {
                if (vlq > 100.0 || rssi > 55) {
                bitrate = bitrate_max;
            }
            else if (vlq < 1 || rssi < 20) {
                bitrate = bitrate_min;
            }
             else {
                bitrate = (int)(bitrate_max * vlq / 100.0);
            
            }
                set_bitrate_async(bitrate);
        }       set_mcs_async(bitrate, driverpath);
        




        fflush(stdout);


        
        
    }

    return 0;

}
