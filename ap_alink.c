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
    else if (bitrateMcs >= 3 && bitrateMcs < 10)  {
        system("wget -qO- \"http://localhost/api/v1/set?fpv.roiQp=0,0,0,0\" > /dev/null 2>&1");
        snprintf(cmd, sizeof(cmd), "echo 0x10 > %s/rate_ctl", mcspath);
        system(cmd);
    }
    else {
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






void config(const char *filename, int *BITRATE_MAX, char *WIFICARD, int *RACE) {
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
        }  
        else if (strncmp(line, "wificard=", 9) == 0) {
            sscanf(line + 9, "%63s", WIFICARD);
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
              //RSSI PATTERN *** ** * 
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
    int bitrate_min = 1;
    int bitrate_max = 0;
    int dbm_Max = -50;
    int dbm_Min = 0;
    int rssi = 0;
    char NIC[10] = {0};
    char driverpath[256] = {0};
    int RaceMode = 0;
    int histeris = 3;
    int minushisteris = -3;
    int aDb = 0;
    int currentDb = 0;
    int dbm = -100;
    //char rssi_pattern[5] = {0};

    config("/etc/ap_alink.conf", &bitrate_max, NIC, &RaceMode);
    

    if (strcmp(NIC, "8812eu2") == 0) {
        strcpy(driverpath, "/proc/net/rtl88x2eu/wlan0");

    }
    else if (strcmp(NIC, "8812au") == 0) {
        strcpy(driverpath, "/proc/net/rtl88xxau/wlan0");
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
        
        //calculation of dbm_Max dbm_Min
        

        dbm_Max = -50;      
        dbm_Min = (rssi > 55) ? -70 : (rssi >= 40 ? -55 : -53);


        double vlq = ((double)((dbm) - (dbm_Min)) / (double)((dbm_Max) - (dbm_Min))) * 100.0;
      
        printf("vlq = %.2f%%\n", vlq);
        printf("rssi = %d\n", rssi);
        printf("adb= %d\n", aDb);
        printf("dbm= %d\n", dbm);
        printf("current %d\n", currentDb);
        mspLQ(rssi);

             
        
        //MAIN LOGIC

        // RSSI fallback
            // Clamp VLQ between 0 and 100
            if ( currentDb > histeris || currentDb < minushisteris ) {
                if (vlq > 100.0 || rssi > 55) {
       
                //system("wget -qO- \"http://localhost/api/v1/set?image.saturation=50\" > /dev/null 2>&1");
                bitrate = bitrate_max;
            }
            else if (vlq < 1 || rssi < 20) {
                bitrate = bitrate_min;

                //BW 
                //system("wget -qO- \"http://localhost/api/v1/set?image.saturation=0\" > /dev/null 2>&1");
                       
            }
             else {
                bitrate = (int)(bitrate_max * vlq / 100.0);
                //system("wget -qO- \"http://localhost/api/v1/set?image.saturation=50\" > /dev/null 2>&1");
            }

                    
                set_bitrate_async(bitrate);
                set_mcs_async(bitrate, driverpath);
                

    
    
    }       


                 




        fflush(stdout);


        
        
    }

    return 0;

}
