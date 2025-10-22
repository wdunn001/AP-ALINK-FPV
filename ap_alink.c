#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sched.h>



void autopower() {
    (void)system("iw wlan0 set tx power auto");
}

// Set real-time priority for ultra-high performance racing VTX
int set_realtime_priority() {
    struct sched_param param;
    
    // Set high real-time priority
    param.sched_priority = 50;  // High priority (1-99 range)
    
    // Use SCHED_FIFO for consistent timing
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("Failed to set real-time priority");
        printf("Note: Real-time priority requires root privileges\n");
        printf("Run with: sudo ./ap_alink\n");
        return -1;
    }
    
    printf("Real-time priority set successfully (SCHED_FIFO, priority 50)\n");
    return 0;
}

// Frame-sync timing for optimal video quality
static struct timespec last_frame_time;
static int target_fps = 120;  // Default racing frame rate
static long frame_interval_ns = 0;

// Initialize frame-sync timing
void init_frame_sync(int fps) {
    target_fps = fps;
    frame_interval_ns = 1000000000L / target_fps;  // Convert to nanoseconds
    
    // Get current time as starting point
    clock_gettime(CLOCK_MONOTONIC, &last_frame_time);
    
    printf("Frame-sync initialized: %d FPS (%.2f ms interval)\n", 
           target_fps, frame_interval_ns / 1000000.0);
}

// Check if it's time for next frame (returns true if frame should be processed)
int is_frame_time() {
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    long elapsed_ns = (current_time.tv_sec - last_frame_time.tv_sec) * 1000000000L + 
                      (current_time.tv_nsec - last_frame_time.tv_nsec);
    
    if (elapsed_ns >= frame_interval_ns) {
        last_frame_time = current_time;
        return 1;  // Time for next frame
    }
    
    return 0;  // Not yet time for next frame
}

// Raw HTTP GET implementation (much faster than wget)
int http_get(const char *path) {
    int s;
    struct sockaddr_in addr;
    char req[256];
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };

    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) return -1;

    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(80);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(s);
        return -1;
    }

    snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\n\r\n", path);
    if (send(s, req, strlen(req), 0) < 0) {
        close(s);
        return -1;
    }

    char buf[64];
    while (recv(s, buf, sizeof(buf), 0) > 0);

    close(s);
    return 0;
}




typedef struct {
    int bitrateMcs;
    char mcspath[256];
} mcs_arg_t;

// Worker thread infrastructure
typedef enum {
    CMD_SET_BITRATE,
    CMD_SET_MCS
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    union {
        int bitrate_kbps;
        mcs_arg_t mcs_data;
    } data;
} worker_cmd_t;

static pthread_t worker_thread;
static pthread_mutex_t worker_mutex = PTHREAD_MUTEX_INITIALIZER;
static sem_t worker_sem;
static worker_cmd_t pending_cmd;
static int worker_running = 0;

// Worker thread that processes API calls
void *worker_thread_func(void *arg) {
    char cmd[512];
    
    while (worker_running) {
        // Wait for work
        sem_wait(&worker_sem);
        
        pthread_mutex_lock(&worker_mutex);
        worker_cmd_t cmd_to_process = pending_cmd;
        pthread_mutex_unlock(&worker_mutex);
        
        switch (cmd_to_process.type) {
            case CMD_SET_BITRATE: {
                int bitrate_kbps = cmd_to_process.data.bitrate_kbps;
                char path[128];
                snprintf(path, sizeof(path), "/api/v1/set?video0.bitrate=%d", bitrate_kbps);
                http_get(path);
                break;
            }
            case CMD_SET_MCS: {
                mcs_arg_t *data = &cmd_to_process.data.mcs_data;
                int bitrateMcs = data->bitrateMcs;
                char *mcspath = data->mcspath;
                
                if (bitrateMcs >= 1 && bitrateMcs < 3) {
                    http_get("/api/v1/set?fpv.roiQp=30,0,0,30");
                    snprintf(cmd, sizeof(cmd), "echo 0x0c > %s/rate_ctl", mcspath);
                    (void)system(cmd);
                }
                else if (bitrateMcs >= 3 && bitrateMcs < 10) {
                    http_get("/api/v1/set?fpv.roiQp=0,0,0,0");
                    snprintf(cmd, sizeof(cmd), "echo 0x10 > %s/rate_ctl", mcspath);
                    (void)system(cmd);
                }
                else {
                    http_get("/api/v1/set?fpv.roiQp=0,0,0,0");
                    snprintf(cmd, sizeof(cmd), "echo 0xFF > %s/rate_ctl", mcspath);
                    (void)system(cmd);
                }
                break;
            }
        }
    }
    return NULL;
}



void set_mcs_async(int bitrateMcs, const char *mcspath) {
    if (!worker_running) return;
    
    pthread_mutex_lock(&worker_mutex);
    pending_cmd.type = CMD_SET_MCS;
    pending_cmd.data.mcs_data.bitrateMcs = bitrateMcs;
    snprintf(pending_cmd.data.mcs_data.mcspath, sizeof(pending_cmd.data.mcs_data.mcspath), "%s", mcspath);
    pthread_mutex_unlock(&worker_mutex);
    
    sem_post(&worker_sem);
}






void config(const char *filename, int *BITRATE_MAX, char *WIFICARD, int *RACE, int *FPS) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("No config file found\n");
        exit(1);
    }

    char line[128];

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
        else if (strncmp(line, "fps=", 4) == 0) {
            sscanf(line + 4, "%d", FPS);
        }
    }

    fclose(fp);
}


int get_dbm() {
    FILE *fp;
    char line[256];
    int dbm = -100;
    char *token;
    int field_count = 0;

    // Open /proc/net/wireless directly - much faster than fork/exec
    fp = fopen("/proc/net/wireless", "r");
    if (!fp) {
        perror("Failed to open /proc/net/wireless");
        return dbm;
    }

    // Skip header lines (first 2 lines)
    if (fgets(line, sizeof(line), fp) == NULL) goto cleanup;
    if (fgets(line, sizeof(line), fp) == NULL) goto cleanup;

    // Read the wlan0 line
    if (fgets(line, sizeof(line), fp) != NULL) {
        // Parse the line: "wlan0: 0000 1234 5678 90ab  cdef  1234  5678  90ab  cdef"
        // Field 2 (index 2) is the signal level in dBm
        token = strtok(line, " \t");
        field_count = 0;
        
        while (token != NULL && field_count < 3) {
            if (field_count == 2) {
                // Convert signal level to dBm (it's in centi-dBm, so divide by 100)
                dbm = atoi(token) / 100;
                break;
            }
            token = strtok(NULL, " \t");
            field_count++;
        }
    }

cleanup:
    fclose(fp);
    return dbm;
}



int get_rssi(const char *readcmd) {
    static FILE *fp = NULL;
    static char last_path[512] = {0};
    char buffer[256];
    int rssi_percent = 0;
    char path[512];

    // Construit le chemin complet vers sta_tp_info
    // English:Builds the full path to sta_tp_info
    snprintf(path, sizeof(path), "%s/sta_tp_info", readcmd);

    // Only reopen if path changed or file not open
    if (fp == NULL || strcmp(path, last_path) != 0) {
        if (fp != NULL) {
            fclose(fp);
        }
        fp = fopen(path, "r");
        if (!fp) {
            perror("fopen");
            return rssi_percent;
        }
        strcpy(last_path, path);
    } else {
        // Rewind to beginning for fresh read
        rewind(fp);
    }

    while (fgets(buffer, sizeof(buffer), fp)) {
        char *pos = strstr(buffer, "rssi");
        if (pos) {
            // Allows optional spaces around the ':'
            if (sscanf(pos, "rssi : %d %%", &rssi_percent) != 1) {
                sscanf(pos, "rssi: %d %%", &rssi_percent);
            }
            break;
        }
    }

    return rssi_percent;
}




void mspLQ(int rssi_osd) {
    char command[128];
    snprintf(command, sizeof(command),
             "echo \"VLQ %d &B &F60 &L30\" > /tmp/MSPOSD.msg",
              rssi_osd);
              //RSSI PATTERN *** ** * 
    (void)system(command);
}


void set_bitrate_async(int bitrate_mbps) {
    if (!worker_running) return;
    
    pthread_mutex_lock(&worker_mutex);
    pending_cmd.type = CMD_SET_BITRATE;
    pending_cmd.data.bitrate_kbps = bitrate_mbps * 1024;
    pthread_mutex_unlock(&worker_mutex);
    
    sem_post(&worker_sem);
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
    int histeris = 1;      // Reduced hysteresis for faster racing response
    int minushisteris = -1; // Reduced hysteresis for faster racing response
    int aDb = 0;
    int currentDb = 0;
    int dbm = -100;
    int loop_counter = 0;  // Counter to reduce I/O frequency
    int target_fps = 120;  // Default racing frame rate
    //char rssi_pattern[5] = {0};

    config("/etc/ap_alink.conf", &bitrate_max, NIC, &RaceMode, &target_fps);
    
    // Set real-time priority for ultra-high performance racing VTX
    set_realtime_priority();
    
    // Initialize frame-sync timing
    init_frame_sync(target_fps);
    
    // Initialize worker thread
    sem_init(&worker_sem, 0, 0);
    worker_running = 1;
    if (pthread_create(&worker_thread, NULL, worker_thread_func, NULL) != 0) {
        perror("Failed to create worker thread");
        exit(1);
    }
    

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
        //SET BITRATE MAX 4MBPS
        bitrate_max=4;
        //SET BUFFER SETTING
        (void)system("sysctl -w net.core.rmem_default=16384");
        (void)system("sysctl -w net.core.rmem_max=65536");
        (void)system("sysctl -w net.core.wmem_default=16384");
        (void)system("sysctl -w net.core.wmem_max=65536");
        (void)system("ifconfig wlan0 txqueuelen 100");
        (void)system("sysctl -w net.core.netdev_max_backlog=64");
        //SET 960x720120FPS
        (void)system("wget -qO- \"http://localhost/api/v1/set?video0.size=1280x720\" > /dev/null 2>&1");
        (void)system("wget -qO- \"http://localhost/api/v1/set?video0.fps=120\" > /dev/null 2>&1");
        (void)system("wget -qO- \"http://localhost/api/v1/set?isp.exposure=11\" > /dev/null 2>&1");                
        
    } else {
        printf("racemode disable\n");

        
    }
    
    while (1) {
        // Frame-sync timing: only process when it's time for next frame
        if (!is_frame_time()) {
            usleep(100);  // Short sleep to avoid busy waiting
            continue;
        }
        
        loop_counter++;
        
        // Only read signal data every 5 frames to reduce I/O overhead
        // This gives us frame-synced control with reduced signal reading frequency
        if (loop_counter % 5 == 0) {
            currentDb = dbm - aDb;
            dbm = get_dbm();
            aDb = dbm; 
            rssi = get_rssi(driverpath);
        }
        
        //calculation of dbm_Max dbm_Min
        

        dbm_Max = -50;      
        dbm_Min = (rssi > 55) ? -70 : (rssi >= 40 ? -55 : -53);


        double vlq = ((double)((dbm) - (dbm_Min)) / (double)((dbm_Max) - (dbm_Min))) * 100.0;
      
#ifdef DEBUG
        // Only show debug output when we read new signal data
        if (loop_counter % 5 == 0) {
            printf("vlq = %.2f%%\n", vlq);
            printf("rssi = %d\n", rssi);
            printf("adb= %d\n", aDb);
            printf("dbm= %d\n", dbm);
            printf("current %d\n", currentDb);
        }
#endif
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
                
#ifdef DEBUG
        fflush(stdout);
#endif
        // Frame-sync timing handles the delay automatically
        }
    }
    
    // Cleanup worker thread
    worker_running = 0;
    sem_post(&worker_sem);  // Wake up worker to exit
    pthread_join(worker_thread, NULL);
    sem_destroy(&worker_sem);

    return 0;
}
