/*
 * Antec (ASETEK) Kuhler-920 Basic Driver [usermode] daemon?
 * Alpha Geek : alphageek@hotmail.com
 *
 * with code and libraries from: libusb-1.0 Daniel Drake (LGPL)
 *
*/

/*
    This file is part of Kuhler 920 Linux Driver Package.

    Kuhler 920 Linux Driver Package is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Kuhler Linux Driver Package is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Kuhler 920 Linux Driver Package.  If not, see <http://www.gnu.org/licenses/>.
*/

// *********************************************************************************************************//
// No warranties, expressed or implied, your mileage may vary, you might fry your cpu, your Kuhler, or both.//
// information taken from clean-room reverse engineering the USB packets from the Kuhler-920                //
// nothing taken from the Kuhler software directly                                                          //
// *********************************************************************************************************//


#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <fcntl.h>
#include <syslog.h>

#include "../common/inc/kuhler_920.h"
#include "/usr/include/libusb-1.0/libusb.h"

// Globals
static struct libusb_device_handle *devh = NULL;
//struct sigaction sigact;

// Named Semaphores
sem_t* cp_sem=NULL;
sem_t* sp_sem=NULL;
sem_t* kuhler_hw_busy_sem=NULL;

// Unnamed Shared Memory
key_t shm_key_sp = shm_key_sp_val;
key_t shm_key_cp = shm_key_cp_val;
	
int shm_id_sp;
int shm_id_cp;

struct control_packet* cp_ptr;
struct status_packet* sp_ptr;


// Pointer - Since this is a permutable structure now
struct kuhler_data* kd_ptr;


// MUTEX Lock
pthread_mutex_t kuhler_mutex;

static void* update_thread(void* ptr) {
	int j = 0;
	while (j == 0) {
		send_status_packet(cp_ptr->mode,kd_ptr);
		sleep(1);
		get_status_packet(sp_ptr);
		update_histogram(sp_ptr);
		//sem_post(sp_sem);
		sleep(10);
	}
}


static int find_kuhler920(void)
{
	// Open the Kuhler H20 - 920
	// Vendor ID : 2433 - ASETEK
	// Product ID : b111 - LC690 
	// Not an Exhaustive Check

	
	//devh = libusb_open_device_with_vid_pid(NULL, 0x2433, 0xb111);
	devh = libusb_open_device_with_vid_pid(NULL, KUHLER_920_USB_VID, KUHLER_920_USB_PID);
	return devh ? 0 : -EIO;
}

static int init_kuhler920(void)
{

	int r,r1;
	unsigned char status_var;
	unsigned char* status;
	status = &status_var;

	// This is the first initialization packet
	// it is expected that this returns a SHORT READ -32,0
	r = libusb_control_transfer(devh, 0x40, 0x00, 0xffff, 0x0000, "", 0, 0);
	//if (r == -32) {
	if (r == -9) {
	} else {
		syslog(LOG_ERR,"Kuhler 920 Not Found, error code %d\n",r);
	}

	// 2nd Control Packet
	// This should return 0,32
        r = libusb_control_transfer(devh, 0x40, 0x02, 0x0002, 0x0000, "", 0, 0);
        //if (r < 0) {
        //        fprintf(stderr, "read hwstat error %d\n", r);
        //        return r;
        //}
        //if ((unsigned int) r < 1) {
        //        fprintf(stderr, "short read (%d)\n", r);
        //        return -1;
       // }

	//printf("hwstat reads %02x\n", *status);
	return r;
}

static int deinit_kuhler920(void) {
	int r;
	unsigned char status_var;
	unsigned char* status;
	status = &status_var;
	// This is expected to return a SHORT READ
        r = libusb_control_transfer(devh, 0x40, 0x02, 0x0004, 0x0000, "", 0, 200);
        if (r < 0) {
                fprintf(stderr, "read hwstat error %d\n", r);
                return r;
        }
        if ((unsigned int) r < 1) {
                //fprintf(stderr, "short read (%d)\n", r);
                return -1;
        }
	return 0;
}

int send_status_packet(int mode, struct kuhler_data* kd) {
//int libusb_bulk_transfer 	(struct libusb_device_handle*  	dev_handle,
//		unsigned char  	endpoint,
//		unsigned char *  	data,
//		int  	length,
//		int *  	transferred,
//		unsigned int  	timeout	 
//	) 	

	int dlength_real;
	int* dlength;
	dlength = &dlength_real;

	// ------------------------------
	// Lock the Hardware
	// ------------------------------
	pthread_mutex_lock(&kuhler_mutex);	
	libusb_bulk_transfer(devh,0x02,kd->usb_status_packet[mode],0x20,dlength,200);

	// ------------------------------
	// unLock the Hardware
	// ------------------------------
	pthread_mutex_unlock(&kuhler_mutex);	

	return 0;
}

void get_status_packet(struct status_packet* sp) {

	int dlength_real;
	int* dlength;

	dlength = &dlength_real;

	// Data is Stored Here, 32 Bytes	
	unsigned char data[0x20];
	// Pointer to data
	unsigned char* data_in;
	// Init Pointer
	data_in = data;

	// -----------------
	// Lock the Hardware
	// -----------------
	pthread_mutex_lock(&kuhler_mutex);
	// Do the Transfer
	libusb_bulk_transfer (devh,0x82,data_in,0x20,dlength,1);
	//printf ("Sent Retr_Status : %i\n",*dlength);
	
	//  Construct the Status Packet
	sp->fan_rpm = (long int) data[0]*16*16 + data[1];
	sp->pump_rpm = (data[8]*16*16)+data[9];
	sp->liquid_temp = (double) data[10] + (double) data[14]/10;
	sp->current_mode = (unsigned short int) data[22];

	// ----------------------
	// Unlock the Hardware
	// ----------------------
	pthread_mutex_unlock(&kuhler_mutex);
}

int change_mode (int mode, struct kuhler_data* kd) {
	int dlength_real;
	int* dlength;
	dlength = &dlength_real;	// ptr to actual nums of bytes sent

	switch (mode) {
		case extreme : { 
	// ------------------------------
			// Lock the Hardware
	// ------------------------------
			pthread_mutex_lock(&kuhler_mutex);	
			syslog (LOG_NOTICE, "Switching to Extreme Mode   \n");
			libusb_bulk_transfer(devh,0x02,kd->usb_mode_control_packet[extreme],0x20,dlength,0);
			sleep(1);
			libusb_bulk_transfer(devh,0x02,kd->usb_status_packet[extreme],0x20,dlength,0);
	// ------------------------------
			// UnLock the Hardware
	// ------------------------------
			pthread_mutex_unlock(&kuhler_mutex);	
			//sleep(1);
			break; 
		 	}
		case silent : { 
	// ------------------------------
			// Lock the Hardware
	// ------------------------------
			pthread_mutex_lock(&kuhler_mutex);	
			syslog (LOG_NOTICE, "Switching to Silent Mode   \n");
			libusb_bulk_transfer(devh,0x02,kd->usb_mode_control_packet[silent],0x20,dlength,0);
			sleep(1); 
			libusb_bulk_transfer(devh,0x02,kd->usb_status_packet[silent],0x20,dlength,0);
	// ------------------------------
			// UnLock the Hardware
	// ------------------------------
			pthread_mutex_unlock(&kuhler_mutex);	
			//sleep(1);
			break; 
			}
		case custom :  {  
			// Lock the Hardware
			pthread_mutex_lock(&kuhler_mutex);	
			syslog (LOG_NOTICE, "Switching to Custom Mode   \n");
			libusb_bulk_transfer(devh,0x02,kd->usb_mode_control_packet[custom],0x20,dlength,0);
			sleep(1);
			libusb_bulk_transfer(devh,0x02,kd->usb_status_packet[custom],0x20,dlength,0);
	// ------------------------------
			// Lock the Hardware
	// ------------------------------
			pthread_mutex_unlock(&kuhler_mutex);	
			//sleep(1);
			break; 
			}
		}
	//sleep(1);
return 0;
}

int change_led_color (uint8_t r,uint8_t g,uint8_t b,int mode, struct kuhler_data* kd) {
	kd->usb_status_packet[mode][1] = r;
	kd->usb_status_packet[mode][2] = g;
	kd->usb_status_packet[mode][3] = b;
	syslog (LOG_DEBUG, "LED Colors Changed to RED: %d, GRN: %d, BLUE: %d \n",r,g,b);
	// Update Status Packet
	sp_ptr->led_r = r;
	sp_ptr->led_g = g;
	sp_ptr->led_b = b;
	return 0;
}

int change_temp_threshold (int ramp_start , int full_fan, struct kuhler_data* kd) {
// Only valid for Custom Mode
	kd->usb_mode_control_packet[custom][1] = ramp_start;
	kd->usb_mode_control_packet[custom][6] = full_fan - ramp_start;
	syslog (LOG_NOTICE, "Custom Mode Temperature Thresholds Changed - Start : %d deg/C, Full %d deg/C \n", ramp_start,full_fan);
	return 0;
}





int main(int argc, char** argv)
{

struct kuhler_data	kd1;

kd1.usb_mode_control_packet[silent] = (char[0x20]) silent_mode_control;
kd1.usb_mode_control_packet[extreme] = (char[0x20]) extreme_mode_control;
kd1.usb_mode_control_packet[custom] = (char[0x20]) custom_mode_control;

kd1.usb_status_packet[silent] = (char[0x20]) silent_status_control;
kd1.usb_status_packet[extreme] = (char[0x20]) extreme_status_control;
kd1.usb_status_packet[custom]  = (char[0x20]) custom_status_control;

// MUTEX Lock
		
	pthread_mutex_init(&kuhler_mutex,NULL);

	int c;
	int debug_level;
	int update_thread_enable = 1;

        while ((c = getopt(argc,argv,"d:hn")) != -1) {
                switch (c) {
                        case 'h' :
                                printargs();	
				exit(0);
                                break;
			case 'd' :
				debug_level = 1;
				break;
			case 'n' :
				update_thread_enable = 0;			
				break;
			default :
				debug_level = 0;
			}
	}

	// Init Log Factiity
	// EMERG,ALRT,CRIT,ERR,WARNING,NOTICE,INFO,DEBUG
	if (debug_level == 1) {
		setlogmask (LOG_UPTO(LOG_DEBUG));
		printf ("Debug Mode Enabled!\n");
	} else {
		setlogmask (LOG_UPTO(LOG_NOTICE));
	}

	openlog ("kuhlerd", LOG_CONS | LOG_PID | LOG_NDELAY , LOG_LOCAL1);
	syslog (LOG_NOTICE, "kuhlerd version %s Started",version_string);

	// Init kd_ptr
	kd_ptr = &kd1;
	
	struct control_packet cp;
	struct status_packet sp;
	
	shm_id_cp = shmget(shm_key_cp,sizeof(cp),IPC_CREAT | 0666);  //rw-,rw-,rw-
	shm_id_sp = shmget(shm_key_sp,sizeof(sp),IPC_CREAT | 0666);	 //rw,r--,r--

	if (shm_id_cp < 0 || shm_id_sp < 0) {
		syslog (LOG_ERR, "Failed to Allocate Shared Memory Segment \n");
		printf ("Kuhlerd : Failed to Allocate Shared Memory Segment\n");
		exit(1);
	}

	syslog (LOG_DEBUG, "Allocated Shared Memory Segment %d %d \n", shm_id_sp,shm_id_cp);

	cp_ptr = shmat(shm_id_cp,NULL,0);
	sp_ptr = shmat(shm_id_sp,NULL,0);

	if (cp_ptr == NULL || sp_ptr == NULL) {
		syslog (LOG_ERR, "Failed to Attach to Shared Memory Segment \n");
		printf ("Kuhlerd : Failed to Attach Shared Memory Segment\n");
		exit(1);
	}


	int r = 1;

	r = libusb_init(NULL);
	if (r < 0) {
		syslog (LOG_ERR, "Failed to Initialized libusb \n");
		fprintf(stderr, "Kuhlerd : failed to initialise libusb\n");
		exit(1);
	}

	r = find_kuhler920();
	if (r < 0) {
		syslog (LOG_ERR, "Could not find Kuhler 920/960 or open device \n");
		fprintf(stderr, "Kuhlerd : Could not find/open device\n");
		exit(1);
	}

	r = libusb_claim_interface(devh, 0);
	if (r < 0) {
		syslog (LOG_ERR, "Could not claim exclusive use of usb device \n");
		fprintf(stderr, "Kuhlerd : usb_claim_interface error %d\n", r);
		exit(1);
	}

	syslog (LOG_DEBUG,"Claimed Interface \n");
	printf("Kuhlerd : claimed interface\n");

	syslog (LOG_DEBUG,"Sending Init Packet\n");
	printf("Kuhlerd : Sending Init Packet\n");
	r = init_kuhler920();
	syslog (LOG_DEBUG,"Init Packet Sent, returned with status %d \n",r);
	printf("Kuhlerd : Init Packet Sent, returned with status %d \n",r);	
	//init_signals();
	syslog (LOG_DEBUG,"Creating Histogram Entries %d \n",r);
	init_histogram(sp_ptr);
	syslog (LOG_DEBUG,"Created Histogram Entries %d \n",r);

	int i=0;
	int current_mode = silent;

	 cp_sem = sem_open(cp_sem_name, O_CREAT , 0666,0); // 1 = unlocked , 0 = locked
	 sp_sem = sem_open(sp_sem_name, O_CREAT , 0666,0); // 1 = unlocked , 0 = locked

	// v0.5 -- HW BUSY Semaphore - IPC
	 kuhler_hw_busy_sem = sem_open(kuhler_hw_busy_sem_name,O_CREAT,0666,0);

	//if (cp_sem == NULL || sp_sem == NULL) {
	if (cp_sem == NULL || sp_sem == NULL || kuhler_hw_busy_sem == NULL) {
		syslog (LOG_ERR, "Failed to Create semaphores \n");
		printf ("Kuhlerd : failed to create semaphores\n");
		exit(1);
	}

	// Initialize Shared Memory For Safety
	cp_ptr->mode = silent;
	cp_ptr->cmd = 0;
	cp_ptr->led_r = (uint8_t) 0;
	cp_ptr->led_g = (uint8_t) 0;
	cp_ptr->led_b = (uint8_t) 0;
	cp_ptr->start_temp = 40;
	cp_ptr->full_temp = 55;

	change_mode(silent,kd_ptr);
	get_status_packet(sp_ptr);

	syslog (LOG_NOTICE, "Entering Main Loop\n");
	printf ("Kuhlerd : Entering Main Loop\n");
	int rv=-99;
	
	// Fork Off and Exit 
	pid_t pidd;
	pidd = fork();

	
	//if (pidd == 0) {
	init_signals();
	// create pthreads
	pthread_t upd;


	// new for V5 - update thread
	if (update_thread_enable == 1) {
		int pthread_val = pthread_create(&upd,NULL,update_thread,NULL);
		syslog(LOG_NOTICE, "Creating Update Thread \n");
		printf ("Kuhlerd : Creating Update Thread \n");
	} else {
		syslog(LOG_NOTICE, "Update thread disabled \n");
		printf ("Kuhlerd : Disabling Update Thread \n");
	}

	if (pidd !=0) {
		syslog(LOG_NOTICE, "Entering Daemon Mode, exiting \n");
		printf ("Kuhlerd : Entering Daemon Mode, exiting \n");
		exit (0);
	}

		while (i==0) {
		// wait for the semaphore to be unlocked - written to 1 by ctl program
		//printf ("Waiting for Semaphore Unlock %i\n",rv);
		// hw busy control semaphore initially LOCKED 
		sem_post(kuhler_hw_busy_sem);
		rv = sem_wait(cp_sem); // set to 0 -- blocking call
		
		// hw busy control semaphore initially LOCKED 
		//sem_post(kuhler_hw_busy_sem);

		if (cp_ptr->cmd == 1 ) {
			change_led_color(cp_ptr->led_r,cp_ptr->led_g,cp_ptr->led_b,cp_ptr->mode,kd_ptr);
			if (cp_ptr->mode == custom) {
				change_temp_threshold(cp_ptr->start_temp,cp_ptr->full_temp,kd_ptr);	
			}
			change_mode(cp_ptr->mode,kd_ptr);
			get_status_packet(sp_ptr);	// required after mode change
		} else {
			send_status_packet(cp_ptr->mode,kd_ptr);
			sleep(1);
			get_status_packet(sp_ptr);
			// unlock status packet semaphore
			sem_post(sp_sem);
			}
		} // end while
	}
//}


// TBD -- Signal Handlers
void signal_handler(int sig) {
	struct shmid_ds* buf;
	int status;
	if (sig == SIGINT || sig == SIGKILL || sig == SIGCHLD || sig == SIGTERM) {
		syslog(LOG_NOTICE, "Received SIGKILL/INT Terminating  \n");
		//fprintf (stderr,"Kuhlerd : Received SIGKILL/INT Terminating \n");
		shmdt(cp_ptr);
		shmdt(sp_ptr);
		status = shmctl(shm_id_cp,IPC_RMID,buf);
		status+=shmctl(shm_id_sp,IPC_RMID,buf);
		// Close Semaphores
		status+=sem_close(cp_sem);
		status+=sem_close(sp_sem);
		status+=sem_close(kuhler_hw_busy_sem);
		// Unlink Semaphores
		status+=sem_unlink(sp_sem_name);
		status+=sem_unlink(cp_sem_name);
		status+=sem_unlink(kuhler_hw_busy_sem_name);
		deinit_kuhler920(); 
		libusb_release_interface(devh, 0);
		libusb_close(devh);
		libusb_exit(NULL);
		syslog(LOG_NOTICE, "Exited with Status : %i \n",status);
		closelog(); // close log connection
		//fprintf (stderr,"Kuhlerd : Exited with Status : %i\n",status);
		exit(status);
	} else {
		syslog(LOG_NOTICE, "Received Signal %i -- Ignoring  \n",sig);
		//fprintf (stderr,"Kuhlerd : Received Signal %i -- Ignoring \n",sig);
	}
}

// ---------------------- 
// ---------------------- This will eventually be used for "overtemperature" events/etc. 
// ----------------------
static void alarm_handler (int alarm_sig) {
	static int count;
	count++;
	syslog(LOG_NOTICE, "Alarm Handler Called With Signal %d %d START times\n", alarm_sig,count);
	send_status_packet(cp_ptr->mode,kd_ptr);
	sleep(1);
	get_status_packet(sp_ptr);
	sem_post(sp_sem);
	syslog(LOG_NOTICE, "Alarm Handler Called With Signal %d %d times FINISHED \n", alarm_sig,count);
}


void init_signals(void) {
	// Register Signal Handlers
	signal (SIGKILL, signal_handler);
	signal (SIGTERM, signal_handler);
	//signal (SIGALRM, alarm_handler);	// 
	syslog (LOG_NOTICE,"Signal Handler Installed \n");
}

void printargs (void) {
	printf ("kuhlerd : Usage \n");
	printf ("-d set DEBUG/MESSAGE level \n");
	printf ("\t 0 : display only NOTICE , ie: Mode Change, Threshold Change, daemon start, stop, and CRITICAL errors \n");
	printf ("\t 1 : display NOTICE and DEBUG level messages - useful for debugging \n");
	printf ("-h display this help information\n");
	printf ("-n disable auto-update thread - disables histogram recording \n");
}


void update_histogram (struct status_packet* sp_ptr) {
        int i;
        double low,high;
        double curr_temp = sp_ptr->liquid_temp;
	double curr_rpm = sp_ptr->fan_rpm;
	double rpm_low,rpm_high;

	
        for (i=0;i<=15;i++) {
              
		low = sp_ptr->temp_histo_entry[i].low_lim;
                high = sp_ptr->temp_histo_entry[i].high_lim;

		rpm_low = sp_ptr->fan_rpm_histo_entry[i].low_lim;
		rpm_high = sp_ptr->fan_rpm_histo_entry[i].high_lim;

                if (curr_temp >= low && curr_temp <= high) {
                        sp_ptr->temp_histo_entry[i].count++;
                }

		if (curr_rpm >= rpm_low && curr_rpm <= rpm_high) {
			sp_ptr->fan_rpm_histo_entry[i].count++;
		}
        }
}

void clear_histogram (struct status_packet* sp_ptr) {
        int i;
        for (i=0;i<=15;i++) {
                sp_ptr->temp_histo_entry[i].count = 0;
        }
}


void init_histogram (struct status_packet* sp_ptr) {
        int i;
        for (i=0;i<=15;i++) {
                sp_ptr->temp_histo_entry[i].low_lim = 14.0 + (4.0*(double)i);
                sp_ptr->temp_histo_entry[i].high_lim = 14.0 + (4.0*(double)i)+3.9;
                sp_ptr->temp_histo_entry[i].count = 0;
                //printf ("Temperature Histogram value: %d low: %2.1f high : %2.1f \n", i,sp_ptr->temp_histo_entry[i].low_lim, sp_ptr->temp_histo_entry[i].high_lim);
        }

	for (i=0;i<=15;i++) {
		sp_ptr->fan_rpm_histo_entry[i].low_lim = 550 + (135*i);
		sp_ptr->fan_rpm_histo_entry[i].high_lim = 550 + (135*i)+134;
		sp_ptr->fan_rpm_histo_entry[i].count = 0;
		//printf ("FAN RPM Histogram value : %d low : %3.0f high : %3.0f \n",i,sp_ptr->fan_rpm_histo_entry[i].low_lim,sp_ptr->fan_rpm_histo_entry[i].high_lim);
	}
}

