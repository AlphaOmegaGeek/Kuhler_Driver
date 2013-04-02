
/*
 * Antec (ASETEK) Kuhler-920 Basic Driver [usermode] daemon
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

    Kuhler 920 Linux Driver Package is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Kuhler 920 Linux Driver Package is free software.  If not, see <http://www.gnu.org/licenses/>.
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
#include "/usr/include/libusb-1.0/libusb.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <fcntl.h>
#include "../common/inc/kuhler_920.h"
#include <semaphore.h>
#include <pthread.h>

// Declarations
void printargs(void);


void print_stats(struct status_packet* sp, struct control_packet* cp) {
        // More Debug
	double temp_f;
	
	temp_f = ((double)(9.0/5.0)*(double)sp->liquid_temp) + 32.0;
	 switch (sp->current_mode) {
		 case 01 :
	 //printf ("Antec Kuhler 920 Stats : Fan RPM : %ld Pump RPM : %ld Liquid Temp: %2.1f deg/C Fan Mode : Silent \n",sp->fan_rpm,sp->pump_rpm,sp->liquid_temp);
                printf ("\nAntec Kuhler 920 Stats : \n");
                printf ("************************ \n");
                printf ("Fan RPM : %i \n",sp->fan_rpm);
                printf ("Pump RPM : %i \n",sp->pump_rpm);
                printf ("Liquid Temp : %2.1f deg/C / %2.1f deg/F \n",sp->liquid_temp,temp_f);
                printf ("Fan Mode : Silent \n");
		printf ("Red LED Value : %d \n",sp->led_r);
		printf ("Grn LED Value : %d \n",sp->led_g);
		printf ("Blu LED Value : %d \n",sp->led_b);
                printf ("************************ \n");

		 break;
	 	case 02 :
	        printf ("\nAntec Kuhler 920 Stats : \n");
                printf ("************************ \n");
                printf ("Fan RPM : %i \n",sp->fan_rpm);
                printf ("Pump RPM : %i \n",sp->pump_rpm);
                printf ("Liquid Temp : %2.1f deg/C \n",sp->liquid_temp);
                printf ("Fan Mode : Custom \n");
		printf ("Fan Start Temp : %d deg/C \n", cp->start_temp);
		printf ("Fan Full Temp : %d deg/C \n", cp->full_temp);
		printf ("Fan Full Delta : %d deg/C \n", cp->full_temp-cp->start_temp);
		printf ("Red LED Value : %d \n",sp->led_r);
		printf ("Grn LED Value : %d \n",sp->led_g);
		printf ("Blu LED Value : %d \n",sp->led_b);
                printf ("************************ \n");

	 //printf ("Antec Kuhler 920 Stats : Fan RPM : %ld Pump RPM : %ld Liquid Temp: %2.1f deg/C Fan Mode : Custom Fan_Start : %d deg/C Fan_Full %d deg/C \n",
	 //sp->fan_rpm,sp->pump_rpm,sp->liquid_temp,cp->start_temp,cp->full_temp);
	 	break;
	 	case 00 : 
	        printf ("\nAntec Kuhler 920 Stats : \n");
                printf ("************************ \n");
                printf ("Fan RPM : %i \n",sp->fan_rpm);
                printf ("Pump RPM : %i \n",sp->pump_rpm);
                printf ("Liquid Temp : %2.1f deg/C  \n",sp->liquid_temp);
                printf ("Fan Mode : Extreme \n");
		printf ("Red LED Value : %d \n",sp->led_r);
		printf ("Grn LED Value : %d \n",sp->led_g);
		printf ("Blu LED Value : %d \n",sp->led_b);
                printf ("************************ \n");

	 //printf ("Antec Kuhler 920 Stats : Fan RPM : %ld Pump RPM : %ld Liquid Temp: %2.1f deg/C Fan Mode : Extreme \n",sp->fan_rpm,sp->pump_rpm,sp->liquid_temp);
	 	break; 
	 	default :
	 printf ("Antec Kuhler 920 Stats : UNKNOWN MODE : ERROR!\n");
		break;
	}
}


int main(int argc, char** argv) {

	struct control_packet cp;
	struct status_packet sp;

	struct control_packet* cp_ptr;
	struct status_packet* sp_ptr;

	// Shared Memory Segments
	key_t shm_key_cp = shm_key_cp_val;
	key_t shm_key_sp = shm_key_sp_val;

	 int cp_shm_id=-1;
	 int sp_shm_id=-1;	
	

	cp_shm_id = shmget(shm_key_cp,sizeof(cp),0666);
	sp_shm_id = shmget(shm_key_sp,sizeof(sp),0444);


	if (cp_shm_id < 0 || sp_shm_id < 0) {
		printf ("Kuhler_ctl : Failed to get Kuhlerd SHM Segment ID %d %d \n",cp_shm_id,sp_shm_id);
		exit(1);
	} else {
		//printf ("CTL : Retrieved SHM Segment %d %d",cp_shm_id,sp_shm_id);
	}

	// This is POIS UNAMED Shared Memory - Convert to Named Memory?
	cp_ptr = shmat(cp_shm_id,NULL,0);
	sp_ptr = shmat(sp_shm_id,NULL,0);

	if (cp_ptr == NULL || sp_ptr == NULL) {
		printf ("Kuhler_ctl : Failed To Attach to Kuhlerd SHM egment\n");
		exit(1);
	}

	int c;
	//unsigned short int red,grn,blu = 0;
	uint8_t red = 0;
	uint8_t grn = 0;
	uint8_t blu = 0;
	uint8_t fan_start = 35;
	uint8_t fan_full =  55;  // Kuher_920.c stores offset as 55-35 = 0x14 [20 deg/c] 
	uint8_t status = 0 ;
	uint8_t mode = silent;   // Initialize into silent mode
	int optflg = 0x00;

	// Semaphores:
	 sem_t* cp_sem=NULL;
	 sem_t* sp_sem=NULL;
	 sem_t* kuhler_hw_busy_sem=NULL;
	// These are POSIX NAMED Semaphores
         cp_sem = sem_open(cp_sem_name, O_EXCL,0x666); // 1 = unlocked , 0 = locked
         sp_sem = sem_open(sp_sem_name, O_EXCL,0x666); // 1 = unlocked , 0 = lockedi
        
	// v0.5 
	 kuhler_hw_busy_sem = sem_open(kuhler_hw_busy_sem_name, O_EXCL,0x666); // 1 = unlocked , 0 = lockedi

	 //if (sp_sem == NULL || cp_sem == NULL) {
	 if (sp_sem == NULL || cp_sem == NULL || kuhler_hw_busy_sem == NULL) {
		printf ("Kuhler_ctl : failed to connect to kuhlerd - Semaphore key not found \n");	
		exit(1);
	}

	if (argc == 1) {
		printargs();
		exit (1);
	}

	while ((c = getopt(argc,argv,"m:r:g:b:s:f:ihvl")) != -1) {
		switch (c) {
			case 'h' :
				printargs();
				break;
			case 'r' :
				red = (uint8_t)atoi(optarg);
				optflg = optflg | 0x01;
				break;
			case 'g' : 
				grn = (uint8_t)atoi(optarg);
				optflg = optflg | 0x02;
				break;
			case 'b' :  	
				blu = (uint8_t)atoi(optarg);
				optflg = optflg | 0x04;
				break;
			case 's' :
				fan_start = (uint8_t)atoi(optarg); 
				optflg = optflg | 0x08;
				break;
			case 'f' : 
				fan_full = (uint8_t)atoi(optarg);
				optflg = optflg | 0x10;
				break;
			case 'i' :
				status = 1; 
				optflg = optflg | 0x20;
				break;
			case 'm' : 
				mode = (unsigned short int)atoi(optarg);
				optflg = optflg | 0x40;
				if (mode > 2 || mode  < 0) {
					printf ("Usage for mode : \n");
					printf (" 0 = silent , 1 = custom , 2 = extreme \n");
					return -1;
				}
				break;
			case 'l' :
				status = 1;
				optflg = optflg | 0x80 | 0x20;
				break;
			default :
				printf ("ERROR : Option Requires %c Integer Argument n",optopt);
				return -1;
				break;
			case '?' :
				printf ("ERROR : Unrecognized Option %c -- Exiting \n",optopt);
				printargs();
				return -1;
				break;
			case ' ' : 
				printf ("Usage : \n");
				break;
			case 'v' :
				printf ("kuhler_ctl : %s \n\n",version_string);
				exit(0);
				break;
		} // end switch
	} //end while 


	int rv = -99;

	sem_wait(kuhler_hw_busy_sem);

	if ((optflg & 0x20) != 0x20) {
		printf ("Setting New Parameters \n");
		cp_ptr->led_r = red;
		cp_ptr->led_b = blu;
		cp_ptr->led_g = grn;
		cp_ptr->start_temp = fan_start;
		cp_ptr->full_temp = fan_full;
		cp_ptr->mode = mode;
		cp_ptr->cmd = 1;
		sem_post(cp_sem);
	} else {
		cp_ptr->cmd = 0;
		//printf ("\nRetrieving Status Only - Ignoring Other Options if Specified: \n");
		sem_post(cp_sem);
		rv = sem_wait(sp_sem);
		print_stats(sp_ptr,cp_ptr);
		if ((optflg & 0x80) == 0x80) {
				print_liquid_temp_histogram(sp_ptr);
				printf ("\n");
				print_fan_rpm_histogram(sp_ptr);
		}
	}


return 0;
}

void printargs(void) {
		printf ("\n");
		printf ("kuhler_ctl : Usage : \n");
		printf (" -h print usage information - this screen \n");
		printf (" -v display version information \n");
		printf (" -r (red led value) 0-255\n");
		printf (" -g (grn led value) 0-255\n");
		printf (" -b (blu led value) 0-255\n");
		printf (" -s fan start temperature in deg/C \n");
		printf (" -f fan full temperature in deg/C \n");
		printf (" -i get Kuhler status, fan RPM, pump RPM , Liquid Temp, and Current Mode \n");
		printf (" -m set mode, 0 = silent, 1 = custom, 2 = extreme \n");
		printf (" -l : display long information / min / max and histogram \n");
		printf ("\n");
		printf ("Note: -i switch will only return info, any other parameters will be ignored \n\n");
		exit (1);
}

void print_liquid_temp_histogram (struct status_packet* sp_ptr) {
        double low,high;
        int i;
        int count;
	printf ("\n");
	printf ("Temperature Histogram \n");
	printf ("********************* \n");
        for (i=0;i<=15;i++) {
                low = sp_ptr->temp_histo_entry[i].low_lim;
                high = sp_ptr->temp_histo_entry[i].high_lim;
                count = sp_ptr->temp_histo_entry[i].count;
                printf (" Liquid Temperature %2.1f to %2.1f : %d \n", low,high,count);
        }
}

void print_fan_rpm_histogram (struct status_packet* sp_ptr) {
        double low,high;
        int i;
        int count;
	printf ("FAN RPM Histogram \n");
	printf ("********************* \n");
        for (i=0;i<=15;i++) {
                low = sp_ptr->fan_rpm_histo_entry[i].low_lim;
                high = sp_ptr->fan_rpm_histo_entry[i].high_lim;
                count = sp_ptr->fan_rpm_histo_entry[i].count;
                printf (" FAN RPM %2.0f to %2.0f : %d \n", low,high,count);
        }
}

