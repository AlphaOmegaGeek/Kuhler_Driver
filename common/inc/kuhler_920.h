/*
 * Antec (ASETEK) Kuhler-920 Basic Driver [usermode] daemon?
 * Alpha Geek : alphageek@hotmail.com
 *
 * with code and libraries from: libusb-1.0 Daniel Drake (GPL)
 *
*/

/*
    This file is part of Kuhler 920 Linux Driver Package.

    Kuhler 920 Linux Driver Package is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/



// *********************************************************************************************************//
// No warranties, expressed or implied, your mileage may vary, you might fry your cpu, your Kuhler, or both.//
// information taken from clean-room reverse engineering the USB packets from the Kuhler-920                //
// nothing taken from the Kuhler software directly                                                          //
// *********************************************************************************************************//

#ifndef __KUHLER920__H
#define __KUHLER920__H

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

#include "/usr/include/libusb-1.0/libusb.h"

enum mode {silent,custom,extreme};

// WriteOnly
struct control_packet { 	// byte aligned
	uint8_t mode;		// silent, ext, cust, enum 
	uint8_t start_temp; 	// 0x00 - 0xff
	uint8_t full_temp;	// (0x00 - 0xff) - start_temp
	uint8_t led_r;		// 0x00 - 0xff - 0x00 = off
	uint8_t led_b;		// 
	uint8_t led_g;		//
	uint8_t cmd;		// 0 = mode_change, 1 = status, 2 = print_histogram, 4 = clear_histogram,  
};


struct histogram_entry {
        double high_lim;	// really cound be int
        double low_lim;
        double count;		// long int fo space-saving?
};

// ReadOnly
struct status_packet { 				
	uint16_t fan_rpm;			// RO 0x0000 - 0xffff expect 500 - 3000 +/- 5%
	double liquid_temp;			// RO - calculation
	uint16_t pump_rpm;			// RO 0x0000 - 0xffff expect 3000 +/- 5%
	uint8_t current_mode;			// RO
	// new for version 0.5
	//double max_liquid_temp;				// v0.5
	//double min_liquid_temp;				// v0.5
	struct histogram_entry 	temp_histo_entry[16];
	struct histogram_entry	fan_rpm_histo_entry[16];
	uint8_t led_r;
	uint8_t led_g;
	uint8_t led_b;
};

// Usb Packets are stored here

struct kuhler_data {
	unsigned char* usb_mode_control_packet[3];	// mapped to enum mode
	unsigned char* usb_status_packet[3];		// mapped to enum mode
};


// USB VID/PID
#define KUHLER_920_USB_VID  0x2433
#define KUHLER_920_USB_PID  0xb111

// Version 4.0 Of Kuhler - Higher Pressure Pump - Split-Flow Radiator
// Requires different Windows Software due to VID/PID Change?
#define KUHLER_920_V4_USB_VID  0x2433
#define KUHLER_920_V4_USB_PID  0xb200

 
// Temperature Control - Bulk Out Packets

// Padded to 32 bytes - all other bytes are 0's
#define silent_mode_control 	{0x12,0x19,0x2d,0x2d,0x2d,0x01,0x19,0x02,0x03,0x04}; // Silent - Constant in SW
#define extreme_mode_control	{0x12,0x19,0x2d,0x2d,0x2d,0x00,0x19,0x02,0x03,0x04}; // Extreme -Constant in SW
#define custom_mode_control	{0x12,0x19,0x2d,0x2d,0x2d,0x01,0x19,0x02,0x03,0x04}; // Custom - Variable 0x1 = start, 0x6 = full

// Padded to 32 bytes - all other bytes are 0's
#define extreme_status_control  {0x10,0x00,0x00,0x00,0x00,0x00,0x00}; // 0x10,RR,GG,BB ...
#define silent_status_control	{0x10,0x00,0x00,0x00,0x00,0x00,0x01}; // 0x10,RR,GG,BB,
#define custom_status_control	{0x10,0x00,0x00,0x00,0x64,0x64,0x02}; // 0x10,RR,GG,BB,0x64,0x64,0x02


// Shared Memory Keys
#define shm_key_cp_val 0xdeadbeef // 32bit value only, need not be globally unique
#define shm_key_sp_val 0xbeefdead // 32bit value only, need not be globally unique

// Named Shared Memory - TBD
#define cp_shm_key_name "/kuhler_cp_shm"
#define sp_shm_key_name "/huhler_sp_shm"

// Named Semaphores
#define cp_sem_name  "/kuhler_cp_sem"
#define sp_sem_name  "/kuhler_sp_sem"
#define kuhler_hw_busy_sem_name "/kuhler_hw_busy_sem"

#define version_string "kuhlerd/kuhler_ctl Control - v0.5"

// Function Declarations -- Kuhlerd
static int find_kuhler920(void);
static int init_kuhler920(void);
static int deinit_kuhler920(void);
void get_status_packet(struct status_packet* sp);
int change_mode (int mode, struct kuhler_data* kd);
int change_led_color (uint8_t r, uint8_t g, uint8_t b, int mode, struct kuhler_data* kd);
int change_temp_threshold (int ramp_start, int full_fan, struct kuhler_data* kd);
void init_signals(void);
void signal_handler(int sig);
void printargs(void);

// function declarations
void update_histogram (struct status_packet* sp_ptr); 
void clear_histogram (struct status_packet* sp_ptr); 
void print_liquid_temp_histogram (struct status_packet* sp_ptr); 
void print_fan_rpm_histogram (struct status_packet* sp_ptr); 
void init_histogram (struct status_packet* sp_ptr);

#endif

