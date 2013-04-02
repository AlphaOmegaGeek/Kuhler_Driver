#include "../common/inc/kuhler_920.h"

void update_histogram (struct status_packet* sp) {
	struct status_packet* sp_ptr;
        int i;
        double low,high;
        double curr_temp = sp_ptr->liquid_temp;
        for (i=0;i<=19;i++) {
                low = sp_ptr->temp_histo_entry[i].low_lim;
                high = sp_ptr->temp_histo_entry[i].high_lim;
                if (curr_temp >= low && curr_temp <= high) {
                        //printf ("Updating Histogram @ %d %2.0f %2.0f \n",i,low,high);         
                        sp_ptr->temp_histo_entry[i].count++;
                }
        }
}

void clear_histogram (struct status_packet* sp_ptr) {
        int i;
        for (i=0;i<=19;i++) {
                sp_ptr->temp_histo_entry[i].count = 0;
        }
}


void print_histogram (struct status_packet* sp_ptr) {
        double low,high;
        int i;
        int count;
        for (i=0;i<=19;i++) {
                low = sp_ptr->temp_histo_entry[i].low_lim;
                high = sp_ptr->temp_histo_entry[i].high_lim;
                count = sp_ptr->temp_histo_entry[i].count;
                printf (" Liquid Temperature %2.0f to %2.0f : %d \n", low,high,count);
        }
}

void init_histogram (struct status_packet* sp_ptr) {
        int i;
        double low_lim;
        double high_lim;
        for (i=0;i<=19;i++) {
                sp_ptr->temp_histo_entry[i].low_lim = 14.0 + (4.0*i);
                sp_ptr->temp_histo_entry[i].high_lim = 14.0 + (4.0*i)+3.0;
                sp_ptr->temp_histo_entry[i].count = 0;
                //printf ("Histogram value %d low: %2.0f high : %2.0f \n", i,sp_ptr->temp_histo_entry[i].low_lim, sp_ptr->temp_histo_entry[i].high_lim);
        }
}

