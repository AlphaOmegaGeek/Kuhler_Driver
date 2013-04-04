CC = gcc
LD = gcc

CFLAGS =   -lusb-1.0 -lpthread  
LDFLAGS =  -lusb-1.0 -lpthread 
INSTALL_DIR = /var/local

all :	kuhlerd kuhler_ctl 


kuhlerd : src/kuhler_920.c common/inc/kuhler_920.h 
	$(CC) src/kuhler_920.c -o kuhlerd $(CFLAGS)

kuhler_ctl : src/kuhler_920_ctl.c common/inc/kuhler_920.h
	$(CC) src/kuhler_920_ctl.c -o kuhler_ctl $(CFLAGS)


.PHONY:	clean

clean :  
	rm -f ./kuhlerd
	rm -f ./kuhler_ctl

install : kuhlerd kuhler_ctl
	cp ./kuhlerd /var/local
	cp ./kuhler_ctl /var/local	

### Don't Error Out on 'can't find file, etc'
