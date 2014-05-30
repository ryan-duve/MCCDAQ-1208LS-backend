#############################################################################
#	
#	makefile for building:
###########################################################################

#  Current Version of the driver
VERSION=1.54

SRCS = pmd.c usb-1208LS.c
HEADERS = pmd.h usb-1208LS.h 
OBJS = $(SRCS:.c=.o)   # same list as SRCS with extension changed
CC=gcc
CFLAGS= -g -Wall -fPIC -O
TARGETS=libmcchid.so libmcchid.a mysql-usb1208LS 
ID=MCCLIBHID
DIST_NAME=$(ID).$(VERSION).tgz
DIST_FILES={README,Makefile,usb-1208LS.h,pmd.h,pmd.c,mysql-usb1208LS.c,}

###### RULES
all: $(TARGETS)

%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@
ifneq ($(MAKECMDGOALS),clean)
include $(SRCS:.c=.d)
endif

libmcchid.so: $(OBJS)
#	$(CC) -O -shared -Wall $(OBJS) -o $@
	$(CC) -shared -Wl,-soname,$@ -o $@ $(OBJS) -lc -lm

libmcchid.a: $(OBJS)
	ar -r libmcchid.a $(OBJS)
	ranlib libmcchid.a

#http://www.daniweb.com/software-development/c/threads/180705/how-to-include-mysql.h-in-makefile
MYSQLCFLAGS= -I/usr/include/mysql -DBIG_JOINS=1 -fno-strict-aliasing -g
MYSQLLIBS= -L/usr/lib/arm-linux-gnueabihf -lmysqlclient -lpthread -lz -lm -lrt -ldl

mysql-usb1208LS:	mysql-usb1208LS.c libmcchid.a
	$(CC) -g -Wall -I. -o $@  $@.c -lmcchid -L. -lm -L/usr/local/lib -lhid -lusb $(MYSQLCFLAGS) $(MYSQLLIBS)


clean:
	rm -rf *.d *.o *~ *.a *.so $(TARGETS)

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) libhid/$(DIST_FILES);

install:
	-install -d /usr/local/lib
	-install -c --mode=0755 ./libmcchid.a libmcchid.so /usr/local/lib
	-/bin/ln -s /usr/local/lib/libmcchid.so /usr/lib/libmcchid.so
	-/bin/ln -s /usr/local/lib/libmcchid.a /usr/lib/libmcchid.a
	-install -d /usr/local/include/libhid
	-install -c --mode=0644 ${HEADERS} /usr/local/include/libhid/

uninstall:
	-rm -f /usr/local/lib/libmcchid*
	-rm -f /usr/lib/libmcchid*
	-rm -rf /usr/local/include/libhid
