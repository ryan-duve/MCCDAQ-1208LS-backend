/*
 *  Copyright (c) 2009   Warren Jasper <wjasper@tx.ncsu.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <sys/types.h>
#include <asm/types.h>

#include "pmd.h"
#include "usb-2416.h"

#define MAX_COUNT     (0xffff)
#define FALSE 0
#define TRUE 1


/* Test Program */
int toContinue()
{
  int answer;
  answer = 0; //answer = getchar();
  printf("Continue [yY]? ");
  while((answer = getchar()) == '\0' ||
    answer == '\n');
  return ( answer == 'y' || answer == 'Y');
}

int main (int argc, char **argv)
{
  int flag;
  usb_dev_handle *udev = NULL;
  int ch;
  int temp;
  int i;
  int tc_type;
  __u8 input;
  __u8 channel, gain, rate, mode, flags;
  char serial[9];
  __u16 version[4];
  float cjc[8];
  int queue_index;
  int data;
  double table_AIN[NGAINS_2416][2];
  double table_AO[NCHAN_AO_2416][2];
  AInScanQueue scanQueue;
  int usb2416_4AO = FALSE;
  double voltage;
  double temperature;
  double frequency;
  double sine[512];
  __s16 sdata[512];  // holds 16 bit signed analog output data
  int   idata[512];   // holds 24 bit signed analog input data

  udev = NULL;
  if ((udev = usb_device_find_USB_MCC(USB2416_PID))) {
    printf("Success, found a USB 2416!\n");
  } else if ((udev = usb_device_find_USB_MCC(USB2416_4AO_PID))) {
    printf("Success, found a USB 2416_4AO!\n");
    usb2416_4AO = TRUE;
  } else {
    printf("Failure, did not find a USB 2416 or 2416_4AO!\n");
    return 0;
  }

  usbBuildGainTable_USB2416(udev, table_AIN);
  for (i = 0; i < NGAINS_2416; i++) {
    printf("Gain: %d    Slope = %lf    Offset = %lf\n", i, table_AIN[i][0], table_AIN[i][1]);
  }

  if (usb2416_4AO) {
    usbBuildGainTable_USB2416_4AO(udev, table_AO);
    printf("\n");
    for (i = 0; i < NCHAN_AO_2416; i++) {
      printf("VDAC%d:    Slope = %lf    Offset = %lf\n", i, table_AO[i][0], table_AO[i][1]);
    }
  }

  while(1) {
    printf("\nUSB 2416 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'j' read CJC sensors.\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Output Scan\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 't' to get TC temperature\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'b': /* test to see if led blinks */
        printf("Enter number or times to blink: ");
	scanf("%d", &temp);
        usbBlink_USB2416(udev, temp);
        break;
      case 'c':
        usbCounterInit_USB2416(udev, COUNTER0);
        printf("Connect DO0 to CTR0\n");
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDOut_USB2416(udev, 0x0, 0);
	  usbDOut_USB2416(udev, 0xff, 0);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB2416(udev, COUNTER0));
        break;      
      case 'd':
        printf("\nTesting Digital I/O....\n");
        do {
  	  printf("Enter a number [0-0xff] : " );
  	  scanf("%x", &temp);
	  usbDOut_USB2416(udev, (__u8)temp, 0);
	  input = usbDOutR_USB2416(udev, 0);
	  printf("The number you entered = %#x\n\n",input);
  	  input = usbDIn_USB2416(udev, 0);
  	  printf("The number you entered = %#x\n\n",input);
	} while (toContinue());
	break;
      case 'e':
        cleanup_USB2416(udev);
        return 0;
      case 'i':
        printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Gain Range for channel %d: 1 = 20V  2 = 10V  3 = 5V  4 = 2.5V Differential: ", channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_20V; break;
  	  case '2': gain = BP_10V; break;
	  case '3': gain = BP_5V; break;
	  case '4': gain = BP_2_5V; break;
	  default:  gain = BP_20V; break;
	}
	rate = HZ1000;
	mode = DIFFERENTIAL;
	for (i = 0; i < 20; i++) {
  	  data = usbAIn_USB2416(udev, channel, mode, gain, rate, &flags);
	  data = data*table_AIN[gain][0] + table_AIN[gain][1];
	  printf("Channel %d  Sample[%d] = %#x  Volts = %lf\n", channel, i, data,
		 volts_USB2416(udev, gain, data));
	  usleep(50000);
	}
	break;
      case 'I':
        printf("Testing USB-2416 Analog Input Scan\n");
	usbAInScanStop_USB2416(udev);
	printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Gain Range for channel %d: 1 = 20V  2 = 10V  3 = 5V  4 = 2.5V Differential: ", channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_20V; break;
  	  case '2': gain = BP_10V; break;
	  case '3': gain = BP_5V; break;
	  case '4': gain = BP_2_5V; break;
	  default:  gain = BP_20V; break;
	}
	rate = HZ1000;
	mode = DIFFERENTIAL;
	scanQueue.count = 1;
	scanQueue.queue[0].channel = channel;
	scanQueue.queue[0].mode = mode;
	scanQueue.queue[0].range = gain;
	scanQueue.queue[0].rate = rate;
	usbAInScanQueueWrite_USB2416(udev, &scanQueue);
	usbAInScanStart_USB2416(udev, 1100., 512, 10, idata);
	usbAInScanStop_USB2416(udev);

	usbAInScanQueueRead_USB2416(udev, &scanQueue);
	for (i = 0; i < 512; i++) {
	  queue_index = idata[i] >> 24;  // MSB of data contains the queue index;
	  gain = scanQueue.queue[queue_index].range;
	  channel = scanQueue.queue[queue_index].channel;
	  data = sint24TOint(idata[i]);                             // convert from signed 24 to signed 32
  	  data = data*table_AIN[gain][0] + table_AIN[gain][1];      // correct for non-linearities in A/D
	  printf("Sample %d Index %d Channel %d   gain = %d raw = %#x  voltage = %f\n",
		 i, queue_index, channel, gain, idata[i], volts_USB2416(udev, gain, data));
	}
	break; 

      case 'j':
	usbCJC_USB2416(udev, cjc);
	for (i = 0; i < 8; i++) {
	  printf("CJC sensor[%d] = %f degree C.\n", i, cjc[i]);
	}
	break;
      case 'O':
        if (!usb2416_4AO) {
	  printf("Analog output only on the USB-2416-4AO model.\n");
	  break;
	}
        channel = 0;
	printf("Test of Analog Output Scan.\n");
	printf("Hook scope up to VDAC 0\n");
	printf("Enter desired frequency of sine wave [Hz]: ");
	scanf("%lf", &frequency);
	for (i = 0; i < 512; i ++) {
	  sine[i] = 10*sin(2.*M_PI*i/128.);
	}
	voltsTos16_USB2416_4AO(sine, sdata, 512, table_AO[channel]);
	usbAOutScanStop_USB2416_4AO(udev);
	usbAOutScanStart_USB2416_4AO(udev, 128.*frequency, 0, (1 << channel));
	printf("Hit \'s <CR>\' to stop ");
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  usb_bulk_write(udev, USB_ENDPOINT_OUT|1, (char *) sdata, sizeof(sdata), 1000);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAOutScanStop_USB2416_4AO(udev);
	break;
      case 'o':
	if (!usb2416_4AO) {
	  printf("Analog output only on the USB-2416-4AO model.\n");
	  break;
	}
        printf("output value on VDAC 0\n");
        do {
	  printf("Enter output voltage [-10 to 10]: ");
	  scanf("%lf", &voltage);
	  usbAOut_USB2416_4AO(udev, 0, voltage, table_AO);
        } while (toContinue());
	break;
      case 'r':
	usbReset_USB2416(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB2416(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB2416(udev));
        break;
      case 't':
	printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case 'J': tc_type = TYPE_J; break;
  	  case 'K': tc_type = TYPE_K; break;
	  case 'R': tc_type = TYPE_R; break;
  	  case 'S': tc_type = TYPE_S; break;
   	  case 'T': tc_type = TYPE_T; break;
     	  case 'N': tc_type = TYPE_N; break;
       	  case 'E': tc_type = TYPE_E; break;
       	  case 'B': tc_type = TYPE_B; break;
	  default: tc_type = TYPE_J; break;
	}
	temperature = tc_temperature_USB2416(udev, tc_type, channel, table_AIN);
	printf("Temperature = %f\n", temperature);
        break;
      case 'v':
        usbGetVersion_USB2416(udev, version);
	printf("USB micro firmware version = %x.%x\n", version[0]/0x100, version[0]%0x100);
	printf("USB update firmware version = %x.%x\n", version[1]/0x100, version[1]%0x100);
	printf("isolated micro firmware version = %x.%x\n", version[2]/0x100, version[2]%0x100);
	printf("isolated update firmware version = %x.%x\n", version[3]/0x100, version[3]%0x100);
	break;
      default:
        break;
    }
  }
}

