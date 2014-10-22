/*
 *
 *  Copyright (c) 2013   Warren Jasper <wjasper@tx.ncsu.edu>
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
#include "usb-1608G.h"

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
  usb_dev_handle *udev = NULL;

  double frequency;
  float temperature;
  float table_AIN[NGAINS_1608G][2];
  ScanList list[NCHAN_1608G];  // scan list used to configure the A/D channels.

  int ch;
  int i;
  int nScans = 0;
  __u8 input;
  int temp, ret;
  __u8 options;
  char serial[9];
  __u32 period;
  __u16 version;

  __u16 value;
  __u16 sdataIn[512]; // holds 16 bit unsigned analog input data

  __u8 mode, gain, channel;

  udev = NULL;
  if ((udev = usb_device_find_USB_MCC(USB1608G_PID))) {
    printf("Success, found a USB 1608G!\n");
  } else {
    printf("Failure, did not find a USB 1608G!\n");
    return 0;
  }
  // some initialization
  usbInit_1608G(udev);
  usbBuildGainTable_USB1608G(udev, table_AIN);
  for (i = 0; i < NGAINS_1608G; i++) {
    printf("Gain: %d   Slope = %f   Offset = %f\n", i, table_AIN[i][0], table_AIN[i][1]);
  }

  while(1) {
    printf("\nUSB 1608G Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Output Scan\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 't' to test the timers\n");
    printf("Hit 'T' to get temperature\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB1608G(udev, options);
	break;
      case 'c':
        usbCounterInit_USB1608G(udev, COUNTER0);
        printf("Connect DIO0 to CTR0\n");
	usbDTristateW_USB1608G(udev, 0xf0);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB1608G(udev, 0x0);
	  usbDLatchW_USB1608G(udev, 0x1);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB1608G(udev, COUNTER0));
        break;      
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO[0-3] <--> DIO[4-7]\n");
	usbDTristateW_USB1608G(udev,0xf0);
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB1608G(udev));
	do {
          printf("Enter a byte number [0-0xf] : " );
          scanf("%x", &temp);
	  temp &= 0xf;
          usbDLatchW_USB1608G(udev, (__u16)temp);
	  temp = usbDLatchR_USB1608G(udev);
          input = usbDPort_USB1608G(udev) >> 4;
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 4; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'e':
        cleanup_USB1608G(udev);
        return 0;
      case 'i':
	printf("Input channel [0-7]: ");
	scanf("%hhd", &channel);
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: ",channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2V; break;
	  case '4': gain = BP_1V; break;
	  default:  gain = BP_10V; break;
	}
	mode = (LAST_CHANNEL | DIFFERENTIAL);
	list[0].range = gain;
        list[0].mode = mode;
	list[0].channel = channel;
	usbAInConfig_USB1608G(udev, list);
	for (i = 0; i < 20; i++) {
	  value = usbAIn_USB1608G(udev, channel);
	  value = rint(value*table_AIN[gain][0] + table_AIN[gain][1]);
	  printf("Channel %d  Mode = %#x  Gain = %d Sample[%d] = %#x Volts = %lf\n",
		 list[0].channel, list[0].mode, list[0].range, i, value, volts_USB1608G(udev,gain,value));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-1608G Analog Input Scan.\n");
        printf("Connect a signal to channels 0 and 1.\n");
	usbAInScanStop_USB1608G(udev);
	usbAInScanClearFIFO_USB1608G(udev);
        printf("Enter number of scans (less than 256): ");
        scanf("%d", &nScans);
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: ",channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2V; break;
	  case '4': gain = BP_1V; break;
	  default:  gain = BP_10V; break;
	}
        list[0].range = gain;
        list[0].mode = SINGLE_ENDED;
        list[0].channel = 0;
        list[1].range = gain;
        list[1].mode = (SINGLE_ENDED |LAST_CHANNEL);
        list[1].channel = 1;
	usbAInConfig_USB1608G(udev, list);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
	usbAInScanStart_USB1608G(udev, nScans, 0, frequency, 2*nScans - 1, 0x0);
	sleep(1);
	ret = usbAInScanRead_USB1608G(udev, nScans, 2, sdataIn);
	printf("Number bytes read = %d  (should be %d)\n", ret, 4*nScans);
	for (i = 0; i < 2*nScans; i++) {
	  sdataIn[i] = rint(sdataIn[i]*table_AIN[gain][0] + table_AIN[gain][1]);
	  printf("Scan = %d   Channel = %d  Mode = %d   Gain = %d Sample[%d] = %#x Volts = %lf\n", i/2, i%2,
  	   mode, gain, i, sdataIn[i], volts_USB1608G(udev,gain,sdataIn[i]));
	}
        break;
      case 'r':
	usbReset_USB1608G(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB1608G(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB1608G(udev));
	break;
      case 't':
        printf("Enter frequency of timer: ");
        scanf("%lf", &frequency);
	period = 64.E6/frequency - 1;
	usbTimerPeriodW_USB1608G(udev, period);
	usbTimerPulseWidthW_USB1608G(udev, period / 2);
	usbTimerCountW_USB1608G(udev, 0);
	usbTimerDelayW_USB1608G(udev, 0);
	usbTimerControlW_USB1608G(udev, 0x1);
	toContinue();
	usbTimerControlW_USB1608G(udev, 0x0);
        break;
      case 'T':
        usbTemperature_USB1608G(udev, &temperature);
	printf("Temperature = %.2f deg C  or  %.2f deg F \n", temperature, 9.0/5.0*temperature + 32.);
	break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB1608G(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
    default:
        break;
    }
  }
}

