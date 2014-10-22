/*
 *
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
#include "usb-1208HS.h"

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
  double voltage;
  double frequency;
  double sine[512];
  float temperature;

  float table_AIN[NMODE][NGAINS_1208HS][2];
  float table_AO[NCHAN_AO_1208HS][2];

  int flag;
  int usb1208HS_2AO = FALSE;
  int usb1208HS_4AO = FALSE;
  usb_dev_handle *udev = NULL;
  int i, j;
  int temp, input;
  __u8 gain, mode, channel;
  int ch;
  int ret;
  
  __u32 period;
  __u16 version;
  __u16 sdataOut[256];   // holds 12 bit unsigned analog output data
  __u16 sdataIn[512]; // holds 13 bit unsigned analog input data
  char serial[9];
  __u8 range[NCHAN_1208HS];
  __u16 value;

  udev = NULL;
  if ((udev = usb_device_find_USB_MCC(USB1208HS_PID))) {
    printf("Success, found a USB 1208HS!\n");
  } else if ((udev = usb_device_find_USB_MCC(USB1208HS_2AO_PID))) {
    printf("Success, found a USB 1208HS-2AO!\n");
    usb1208HS_2AO = TRUE;
  } else if ((udev = usb_device_find_USB_MCC(USB1208HS_4AO_PID))) {
    printf("Success, found a USB 1208HS-4AO!\n");
    usb1208HS_4AO = TRUE;
  } else {
    printf("Failure, did not find a USB 1208HS, 1208HS-2AO or 1208HS-4AO!\n");
    return 0;
  }

  usbInit_1208HS(udev);
  usbBuildGainTable_USB1208HS(udev, table_AIN);
  for (i = 0; i < NMODE; i++ ) {
    for (j = 0; j < NGAINS_1208HS; j++) {
      printf("Mode = %d   Gain: %d    Slope = %f    Offset = %f\n", i, j, table_AIN[i][j][0], table_AIN[i][j][1]);
    }
  }

  if (usb1208HS_4AO) {
    usbBuildGainTable_USB1208HS_4AO(udev, table_AO);
    printf("\n");
    for (i = 0; i < NCHAN_AO_1208HS; i++) {
      printf("VDAC%d:    Slope = %f    Offset = %f\n", i, table_AO[i][0], table_AO[i][1]);
    }
  }

  while(1) {
    printf("\nUSB 1208HS Testing\n");
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
	scanf("%d", &temp);
        usbBlink_USB1208HS(udev, temp);
        break;
      case 'c':
        usbCounterInit_USB1208HS(udev, COUNTER0);
        printf("Connect DIO0 to CTR0\n");
	usbDTristateW_USB1208HS(udev, 0xff00);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB1208HS(udev, 0x0);
	  usbDLatchW_USB1208HS(udev, 0x1);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB1208HS(udev, COUNTER0));
        break;      
      case 'd':
	printf("\nTesting Digital I/O....\n");
	printf("connect pins DIO 0 <=>  DIO 8\n");
	printf("             DIO 1 <=>  DIO 9\n");
	printf("             DIO 2 <=>  DIO 10\n");
	printf("             DIO 3 <=>  DIO 11\n");
	printf("             DIO 4 <=>  DIO 12\n");
	printf("             DIO 5 <=>  DIO 13\n");
	printf("             DIO 6 <=>  DIO 14\n");
	printf("             DIO 7 <=>  DIO 15\n");
	usbDTristateW_USB1208HS(udev, 0xff00);
	do {
          printf("Enter a byte number [0-0xff] : " );
          scanf("%x", &temp);
          usbDLatchW_USB1208HS(udev, (__u16)temp);
	  temp = usbDLatchR_USB1208HS(udev);
          input = usbDPort_USB1208HS(udev);
	  input >>= 8;
          printf("The number you entered = %#x  Latched value = %#x\n\n", input, temp);
	} while (toContinue());
	break;
      case 'e':
        cleanup_USB1208HS(udev);
        return 0;
	break;
      case 'i':
	printf("Input channel [0-7]: ");
	scanf("%hhd", &channel);
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2.5V  4 = 0-10V Single Ended: ",
	       channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2_5V; break;
	  case '4': gain = UP_10V; break;
	  default:  gain = BP_10V; break;
	}
	mode = SINGLE_ENDED;
	for (i = 0; i < NCHAN_1208HS; i++) {
	  range[i] = gain;
	}
	usbAInConfig_USB1208HS(udev, mode, range);
	for (i = 0; i < 20; i++) {
	  value = usbAIn_USB1208HS(udev, channel);
	  value = rint(value*table_AIN[mode][gain][0] + table_AIN[mode][gain][1]);
	  printf("Channel %d  Mode = %d  Gain = %d Sample[%d] = %#x Volts = %lf\n", channel,
		 mode, gain, i, value, volts_USB1208HS(udev,mode,gain,value));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-1208HS Analog Input Scan.\n");
	usbAInScanStop_USB1208HS(udev);
	printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2.5V  4 = 0-10V Single Ended: ", channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2_5V; break;
	  case '4': gain = UP_10V; break;
	  default:  gain = BP_10V; break;
	}
	mode = SINGLE_ENDED;
	for (i = 0; i < NCHAN_1208HS; i++) {
	  range[i] = gain;
	}
	usbAInConfig_USB1208HS(udev, mode, range);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
	usbAInScanStart_USB1208HS(udev, 512, 0, frequency, (0x1<<channel), 0xff, 0);
	usbAInScanRead_USB1208HS(udev, 512, 1, sdataIn);
	for (i = 0; i < 512; i++) {
	  sdataIn[i] = rint(sdataIn[i]*table_AIN[mode][gain][0] + table_AIN[mode][gain][1]);
	  printf("Channel %d  Mode = %d  Gain = %d Sample[%d] = %#x Volts = %lf\n", channel,
		 mode, gain, i, sdataIn[i], volts_USB1208HS(udev,mode,gain,sdataIn[i]));
	}
        break;
      case 'o':
        if (!(usb1208HS_4AO || usb1208HS_2AO)) {
	  printf("Analog output only on the USB-1208HS-[24]AO model.\n");
  	  break;
        }
        printf("Enter voltage: ");
	scanf("%lf", &voltage);
	usbAOut_USB1208HS(udev, 0, voltage, table_AO);
	usbAOutR_USB1208HS(udev, 0, &voltage, table_AO);
	printf("Analog Output Voltage = %f V\n", voltage);
        break;
      case 'O':
	if (!(usb1208HS_4AO || usb1208HS_2AO)) {
	  printf("Analog output only on the USB-1208HS-[24]AO model.\n");
	  break;
	}
	channel = 0;
	printf("Test of Analog Output Scan.\n");
	printf("Hook scope up to VDAC 0\n");
	printf("Enter desired frequency of sine wave [Hz]: ");
	scanf("%lf", &frequency);
	for (i = 0; i < 256; i ++) {
	  sine[i] = 10.0*sin(2.*M_PI*i/128.);
          sdataOut[i] = voltsTou12_USB1208HS_AO(sine[i], 0, table_AO);
	  // printf("sin[%d] = %f    sdataOut[%d] = %#x\n", i, sine[i], i, sdataOut[i]);
	}
	// usbAOutScanStop_USB1208HS(udev);
	// usbAOutScanClearFIFO_USB1208HS(udev);
	usbAOutScanStart_USB1208HS(udev, 0, 0, 128.*frequency,  AO_CHAN0);
	printf("status = %#x\n",  usbStatus_USB1208HS(udev));
	printf("Hit \'s <CR>\' to stop ");
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  ret = usb_bulk_write(udev, USB_ENDPOINT_OUT|2, (char *) sdataOut, sizeof(sdataOut), 400);
	  // printf("ret = %d  status = %#x\n", ret, usbStatus_USB1208HS(udev));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAOutScanStop_USB1208HS(udev);
	break;
      case 'r':
	usbReset_USB1208HS(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB1208HS(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB1208HS(udev));
	break;
      case 't':
        printf("Enter frequency of timer: ");
        scanf("%lf", &frequency);
	period = 40.E6/frequency - 1;
	usbTimerPeriodW_USB1208HS(udev, period);
	usbTimerPulseWidthW_USB1208HS(udev, period / 2);
	usbTimerCountW_USB1208HS(udev, 0);
	usbTimerDelayW_USB1208HS(udev, 0);
	usbTimerControlW_USB1208HS(udev, 0x1);
	toContinue();
	usbTimerControlW_USB1208HS(udev, 0x0);
        break;
      case 'T':
        usbTemperature_USB1208HS(udev, &temperature);
	printf("Temperature = %.2f deg C  or  %.2f deg F \n", temperature, 9.0/5.0*temperature + 32.);
	break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB1208HS(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
    }
  }
}

