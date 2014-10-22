/*
 *
 *  Copyright (c) 2004-2012  Warren Jasper <wjasper@tx.ncsu.edu>
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
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "pmd.h"
#include "usb-1608FS.h"

// delay for full speed devices in ms
#define FS_DELAY 100

void usbBuildCalTable_USB1608FS(HIDInterface* hid, Calibration_AIN table_AIN[NGAINS_1608FS][NCHAN_1608FS])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
       voltage = value*table[gain#][chan#][0] + table[gain#][chan#][1]
     only needed for fast lookup.
  */

  float y0, y1, y2, v0, v1, v2, x0, x1, x2;
  float m, b, target_sum, raw_sum, raw_sqr;
  int j;
  __u16 addr;
  __u16 data[2*NCHAN_1608FS], data1[NCHAN_1608FS];

  /* Read in the internal ground reference at 0x90 in the EEPROM */
  addr = 0x90;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v0);
  y0 = v0*65536./20. + 0x8000;  //Calculate the corresponding calibrated value y0 
  
  /* Read in the internal reference for +/- 10V at 0x80 in the EEPROM  (+5.0 Nonminal) */
  addr = 0x80;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v1);
  y1 = v1*65536./20. + 0x8000;  //  Calculate the corresponding calibrated value y1

  /* Read in the internal reference for +/- 10V at 0xa0 in the EEPROM  (-5.0 Nonminal) */
  addr = 0xa0;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v2);
  y2 = v2*65536./20. + 0x8000;  // Calculate the corresponding calibrated value y2 

  addr = 0xb0;         // +/- 10V Uncalibrated readings
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 32, (__u8 *) data);
  addr = 0x130;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 16, (__u8 *) data1);

  for (j = 0; j < NCHAN_1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[0][j].slope = m;   // slope
    table_AIN[0][j].offset = b;   // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./10. + 0x8000;

  /* Read in the internal reference for +/- 5V at 0x84 in the EEPROM */
  addr = 0x84;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v1);
  y1 = v1*65536./10. + 0x8000;    // Calculate the corresponding calibrated value y1.

  /* Read in the internal reference for +/- 5V at 0x9c in the EEPROM  (-2.5 Nonminal) */
  addr = 0x9c;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v2);
  y2 = v2*65536./10. + 0x8000;  // Calculate the corresponding calibrated value y2 

  addr = 0xd0;         // +/- 5V Uncalibrated readings
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 32, (__u8 *) data);
  addr = 0x140;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 16, (__u8 *) data1);

  for (j = 0; j < NCHAN_1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[1][j].slope = m;   // slope
    table_AIN[1][j].offset = b;   // intercept
  }
     
  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./4. + 0x8000;

  /* Read in the internal reference for +/- 2V at 0x88 in the EEPROM */
  addr = 0x88;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v1);
  y1 = v1*65536./4. + 0x8000;    // Calculate the corresponding calibrated value y1.

  /* Read in the internal reference for +/- 2V at 0x98 in the EEPROM  (-2.5 Nonminal) */
  addr = 0x98;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v2);
  y2 = v2*65536./4. + 0x8000;  // Calculate the corresponding calibrated value y2 
  
  addr = 0xf0;         // +/- 2V Uncalibrated readings
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 32, (__u8 *) &data);
  addr = 0x150;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 16, (__u8 *) data1);

  for (j = 0; j < NCHAN_1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[2][j].slope = m;   // slope
    table_AIN[2][j].offset = b;   // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./2. + 0x8000;
     
  /* Read in the internal reference for +/- 1V at 0x8c in the EEPROM */
  addr = 0x8c;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v1);
  y1 = v1*65536./2. + 0x8000;   // Calculate the corresponding calibrated value y1.

  /* Read in the internal reference for +/- 1V at 0x94 in the EEPROM  (-2.5 Nonminal) */
  addr = 0x94;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 4, (__u8 *) &v2);
  y2 = v2*65536./2. + 0x8000;   // Calculate the corresponding calibrated value y2 

  addr = 0x110;         // +/- 1V Uncalibrated readings
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 32, (__u8 *) &data);
  addr = 0x160;
  usbReadMemory_USB1608FS(hid, addr, EEPROM, 16, (__u8 *) data1);
  
  for (j = 0; j < NCHAN_1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[3][j].slope = m;   // slope
    table_AIN[3][j].offset = b;   // intercept
  }
  return;
}

/* configures digital port */
void usbDConfigPort_USB1608FS(HIDInterface* hid, __u8 direction)
{
  struct t_config_port {
    __u8 reportID;
    __u8 direction;
  } config_port;

  config_port.reportID = DCONFIG;
  config_port.direction = direction;

  PMD_SendOutputReport(hid, DCONFIG, (__u8*) &config_port, sizeof(config_port), FS_DELAY);
}

/* configures digital bit */
void usbDConfigBit_USB1608FS(HIDInterface* hid, __u8 bit_num, __u8 direction)
{
  struct t_config_bit {
    __u8 reportID;
    __u8 bit_num;      
    __u8 direction;
  } config_bit;

  config_bit.reportID = DCONFIG_BIT;
  config_bit.bit_num = bit_num;
  config_bit.direction = direction;

  PMD_SendOutputReport(hid, DCONFIG_BIT, (__u8*) &config_bit, sizeof(config_bit), FS_DELAY);
}

/* reads digital port  */
void usbDIn_USB1608FS(HIDInterface* hid, __u8* value)
{
  __u8 reportID = DIN;
  struct t_read_port {
    __u8 reportID;
    __u8 value;
  } read_port;

  PMD_SendOutputReport(hid, DIN, &reportID, 1, FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &read_port, sizeof(read_port), FS_DELAY);
  *value = read_port.value;
  return;
}

/* reads digital bit  */
void usbDInBit_USB1608FS(HIDInterface* hid, __u8 bit_num, __u8* value)
{
  struct t_read_bit {
    __u8 reportID;
    __u8 value;  // contains bit_number on send and value on receive.
  } read_bit;

  read_bit.reportID = DBIT_IN;
  read_bit.value = bit_num;

  PMD_SendOutputReport(hid, DBIT_IN, (__u8*) &read_bit, 2, FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &read_bit, sizeof(read_bit), FS_DELAY);
  *value = read_bit.value;
  return;
}

/* writes digital port */
void usbDOut_USB1608FS(HIDInterface* hid, __u8 value)
{
  struct t_write_port {
    __u8 reportID;
    __u8 value;
  } write_port;

  write_port.reportID = DOUT;
  write_port.value = value;

  PMD_SendOutputReport(hid, DOUT, (__u8*) &write_port, sizeof(write_port), FS_DELAY);
}

/* writes digital bit  */
void usbDOutBit_USB1608FS(HIDInterface* hid, __u8 bit_num, __u8 value)
{
  struct t_write_bit {
    __u8 reportID;
    __u8 bit_num;
    __u8 value;
  } write_bit;

  write_bit.reportID = DBIT_OUT;
  write_bit.bit_num = bit_num;
  write_bit.value = value;

  PMD_SendOutputReport(hid, DBIT_OUT, (__u8*) &write_bit, sizeof(write_bit), FS_DELAY);
  return;
}

/* reads from analog in */
__s16 usbAIn_USB1608FS(HIDInterface* hid, __u8 channel, __u8 range, Calibration_AIN table_AIN[NGAINS_1608FS][NCHAN_1608FS])
{
  __u16 data;
  __s16 value = 0;
  __u8 report[3];

  struct t_ain {
    __u8 reportID;
    __u8 channel;
    __u8 range;
  } ain;

  ain.reportID = AIN;
  ain.channel = channel;
  ain.range = range;
  if (channel > NCHAN_1608FS - 1) {
    printf("usbAIN: channel out of range for differential mode.\n");
    return -1;
  }
  if (range > 7) {
    printf("usbAIN: range setting too large.\n");
    return -1;
  }

  PMD_SendOutputReport(hid, AIN, (__u8*) &ain, sizeof(ain), FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) report, sizeof(report), FS_DELAY);

  data = (__u16) ( report[1] | (report[2] << 8));
  
  /* apply calibration correction 
        slope:  m = table_AIN[gain][channel].slope
        offset: b = table_AIN[gain][channel].offset
        correction = m*raw_value + b
  */

  switch (range) {
    case  BP_10_00V:
      value = (int) (table_AIN[0][channel].slope*((float) data) + table_AIN[0][channel].offset);
      break;
    case BP_5_00V:
      value = (int) (table_AIN[1][channel].slope*((float) data) + table_AIN[1][channel].offset);
      break;
    case BP_2_50V:  // use data from 2 V
    case BP_2_00V:
      value = (int) (table_AIN[2][channel].slope*((float) data) + table_AIN[2][channel].offset);
      break;
    case BP_1_25V:  // use data from 1 V
    case BP_1_00V:
    case BP_0_625V:
    case BP_0_3125V:
      value = (int) (table_AIN[3][channel].slope*((float) data) + table_AIN[3][channel].offset);
      break;
    default:
      break;
  }

  if (value >= 0x8000) {
    value -=  0x8000;
  } else {
    value = (0x8000 - value);
    value *= (-1);
  }

  return value;
}

void usbAInStop_USB1608FS(HIDInterface* hid)
{
  __u8 reportID = AIN_STOP;;

  PMD_SendOutputReport(hid, AIN_STOP, &reportID, sizeof(reportID), FS_DELAY);
}

int usbAInScan_USB1608FS(HIDInterface* hid[], __u8 lowchannel, __u8 highchannel, __u32 num_samples,
			 float *frequency, __u8 options, __s16 sdata[], Calibration_AIN table_AIN[NGAINS_1608FS][NCHAN_1608FS],
			 __u8 gainArray[NCHAN_1608FS])
{
  /*
    Burst I/O mode will sample data to the onboard SRAM FIFO until
    full, and then return the data in continuous messages using all 5
    endpoints.  Prescaler values above 1:8 are not allowed in burst
    I/O mode.  Single execution and immediate transfer bits will be
    ignored in this mode.

    Immediate transfer mode is used for low sampling rates to avoid
    delays in receiving the sampled data.  The data will be sent at
    the end of every timer period, rather than waiting for the buffer
    to fill.  All 6 endpoints will still be used in a sequential
    manner.  This mode should not be used if the aggregate sampling
    rate is greater then 32,000 samples per second in order to avoid
    data loss.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data the
    specified rate.  No messages will be sent until the trigger is
    detected.

    External sync may be used to synchronize the sampling of multiple
    USB-1608FS devices, or to sample data using an external clock.
    The device must be set to be a sync slave with the usbSetSync
    command prior to using this mode.  Data will be acquired on all
    specified channels when the sync edge is detected.
   */

  int i, k;
  int pipe, ret;
  __u32 preload;
  __u32 nscans;
  int nchan;
  int timeout = FS_DELAY;

  struct t_data{
    __u16 value[31];        // 31 16-bit samples
    __u16 scan_index;       //  1 16 bit scan count
  } data;
    
  struct arg {
    __u8  reportID;
    __u8  lowchannel;
    __u8  highchannel;
    __u8  count[4];
    __u8  prescale;
    __u8  preload[2];
    __u8  options;
  } arg;

  if (highchannel > 7) {
    printf("usbAInScan: highchannel out of range.\n");
    return -1;
  }

  if (lowchannel > 7) {
    printf("usbAInScan: lowchannel out of range.\n");
    return -1;
  }

  if (lowchannel > highchannel) {
    printf("usbAInScan: lowchannel greater than highchannel.\n");
    return -1;
  }

  nchan = highchannel - lowchannel + 1;  // total number of channels in a scan.

  // nscans is the number of scans to perform.
  if (options & AIN_TRANSFER_MODE) {
    // return after the end of each scan of the channels.
    nscans = num_samples;
    timeout = ((1000) / *frequency) + FS_DELAY;
  } else {
    // return after 31 samples have been collected.
    nscans = ((num_samples - 1)*nchan)/31 + 1;
    timeout = 31*(1000 / *frequency) + FS_DELAY;
  }
  num_samples *= nchan;

  if (options & AIN_BURST_MODE) {
    timeout = ((nscans*1000) / *frequency) + FS_DELAY;
  }

  if (options & AIN_TRIGGER) {
    timeout = 0;  // set wait forever
  }

  arg.reportID = AIN_SCAN;
  arg.lowchannel = lowchannel;
  arg.highchannel = highchannel;
  arg.count[0] = (__u8) nscans & 0xff;           // low byte
  arg.count[1] = (__u8) (nscans >>  8) & 0xff;
  arg.count[2] = (__u8) (nscans >> 16) & 0xff;
  arg.count[3] = (__u8) (nscans >> 24) & 0xff;   // high byte
  arg.options = options;

  for (arg.prescale = 0; arg.prescale <= 8; arg.prescale++) {
    preload = 1.0e7/((*frequency) * (1<<arg.prescale));
    if (preload <= 0xffff) {
      arg.preload[0] = (__u8) preload & 0xff;          // low byte
      arg.preload[1] = (__u8) (preload >> 8) & 0xff;   // high byte
      break;
    }
  }

  if (arg.prescale == 9 || preload == 0) {
    printf("usbAInScan_USB1608FS: frequency out of range.\n");
    return -1;
  }

  *frequency = 1.0e7/(preload*(1<<arg.prescale));
  PMD_SendOutputReport(hid[0], AIN_SCAN, (__u8 *) &arg, sizeof(arg), FS_DELAY);

  pipe = 1;  // Initial Endpoint to receive data.
  for (i = 0; i < nscans; i++, pipe = (pipe)%6 + 1) {  //pipe should take the values 1-6
    ret = usb_interrupt_read(hid[pipe]->dev_handle, USB_ENDPOINT_IN |(pipe+2), (char *) &data, sizeof(data), timeout);
    if (ret < 0) {
      perror("usbAInScan_USB1608FS");
      return 1;
    }
    //printf("AInScan_USB1608FS: ret = %d  scan = %d    num_samples = %d       scan_index = %d  Endpoint = %#x\n",
    //	   ret, i, num_samples, data.scan_index, USB_ENDPOINT_IN |(pipe+2));	
    if (options & AIN_TRANSFER_MODE) {
      for (k = 0; k < nchan; k++) {
	if (data.value[k] >= 0x8000) {
	  sdata[data.scan_index*nchan+k] = (data.value[k] - 0x8000);
	} else {
	  sdata[data.scan_index*nchan+k] = (0x8000 - data.value[k]);
	  sdata[data.scan_index*nchan+k] *= (-1);
	}
      }
      num_samples -= nchan;
      if (num_samples == 0) break;
    } else {
      // not in Immediate Transfer Mode.  Read 31 or fewer samples per scan.	
      if (num_samples > 31) {
        for (k = 0; k < 31;  k++) {
  	  if (data.value[k] >= 0x8000) {
	    sdata[data.scan_index*31+k] = (data.value[k] - 0x8000);
  	  } else {
	    sdata[data.scan_index*31+k] = (0x8000 - data.value[k]);
	    sdata[data.scan_index*31+k] *= (-1);
	  }
	}
	num_samples -= 31;
      } else {   // only copy in a partial scan
	for (k = 0; k < num_samples;  k++) {
	  if (data.value[k] >= 0x8000) {
	    sdata[data.scan_index*31+k] = (data.value[k] - 0x8000);
	  } else {
	    sdata[data.scan_index*31+k] = (0x8000 - data.value[k]);
	    sdata[data.scan_index*31+k] *= (-1);
	  }
	}
	num_samples = 0 ;
	break;
      }
    }
  }

  usbAInStop_USB1608FS(hid[0]);
  // Make sure all the buffers are flushed:
  for (pipe = 1; pipe <= 6; pipe++) {
    ret = usb_interrupt_read(hid[pipe]->dev_handle, USB_ENDPOINT_IN |(pipe+2), (char *) &data, sizeof(data), 10);
  }
  return nscans;
}


void usbAInLoadQueue_USB1608FS(HIDInterface* hid, __u8 gainArray[8])
{
  struct {
    __u8 reportID;
    __u8 gainArray[8];
  } in;

  in.reportID = ALOAD_QUEUE;
  memcpy(in.gainArray, gainArray, 8);
  
  PMD_SendOutputReport(hid, ALOAD_QUEUE, (__u8 *) &in, sizeof(in), FS_DELAY);
}

/* Initialize the counter */
void usbInitCounter_USB1608FS(HIDInterface* hid)
{
  __u8 cmd[1];

  cmd[0] = CINIT;

  PMD_SendOutputReport(hid, CINIT, cmd, sizeof(cmd), FS_DELAY);
}

__u32 usbReadCounter_USB1608FS(HIDInterface* hid)
{
   __u32 value;
  struct t_counter {
    __u8 reportID;
    __u8 value[4];
  } counter;

  counter.reportID = CIN;

  PMD_SendOutputReport(hid, CIN, (__u8*) &counter, 1, FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &counter, sizeof(counter), FS_DELAY);
  value =   counter.value[0] | (counter.value[1] << 8) |
    (counter.value[2] << 16) | (counter.value[3] << 24);

  return value;
}

/* blinks the LED of USB device */
void usbBlink_USB1608FS(HIDInterface* hid)
{
  __u8 reportID = BLINK_LED;

  PMD_SendOutputReport(hid, BLINK_LED, &reportID, sizeof(reportID), FS_DELAY);
}

int usbReset_USB1608FS(HIDInterface* hid)
{
  __u8 reportID = RESET;

  return PMD_SendOutputReport(hid, RESET, &reportID, sizeof(reportID), FS_DELAY);
}

void usbSetTrigger_USB1608FS(HIDInterface* hid, __u8 type)
{
  __u8 cmd[2];
  
  cmd[0] = SET_TRIGGER;
  cmd[1] = type;
  
  PMD_SendOutputReport(hid, SET_TRIGGER, cmd, sizeof(cmd), FS_DELAY);
}

void usbSetSync_USB1608FS(HIDInterface* hid, __u8 type)
{
  /*
    This command configures the sync signal.  The sync signal may be
    used to synchronize the analog input scan of multiple PMD-160FS
    devices.  When multiple devices are to be used, one device is
    selected as the master and the rest as slaves.  The sync signal of
    all devices must be wired together.  The master will output a
    pulse every sample, and all of the devices will acquire their
    samples simultaneously. This may also be used to pace one or more
    PMD-1608 devices from an external TTL/CMOS clock signal (max rate = 50 kHz).

    This may also be used with an external trigger.  The external
    trigger signal should be brought to the master device, and all
    devices will begin sampling when the master is triggered.

    If a device is configured as a slave, it will not acquire data
    given an AInScan command until it detects a pulse on the sync
    input.
   */
  __u8 cmd[2];

  cmd[0] = SET_SYNC;
  cmd[1] = type;   // 0 = master, 1 = slave
  
  PMD_SendOutputReport(hid, SET_SYNC, cmd, sizeof(cmd), FS_DELAY);
}

__u16 usbGetStatus_USB1608FS(HIDInterface* hid)
{
  /*
    This command retrieves the status of the device.

    Bit 0: 0 = Sync slave,   1 = sync master
    Bit 1: 0 = trigger falling edge,  1 = trigger rising edge
    Bit 2-14:  Not used
    Bit 15:   1 = program memory update mode
   */
  __u16 status;
    
  struct t_statusReport {
  __u8 reportID;
  __u8 status[2];
  } statusReport;

  statusReport.reportID = GET_STATUS;

  PMD_SendOutputReport(hid, GET_STATUS, &statusReport.reportID, 1, FS_DELAY);
  do {
    statusReport.reportID = 0;
    usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		       (char *) &statusReport, sizeof(statusReport), FS_DELAY);
  } while ( statusReport.reportID != GET_STATUS);
  status = (__u16) (statusReport.status[0] | (statusReport.status[1] << 8));

  return status;
}

void usbReadMemory_USB1608FS(HIDInterface* hid, __u16 address, __u8 type, __u8 count, __u8* memory)
{
  // Addresses 0x000 - 0x07F are reserved for firmware data
  // Addresses 0x080 - 0x3FF are available for use as calibraion or user data

  int ret;

  struct arg_t {
    __u8 reportID;
    __u8 address[2];
    __u8 type;
    __u8 count;
  } arg;

  struct memRead_t {
    __u8 reportID;
    __u8 data[62];
    __u8 pad[2];
  } memRead;

  if (count > 62) count = 62;
  arg.reportID = MEM_READ;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte
  arg.type = type;
  arg.count = count;

  ret = PMD_SendOutputReport(hid, MEM_READ, (__u8 *) &arg, (int) sizeof(arg), FS_DELAY);
  if (ret < 0) {
    printf("Error in usbReadMemory_USB1608FS: PMD_SendOutputReport() returned %d.\n", ret);
  }
  
  memRead.reportID = 0x0;
  // always read 64 bytes regardless.  Only the first count are meaningful.
  ret = usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2, (char *) &memRead, 64, FS_DELAY);
  if (ret <= 0) {
    printf("Error in usbReadMemory_USB1608FS: usb_interrupt_read() returned %d.\n", ret);
    printf("Address = %#x  Count = %d  Number of bytes read = %d \n", address, count, ret);
  }
  if (memRead.reportID != MEM_READ) {
    printf("Error in Reading Memory from EEPROM!\n");
    PMD_SendOutputReport(hid, MEM_READ, (__u8 *) &arg, sizeof(arg), FS_DELAY);
    usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2, (char *) &memRead, count+1, FS_DELAY);
  }
  memcpy(memory, &memRead.data[0], count);
}

int usbWriteMemory_USB1608FS(HIDInterface* hid, __u16 address, __u8 count, __u8* data)
{
  // Locations 0x00-0x7F are reserved for firmware and my not be written.
  int i;
  struct t_mem_write_report {
    __u8 reportID;
    __u8 address[2];
    __u8 count;
    __u8 data[count];
  } arg;

  if (address <=0x7f) return -1;
  if (count > 59) count = 59;

  arg.reportID = MEM_WRITE;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte

  arg.count = count;
  for ( i = 0; i < count; i++ ) {
    arg.data[i] = data[i];
  }
  PMD_SendOutputReport(hid, MEM_WRITE, (__u8 *) &arg, sizeof(arg), FS_DELAY);
  return 0;
}

void usbGetAll_USB1608FS(HIDInterface* hid, __u8 data[])
{
  __u8 reportID = GET_ALL;
  struct t_get_all {
    __u8 reportID;
    __u8 values[19];
  } get_all;

  PMD_SendOutputReport(hid, GET_ALL, &reportID, sizeof(reportID), FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &get_all, sizeof(get_all), FS_DELAY);
  memcpy(data, get_all.values, 19);
  return;
}

float volts_USB1608FS( const int gain, const signed short num )
{
  float volt = 0.0;
  
  switch( gain ) {
    case BP_10_00V:
      volt = num * 10.0 / 0x7fff;
      break;
    case BP_5_00V:
      volt = num * 5.00 / 0x7fff;
      break;
    case BP_2_50V:
      volt = num * 2.50 / 0x7fff;
      break;
    case BP_2_00V:
      volt = num * 2.00 / 0x7fff;
      break;
    case BP_1_25V:
      volt = num * 1.25 / 0x7fff;
      break;
    case BP_1_00V:
      volt = num * 1.0 / 0x7fff;
      break;
    case BP_0_625V:
      volt = num * 0.625 / 0x7fff;
      break;
    case BP_0_3125V:
      volt = num * 0.3125 / 0x7fff;
      break;
  }

  return volt;
}
