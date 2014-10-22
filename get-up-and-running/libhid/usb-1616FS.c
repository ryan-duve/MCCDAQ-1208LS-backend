/*
 *
 *  Copyright (c) 2005-2009  Warren Jasper <wjasper@tx.ncsu.edu>
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
#include "usb-1616FS.h"

#define FS_DELAY 300

void usbBuildCalTable_USB1616FS(HIDInterface* hid, float table_AIN[NGAINS_1616FS][NCHAN_1616FS][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
       voltage = value*table[gain#][chan#][0] + table[gain#][chan#][1]
     only needed for fast lookup.
  */

  float y0, y1, v0, v1, x0, x1;
  int j;
  __u16 addr;
  __u16 data[32];

  /* Read in the internal ground reference at 0x90 in the EEPROM */
  addr = 0x90;
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 4, (__u8 *) &v0);

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./20. + 0x8000;
  
  /* Read in the internal reference for +/- 10V at 0x80 in the EEPROM */
  addr = 0x80;
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 4, (__u8 *) &v1);

  /* Calculate the corresponding calibrated value y1 */
  y1 = v1*65536./20. + 0x8000;

  addr = 0xb0;         // +/- 10V Uncalibrated readings
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) data);
  addr += 32;
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) &data[16]);
  for (j = 0; j < NCHAN_1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[0][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[0][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./10. + 0x8000;

  /* Read in the internal reference for +/- 5V at 0x84 in the EEPROM */
  addr = 0x84;
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 4, (__u8 *) &v1);

  /* Calculate the corresponding calibrated value y1. */
  y1 = v1*65536./10. + 0x8000;

  addr = 0xf0;         // +/- 5V Uncalibrated readings
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) data);
  addr += 32;			    
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) &data[16]);
  for (j = 0; j < NCHAN_1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[1][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[1][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }
     
  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./4. + 0x8000;

  /* Read in the internal reference for +/- 2V at 0x88 in the EEPROM */
  addr = 0x88;
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 4, (__u8 *) &v1);

  /* Calculate the corresponding calibrated value y1. */
  y1 = v1*65536./4. + 0x8000;
  
  addr = 0x130;         // +/- 2V Uncalibrated readings
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) &data);
  addr += 32;			    
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) &data[16]);
  for (j = 0; j < NCHAN_1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[2][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[2][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./2. + 0x8000;
     
  /* Read in the internal reference for +/- 1V at 0x8c in the EEPROM */
  addr = 0x8c;
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 4, (__u8 *) &v1);

  /* Calculate the corresponding calibrated value y1. */
  y1 = v1*65536./2. + 0x8000;

  addr = 0x170;         // +/- 1V Uncalibrated readings
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) &data);
  addr += 32;			    
  usbReadMemory_USB1616FS(hid, addr, EEPROM, 32, (__u8 *) &data[16]);
  for (j = 0; j < NCHAN_1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[3][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[3][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }
  
  return;
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/


/* configures digital port */
void usbDConfigPort_USB1616FS(HIDInterface* hid, __u8 direction)
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
void usbDConfigBit_USB1616FS(HIDInterface* hid, __u8 bit_num, __u8 direction)
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
__u8 usbDIn_USB1616FS(HIDInterface* hid)
{
  __u8 reportID = DIN;
  struct t_read_port {
    __u8 reportID;
    __u8 value;
  } read_port;

  PMD_SendOutputReport(hid, DIN, &reportID, 1, FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &read_port, sizeof(read_port), FS_DELAY);
  return (read_port.value);
}

/* reads digital bit  */
__u8 usbDInBit_USB1616FS(HIDInterface* hid, __u8 bit_num)
{
  struct t_read_bit {
    __u8 reportID;
    __u8 value;
  } read_bit;

  read_bit.reportID = DBIT_IN;
  read_bit.value = bit_num;

  PMD_SendOutputReport(hid, DBIT_IN, (__u8*) &read_bit, 2, FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &read_bit, sizeof(read_bit), FS_DELAY);
  return  (read_bit.value);
}

/* writes digital port */
void usbDOut_USB1616FS(HIDInterface* hid, __u8 value)
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
void usbDOutBit_USB1616FS(HIDInterface* hid, __u8 bit_num, __u8 value)
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
__s16 usbAIn_USB1616FS(HIDInterface* hid, __u8 channel, __u8 range, float table_AIN[NGAINS_1616FS][NCHAN_1616FS][2])
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
  if (channel > NCHAN_1616FS - 1 ) {
    printf("usbAIN: channel out of range for differential mode.\n");
    return -1;
  }
  if (range > 7) {
    printf("usbAIN: range setting too large.\n");
    return -1;
  }

  PMD_SendOutputReport(hid, AIN, (__u8*) &ain, sizeof(ain), FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2, (char *) report, sizeof(report), FS_DELAY);

  data = (__u16) ( report[1] | (report[2] << 8));

  /* apply calibration correction 
        slope:  m = table_AIN[gain][channel][0]
        offset: b = table_AIN[gain][channel][1]
        correction = m*raw_value + b
  */

  switch (range) {
    case  BP_10_00V:
      value = (int) (table_AIN[0][channel][0]*((float) data) + table_AIN[0][channel][1]);
      break;
    case BP_5_00V:
      value = (int) (table_AIN[1][channel][0]*((float) data) + table_AIN[1][channel][1]);
      break;
    case BP_2_50V:  // use data from 2 V
    case BP_2_00V:
      value = (int) (table_AIN[2][channel][0]*((float) data) + table_AIN[2][channel][1]);
      break;
    case BP_1_25V:  // use data from 1 V
    case BP_1_00V:
    case BP_0_625V:
    case BP_0_3125V:
      value = (int) (table_AIN[3][channel][0]*((float) data) + table_AIN[3][channel][1]);
      break;
    default:
      break;
  }
	 
  if ( value >= 0x8000 ) {
    value -=  0x8000;
  } else {
    value = (0x8000 - value);
    value *= (-1);
  }

  return value;
}

void usbAInStop_USB1616FS(HIDInterface* hid)
{
  __u8 reportID = AIN_STOP;;

  PMD_SendOutputReport(hid, AIN_STOP, &reportID, sizeof(reportID), FS_DELAY);
}

int usbAInScan_USB1616FS(HIDInterface* hid[], __u8 lowchannel, __u8 highchannel, __u32 nScan,
			 float *frequency, __u8 options, __s16 sdata[], float table_AIN[NGAINS_1616FS][NCHAN_1616FS][2], __u8 gainArray[NCHAN_1616FS])
{
  int nSamples = 0;  // current number of samples taken;
  int tSamples;      // total number of samples to take;
  int bytes;         // number of bytes read from USB endpoint (64 bytes)
  int i;
  int chan = lowchannel;
  int ep;
  __u32 preload;

  __s16 value;
  __u16 uvalue;
  __u8 gain;    

  struct t_data{
    __u16 scan_index0;       //    16 bit scan count
    __u16 value0[31];        // 31 16-bit samples
    __u16 scan_index1;       //    16 bit scan count
    __u16 value1[31];        // 31 16-bit samples
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

  if (highchannel > NCHAN_1616FS - 1) {
    printf("usbAInScan: highchannel out of range.\n");
    return -1;
  }
  if (lowchannel > NCHAN_1616FS - 1) {
    printf("usbAInScan: lowchannel out of range.\n");
    return -1;
  }
  if (highchannel >= lowchannel) {
    chan = lowchannel;
  } else {
    printf("usbAINScan: lowchannel can not be greater than higchannel.\n");
    return -1;
  }

  arg.reportID = AIN_SCAN;
  arg.lowchannel = lowchannel;
  arg.highchannel = highchannel;
  arg.count[0] = (__u8) nScan & 0xff;           // low byte
  arg.count[1] = (__u8) (nScan >>  8) & 0xff;
  arg.count[2] = (__u8) (nScan >> 16) & 0xff;
  arg.count[3] = (__u8) (nScan >> 24) & 0xff;   // high byte
  arg.options = options;                        

  for ( arg.prescale = 0; arg.prescale <= 8; arg.prescale++ ) {
    preload = 10e6/((*frequency) * (1<<arg.prescale));
    if ( preload <= 0xffff ) {
      arg.preload[0] = (__u8) preload & 0xff;          // low byte
      arg.preload[1] = (__u8) (preload >> 8) & 0xff;   // high byte
      break;
    }
  }

  *frequency = 10.e6/(preload*(1<<arg.prescale));

  if ( arg.prescale == 9 || preload == 0) {
    printf("usbAInScan_PMD1608FS: frequency out of range.\n");
    return -1;
  }

  tSamples = nScan*(highchannel - lowchannel + 1);
  printf("total Samples = %d\n", tSamples);
  usbAInStop_USB1616FS(hid[0]);
  PMD_SendOutputReport(hid[0], AIN_SCAN, (__u8 *) &arg, sizeof(arg), FS_DELAY);
  ep = 0;   //cycle through the endpoints 1 - 6;

  while (tSamples > nSamples) {
    bytes = usb_interrupt_read(hid[ep+1]->dev_handle, USB_ENDPOINT_IN |(ep+3), (char *) &data, sizeof(data), FS_DELAY);
    if (bytes < 0 ) {
      printf("usbAInScan: usb_interrupt_read error. scan index = %d return code = %d\n", data.scan_index0, bytes);
    }
    for (i = 0; i < 31; i++) {
      if (nSamples == tSamples) goto cont;
      sdata[nSamples++] = data.value0[i];
      //       printf("sample # = %d   ep = %d raw value of data = %#x  %#x index = %hd  bytes = %d\n",
      //      	     nSamples, ep+1, data.value0[i], sdata[i], data.scan_index0, bytes);

    }
    for (i = 0; i < 31; i++) {
      if (nSamples == tSamples) goto cont;
      sdata[nSamples++] = data.value1[i];
      //       printf("sample # = %d   ep = %d raw value of data = %#x  %#x index = %hd  bytes = %d\n",
      //      	     nSamples, ep+1, data.value1[i], sdata[i], data.scan_index1, bytes);

    }
    ep = (ep + 1) % 6;  // increment to the next endpoint
  }

cont:  
  chan = lowchannel;
  for (i = 0; i < nSamples; i++) {
    /* apply calibration correction 
       slope:  m = table_AIN[gain][channel][0]
       offset: b = table_AIN[gain][channel][1]
       correction = m*raw_value + b
    */
    switch (gainArray[chan]) {
      case  BP_10_00V:
	gain = 0;
	break;
      case BP_5_00V:
	gain = 1;
	break;
      case BP_2_50V:  // use data from 2 V
      case BP_2_00V:
	gain = 2;
	break;
      case BP_1_25V:  // use data from 1 V
      case BP_1_00V:
      case BP_0_625V:
      case BP_0_3125V:
	gain = 3;
	break;
      default:
	gain = 0;
	break;
    }

    //    printf("sample = %d  chan = %d  gain = %d sdata = %#x\n", i, chan, gainArray[chan], sdata[i]);
    uvalue = sdata[i];
    value = (int) (table_AIN[gain][chan][0]*((float) uvalue) + table_AIN[gain][chan][1]);

    if ( value >= 0x8000 ) {
      value -=  0x8000;
    } else {
      value = (0x8000 - value);
      value *= (-1);
    }
    sdata[i] = value;
    chan++;                    // each sample is one channel higher.
    if (chan > highchannel) {  // wrap around which is one scan.
      chan = lowchannel;
    }
  }	

  printf("Total number of samples returned = %d\n", nSamples);
  usbAInStop_USB1616FS(hid[0]);
  return nSamples;
}

void usbAInLoadQueue_USB1616FS(HIDInterface* hid, __u8 gainArray[NCHAN_1616FS])
{
   struct {
    __u8 reportID;
    __u8 gainArray[NCHAN_1616FS];
  } in;

  in.reportID = ALOAD_QUEUE;
  memcpy(in.gainArray, gainArray, NCHAN_1616FS);
  
  PMD_SendOutputReport(hid, ALOAD_QUEUE, (__u8 *) &in, sizeof(in), FS_DELAY);
}

/* Initialize the counter */
void usbInitCounter_USB1616FS(HIDInterface* hid)
{
  __u8 cmd[1];

  cmd[0] = CINIT;

  PMD_SendOutputReport(hid, CINIT, cmd, sizeof(cmd), FS_DELAY);
}

__u32 usbReadCounter_USB1616FS(HIDInterface* hid)
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
void usbBlink_USB1616FS(HIDInterface* hid)
{
  __u8 reportID = BLINK_LED;

  PMD_SendOutputReport(hid, BLINK_LED, &reportID, sizeof(reportID), FS_DELAY);
}

int usbReset_USB1616FS(HIDInterface* hid)
{
  __u8 reportID = RESET;

  return PMD_SendOutputReport(hid, RESET, &reportID, sizeof(reportID), FS_DELAY);
}

void usbSetTrigger_USB1616FS(HIDInterface* hid, __u8 type)
{
  __u8 cmd[2];
  
  cmd[0] = SET_TRIGGER;
  cmd[1] = type;
  
  PMD_SendOutputReport(hid, SET_TRIGGER, cmd, sizeof(cmd), FS_DELAY);
}

void usbSetSync_USB1616FS(HIDInterface* hid, __u8 type)
{
  /*
    This command configures the sync signal.  The sync signal may be used to synchronize the
    analog input scan of multiple devices.  When multiple devices are to be used, one device is
    selected as the master and the rest as slaves.  The sync signal of all devices must be wired
    together.  The master will output a pulse every sample, and all the devices will acquire thier
    samples simultaneously.  The may also be used to pace one or more devices from an external
    TTL/CMOS clock signal (max rate = 50kHz)

    This may also be used with an external trigger; the external trigger signal should be brought to
    the master device, and all devices will begin sampling wihen the master is triggered.

    If the device is configured as a slave, it will not acquire data when given an AInScan command
    until it detects a pulse onthe syc input.

    type   0 = master,   1 = slave
  */
  __u8 cmd[2];

  cmd[0] = SET_SYNC;
  cmd[1] = type;
  
  PMD_SendOutputReport(hid, SET_SYNC, cmd, sizeof(cmd), FS_DELAY);
}

__u16 usbGetStatus_USB1616FS(HIDInterface* hid)
{
  /*
    Bit  0:   0 = Sync slave,            1 = sync master
    Bit  1:   0 = trigger falling edge,  1 = trigger rising edge
    Bit 16:   1 = program memory update mode
  */

  __u16 status;

  struct t_statusReport {
  __u8 reportID;
  __u8 status[2];
  } statusReport;

  statusReport.reportID = GET_STATUS;

  PMD_SendOutputReport(hid, GET_STATUS, (__u8*) &statusReport, 1, FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &statusReport, sizeof(statusReport), FS_DELAY);
  status = (__u16) (statusReport.status[0] | (statusReport.status[1] << 8));
  return status;
}

void usbReadMemory_USB1616FS( HIDInterface* hid, __u16 address, __u8 type, __u8 count, __u8* memory)
{
  struct arg_t {
    __u8 reportID;
    __u8 address[2];
    __u8 type;
    __u8 count;
  } arg;

  struct memRead_t {
    __u8 reportID;
    __u8 data[62];
  } memRead;

  if ( count > 62 ) count = 62;
  arg.reportID = MEM_READ;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte
  arg.type = type;
  arg.count = count;

  PMD_SendOutputReport(hid, MEM_READ, (__u8 *) &arg, sizeof(arg), FS_DELAY);
  memRead.reportID = 0x0;
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2, (char *) &memRead, sizeof(memRead), FS_DELAY);
  if (memRead.reportID != MEM_READ) {
    PMD_SendOutputReport(hid, MEM_READ, (__u8 *) &arg, sizeof(arg), FS_DELAY);
    usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2, (char *) &memRead, sizeof(memRead), FS_DELAY);
  }
  //  printf("%#x  %#x  %#x  \n", memRead.reportID, memRead.data[0], memRead.data[1]);
  memcpy(memory, &memRead.data[0], count);
}

int usbWriteMemory_USB1616FS(HIDInterface* hid, __u16 address, __u8 count, __u8 data[])
{
  // Locations 0x00-0x7F are reserved for firmware and my not be written.
  int i;
  struct t_mem_write_report {
    __u8 reportID;
    __u8 address[2];
    __u8 count;
    __u8 data[count];
  } arg;

  if ( address <=0x7f ) return -1;
  if ( count > 59 ) count = 59;

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

void usbPrepareDownload_USB1616FS(HIDInterface* hid)
{
  /*
    This command puts the device into code update mode.  The unlock code must be correct as a
    further safety device.  Call this once before sending code with usbWriteCode.  If not in
    code update mode, any usbWriteCode will be ignored.  A usbReset command must be issued at
    the end of the code download in order to return the device to operation with the new code.
  */

  struct t_prepare_download_report {
    __u8 reportID;
    __u8 unlock_code;
  } prepare_download_report;

  prepare_download_report.reportID = PREPARE_DOWNLOAD;
  prepare_download_report.unlock_code = 0xad;

  PMD_SendOutputReport(hid, PREPARE_DOWNLOAD, (__u8 *) &prepare_download_report,
		       sizeof(prepare_download_report), FS_DELAY);
}

void usbWriteCode_USB1616FS(HIDInterface* hid, __u32 address, __u8 count, __u8 data[])
{
  /*
    This command send the new program memory image to the device.  The download program
    memory image is stored in external SRAM.  The image will be written to FLASH program
    memory when the UpdateCode command is issued (updates must be enabled with UnlockCode
    first.)  A running checksum will be calculated a the program memory image is sent, and the host
    should compare its own checksum with this value (retrieved with ReadChecksum) prior to
    sending the UpdateCode command.
  */

  struct t_arg {
    __u8 reportID;
    __u8 address[3];
    __u8 count;        // 58 bytes max
    __u8 data[58];     // the program data, 58 bytes max
  } arg;

  if (count > 58) return;

  arg.reportID = WRITE_CODE;
  memcpy(&arg.address[0], &address, 3);   // 24 bit address
  arg.count = count;
  memcpy(&arg.data[0], data, count);      // 24 bit address
  PMD_SendOutputReport(hid, WRITE_CODE, (__u8 *) &arg, count+5, FS_DELAY);
}

__u16 usbReadChecksum_USB1616FS(HIDInterface* hid)
{
  struct t_checksum_report {
    __u8 reportID;
    __u8 checksum[2];
  } checksum_report;

  checksum_report.reportID = READ_CHECKSUM;
      
  PMD_SendOutputReport(hid, READ_CHECKSUM, (__u8 *) &checksum_report, sizeof(__u8), FS_DELAY);
  usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
		     (char *) &checksum_report, sizeof(checksum_report), FS_DELAY);
  return (checksum_report.checksum[0] | (checksum_report.checksum[1] << 8));
}

void usbWriteSerial_USB1616FS(HIDInterface* hid, __u8 serial[8])
{
  // Note: The new serial number will be programmed but not used until hardware reset.
  struct t_write_serial {
    __u8 reportID;
    __u8 serial[8];
  } write_serial;

  write_serial.reportID = WRITE_SERIAL;
  memcpy(write_serial.serial, serial, 8);
  
  PMD_SendOutputReport(hid, WRITE_SERIAL, (__u8 *) &write_serial, sizeof(write_serial), FS_DELAY);
}

void usbUpdateCode_USB1616FS(HIDInterface* hid)
{
  __u8 reportID = UPDATE_CODE;
  PMD_SendOutputReport(hid, UPDATE_CODE, &reportID, sizeof(reportID), FS_DELAY);
}

int usbReadCode_USB1616FS(HIDInterface* hid, __u32 address, __u8 count, __u8 data[])
{
  struct t_arg {
    __u8 reportID;
    __u8 address[3];
    __u8 count;     // 62 max.
  } arg;

  struct t_read_code {
    __u8 reportID;
    __u8 data[62];
  } read_code;

  int bRead;

  if (count > 62) count = 62;  

  memcpy(&arg.address[0], &address, 3);   // 24 bit address
  arg.reportID = READ_CODE;
  arg.count = count;

  PMD_SendOutputReport(hid, READ_CODE, (__u8 *) &arg, sizeof(arg), FS_DELAY);

  bRead = usb_interrupt_read(hid->dev_handle, USB_ENDPOINT_IN | 2,
			     (char *) &read_code, count+1, FS_DELAY);
  memcpy(data, read_code.data, count);
  return bRead;
}

float volts_USB1616FS( const int gain, const signed short num )
{
  float volt = 0.0;

  switch( gain ) {
    case BP_10_00V:
      volt = num * 10.0 / 0x7fff;
      break;
    case BP_5_00V:
      volt = num * 5.0 / 0x7fff;
      break;
    case BP_2_50V:
      volt = num * 2.5 / 0x7fff;
      break;
    case BP_2_00V:
      volt = num * 2.0 / 0x7fff;
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
