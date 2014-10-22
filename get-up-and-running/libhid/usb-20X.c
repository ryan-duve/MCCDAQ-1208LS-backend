/*
 *
 *  Copyright (c) 2013  Warren Jasper <wjasper@tx.ncsu.edu>
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
#include <math.h>

#include "usb-20X.h"

#define HS_DELAY 2000


void usbBuildGainTable_USB20X(usb_dev_handle *udev, float table[NCHAN_USB20X][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
     The calibration coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j;
  __u16 address = 0;

  for (i = 0; i < NCHAN_USB20X; i++) {
    for (j = 0; j < 2; j++) {
      usbReadCalMemory_USB20X(udev, address, 4, (__u8 *) &table[i][j]);
      address += 4;
    }
  }
}
			      

/***********************************************
 *            Digital I/O                      *
 ***********************************************/
/* reads tristate port regiser */
__u8  usbDTristateR_USB20X(usb_dev_handle *udev)
{
  /*
    This command reads or writes the digital port tristate
    register.  The tristate register determines if the
    latch register value is driven onto the port pin.  A
    '1' in the tristate register makes the corresponding
    pin an input, a '0' makes it an output.
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data = 0x0;

  if (usb_control_msg(udev, requesttype, DTRISTATE, 0x0, 0x0, (char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDTristateR_USB20X: error in usb_control_msg().\n");
  }
  return data;
}

void usbDTristateW_USB20X(usb_dev_handle *udev, __u8 value)
{
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usb_control_msg(udev, requesttype, DTRISTATE, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDTristateW_USB1208HS: error in usb_control_msg().\n");
  }
  return;
}

/* reads digital port  */
__u8 usbDPort_USB20X(usb_dev_handle *udev)
{
  /*
    This command reads the current state of the digital port pins
   */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data;

  if (usb_control_msg(udev, requesttype, DPORT, 0x0, 0x0, (char*) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDPort_USB20X: error in usb_control_msg().\n");
  }
  return data;
}

/* read digital port */
__u8 usbDLatchR_USB20X(usb_dev_handle *udev)
{
  /*
    This command reads or writes the digital port latch register
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u16 data = 0x0;
  
  if (usb_control_msg(udev, requesttype, DLATCH, 0x0, 0x0, (char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDLatchR_USB20X: error in usb_control_msg().\n");
  }
  return data;
}

void usbDLatchW_USB20X(usb_dev_handle *udev, __u8 data)
{
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usb_control_msg(udev, requesttype, DLATCH, data, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDLatchW_USB20X: error in usb_control_msg().\n");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/
__u16 usbAIn_USB20X(usb_dev_handle *udev, __u8 channel)
{
  /*
    This command reads the value of an anlog input channel.  This command will result
    in a bus stall if an AInScan is currently running.
  */
  __u16 value;
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, AIN, channel, 0x0, (char *) &value, sizeof(value), HS_DELAY);
  return value;
}

void usbAInScanStart_USB20X(usb_dev_handle *udev, __u32 count, double frequency, __u8 channels, __u8 options, __u8 trigger_source, __u8 trigger_mode)
{
  /*
     The command starts an analog input scan.  The command will result
     in a bus stall if an AInScan is currently running.  The USB-201
     will not generate an internal pacer faster than 100 kHz; the
     USB-204 will not generate an interal pacer faster than 500 kHz.

     The ADC is paced such that the pacer controls the ADC
     conversions.  The internal pacer rate is set by an internal
     32-bit timer running at a base rate of 70 MHz.  The timer is
     controlled by pacer_period.  The value is the period of the ADC
     conversions.  A pulse will be output at the PACER_OUT pin at
     every pacer_period internval The equation for calculating
     pacer_period is:

     pacer_period = [70 MHz / (A/D frequency)] - 1

     If pacer_period is set to 0, the device does not generate an A/D
     clock.  Th uses the PACER_IN pin as the pacer source.  Each
     rising edge of PACER_IN starts a conversion.

     The data will be returned in packets untilizing a bulk endpoint.
     The data will be in the format:

     lowchannel sample 0: lowchannel + 1 sample 0: ... : hichannel sample 0
     lowchannel sample 1: lowchannel + 1 sample 1: ... : hichannel sample 1
     ...
     lowchannel sample n: lowchannel + 1 sample n: ... : hichannel sample n

     The scan will not begin until the AInScan Start command is sent
     and any trigger conditions are met.  Data will be sent until
     reaching the specified count or an usbAInScanStop_USB20X()
     command is sent.

     The external trigger may be used to start the scan.  If enabled,
     the device will wait until the appropriate trigger condition is
     detected than begin sampling data at the specified rate.  No
     packets will be sent until the trigger is detected.

     In block transfer mode, the data is sent in 64-byte packets as
     soon as data is available from the A/D.  In immediate transfer
     mode, the data is sent after each scan, resulting in packets that
     are 1-8 samples (2-16 bytes) long.  This mode should only be used
     for low pacer rates, typically under 100 Hz, because overruns
     will occur if the rate is too high.

     Overruns are indicated by the device stalling the bulk endpoint
     during the scan.  The host may read the status to verify and ust
     clear the stall condition before further scan can be performed.
     
  */
  struct t_AInScan {
    __u32 count;         // The total number of scans to perform (0 for continuous scan)
    __u32 pacer_period;  // The pacer timer period value. (0 for external clock PACER_IN).
    __u8 channels;       // bit field that selects the channels in the scan;
    __u8 options;        /* bit field that controls scan options:
			  bit 0:  0 = block transfer mode, 1 = immediate transfer mode
                          bit 1:  Reserved
	                  bit 2:  Reserved
                          bit 3:  Reserved
                          bit 4:  Reserved
			  bit 5:  Reserved
			  bit 6:  Reserved
			  bit 7:  0 = stall on overrun, 1 = inhibit stall
		         */
    __u8 trigger_source; // 0 = no trigger, 1 = digital trigger
    __u8 trigger_mode;   // 0 = Edge/rising, 1 = Edge/falling, 2 = Level/high 3 = Level/low
  } AInScan;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);


  if (frequency > 500000.) frequency = 500000.;
  if (frequency < 0.) frequency = 0.;
  if (frequency < 100.) {
    options |= IMMEDIATE_TRANSFER_MODE;
  } else {
    options |= BLOCK_TRANSFER_MODE;
  }

  AInScan.count = count;
  AInScan.pacer_period = rint((70.E6 / frequency) - 1);
  AInScan.channels = channels;
  AInScan.options = options;
  AInScan.trigger_source = trigger_source;
  AInScan.trigger_mode = trigger_mode;

  usbAInScanStop_USB20X(udev);
  usbAInScanClearFIFO_USB20X(udev);
  /* Pack the data into 12 bytes */
  usb_control_msg(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (char *) &AInScan, 12, HS_DELAY);
}

int usbAInScanRead_USB20X(usb_dev_handle *udev, int nScan, int nChan, __u16 *data)
{
  int ret = -1;
  int nbytes = nChan*nScan*2;    // number of bytes to read in 64 bit chunks
  __u16 status;

  ret = usb_bulk_read(udev, USB_ENDPOINT_IN|1, (char *) data, nbytes, HS_DELAY);
  if (ret != nbytes) {
    printf("usbAInScanRead_USB20X: error in usb_bulk_read.  ret = %d\n", ret);
  }

  status = usbStatus_USB20X(udev);
  if (status & AIN_SCAN_RUNNING) {
    printf("Analog In scan not done.\n");
    usbAInScanStop_USB20X(udev);
  }

  if ((status & AIN_SCAN_OVERRUN)) {
    printf("Analog AIn scan overrun.\n");
  }

  return ret;
}

void usbAInScanStop_USB20X(usb_dev_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInScanClearFIFO_USB20X(usb_dev_handle *udev)
{
  /*
    This command clears the internal scan endpoint FIFOs.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AIN_SCAN_CLR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

/***********************************************
 *            Counter/Timer                    *
 ***********************************************/
void usbCounterInit_USB20X(usb_dev_handle *udev)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
    specified counter (0 or 1) will be reset to zero.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, NULL, 0x0, HS_DELAY);
  return;
}

__u32 usbCounter_USB20X(usb_dev_handle *udev)
{
  /*
    This command reads the 32-bit event counter.  
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u32 counts = 0x0;

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, (char *) &counts, sizeof(counts), HS_DELAY);
  return counts;
}

/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbReadCalMemory_USB20X(usb_dev_handle *udev, __u16 address, __u16 count, __u8 data[])
{
  /*
    This command allows for reading and writing the nonvolatile
     calibration memory.  The cal memory is 768 bytes (address
     0-0x2FF).  The cal memory is write protected and must be unlocked
     in order to write the memory.  The unlock procedure is to write
     the unlock code 0xAA55 to address 0x300.  Writes to the entire
     memory range is then possible.  Write any other value to address
     0x300 to lock the memory after writing.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbReadCalMemory_USB20X: max bytes that can be written is 768.");
    return;
  }

  if (address > 0x2ff) {
    printf("usbCalMemoryR_USB20X: address must be in the range 0 - 0x2ff.");
    return;
  }
  usb_control_msg(udev, requesttype, CAL_MEMORY, address, 0x0, (char *) data, count, HS_DELAY);
}

void usbWriteCalMemory_USB20X(usb_dev_handle *udev, __u16 address, __u16 count, __u8 data[])
{

  __u16 unlock_code = 0xaa55;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbWriteCalMemory_USB20X: max bytes that can be written is 768.");
    return;
  }

  if (address > 0x2ff) {
    printf("usbWriteCalMemory_USB20X: address must be in the range 0 - 0x2ff.");
    return;
  }

  usb_control_msg(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (char *) &unlock_code, sizeof(unlock_code), HS_DELAY); // unlock memory
  usb_control_msg(udev, requesttype, CAL_MEMORY, address, 0x0, (char *) data, count, HS_DELAY);
  usb_control_msg(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (char *) 0x0, sizeof(__u16), HS_DELAY); // lock memory
}

void usbReadUserMemory_USB20X(usb_dev_handle *udev, __u16 address, __u16 count, __u8 data[])
{
    
  /*
    These commands allow for reading and writing the nonvolatile user memory. count must
    be less than or equal to 256
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 256) {
    printf("usbReadUserMemory_USB20X: max bytes that can be written is 256.");
    return;
  }

  if (address > 0xff) {
    printf("usbReadUserMemory_USB20X: address must be in the range 0 - 0xff.");
    return;
  }
  usb_control_msg(udev, requesttype, USER_MEMORY, address, 0x0, (char *) data, count, HS_DELAY);
}

void usbWriteUserMemory_USB20X(usb_dev_handle *udev, __u16 address, __u16 count, __u8 data[])
{

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 255) {
    printf("usbWriteUserMemory_USB20X: max bytes that can be written is 768.");
    return;
  }

  if (address > 0xff) {
    printf("usbWriteUserMemory_USB20X: address must be in the range 0 - 0x2ff.");
    return;
  }
  usb_control_msg(udev, requesttype, USER_MEMORY, address, 0x0, (char *) data, count, HS_DELAY);
}

void usbReadMBDMemory_USB20X(usb_dev_handle *udev, __u16 address, __u16 count, __u8 data[])
{
  /*
    These commands allow for reading and writing the nonvolatile MBD memory. count must
    be less than or equal to 1024
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbReadMBDMemory_USB20X: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x400) {
    printf("usbReadMBDMemory_USB20X: address must be in the range 0 - 0x400.");
    return;
  }
  usb_control_msg(udev, requesttype, MBD_MEMORY, address, 0x0, (char *) data, count, HS_DELAY);
}

void usbWriteMBDMemory_USB20X(usb_dev_handle *udev, __u16 address, __u16 count, __u8 data[])
{

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbWriteMBDMemory_USB20X: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x400) {
    printf("usbWriteUserMemory_USB20X: address must be in the range 0 - 0x400.");
    return;
  }
  usb_control_msg(udev, requesttype, USER_MEMORY, address, 0x0, (char *) data, count, HS_DELAY);
}

  
/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbReset_USB20X(usb_dev_handle *udev)
{
  /*
    This command resets the device
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

__u16 usbStatus_USB20X(usb_dev_handle *udev)
{
  /*
    This command retrieves the status of the device.  Writing the command will clear
    the error indicators.
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u16 status = 0x0;

  usb_control_msg(udev, requesttype, STATUS, 0x0, 0x0, (char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbGetSerialNumber_USB20X(usb_dev_handle *udev, char serial[9])
{
  /*
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). 
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, SERIAL, 0x0, 0x0, serial, 8, HS_DELAY);
  serial[8] = '\0';
  return;
}

void usbBlink_USB20X(usb_dev_handle *udev, __u8 count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, BLINK_LED, 0x0, 0x0, (char *) &count, sizeof(count), HS_DELAY);
  return;
}

void cleanup_USB20X(usb_dev_handle *udev)
{
  if (udev) {
    usb_clear_halt(udev, USB_ENDPOINT_IN|1);
    usb_clear_halt(udev, USB_ENDPOINT_OUT|1);
    usb_release_interface(udev, 0);
    usb_close(udev);
  }
}

double volts_USB20X(usb_dev_handle *udev, __u16 value)
{
  double volt = 0.0;
  volt = (value - 2048.)*10./2048.;
  return volt;
}
