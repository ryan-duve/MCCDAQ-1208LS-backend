/*
 *
 *  Copyright (c) 2008  Warren Jasper <wjasper@tx.ncsu.edu>
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

#include "usb-1608HS.h"

#define HS_DELAY 1000

void usbBuildGainTable_USB1608HS(usb_dev_handle *udev, float *table[NCHAN_1608HS][NGAINS_1608HS][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
       voltage = value*table[chan#][gain#][0] + table[chan#][gain#][1]
     only needed for fast lookup.
  */
  int i, j, k;
  __u8 data[4];

  usbSetMemAddress_USB1608HS(udev, 0x20);  // ch0, +/- 10V calculated slope
  for (i = 0; i < NCHAN_1608HS; i++) {
    for (j = 0; j < NGAINS_1608HS; j++) {
      for (k = 0; k < 2; k++) {
	usbReadMemory_USB1608HS(udev, 4, data);
	memcpy(&table[i][j][k], data, 4);
      }
    }
  }
  return;
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* reads digital port  */
__u8  usbDIn_USB1608HS(usb_dev_handle *udev)
{
  /*
    This command reads the current state of the DIn port.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data = 0x0;

  usb_control_msg(udev, requesttype, DIN, 0x0, 0x0, (char *) &data, sizeof(data), HS_DELAY);
  return data;
}

/* reads digital bit  */
__u8 usbDInBit_USB1608HS(usb_dev_handle *udev, __u8 bit_num)
{
  /*
    This command reads an individual DIn port bit.
   */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data;

  usb_control_msg(udev, requesttype, DIN_BIT, bit_num, 0x0, (char*) &data, sizeof(data), HS_DELAY);
  return data;
}

/* writes digital port */
void usbDOut_USB1608HS(usb_dev_handle *udev, __u8 data)
{
  /*
    This command writes the DOut port latch.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, DOUT, 0x0, 0x0, (char *) &data, sizeof(data), HS_DELAY);
  return;
}

void usbDOutBit_USB1608HS(usb_dev_handle *udev, __u8 bit_num, __u8 data)
{
  /*
    This command writes an individual DOut port bit latch.
  */
  
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 request = DOUT_BIT;
  char buf[2];
  
  buf[0] = bit_num;
  buf[1] = data;

  usb_control_msg(udev, requesttype, request, 0x0, 0x0, buf, sizeof(buf), HS_DELAY);
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

void usbAInScanStart_USB1608HS(usb_dev_handle *udev)
{
  /*
     This command starts the analog input scan using the parameters previously written
     with AInScanConfig.  The command will result in a bus stall if an AIn scan is
     currently running.
  */
  
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB1608HS(udev) & INPUT_SCAN_RUNNING) {
    usbAInScanStop_USB1608HS(udev);
  }
  usb_control_msg(udev, requesttype, AIN_START, 0x0, 0x0, NULL, 0, HS_DELAY);
}

void usbAInScanStop_USB1608HS(usb_dev_handle *udev)
{
  /*
     This command stops the analog input scan (if running).
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  
  usb_control_msg(udev, requesttype, AIN_STOP, 0x0, 0x0, NULL, 0, HS_DELAY);
}

void usbAInConfig_USB1608HS(usb_dev_handle *udev,  __u8 config_array[8])
{
  /*
    The command writes the analog input channel configurations. This
    command will result in a bus stall if an AIn scan is currently running.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB1608HS(udev) & INPUT_SCAN_RUNNING) {
    usbAInScanStop_USB1608HS(udev);
  }
  usb_control_msg(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (char *) config_array, 8, HS_DELAY);
}

void usbAInConfigRead_USB1608HS(usb_dev_handle *udev,  __u8 config_array[8])
{
  /*
    The command reads the analog input channel configurations. This
    command will result in a bus stall if an AIn scan is currently running.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB1608HS(udev) & INPUT_SCAN_RUNNING) {
    usbAInScanStop_USB1608HS(udev);
  }
  usb_control_msg(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (char *) config_array, 8, HS_DELAY);
  return;
}

void usbAInScanConfig_USB1608HS(usb_dev_handle *udev, __u8 lowChan, __u8 numChan, int count, float freq, __u8 options)
{
  struct t_AInScanConfig {
    __u8 lowchannel;      // the first channel of the scan (0-7)
    __u8 numchannels;     // the number of channels in the scan - 1 (0-7)
    __u8 count[3];        // uint24; Total numer of scans to perform, used only in single execution mode.
    __u8 timer_period[4]; // pacer time period value
    __u8 options;         /* bit field that controls various options:
			    bit 0:  1 = single execution, 0 = continuous execution
			    bit 1:  1 = burst mode (single execution only), 0 = normal mode
                            bit 2:  1 = immediate transfer (N/A in burst mode), 0 = block transfer
                            bit 3:  1 = user extgernal trigger,
			    bit 4:  1 = user SYNC_IN, 0 = use internal pacer
                            bit 5:  1 = debug mode, 0 = normal data
                            bit 6:  1 = retrigger mode, 0 = normal trigger
			    bit 7:  1 = include DIn data with each scan
		          */
  } AInScanConfig;

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u32 timer_period;

  AInScanConfig.lowchannel = lowChan;
  AInScanConfig.numchannels = numChan;
  timer_period = (40e6/freq) - 1;
  AInScanConfig.timer_period[0] = timer_period & 0xff;
  AInScanConfig.timer_period[1] = (timer_period >> 8) & 0xff;
  AInScanConfig.timer_period[2] = (timer_period >> 16) & 0xff;
  AInScanConfig.timer_period[3] = (timer_period >> 24) & 0xff;
  AInScanConfig.count[0] = count & 0xff;
  AInScanConfig.count[1] = (count >> 8)  & 0xff;
  AInScanConfig.count[2] = (count >> 16) & 0xff;
  AInScanConfig.options = options;

  if (usbStatus_USB1608HS(udev) & INPUT_SCAN_RUNNING) {
    usbAInScanStop_USB1608HS(udev);
  }
  usb_control_msg(udev, requesttype, AIN_SCAN_CFG, 0x0, 0x0, (char *) &AInScanConfig, sizeof(AInScanConfig), HS_DELAY);
}

int usbAInScan_USB1608HS(usb_dev_handle *udev, __u16 *data)
{

  /*

  Data format:

  low channel samlpe 0: lowchannel + 1 sample 0: ... hichannel sample 0: [DIn value]
  low channel samlpe 1: lowchannel + 1 sample 1: ... hichannel sample 1: [DIn value]
  ...
  low channel samle n: lowchannel + 1 sample n: ... hichannel sample n: [DIn value]

  The DIn data vill be a 16-bit value in order to keep the data word-aligned and the
  upper byte will always be 0.
  
  */
   struct t_AInScanConfig {
    __u8 lowchannel;      // the first channel of the scan (0-7)
    __u8 numchannels;     // the number of channels in the scan - 1 (0-7)
    __u8 count[3];        // uint24; Total numer of scans to perform, used only in single execution mode.
    __u8 timer_period[4]; // pacer time period value
    __u8 options;         /* bit field that controls various options:
			    bit 0:  1 = single execution, 0 = continuous execution
			    bit 1:  1 = burst mode (single execution only), 0 = normal mode
                            bit 2:  1 = immediate transfer (N/A in burst mode), 0 = block transfer
                            bit 3:  1 = user extgernal trigger,
			    bit 4:  1 = user SYNC_IN, 0 = use internal pacer
                            bit 5:  1 = debug mode, 0 = normal data
                            bit 6:  1 = retrigger mode, 0 = normal trigger
			    bit 7:  1 = include DIn data with each scan
		          */
  } AInScanConfig;
   __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret = -1;
  int nbytes;
  int nscan;

  usb_control_msg(udev, requesttype, AIN_SCAN_CFG, 0x0, 0x0, (char *) &AInScanConfig, sizeof(AInScanConfig), HS_DELAY);
  nscan = AInScanConfig.count[0] + (1<<8)*AInScanConfig.count[1] + (1<<16)*AInScanConfig.count[2];

  // number of byes read = 2 * number of scans * number of channels (without DIn)
  if (AInScanConfig.options & AIN_DIN_DATA) {
    nbytes = 2*nscan*(AInScanConfig.numchannels+2);   
  } else {
    nbytes = 2*nscan*(AInScanConfig.numchannels+1);   
  }

  ret = usb_bulk_read(udev, USB_ENDPOINT_IN|1, (char *) data, nbytes, HS_DELAY);
  if (ret < 0) {
    printf("usbAInScan_USB1608HS error: ret = %d\n", ret);
    return ret;
  }
  return ret;
}

int usbAIn_USB1608HS(usb_dev_handle *udev, __u16 *data)
{
  /*
    This command returns the value from the analog input channels.  The channels
    are configured with AInConfig.  This command will result in a bus stall if
    an AIn scan is currently runnning.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB1608HS(udev) & INPUT_SCAN_RUNNING) {
    usbAInScanStop_USB1608HS(udev);
  }

  usb_control_msg(udev, requesttype, AIN, 0x0, 0x0, (char *) data, 16, HS_DELAY);
  return 8;
}

/***********************************************
 *            Analog Output                    *
 ***********************************************/

void usbAOut_USB1608HS(usb_dev_handle *udev, __u16 data[2])
{
  /* This command writes the values for the analog output channels.
     The values are 16-bit unsigned numbers.  Both read and write will
     result in a control pipe stall if an output scan is running. The
     equation for the output voltage is:

     V_out = (value - 0x8000)/(2^16 - 1) * V_ref

     where value is the value written to the channel and V_ref = 10.0V.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, AOUT, 0x0, 0x0, (char *) data, 4, HS_DELAY);
  return;
}

void usbAOutRead_USB1608HS(usb_dev_handle *udev, __u16 *data)
{
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, AOUT, 0x0, 0x0, (char *) data, 4, HS_DELAY);
  return;
}

void usbAOutScanStart_USB1608HS(usb_dev_handle *udev)
{
  /*
    This command starts the analog output scan using the parameters previouly written
    with AOutScanConfig.  This command will result in a bus stall if an AOut scan
    is currently running.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB1608HS(udev) & OUTPUT_SCAN_RUNNING) {
    usbAOutScanStop_USB1608HS(udev);
  }
  usb_control_msg(udev, requesttype, AOUT_START, 0x0, 0x0, NULL, 0, HS_DELAY);
}

void usbAOutScanStop_USB1608HS(usb_dev_handle *udev)
{
  /*
    This command stops the analog output scan (if running) and clears the
    output FIFO data.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  
  usb_control_msg(udev, requesttype, AOUT_STOP, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbAOutConfig_USB1608HS(usb_dev_handle *udev, __u8 config)
{
  /*
     This command writes the analog output channel configurations.  This
     command will result in a bus stall if AOutScan is currently running.

     config: bit 0: channel 0 0-force/sense disable  1-force/sense enable
             bit 1: channel 1 0-force/sense disable  1-force/sense enable
             bits 2-7: reserved
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB1608HS(udev) & OUTPUT_SCAN_RUNNING) {
    usbAOutScanStop_USB1608HS(udev);
  }
  usb_control_msg(udev, requesttype, AOUT_CONFIG, 0x0, 0x0, (char *) &config, sizeof(config), HS_DELAY);
  return; 
}

void usbAOutConfigRead_USB1608HS(usb_dev_handle *udev, __u8 *config)
{
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB1608HS(udev) & OUTPUT_SCAN_RUNNING) {
    usbAOutScanStop_USB1608HS(udev);
  }
  usb_control_msg(udev, requesttype, AOUT_CONFIG, 0x0, 0x0, (char *) config, 1, HS_DELAY);
  return;
}

int usbAOutScan_USB1608HS(usb_dev_handle *udev, __u16 *data, __u32 ndata)
{
  int ret;
  int i = 0;
  int nwrite = 0;

  while (ndata >= 512) {
    ret = usb_bulk_write(udev, USB_ENDPOINT_OUT | 1,  (char *) &data[i], 1024, HS_DELAY);
    if (ret < 0) {
      printf("usbAOutScan_USB1608HS: Error in usb_bulk_write.  Error = %d\n", ret);
      return ret;
    } else {
      nwrite += ret/2;
    }
    ndata -= 512;
    i += 512;
  }
  if (ndata > 0) {
    ret = usb_bulk_write(udev, USB_ENDPOINT_OUT | 1,  (char *) &data[i], ndata*2, HS_DELAY);
    if (ret < 0) {
      printf("usbAOutScan_USB1608HS: Error in usb_bulk_write.  Error = %d\n", ret);
      return ret;
    } else {
      nwrite += ret/2;
    }
  }
  return nwrite;
}

void usbAOutScanConfig_USB1608HS(usb_dev_handle *udev, __u32 nscans, float frequency, __u8 options)
{
  /*
    This command configures the analog output channel scan.  This command will
    result in a bus stall if an AOutScan is currently running.

    Note: The output scan operates with the host continuously transferring data for the
    outputs until the end of the scan.  If the nscans parameter is 0, the scan will run
    until the AOutScanStop command is issued by the host;  if it is nonzero, the scan
    will stop automatically after the specified number of scans have been output.
    The channels in the scan are selected in the options bit field.  Scan refers to
    the number of updates to the channels (if both channels are used, one scan is an
    update to both channels).

    The time base is controlled by an internal 16-bit timer running at a base rate of
    10MHz.  The timer is controlled by timer_prescale and timer_period.  The timer
    will be reset and provide an internal interrupt when its value equals
    timer_preload.  It is preferable to keep the prescaler to the lowest value that will
    achieve the desired rate.

    Timer calculation:
    preload = (10MHz/ (frequency*prescaler)) - 1;
    Example: Data rate = 10kHz
    With a prescaler = 1: prelaod = (10MHz/(10kHz*1)) - 1 = 999 (0x03E7)

    The same time base is used for both channels when the scan involves two channels.

    The output data is to be sent using the bulk out endpoint.  The data must be in the format:

    low channel sample 0 [high channel sample 0]
    low channel sample 1 [high channel sample 1]
    ...
    low channel sample n [high channel sample n]

    The ouptut data is written to a 512-sample FIFO in the device.  The bulk endpoint
    data is only accepted if there is room in the FIFO.  Output data may be sent to
    the FIFO before the start of a scan, and the FIFO is cleared when the
    AOutScanConfig or AOutScanStop commands are received.  The scan will not
    begin until the AOutScanStart command is sent (and output data is in the FIFO).
    Data will be output until reaching the specified number of scans (in single
    execution mode) or an AOutScanStop commend is sent.
  */
  
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u32 preload;

  struct t_AOutScan {
    __u32 nscans;          // The total number of scans to perform. 0 = run forever.
    __u8 timer_prescale;   // Timer prescaler
    __u8 timer_preload[2]; // Timer preload value
    __u8 options;          // bit 0: 1 = include channel 0 in output scan
                           // bit 1: 1 = include channel 0 in output scan
  } AOutScan;
  AOutScan.nscans = nscans;
  AOutScan.options = options;

  for (AOutScan.timer_prescale = 0; AOutScan.timer_prescale <= 8; AOutScan.timer_prescale++) {
    preload = 10e6/((frequency) * (1<<AOutScan.timer_prescale)) - 1;
    if (preload <= 0xffff) {
      AOutScan.timer_preload[0] = (__u8) preload & 0xff;         // low byte
      AOutScan.timer_preload[1] = (__u8) (preload >> 8) & 0xff;  // high byte
      break;
    }
  }
  if (AOutScan.timer_prescale > 8 || preload == 0) {
    printf("Error AOutScanConfig.  frequency out of range.\n");
    return;
  }
  usb_control_msg(udev, requesttype, AOUT_SCAN_CFG, 0x0, 0x0, (char *) &AOutScan, sizeof(AOutScan), HS_DELAY);
}

/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbCounterInit_USB1608HS(usb_dev_handle *udev)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
     counter will be initialized to zero.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, NULL, 0, HS_DELAY);
  
  return;
}

__u32 usbCounter_USB1608HS(usb_dev_handle *udev)
{
  /*
    This command reads the 32-bit event counter.  
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u32 count = 0x0;

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, (char *) &count, sizeof(count), HS_DELAY);
  return count;
}

__u8 usbStatus_USB1608HS(usb_dev_handle *udev)
{
  /*
    This command retrieves the status of the device.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 status;

  usb_control_msg(udev, requesttype, STATUS, 0x0, 0x0, (char *) &status, sizeof(status), HS_DELAY);
  return status;
}  
  
/* blinks the LED of USB device */
void usbBlink_USB1608HS(usb_dev_handle *udev, __u8 count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, BLINK_LED, 0x0, 0x0, (char *) &count, sizeof(count), HS_DELAY);
  return;
}

void usbReset_USB1608HS(usb_dev_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device disconnects from the USB bus and resets
    its microcontroller.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbTrigger_USB1608HS(usb_dev_handle *udev, __u16 data, __u8 options)
{
  /*
    This function configures the external trigger for analog input.  The trigger has
    an analog level and edge setting, and uses a 12-bit DAC for the analog level.
    Once the trigger is received, the AInScan will proceed as configured.  The "use
    trigger" option must be used in the AInScanConfig command to utilize this
    feature.  The command may be sent during an AInScan to change the trigger
    level, but the options will not be applied until the following AInScan.

   The trigger value is calculated as follows:

    V_trigger = (k - 0x800/(2^12 - 1)) * 10V  where k is the level value written to the device
  */

  struct t_trigger {
    __u16 level;  // the trigger level (0-4095)
    __u8 options; /* bit field options:
                     bit 0: trigger mode (0 = level, 1 = edge)
                     bit 1: trigger polarity (0 = low / falling, 1 = high / rising)
                     bits 2-7: reserved */
  } trigger;

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  trigger.level = data;
  trigger.options = options;

  usb_control_msg(udev, requesttype, TRIGGER_CFG, 0x0, 0x0, (char *) &trigger, sizeof(trigger), HS_DELAY);
  return;
}

float usbTemperature_(usb_dev_handle *udev)
{
  /*
    This command reads the internal temperature.  The returned temperature is in degrees Celsius
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u16 data;
  float temperature;

  usb_control_msg(udev, requesttype, TEMPERATURE, 0x0, 0x0, (char *) &data, sizeof(data), HS_DELAY);
  temperature = (((data/65536.)*3.3) - 0.4)/(.0195);
  return temperature;
}

void usbGetSerialNumber_USB1608HS(usb_dev_handle *udev, char serial[9])
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

void usbSetSerialNumber_USB1608HS(usb_dev_handle *udev, char serial[9])
{
  /*
    This commands writes the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). The new serial number will be programmed but not used until
    hardware reset.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, SERIAL, 0x0, 0x0, serial, 8, HS_DELAY);
  return;
}

void usbReadMemory_USB1608HS(usb_dev_handle *udev, __u16 length, __u8 *data)
{
  /* This command reads data from the available data EEPROM memory.  The
     read will begin at the current address, which may be set with MemAddress.
     The address will automatically increment during a read or write but stay
     within the range allowed by the EEPROM.  The amount of data to be written
     or read is specified in wLength.
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, MEMORY, 0x0, 0x0, (char *) data, length, HS_DELAY);
  return;
}

void usbWriteMemory_USB1608HS(usb_dev_handle *udev, __u16 length, __u8 *data)
{
  /* This command reads data from the available data EEPROM memory.  The
     read will begin at the current address, which may be set with MemAddress.
     The address will automatically increment during a read or write but stay
     within the range allowed by the EEPROM.  The amount of data to be written
     or read is specified in wLength.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, MEMORY, 0x0, 0x0, (char *) data, length, HS_DELAY);
  return;
}

void usbSetMemAddress_USB1608HS(usb_dev_handle *udev, __u16 address)
{
  /*
    The command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas. The memory
    map for this device is:

    Address             Description
    -------             ----------                  
    0x0000              EEPROM (512 bytes)
    0x01FF

  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (char *) &address, sizeof(address), HS_DELAY);
  return;
}

void usbGetMemAddress_USB1608HS(usb_dev_handle *udev, __u16 *address)
{
  /*
    The command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas. The memory
    map for this device is:

    Address             Description
    -------             ----------                  
    0x0000              EEPROM (512 bytes)
    0x01FF

  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (char *) address, sizeof(__u16), HS_DELAY);
  return;
}

void usbUpdateMode_USB1608HS(usb_dev_handle *udev, __u8 device)
{
  /*
    This command puts the device into code update mode, which allows updating
    the code for the microcontroller or FPGA.  The unlock code must be correct
    as a futher safety device.  If the device is not in code update mode all of
    the other ocde uipdate commands will result in a control pipe stall.  A Reset
    command must be issued at the end of the code update in order to return the
    device to operation with the new code.

    Note:  The FPGA FLASH configuration memory will be erased on receiveing this
    command if the FPGA is selected, which will cause the device to stop responding
    for approximately 2.5 seconds.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct t_updateMode {
    __u8 unlock_code;      // must be the value 0xAD
    __u8 device;           // 0 = Microcontroller  1 = FPGA
  } updateMode;

  updateMode.device = device;
  updateMode.unlock_code = 0xad;
  usb_control_msg(udev, requesttype, UPDATE_MODE, 0x0, 0x0, (char *) &updateMode, sizeof(updateMode), HS_DELAY);
}

void usbUpdateAddress_USB1608HS(usb_dev_handle *udev, __u32 address)
{
  /*
    This command reads or writes the address used for code download.  The destination
    for the download (mirocontroller or FPGA) is selected when calling UpdateMode.

    address is a unsigned 24 bit value.

    The microcontroller memory map is:
    Address              Description
    ------               ----------
    0x000800          FLASH program memory
    0x007FFF     

    The FPGA memory map is:
    Address              Description
    ------               ----------
    0x000000          FLASH configuration memory
    0x01FFFF

  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, UPDATE_ADDRESS, 0x0, 0x0, (char *) &address, 3, HS_DELAY);
}

void usbUpdateData_USB1608HS(usb_dev_handle *udev, __u16 length, __u8 *data)
{
  /*
    This command writes to the program memory in the device and checks the
    status of a previous write.  This command is not accepted unless the device
    is in updteMode.  This command will normally be used when downloading a new
    hex file, so it supports the memory ranges that may be found in the hex file.
    The number of bytes to be written must be specified in wLength.  The status
    of each write should be checked by reading this command.

    Microcontroller: The device must receive data in 64-byte segments that begin
    on a 64-byte boundary

    FPGA: The FPGA data must always be sent in 64-byte segments that begin on a
    a 64-byte boundary.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 status = 0;

  usb_control_msg(udev, requesttype, UPDATE_DATA, 0x0, 0x0, (char *) data, length, HS_DELAY);
  requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, UPDATE_DATA, 0x0, 0x0, (char *) &status, sizeof(status), HS_DELAY);
  if ((status & 0x1) == 1) {
    printf("usbUpdateData_USB1608HS: Error writing data.\n");
  }
}

__u16 usbUpdateVersion_USB1608HS(usb_dev_handle *udev)
{
  /*
    This command retrieves the firmware version of the update code.  This
    command is not accepted unless the device is in update mode and the
    microcontroller is selected.
  */

  __u16 version;  // Hex BCD representation of the updater version.  e.g. Version 1.25 would return 0x0125.
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usbUpdateMode_USB1608HS(udev, 0);
  usb_control_msg(udev, requesttype, UPDATE_VERSION, 0x0, 0x0, (char *) &version, sizeof(version), HS_DELAY);
  return version;
}

__u16 usbUpdateChecksum_USB1608HS(usb_dev_handle *udev)
{
  /*
    The command reads or resets the checksum value that is calculated by the update
    code.  This command is not accepted unless the device is in update mode.  Each
    block of data to be written will add to the cheksum.  This is used by the
    application to verify that all of the blocks were received by the update code.
    The checksum will be reset to 0 whenthis is sent as an output.
  */
  __u16 checksum;  // The current checksum of received data, equal to sum(data)
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, UPDATE_CHECKSUM, 0x0, 0x0, (char *) &checksum, sizeof(checksum), HS_DELAY);
  return checksum;
}

void usbUpdateChecksumReset_USB1608HS(usb_dev_handle *udev)
{
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, UPDATE_CHECKSUM, 0x0, 0x0, NULL, 0, HS_DELAY);
}

float volts_USB1608HS(usb_dev_handle *udev, const int channel, const int gain, const unsigned short value)
{
  float volt = 0.0;
  unsigned short num;
  int i, j, k;
  __u8 data[4];
  float table[NCHAN_1608HS][NGAINS_1608HS][2];
  
  /*
    Build a table of slopes and offsets.
    NOTE: Lookup table not global so routine is reentriant
    For faster implementation, call usbBuildGainTable_USB1608HS
  */

  usbSetMemAddress_USB1608HS(udev, 0x20);  // ch0, +/- 10V calculated slope
  for (i = 0; i < NCHAN_1608HS; i++) {
    for (j = 0; j < NGAINS_1608HS; j++) {
      for (k = 0; k < 2; k++) {
	usbReadMemory_USB1608HS(udev, 4, data);
	memcpy(&table[i][j][k], data, 4);
      }
    }
  }

  num = value*table[channel][gain][0] + table[channel][gain][1];

  switch (gain) {
    case DE_10_00V:
      volt = (num - 0x8000) * 10.0 / 0x7fff;
      break;
    case DE_5_00V:
      volt = (num - 0x8000) * 5.0 / 0x7fff;
      break;
    case DE_2_00V:
      volt = (num - 0x8000) * 2.0 / 0x7fff;
      break;
    case DE_1_00V:
      volt = (num - 0x8000) * 1.0 / 0x7fff;
      break;
  }

  return volt;
}

void cleanup_USB1608HS( usb_dev_handle *udev )
{
  if (udev) {
    usb_clear_halt(udev, USB_ENDPOINT_IN|1);
    usb_clear_halt(udev, USB_ENDPOINT_OUT|1);
    usb_release_interface(udev, 0);
    usb_close(udev);
  }
}

