/*
 *  Copyright (c) 2009-2013  Warren Jasper <wjasper@tx.ncsu.edu>
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

#include "usb-2416.h"

#define HS_DELAY 1000


//*******************************************************************
// NIST Thermocouple coefficients
//
// The following types are supported:
//
//	J, K, R, S, T, N, E, B

typedef struct t_NIST_Table {
  unsigned char nCoefficients;
  double VThreshold;
  const double* Coefficients;
} NIST_Table;

typedef struct t_NIST_Reverse {
  unsigned char nCoefficients;
  const double* Coefficients;
} NIST_Reverse;
	
typedef struct t_Thermocouple_Data {
  unsigned char nTables;
  const NIST_Reverse* ReverseTable;
  const NIST_Table* Tables;
} Thermocouple_Data;

// ****************************************************************************
// Type J data

const double TypeJTable0[] = 
{
  0.0000000E+00,
  1.9528268E+01,
 -1.2286185E+00,
 -1.0752178E+00,
 -5.9086933E-01,
 -1.7256713E-01,
 -2.8131513E-02,
 -2.3963370E-03,
 -8.3823321E-05
};

const double TypeJTable1[] = 
{	
  0.000000E+00,	
  1.978425E+01,	
 -2.001204E-01,	
  1.036969E-02,	
 -2.549687E-04,	
  3.585153E-06,	
 -5.344285E-08,	
  5.099890E-10
};

const double TypeJTable2[] = 
{
 -3.11358187E+03,
  3.00543684E+02, 		
 -9.94773230E+00, 		
  1.70276630E-01, 		
 -1.43033468E-03, 		
  4.73886084E-06		
};

const NIST_Table TypeJTables[] =
{
  {9,    0.0, TypeJTable0},
  {8, 42.919, TypeJTable1},
  {6, 69.553, TypeJTable2}
};

const double TypeJReverse[] =
  {
    0.000000000000E+00,
    0.503811878150E-01,
    0.304758369300E-04,
   -0.856810657200E-07,
    0.132281952950E-09,
   -0.170529583370E-12,
    0.209480906970E-15,
   -0.125383953360E-18,
    0.156317256970E-22
};

const NIST_Reverse TypeJReverseTable = {
  9,  // nCoefficients
  TypeJReverse
};

// ****************************************************************************
// Type K data
const double TypeKTable0[] =
{
   0.0000000E+00,
   2.5173462E+01,
  -1.1662878E+00,
  -1.0833638E+00,
  -8.9773540E-01,
  -3.7342377E-01,
  -8.6632643E-02,
  -1.0450598E-02,
  -5.1920577E-04
};

const double TypeKTable1[] =
{
   0.000000E+00,
   2.508355E+01,
   7.860106E-02,
  -2.503131E-01,
   8.315270E-02,
  -1.228034E-02,
   9.804036E-04,
  -4.413030E-05,
   1.057734E-06,
  -1.052755E-08
};

const double TypeKTable2[] =
{
  -1.318058E+02,
   4.830222E+01,
  -1.646031E+00,
   5.464731E-02,
  -9.650715E-04,
   8.802193E-06,
  -3.110810E-08
};

const NIST_Table TypeKTables[] =
{
  { 9,   0.00, TypeKTable0},
  {10, 20.644, TypeKTable1},
  { 7, 54.886, TypeKTable2}
};

const double TypeKReverse[] =
{
  -0.176004136860E-01,
   0.389212049750E-01,
   0.185587700320E-04,
  -0.994575928740E-07,
   0.318409457190E-09,
  -0.560728448890E-12,
   0.560750590590E-15,
  -0.320207200030E-18,
   0.971511471520E-22,
  -0.121047212750E-25
};

const double TypeKReverseExtra[] =
{
  0.118597600000E+00,
 -0.118343200000E-03,
  0.126968600000E+03
};

const NIST_Reverse TypeKReverseTable = 
{
  10,						// nCoefficients
  TypeKReverse
};

// ****************************************************************************
// Type R data
const double TypeRTable0[] = 
{
   0.0000000E+00,
   1.8891380E+02,
  -9.3835290E+01,
   1.3068619E+02,
  -2.2703580E+02,
   3.5145659E+02,
  -3.8953900E+02,
   2.8239471E+02,
  -1.2607281E+02,
   3.1353611E+01,
  -3.3187769E+00
};

const double TypeRTable1[] = 
{
   1.334584505E+01,
   1.472644573E+02,
  -1.844024844E+01,
   4.031129726E+00,
  -6.249428360E-01,
   6.468412046E-02,
  -4.458750426E-03,
   1.994710149E-04,
  -5.313401790E-06,
   6.481976217E-08,
};	

const double TypeRTable2[] = 
{
  -8.199599416E+01,
   1.553962042E+02,
  -8.342197663E+00,
   4.279433549E-01,
  -1.191577910E-02,
   1.492290091E-04
};

const double TypeRTable3[] = 
{
   3.406177836E+04,
  -7.023729171E+03,
   5.582903813E+02,
  -1.952394635E+01,
   2.560740231E-01
};

const NIST_Table TypeRTables[] =
{
  {11,  1.923, TypeRTable0},
  {10, 13.228, TypeRTable1},
  { 6, 19.739, TypeRTable2},
  { 5, 21.103, TypeRTable3}
};

const double TypeRReverse[] = 
{
   0.000000000000E+00,
   0.528961729765E-02,
   0.139166589782E-04,
  -0.238855693017E-07,
   0.356916001063E-10,
  -0.462347666298E-13,
   0.500777441034E-16,
  -0.373105886191E-19,
   0.157716482367E-22,
  -0.281038625251E-26
};

const NIST_Reverse TypeRReverseTable = 
{
  10,  	        // nCoefficients
  TypeRReverse
};	

// ****************************************************************************
// Type S data
const double TypeSTable0[] = 
{
   0.00000000E+00,
   1.84949460E+02,
  -8.00504062E+01,
   1.02237430E+02,
  -1.52248592E+02,
   1.88821343E+02,
  -1.59085941E+02,
   8.23027880E+01,
  -2.34181944E+01,
   2.79786260E+00
};

const double TypeSTable1[] = 
{
   1.291507177E+01,
   1.466298863E+02,
  -1.534713402E+01,
   3.145945973E+00,
  -4.163257839E-01,
   3.187963771E-02,
  -1.291637500E-03,
   2.183475087E-05,
  -1.447379511E-07,
   8.211272125E-09
};

const double TypeSTable2[] = 
{
  -8.087801117E+01,
   1.621573104E+02,
  -8.536869453E+00,
   4.719686976E-01,
  -1.441693666E-02,
   2.081618890E-04
};
const double TypeSTable3[] = 
{
   5.333875126E+04,
  -1.235892298E+04,
   1.092657613E+03,
  -4.265693686E+01,
   6.247205420E-01
};

const NIST_Table TypeSTables[4] =
{
  {10,  1.874, TypeSTable0},
  {10, 11.950, TypeSTable1},
  { 6, 17.536, TypeSTable2},
  { 5, 18.693, TypeSTable3}
};

const double TypeSReverse[] = 
{
   0.000000000000E+00,
   0.540313308631E-02,
   0.125934289740E-04,
  -0.232477968689E-07,
   0.322028823036E-10,
  -0.331465196389E-13,
   0.255744251786E-16,
  -0.125068871393E-19,
   0.271443176145E-23
};

const NIST_Reverse TypeSReverseTable = 
{
  9, 		// nCoefficients
  TypeSReverse
};

// ****************************************************************************
// Type T data
const double TypeTTable0[] = 
{
   0.0000000E+00,
   2.5949192E+01,
  -2.1316967E-01,
   7.9018692E-01,
   4.2527777E-01,
   1.3304473E-01,
   2.0241446E-02,
   1.2668171E-03
};

const double TypeTTable1[] = 
{
   0.000000E+00,
   2.592800E+01,
  -7.602961E-01,
   4.637791E-02,
  -2.165394E-03,
   6.048144E-05,
  -7.293422E-07
};

const NIST_Table TypeTTables[2] =
{
  {8,   0.00, TypeTTable0},
  {7, 20.872, TypeTTable1}
};

const double TypeTReverse[] = 
{
   0.000000000000E+00,
   0.387481063640E-01,
   0.332922278800E-04,
   0.206182434040E-06,
  -0.218822568460E-08,
   0.109968809280E-10,
  -0.308157587720E-13,
   0.454791352900E-16,
  -0.275129016730E-19
};

const NIST_Reverse TypeTReverseTable = 
{
  9,		// nCoefficients
  TypeTReverse
};

// ****************************************************************************
// Type N data
const double TypeNTable0[] =
{
  0.0000000E+00,
  3.8436847E+01,
  1.1010485E+00,
  5.2229312E+00,
  7.2060525E+00,
  5.8488586E+00,
  2.7754916E+00,
  7.7075166E-01,
  1.1582665E-01,
  7.3138868E-03
};

const double TypeNTable1[] =
{
  0.00000E+00,
  3.86896E+01,
  -1.08267E+00,
  4.70205E-02,
  -2.12169E-06,
  -1.17272E-04,
  5.39280E-06,
  -7.98156E-08
};

const double TypeNTable2[] =
{
   1.972485E+01,
   3.300943E+01,
  -3.915159E-01,
   9.855391E-03,
  -1.274371E-04,
   7.767022E-07
};

const NIST_Table TypeNTables[3] =
{
  {10,   0.00, TypeNTable0},
  { 8, 20.613, TypeNTable1},
  { 6, 47.513, TypeNTable2}
};

const double TypeNReverse[] = 
{
   0.000000000000E+00,
   0.259293946010E-01,
   0.157101418800E-04,
   0.438256272370E-07,
  -0.252611697940E-09,
   0.643118193390E-12,
  -0.100634715190E-14,
   0.997453389920E-18,
  -0.608632456070E-21,
   0.208492293390E-24,
  -0.306821961510E-28
};

const NIST_Reverse TypeNReverseTable = 
{
  11,	    // nCoefficients
  TypeNReverse
};

// ****************************************************************************
// Type E data
const double TypeETable0[] = 
{
   0.0000000E+00,
   1.6977288E+01,
  -4.3514970E-01,
  -1.5859697E-01,
  -9.2502871E-02,
  -2.6084314E-02,
  -4.1360199E-03,
  -3.4034030E-04,
  -1.1564890E-05
};

const double TypeETable1[] = 
{
   0.0000000E+00,
   1.7057035E+01,
  -2.3301759E-01,
   6.5435585E-03,
  -7.3562749E-05,
  -1.7896001E-06,
   8.4036165E-08,
  -1.3735879E-09,
   1.0629823E-11,
  -3.2447087E-14
};

const NIST_Table TypeETables[2] =
{
  { 9,   0.00, TypeETable0},
  {10, 76.373, TypeETable1}
};

const double TypeEReverse[] =
{
   0.000000000000E+00,
   0.586655087100E-01,
   0.450322755820E-04,
   0.289084072120E-07,
  -0.330568966520E-09,
   0.650244032700E-12,
  -0.191974955040E-15,
  -0.125366004970E-17,
   0.214892175690E-20,
  -0.143880417820E-23,
   0.359608994810E-27
};

const NIST_Reverse TypeEReverseTable = 
{
  11,		// nCoefficients
  TypeEReverse
};

// ****************************************************************************
// Type B data
const double TypeBTable0[] = 
{
	 9.8423321E+01,
	 6.9971500E+02,
	-8.4765304E+02,
	 1.0052644E+03,
	-8.3345952E+02,
	 4.5508542E+02,
	-1.5523037E+02,
	 2.9886750E+01,
	-2.4742860E+00
};

const double TypeBTable1[] = 
{
	 2.1315071E+02,
	 2.8510504E+02,
	-5.2742887E+01,
	 9.9160804E+00,
	-1.2965303E+00,
	 1.1195870E-01,
	-6.0625199E-03,
	 1.8661696E-04,
	-2.4878585E-06
};

const NIST_Table TypeBTables[2] =
{
  {9,  2.431, TypeBTable0},
  {9, 13.820, TypeBTable1}
};

const double TypeBReverse[] = 
{
   0.000000000000E+00,
  -0.246508183460E-03,
   0.590404211710E-05,
  -0.132579316360E-08,
   0.156682919010E-11,
  -0.169445292400E-14,
   0.629903470940E-18
};

const NIST_Reverse TypeBReverseTable = 
{
  7,		// nCoefficients
  TypeBReverse
};

const double CJCGradients[16] =
  { 1.310,  0.940,  0.0566,  0.256,  1.014,  0.880,  0.850,  0.870,
    0.880,  1.036,  1.282,  1.506,   0.188,   0.530, 0.934,  1.136
  };


// ****************************************************************************

const Thermocouple_Data ThermocoupleData[8] =
{
  {
    3,	                 // nTables
    &TypeJReverseTable,  // Reverse Table
    TypeJTables		 // Tables
  },
  {
    3, 			 // nTables
    &TypeKReverseTable,  // Reverse Table
    TypeKTables		 // Tables
  },
  {
    2, 			// nTables
    &TypeTReverseTable, // Reverse Table
    TypeTTables		// Tables
  },
  {
    2, 			// nTables
    &TypeEReverseTable, // Reverse Table
    TypeETables		// Tables
  },
  {
    4, 			// nTables
    &TypeRReverseTable, // Reverse Table
    TypeRTables		// Tables
  },
  {
    4, 			// nTables
    &TypeSReverseTable, // Reverse Table
    TypeSTables		// Tables
  },
  {
    2, 			// nTables
    &TypeBReverseTable, // Reverse Table
    TypeBTables		// Tables
  },
  {
    3, 			// nTables
    &TypeNReverseTable, // Reverse Table
    TypeNTables		// Tables
  }
};	
//*********************************************************************************

void usbBuildGainTable_USB2416(usb_dev_handle *udev, double table[NGAINS_2416][2])
{
  /*
    Builds a lookup table of calibration coefficents to translate values into voltages:
         voltage = value*table[gain#][0] + table[gain#][1]
    only needed for fast lookup.
  */
  int j, k;
  __u16 address = 0x00A0;

  for (j = 0; j < NGAINS_2416; j++) {
    for (k = 0; k < 2; k++) {
      usbReadMemory_USB2416(udev, 8, address, (__u8 *) &table[j][k]);
      address += 0x8;
    }
  }
  return;
}

void usbBuildGainTable_USB2416_4AO(usb_dev_handle *udev, double table_AO[NCHAN_AO_2416][2])
{
  /*
    Builds a lookup table of calibration coefficents to translate values into voltages:
       corrected value = value*table[VDAC#][0] + table[VDAC][1]
  */

  int j, k;
  __u16 address = 0x0180;

  for (j = 0; j < NCHAN_AO_2416; j++) {
    for (k = 0; k < 2; k++) {
      usbReadMemory_USB2416(udev, 8, address, (__u8 *) &table_AO[j][k]);
      address += 0x8;
    }
  }
  return;
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* reads digital port  */
__u8  usbDIn_USB2416(usb_dev_handle *udev, __u8 port)
{
  /*
    This command reads the current state of the DIn port.
    port:  0  onboard (pins 0-7)
           1  Expansion 1 (pins 8-15)
           2  Expansion 2 (pins 16-23)
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data = 0x0;

  usb_control_msg(udev, requesttype, DIN, (__u16) port,  0x0, (char *) &data, sizeof(data), HS_DELAY);
  return data;
}

/* read/writes digital port latch */
void usbDOut_USB2416(usb_dev_handle *udev, __u8 value, __u8 port)
{
  /*
    This command writes the DOut port latch.
    port:  0  onboard (pins 0-7)
           1  Expansion 1 (pins 8-15)
           2  Expansion 2 (pins 16-23)

	   NOTE: The DIO are open-drain, which when used as an output is capable of sinking up to 150 mA.
	   Writing a "1" to a bit will cause its voltage to go LOW (0V), and writing a "0" to
	   the bit will cause the voltage to go HIGH (5V) by the 47k Ohm pullup resister.
	   See page 23 of the users manual.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  char buf[2];

  buf[0] = port;
  buf[1] = value;

  usb_control_msg( udev, requesttype, DOUT, 0x0, 0x0, buf, sizeof(buf), HS_DELAY );
  return;
}

__u8 usbDOutR_USB2416(usb_dev_handle *udev, __u8 port)
{
  /*
    This command reads the DOut port latch.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 data;

  usb_control_msg(udev, requesttype, DOUT, port, 0x0, (char *) &data, sizeof(data), HS_DELAY);
  return data;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

#define  SIGN_BITMASK (1 << 23)
#define  FULL_SCALE24_BITMASK ((1<<24) - 1)
#define  SIGN_EXT_BITMASK (~FULL_SCALE24_BITMASK)

int  sint24TOint(int int24val)
// Converts a 2's complement signed 24 bit number to a int (32 or 64 bit)
{
  if (int24val & SIGN_BITMASK) {
    int24val |= SIGN_EXT_BITMASK;
  } else {
    int24val &= FULL_SCALE24_BITMASK;
  }

  return int24val;
}

__u32  intTOsint24(int int32)
{
  if (int32 < 0) {
    int32 &= SIGN_EXT_BITMASK;
    int32 |= SIGN_BITMASK;
  } else {
    int32 &= FULL_SCALE24_BITMASK;
  }
  return int32;
}

int usbAIn_USB2416(usb_dev_handle *udev, __u8 channel, __u8 mode, __u8 range, __u8 rate, __u8 *flags)
{
  __u32 data;
  __u16 input1;
  __u16 input2;
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  input1 = (mode << 8) | channel;
  input2 = (rate << 8) | range;

  usb_control_msg(udev, requesttype, AIN, input1, input2, (char *) &data, sizeof(data), HS_DELAY);
  //  printf("input1 = %#x    input2 = %#x    data = %#x\n", input1, input2, data);
  *flags = (data >> 24);
  data &= 0x00ffffff;
  return sint24TOint(data);
}

void usbAInScanStop_USB2416(usb_dev_handle *udev)
{
  /*
    This command stops the analog input scan (if running)
  */
  
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

__u8  usbAInScanStatus_USB2416(usb_dev_handle *udev, __u16 *depth)
{
  /*
    This command reads the status of the analog input scan.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct t_AInStatus {
    __u16 depth;    // number of samples currently in the FIFO (max 512)
    __u8 status;    // bit 0: 1 = scan running
                    // bit 1: 1 = scan overrun due to fifo full
                    // bit 2: 1 = scan overrun due to pacer period too short for queue
                    // bit 3-7: reserved
  } AInStatus;
  usb_control_msg(udev, requesttype, AIN_SCAN_STATUS, 0x0, 0x0, (char *) &AInStatus, sizeof(AInStatus), HS_DELAY);
  *depth =  AInStatus.depth;
  return AInStatus.status;
}

void usbAInScanQueueWrite_USB2416(usb_dev_handle *udev, AInScanQueue *queue)
{
  /*
    This command reads or writes the analog input scan channel queue.  The
    queue may have a maximum of 64 entries.  The queue can not be mondified
    during an AInScan.
  */
  
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (queue->count > MAX_QUEUE_SIZE) queue->count = MAX_QUEUE_SIZE;
  usb_control_msg(udev, requesttype, AIN_SCAN_QUEUE, 0x0, 0x0, (char *) queue, (1+queue->count*4), HS_DELAY);
}

void usbAInScanQueueRead_USB2416(usb_dev_handle *udev, AInScanQueue *queue)
{
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AIN_SCAN_QUEUE, 0x0, 0x0, (char *) queue, sizeof(AInScanQueue), HS_DELAY);
}

double usbAInMinPacerPeriod_USB2416(usb_dev_handle *udev)
{
  AInScanQueue scanQueue;
  double period = 0.0;
  int i;

  // Calculate the minimum allowable pacer period
  usbAInScanQueueRead_USB2416(udev, &scanQueue);
  for(i = 0; i < scanQueue.count; i++) {
    switch(scanQueue.queue[i].rate) {
      case HZ30000: period += 1./30000. + 640.E-6; break;
      case HZ15000: period += 1./15000. + 640.E-6; break;
      case HZ7500:  period += 1./7500.  + 640.E-6; break;
      case HZ3750:  period += 1./3750.  + 640.E-6; break;
      case HZ2000:  period += 1./2000.  + 640.E-6; break;
      case HZ1000:  period += 1./1000.  + 640.E-6; break;
      case HZ500:   period += 1./500.   + 640.E-6; break;
      case HZ100:   period += 1./100.   + 640.E-6; break;
      case HZ60:    period += 1./60.    + 640.E-6; break;
      case HZ50:    period += 1./50.    + 640.E-6; break;
      case HZ30:    period += 1./30.    + 640.E-6; break;
      case HZ25:    period += 1./25.    + 640.E-6; break;
      case HZ15:    period += 1./15.    + 640.E-6; break;
      case HZ10:    period += 1./10.    + 640.E-6; break;
      case HZ5:     period += 1./5.     + 640.E-6; break;
      case HZ2_5:   period += 1./2.5    + 640.E-6; break;      
      default:  printf("Unknown rate.\n"); break;
    }      
  }
  return period;
}

void usbAInScanStart_USB2416(usb_dev_handle *udev, double frequency, __u16 count, __u8 packet_size, int *data)
{
  /*
    This command starts an analog input channel scan.  The channel
    configuration for the scan is set with AInScanQueue_US2416Write().
    This command will result in a bus stall if usbAInScan is currently
    running.
  */

  double period = 0.0;
  int nbytes;
  int ret = -1;
  __u32 pacer_period;
  __u16 depth;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 status;

  struct t_scanPacket {
    __u8 pacer_period[4]; // pacer timer period = 50 kHz / (sample frequency)
    __u8 count[2];        // the total number of scans to perform (0 = continuous)
    __u8 packet_size;     // the number of samples per bulk transfer (0-15)
  } scanPacket;

  period = usbAInMinPacerPeriod_USB2416(udev);
  if (period > 1./frequency) {
    pacer_period = rint(period*50000.);
  } else {
    pacer_period = rint(50000./frequency);
  }
  
  memcpy(scanPacket.pacer_period, &pacer_period, 4);
  memcpy(scanPacket.count, &count, 2);
  scanPacket.packet_size = packet_size;

  if (usbAInScanStatus_USB2416(udev, &depth) & INPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the FIFO buffer.\n", depth);
    return;
  }
  usb_control_msg(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (char *) &scanPacket, sizeof(scanPacket), HS_DELAY);

  nbytes = (packet_size+1)*sizeof(int);
  status = usbStatus_USB2416(udev);

  while (status & INPUT_SCAN_RUNNING) {
    ret = usb_bulk_read(udev, USB_ENDPOINT_IN|1, (char *) data, nbytes, HS_DELAY);
    if (ret <= 0) {
      return;
    } else {
      data += ret/sizeof(int);
    }
    status = usbStatus_USB2416(udev);
  }
  
}
/***********************************************
 *          Analog Output                      *
 ***********************************************/
void usbAOutScanStop_USB2416_4AO(usb_dev_handle *udev)
{
  /* This command stops the analog output scan (if running) and
     clears the output FIFO data.  Any data in the endpoint buffers will
     be flushed, so this command is useful to issue prior to the
     beginning of an output scan.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, AOUT_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

__u8  usbAOutScanStatus_USB2416_4AO(usb_dev_handle *udev, __u16 *depth)
{
  /*  This comamnd reads the status of the analog output scan:
      depth: the number of samples currently in the FIFO (max 1024)
      status: bit 0: 1 = scan running
              bit 1: 1 = scan underrun
              bits 2-7: reserved
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct t_AOutStatus {
    __u16 depth;    // number of samples currently in the FIFO (max 512)
    __u8 status;    // bit 0: 1 = scan running
                    // bit 1: 1 = scan overrun due to fifo full
                    // bit 2: 1 = scan overrun due to pacer period too short for queue
                    // bit 3-7: reserved
  } AOutStatus;
  usb_control_msg(udev, requesttype, AOUT_SCAN_STATUS, 0x0, 0x0, (char *) &AOutStatus, sizeof(AOutStatus), HS_DELAY);
  *depth =  AOutStatus.depth;
  return AOutStatus.status;
}

void usbAOutScanStart_USB2416_4AO(usb_dev_handle *udev, double frequency, __u16 scans, __u8 options)
{
  /* This command configures the analog output channel scan.
     This command will result in a bus stall if an AOUT_SCAN is
     currently running.

     Notes:
     The output scan operates with the host continuously transferring data for the
     outputs until the end of the scan.  If the "scans" parameter is 0, the scan will run
     until the AOutScanStop command is issued by the host; if it is nonzero, the scan
     will stop automatically after the specified number of scans have been output.
     The channels in the scan are selected in the options bit field.  "Scans" refers to
     the number of updates to the channels (if all channels are used, one scan s an
     update to all 4 channels).

     period = 50kHz / frequency

     Multiple channels are updated simultaneously using the same time base.

     The output data is sent using the bulk out endpoint.  The data format is:
     low channel sample 0 : ... : [high channel sample 0]
     low channel sample 1 : ... : [high channel sample 1]
     .
     .
     .
     low channel sample n : ... : [high channel sample n]

     The output data is written to a 512-sample FIFO in the device.  The bulk endpoint
     data is only accepted if there is room in the FIFO.  Output data may be sent to the
     FIFO before the start of the scan, and the FIFO is cleared when the AOutScanStop command
     is received.  The scan will not begin until the command is sent (and output data is in
     the FIFO).  Data will be output until reaching the specified number of scans (in single
     execution mode)or an AOutScanStop command is sent.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct t_scanPacket {
    __u16 pacer_period;  // pacer timer period = 50 kHz / (scan frequency)
    __u8 scans[2];       // the total number of scans to perform (0 = continuous)
    __u8 options;        // bit 0: 1 = include channel 0 in output scan
			 // bit 1: 1 = include channel 1 in output scan
			 // bit 2: 1 = include channel 2 in output scan
			 // bit 3: 1 = include channel 3 in output scan
			 // bits 4-7 reserved
  } scanPacket;
  __u16 depth;
  
  scanPacket.pacer_period = (__u16) rint(50000./frequency);
  memcpy(scanPacket.scans, &scans, 2);
  scanPacket.options = options;

  if (usbAOutScanStatus_USB2416_4AO(udev, &depth) & OUTPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the Output FIFO buffer.\n", depth);
    return;
  }
  usb_control_msg(udev, requesttype, AOUT_SCAN_START, 0x0, 0x0, (char *) &scanPacket, sizeof(scanPacket), HS_DELAY);
}

void usbAOut_USB2416_4AO(usb_dev_handle *udev, int channel, double voltage, double table_AO[NCHAN_AO_2416][2])
{
  /* This command writes the values for the analog output channels.  The
     values are 16-bit signed numbers.  This command will result in a control
     pipe stall if an output scan is running.  The equation for the output voltage is:

           V_out = (value / 2^15)* V_ref

     where "value" is the value written to the channel and V_ref = 10V.  
  */
  double dvalue;
  __u16 depth;
  short int value;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  struct t_aOut {
    __u8 value[2];
    __u8 command;
  } aOut;

  dvalue = voltage*(1<<15)/10.;
  dvalue = dvalue*table_AO[channel][0] + table_AO[channel][1];  

  if (dvalue >= 32767.) {
    value = 0x7fff;
  } else if (dvalue <= -32768.) {
    value = 0x8000;
  } else {
    value = (short int) dvalue;
  }

  memcpy(aOut.value, &value, 2);
  aOut.command = 0x10 | (channel << 1);
  
  if (usbAOutScanStatus_USB2416_4AO(udev, &depth) & OUTPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the Output FIFO buffer.\n", depth);
    return;
  }
  usb_control_msg(udev, requesttype, AOUT, 0x0, 0x0, (char *) &aOut, sizeof(aOut), HS_DELAY);
}


/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbCounterInit_USB2416(usb_dev_handle *udev, __u8 counter)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
     counter will be initialized to zero.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, (char *) &counter, sizeof(counter), HS_DELAY);
  
  return;
}

__u32 usbCounter_USB2416(usb_dev_handle *udev, __u8 counter)
{
  /*
    This command reads the 32-bit event counter.  
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u32 counts[2] = {0x0, 0x0};

  usb_control_msg(udev, requesttype, COUNTER, 0x0, 0x0, (char *) &counts, sizeof(counts), HS_DELAY);
  if (counter == COUNTER0) {
    return counts[0];
  } else {
    return counts[1];
  }
}

void usbCJC_USB2416(usb_dev_handle *udev, float temp[8])
{
  /*
    This command reads the CJC sensors.  The temperature in degrees
    Celsius is calculated as:

     T = 128.(value/2^15)
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  short int value[8];
  int i;

  usb_control_msg(udev, requesttype, CJC, 0x0, 0x0, (char *) &value, sizeof(value), HS_DELAY);
  for (i = 0; i < 8; i++) {
    temp[i] = value[i]/256.0;
  }
}

/* blinks the LED of USB device */
void usbBlink_USB2416(usb_dev_handle *udev, __u8 bcount)
{
  /*
    This command will blink the device LED "count" number of times
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 cmd = BLINK_LED;

  printf("Blinking LED (%x) for %d counts\n", cmd, bcount);
  usb_control_msg(udev, requesttype, BLINK_LED, 0x0, 0x0, (char *) &bcount, 1, HS_DELAY);
  return;
}

__u8 usbStatus_USB2416(usb_dev_handle *udev)
{
  /*
    This command retrieves the status of the device.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  __u8 status = 0x0;

  usb_control_msg(udev, requesttype, GET_STATUS, 0x0, 0x0, (char *) &status, sizeof(status), HS_DELAY);
  return status;
}  

void usbGetSerialNumber_USB2416(usb_dev_handle *udev, char serial[9])
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

void usbSetSerialNumber_USB2416(usb_dev_handle *udev, char serial[9])
{
  /*
    This command writes the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). The new serial number will be programmed but not used until
    hardware reset.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, SERIAL, 0x0, 0x0, serial, 8, HS_DELAY);
  return;
}

void usbGetVersion_USB2416(usb_dev_handle *udev, __u16 version[4])
{
  /*
    This command reads the microcontroller firmware versions.  The firmware
    versions are returned as packed hexadecmal BCD values, i.e. if version
    = 0x0132, then the firmware version is 1.32.
  */
  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, VERSION, 0x0, 0x0, (char *) version, 8, HS_DELAY);
}

void usbReadMemory_USB2416(usb_dev_handle *udev, __u16 length,  __u16 address, __u8 *data)
{
  /* This command reads data from the available data EEPROM memory.
     The number of bytes to read is specified in the wLength (for
     writes it is wLength - sizeof(address)).
  */

  __u8 requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, MEMORY, address, 0x0, (char *) data, length, HS_DELAY);
  return;
}

void usbWriteMemory_USB2416(usb_dev_handle *udev, __u16 length,  __u16 address, __u8 *data)
{
  /* This command writes data to the available data EEPROM memory. 
     The number of bytes to read is specified in the wLength (for
     writes it is wLength - sizeof(address)).  The first 2 byes of data is
     the address.

     Note: this function is not reentrant
  */

  char *buf;
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  buf = malloc(length + 2);
  memcpy(buf, &address, 2);
  memcpy(&buf[2], data, length);
  usb_control_msg(udev, requesttype, MEMORY, 0x0, 0x0, buf, length+2, HS_DELAY);
  free(buf);
  return;
}

void usbReset_USB2416(usb_dev_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device disconnects from the USB bus and resets
    its microcontroller.
  */

  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbCalConfig_USB2416(usb_dev_handle *udev, __u8 value)
{
  /*
    This command will configure the calibration source.
    value =  0:  +0.078V
             1:  -0.078V
             2:  +0.156V
	     3:  -0.156V
	     4:  +0.325V
     	     5:  -0.325V
	     6:  +0.626V
     	     7:  -0.626V
	     8:  +1.25V
     	     9:  -1.25V
	    10:  +2.50V
     	    11:  -2.50V
	    12:  +5.00V
     	    13:  -5.00V
	    14:  +10.0V
     	    15:  -10.0V
	    16:  +18.0V
     	    17:  -18.0V
            18: External calibration source.	     
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  usb_control_msg(udev, requesttype, CAL_CONFIG, 0x0, 0x0, (char *) &value, sizeof(value), HS_DELAY);
  return;
}

void usbADCal_USB2416(usb_dev_handle *udev)
{
  /*
    The command will perform A/D self calibration.
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, ADCAL, 0x0, 0x0, NULL, 0x0, HS_DELAY);
  return;
}

void usbTCCalMeasure(usb_dev_handle *udev, __u8 value)
{
  /* The command will enable measurement of the TC cal source
     value: 0: normal operation
            1: TC cal source measurment mode (JP3)
  */
  __u8 requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  usb_control_msg(udev, requesttype, TC_CAL, 0x0, 0x0, (char *) &value, sizeof(value), HS_DELAY);
  return;
}

void cleanup_USB2416( usb_dev_handle *udev )
{
  if (udev) {
    usb_clear_halt(udev, USB_ENDPOINT_IN|1);
    usb_clear_halt(udev, USB_ENDPOINT_OUT|1);
    usb_release_interface(udev, 0);
    usb_close(udev);
  }
}

void voltsTos16_USB2416_4AO(double *voltage, __s16 *data, int nSamples, double table_AO[])
{
  /* This routine converts an array of voltages (-10 to 10 volts) to singed 24 bit ints for the DAC */
  int i;
  double dvalue;

  for (i = 0; i < nSamples; i++) {
    dvalue = voltage[i]*(1<<15)/10.;                             /* convert voltage to signed value */
    dvalue = dvalue*table_AO[0] + table_AO[1];                   /* correct for calibration errors */
    if (dvalue >= 32767.) {
      data[i] = 0x7fff;
    } else if (dvalue <= -32768.) {
      data[i] = 0x8000;
    } else {
      data[i] = (short int) dvalue;
    }
  }
}

double volts_USB2416(usb_dev_handle *udev, const int gain, const int value)
{
  double volt = 0.0;
  
  switch (gain) {
    case BP_20V:
      volt = value * 20.0 / 0x7fffff;
      break;
    case BP_10V:
      volt = value * 10.0 / 0x7fffff;
      break;
    case BP_5V:
      volt = value * 5.0 / 0x7fffff;
      break;
    case BP_2_5V:
      volt = value * 2.5 / 0x7fffff;
      break;
    case BP_1_25V:
      volt = value * 1.25 / 0x7fffff;
      break;
    case BP_625V:
      volt = value * 0.625 / 0x7fffff;
      break;
    case BP_312V:
      volt = value * 0.312 / 0x7fffff;
      break;
    case BP_156V:
      volt = value * 0.156 / 0x7fffff;
      break;
    case BP_078V:
      volt = value * 0.078 / 0x7fffff;
      break;
  }

  return volt;
}


double NISTCalcVoltage(unsigned char tc_type, double temp)
{
  unsigned char nCoef;
  unsigned char index;
  double fVoltage;
  double fTemp;
  double fExtra = 0.0;
	
  // select appropriate NIST table data
  nCoef = ThermocoupleData[tc_type].ReverseTable->nCoefficients;
	
  // calc V
  if (tc_type == TYPE_K) {
    // extra calcs for type K
    fTemp = temp - TypeKReverseExtra[2];
    fTemp *= fTemp;
    fTemp *= TypeKReverseExtra[1];
    fExtra = exp(fTemp);
    fExtra *= TypeKReverseExtra[0];
  }
  
  fTemp = 1.0;
  fVoltage = ThermocoupleData[tc_type].ReverseTable->Coefficients[0];
  for (index = 1; index < nCoef; index++) {
    fTemp *= temp;
    fVoltage += fTemp *
      ThermocoupleData[tc_type].ReverseTable->Coefficients[index];
  }
  
  if (tc_type == TYPE_K)
    fVoltage += fExtra;

  return fVoltage;
}

double NISTCalcTemp(unsigned char tc_type, double voltage)
{
  unsigned char index;
  unsigned char num;
  unsigned char mytable;
  double fVoltage;
  double fResult;
	
  
  // determine which temp range table to use with the threshold V
  num = ThermocoupleData[tc_type].nTables;
  index = 0;
  mytable = 0;
	
  while ((index < num) && (voltage > ThermocoupleData[tc_type].Tables[index].VThreshold)) {
    index++;
  }
	
  if (index == num) {
    mytable = index - 1;
  } else {
    mytable = index;
  }
	
  // calculate T using NIST table
  num = ThermocoupleData[tc_type].Tables[mytable].nCoefficients;
  fVoltage = 1.0;
  fResult = ThermocoupleData[tc_type].Tables[mytable].Coefficients[0];
  for (index = 1; index < num; index++)	{
    fVoltage *= voltage;
    fResult += fVoltage *
      ThermocoupleData[tc_type].Tables[mytable].Coefficients[index];
  }
  
  return fResult;
}

double tc_temperature_USB2416(usb_dev_handle *udev, int tc_type, __u8 channel, double table_AI[NGAINS_2416][2])
{
  int value;          // integer value of the temperature
  __u8 flag;
  double tc_voltage;
  double CJC_Temp;
  float cjc_array[8];

  // Read the raw voltage (Mode = 4, Range = +/- .078V, Rate = 1kS/s)
  value = usbAIn_USB2416(udev, channel, 4, 8, 5, &flag);
  if (flag & 0x80) {
    printf("TC open detected.  Check wiring on channel %d\n", channel);
    return -1;
  }
  // Apply calibration offset from Gain Table (EEPROM) address 0x0130 (slope) and 0x0138 (offset)
  value = value*table_AI[9][0] + table_AI[9][1];
  // Calculate the TC voltage from the corrected values
  tc_voltage = (value * 2. * 0.078125) / 16777216.;
  // Read the correct CJC block from the array
  usbCJC_USB2416(udev, cjc_array);
  // Correct the CJC Temperature by the CJCGradiant for the appropriate channel
  CJC_Temp = cjc_array[channel/4] - CJCGradients[channel];
  // Calculate the CJC voltage using the NIST polynomials and add to tc_voltage in millivolts
  tc_voltage = NISTCalcVoltage(tc_type, CJC_Temp) + 1000.*tc_voltage;
  // Calcualate actual temperature using reverse NIST polynomial.
  return (NISTCalcTemp(tc_type, tc_voltage));
}
