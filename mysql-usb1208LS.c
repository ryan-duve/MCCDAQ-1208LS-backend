#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/types.h>
#include <asm/types.h>

#include "pmd.h"
#include "usb-1208LS.h"

//mysql libraries
#include <my_global.h>
#include <mysql.h>

//mysql crap out function
void finish_with_error(MYSQL *con)
{
	fprintf(stderr, "%s\n", mysql_error(con));
	mysql_close(con);
	exit(1);
}

char *getPassword(char *password){
	FILE *pFile;

	//password is stored in a file called "password"
	pFile=fopen("password","r");
	fscanf(pFile,"%s",password);
	fclose(pFile);

	return password;
}
			


int main (int argc, char **argv)
{
	//database password
	char password[16];

	//build MySQL connection
	MYSQL *con = mysql_init(NULL);

	if (con == NULL){
		fprintf(stderr, "%s\n", mysql_error(con));
		exit(1);
	}

	if (mysql_real_connect(con, "localhost", "uva", getPassword(password),"slowcontrols", 0, NULL, 0) == NULL){
		finish_with_error(con);
	}



  //everything between the asterix lines is crap you don't understand
  //***************************************************

  int flag;
  signed short svalue;
  __u8 gain;

  HIDInterface*  hid = 0x0;
  hid_return ret;
  int interface;

  // Debug information.  Delete when not needed    
  // hid_set_debug(HID_DEBUG_ALL);
  // hid_set_debug_stream(stderr);
  // hid_set_usb_debug(2);
  
  ret = hid_init();
  if (ret != HID_RET_SUCCESS) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((interface = PMD_Find_Interface(&hid, 0, USB1208LS_PID)) < 0) {
    fprintf(stderr, "USB 1208LS not found.\n");
    exit(1);
  }else{
    fprintf(stdout, "Acquiring data.  Press 'x' then enter to stop.\n");
  }

  /* config mask 0x01 means all inputs */
  usbDConfigPort_USB1208LS(hid, DIO_PORTB, DIO_DIR_IN);
  usbDConfigPort_USB1208LS(hid, DIO_PORTA, DIO_DIR_OUT);
  usbDOut_USB1208LS(hid, DIO_PORTA, 0x0);
  usbDOut_USB1208LS(hid, DIO_PORTA, 0x0);


  //***************************************************
  //begin program here

  gain = BP_20_00V;//set gain to 20 V
  
  //these two lines make getchar() in the while loop non-blocking
  flag = fcntl(fileno(stdin), F_GETFL);
  fcntl(0, F_SETFL, flag | O_NONBLOCK);
  
  //Voltage to flow rate requires fitting voltages to a line, y=mx+b.  Values for m and b are different for each of the 4 cases:
  //0: separator
  //1: shield
  //2: helium-3
  //3: evaporator
  //
  //Each value of m and b was found empirically with a flow meter and gas bottle
  float m[4]={
	  11.4418,//separator
	  11.2594,//shield
	  10.1275,//helium-3
	  60.7119 //evap
  };
  float b[4]={
	  0.1215,  //separator
	  -0.0256, //shield
	  0.0219,  //helium-3
	  0.1746   //evaporator
  };
  float calculated_flow=0.0;

  int i;
  char buffer[1000];

  do {
	  for(i=0;i<4;++i){
		  svalue = usbAIn_USB1208LS(hid, i, gain);

		  //calculate measured value in SLPM
		  //y=mx+b
		  calculated_flow=m[i]*volts_LS(gain, svalue)+b[i];

		  //make and execute the db entry
		  sprintf(buffer, "INSERT INTO usb1208ls (device, raw_reading, measurement_reading) VALUES('d%d',%1f,%1f)",i,volts_LS(gain, svalue),calculated_flow);
		  if (mysql_query(con,buffer)) {
			  finish_with_error(con);
		  }
	  }

	  sleep(1);
	  //usleep(100000);

  } while (!isalpha(getchar()));
      return 0;
}
