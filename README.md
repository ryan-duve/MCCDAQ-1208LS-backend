MCCDAQ-1208LS-backend
=====================

Development of 1208LS DAQ and MySQL recording.

A C program that reads in info from the MCCDAQ 1208LS USB data acquisition module and writes to a MySQL database.  The volts_LS calls the reading from the module and the "INSERT INTO" statement writes it to the database.  
