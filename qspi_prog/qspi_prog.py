# Python script to read / write / erase a board's QSPI Flash chip.
import os
import serial
import sys

PORT = '/dev/ttyACM0'
BAUD = 115200

# Reset the serial port's input buffer, otherwise data
# might be read from the past.
with serial.Serial( PORT, BAUD, timeout = 5 ) as tty:
  tty.reset_input_buffer()

# If the input parameters aren't valid, print a help message.
# TODO: Verify parameter types too.
if len( sys.argv ) != 3:
  print( "Usage: python3 qspi_prog.py <start_address> <file_to_write>\r\n" )

# Write a file and verify it by reading it back.
else:
  # Get the file size and address offset as numbers.
  fsize = os.path.getsize( sys.argv[ 2 ] )
  sadr = int( sys.argv[ 1 ] )

  # Print a message describing what the script will do.
  print( "Write %d bytes @ 0x%08X"%( fsize, sadr ) )

  # Write and verify the file.
  with serial.Serial( PORT, BAUD, timeout = 5 ) as tty:
    # Send the "Write data" command.
    tty.write( "W{0}@{1}\r\n".format( fsize, sadr ).encode() )

    # Open the target file as a read-only binary file.
    with open( sys.argv[ 2 ], 'rb' ) as f:
      # Set up some intermediary values to track progress.
      dat_left = fsize
      read_len = 256 - ( sadr % 256 )
      pv = 0
      rxb = ""

      # Wait for the program to finish erasing sectors.
      while rxb != "RDY\r\n":
        rxb = tty.readline()

      # Send data one page at a time.
      while dat_left > 0:
        # File reads can't be larger than one Flash page (256B).
        if read_len > dat_left:
          read_len = dat_left

        # Read N bytes from the file, and write them to the serial connection.
        fbuf = f.read( read_len )
        tty.write( fbuf );

        # Update bookkeeping values.
        dat_left -= read_len
        read_len = 256
        pv += 1
        # Only print every hundredth page, for brevity.
        if pv % 100 == 0:
          print( "Page %d..."%pv )

        # Wait for the chip to respond after each page of data.
        rxb = tty.readline()

    # Done writing data; the target file gets closed here.
    print( "Done.\r\nVerifying..." )

    # Re-open the target file to verify the written data.
    fb = open( sys.argv[ 2 ], 'rb' )

    # Set up values to track verification progress / status.
    prog = 0
    fail = 0
    # Verify data one 4KiB sector at a time.
    for i in range( ( ( fsize - 1 ) / 4096 ) + 1 ):
      # Wait for the chip to finish its previous command.
      rxb = tty.readline()

      # Calculate the number of bytes to read in this sector.
      read_len = 4096
      if fsize - prog < 4096:
        read_len = fsize - prog

      # Send the "Read byte" command.
      tty.write( "R{0}@{1}\r\n".format( read_len, sadr + prog ).encode() )
      # Read the response and its newline.
      bytes_in = tty.read( read_len )
      rxb = tty.readline()

      # Compare the received data to the target file.
      file_bytes = fb.read( read_len )

      # Mark failures, and print a message every 10 sectors.
      if bytes_in != file_bytes:
        print( "Fail: Sector %d does not match."%i )
        fail = 1
      if i % 10 == 0:
        print( "Checked sector %d"%i )

      # Update "progress" value.
      prog += read_len

    # Done verifying data; print a success or failure message.
    if fail:
      print( "Verification failed." )
    else:
      print( "Verification complete." )
