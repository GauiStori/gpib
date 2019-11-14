import sys
sys.path.append('/usr/local/lib64/python2.7/site-packages')
import gpib

board  = 1
device = 1

def query(handle, command, numbytes=100):
        gpib.write(handle,command)
        response = gpib.read(handle,numbytes)
        response = response.rstrip("\r\n")
        return response

def initialise_device(handle):            # set up device to assert SRQ/RQS
        gpib.write(handle,"*CLS")         # Clear status registers
        gpib.write(handle,"*SRE 32")      # Assert SRQ on OPC
        return

def show_devid(handle):                   # Show device ID
        print query(handle,"*IDN?")      
        return

print gpib.version()                      # Show package version
ud = gpib.dev(board,device)               # Open the device
gpib.config(board,gpib.IbcTMO, gpib.T30s) # Set timeout to 30 seconds
show_devid(ud);
initialise_device(ud);
gpib.write(handle,"*TST;*OPC")            # Initiate selftest and request OPC
# Wait for Timeout or Service Request on board
sta = gpib.wait(board, gpib.TIMO | gpib.SRQI)
if (sta & gpib.TIMO) != 0:
   print "Timed out"
else:
  print "SRQ asserted "
# For each device which might pull SRQ
  stb = gpib.serial_poll(ud)            # Read status byte
  print "stb = %#x"%(stb)
  if (stb & gpib.IbStbRQS) != 0:        # Check for RQS bit
    print "Device asserted RQS"
    if (stb & gpib.IbStbESB) != 0:      # Check for Event Status bit
            esr = int(query(ud,"ESR?")) # Read Event Status Register
            if (esr & 1) != 0:          # Check for operation completed
                    print "Device Operation Completed"

# done  
gpib.close(ud)
