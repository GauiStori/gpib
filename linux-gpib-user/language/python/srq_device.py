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

def initialise_device(handle):         # set up device to assert SRQ/RQS
        gpib.write(handle,"*CLS")      # Clear status registers
        gpib.write(handle,"*SRE 32")   # Assert SRQ on Event
        return

def show_devid(handle):                # Show device ID
        print query(handle,"*IDN?")      
        return

print gpib.version()                   # Show package version
ud = gpib.dev(board,device)            # Open the device
gpib.config(board,gpib.IbcAUTOPOLL,1)  # Enable automatic serial polling
gpib.config(ud,gpib.IbcTMO, gpib.T30s) # Set timeout to 30 seconds
show_devid(ud);
initialise_device(ud);
gpib.write(handle,"*TST;*OPC")         # Selftest and request OPC event
# Wait for Timeout or Request Service on device
sta = gpib.wait(ud, gpib.TIMO | gpib.RQS)
if (sta & gpib.TIMO) != 0:
   print "Timed out"
else:
    print "Device asserted RQS"
    stb = gpib.serial_poll(ud)          # Read status byte
    print "stb = %#x"%(stb)
    if (stb & gpib.IbStbESB) != 0:      # Check for Event Status bit
            esr = int(query(ud,"ESR?")) # Read Event Status Register
            if (esr & 1) != 0:          # Check for operation completed
                    print "Device Operation Completed"

# done  
gpib.close(ud)
