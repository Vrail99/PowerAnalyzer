from matplotlib.animation import AVConvBase
import serial
import time
import serial.tools.list_ports
import tkinter.messagebox


class SerialBus:
    def __init__(self):
        self.sBus = serial.Serial()
        self.name = "None"
        self.linereader = None
        self.writeTimeout = 2  # Seconds
        self.readTimeout = 2  # Seconds
        self.currPort = None  # Current Port
        self.port_list = []  # Empty Port List

    def openPort(self, port: str):
        try:
            if (self.deviceOpen()):  # If open: close port and reopen with timings
                self.sBus.close()
                time.sleep(1)  # Wait for 1s
            self.sBus = serial.Serial(port, timeout=self.readTimeout,
                                      write_timeout=self.writeTimeout, baudrate=115200)

            self.currPort = port
            self.linereader = ReadLine(self.sBus)
            # Check if the port is correct
            try:
                self.writeString('ci')
                answer = self.linereader.readline().decode('utf-8').rstrip()
                if (answer != "tconn"):
                    print("Error connecting")
                    self.sBus.close()
                    return 0

            except serial.serialutil.SerialTimeoutException as e:
                tkinter.messagebox.showinfo("Info",
                                            "Timeout. Connection with Teensy-port '"+str(self.currPort) +
                                            "' not possible.\n Maybe a wrong port?")
                self.sBus.close()
                return 0

        except ValueError:
            print("Error, couldn't open Serial port. "
                  "Maybe already connected in another program?")
            return 0
        except serial.serialutil.SerialException as e:
            print("The Port does not exist", e)
            return 0

        return 1

    def closePort(self):
        try:
            self.sBus.close()
        except:
            print("Error! Port cannot be closed")

    def deviceOpen(self):
        return self.sBus.is_open

    # Read-Functions
    def readIntValue(self, address=None):
        value = -1
        if (self.deviceOpen() and address != None):
            try:
                self.sBus.write(address.encode())
                value = int(self.linereader.readline())
            except:
                print("Error while reading/writing integer value")
                print("{}\t{}".format(address, value))
        return value

    def readEEPROMValue(self, address, mask, pos):
        value = -999
        if (self.deviceOpen()):
            try:
                stri = "ev<" + address + " " + mask + " " + pos + ">"
                self.sBus.write(stri.encode())
                value = int(self.linereader.readline())
            except:
                print("Error while reading/writing EEPROM value")
                print("{}\t{}".format(address, value))

        return value

    # Just reads a line
    def readLine(self):
        k = 0
        if (self.deviceOpen()):
            try:
                k = self.linereader.readline()
            except TimeoutError:
                print("Timout while reading string. (Serial bus timed out)")

        return k

    # Write-Functions

    #
    # Writes an EEPROMValue
    #   @param adress   Address of EEPROM-Register
    #   @param value    EEPROM Value
    #   @param mask     Value Mask
    #   @param pos      Position of LSB of value
    #

    def writeEEPROMValue(self, adress, value, mask, pos):
        if (self.deviceOpen()):
            try:
                stream = 'ww<' + adress + ' ' + \
                    str(value) + ' ' + mask + ' ' + str(pos) + '>'
                print(stream)
                self.sBus.write(stream.encode())
            except:
                print("Error while writing")
        else:
            print("Cannot write: device not open")
            return 0

    #
    # Writes a String to the serial Bus
    #   @param string   String to be sent
    #
    def writeString(self, string):
        if (self.deviceOpen()):
            try:
                self.sBus.write(string.encode())
            except serial.SerialTimeoutException:
                print(
                    "Timeout while writing. Not able to reach the teensy or is it busy?")
            except serial.SerialException as e:
                print("Serial Error", e)

    # Getter

    #
    # Makes use of serial.tools to search for available serial ports
    #   @return     port list
    #
    def getPorts(self):
        self.port_list = [port[0]
                          for port in serial.tools.list_ports.comports()]
        if len(self.port_list) == 0:
            self.port_list.append("No serial port found")

        return self.port_list

    #
    # Returns the current Port
    #   @return     current port
    #
    def getCurrentPort(self):
        return self.currPort

    # Setter

    #
    # Sets the read timeout of the serial bus
    #   @param t    time in seconds
    #
    def setReadTimeout(self, t):
        self.readTimeout = t
        self.openPort(self.currPort)
        print("Read Timeout set to ", t, "seconds")

    #
    # Sets the write timeout of the serial bus
    #   @param t    time in seconds
    #
    def setWriteTimeout(self, t):
        self.writeTimeout = t
        self.openPort(self.currPort)
        print("Write Timeout set to ", t, "seconds")

    # Auxiliary

    #
    #   Reads all data out of the bus
    #
    def flushBus(self):
        try:
            self.linereader.flush()
            if (self.sBus.in_waiting > 0):
                self.sBus.read(self.sBus.in_waiting)
        except OSError as e:
            print("OS-Error", e)

    #
    # Prints in-waiting in serial bus and linereader helperclass
    #
    def inwaiting(self):
        if self.deviceOpen():
            print("in_waiting/in Buf:", self.sBus.in_waiting,
                  "/", self.linereader.getBufLen())

    #
    # Stops reading of serial bus
    #
    def stopReading(self, stop):
        self.linereader.stopReading(stop)

    #
    # Reads all in-waiting symbols
    #   @return number of bytes in serial bus and linereader helperclass
    #
    def getInWaiting(self):
        try:
            return self.sBus.in_waiting + self.linereader.getBufLen()
        except AttributeError as e:
            print("None Type", e)
            return 0


##############################################################################################################
################                        Readline Helperclass                          ########################
##############################################################################################################


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
        self.stopped = False

    def readline(self):
        self.counter = 0
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(8192, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i]
                self.buf[0:] = data[i + 1:]
                self.counter = 0
                return r
            else:
                if (self.stopped):
                    self.buf = bytearray()
                    return
                else:
                    if self.counter < 2:
                        self.buf.extend(data)
                        self.counter += 1
                    else:
                        self.counter = 0
                        raise serial.serialutil.SerialTimeoutException

    def getBufLen(self):
        if (self.buf.find(b"\n") >= 0):
            return len(self.buf)
        else:
            self.buf = bytearray()

        return 0

    def flush(self):
        self.buf = bytearray()

    def stopReading(self, stop):
        self.stopped = stop
