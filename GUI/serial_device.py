import serial
import time
from serial.serialutil import SerialTimeoutException, SerialException
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

    def openPort(self, port: str) -> int:
        """Opens the port. Reopens the port, if it was already open.
        Performs a handshake with the teensy to check if the connection can be established.

        Keyword Arguments:

        port -- String of the port to connect to.

        Returns 1 if successful, or 0 if an error occurred."""
        try:
            if (self.deviceOpen()):  # If open: close port and reopen with timings
                self.sBus.close()
                time.sleep(0.5)  # Wait for 1s
            self.sBus = serial.Serial(port, timeout=self.readTimeout,
                                      write_timeout=self.writeTimeout, baudrate=115200)

            self.currPort = port
            self.linereader = ReadLine(self.sBus)
            # Check if the port is correct
            try:
                self.writeString('x')
                time.sleep(0.25)
                self.writeString('ci')
                answer = self.linereader.readline().decode(
                    'utf-8').rstrip()  # Remove any control-characters
                if (answer != "tconn"):
                    print("Error connecting")
                    self.sBus.close()
                    return 0
                else:
                    print("Connected")

            except SerialTimeoutException as e:
                tkinter.messagebox.showinfo("Info",
                                            "Timeout. Connection with Teensy-port '"+str(self.currPort) +
                                            "' not possible.\n Maybe a wrong port?")
                self.sBus.close()
                return 0

        except ValueError:
            print("Error, couldn't open Serial port. "
                  "Maybe already connected in another program?")
            return 0
        except SerialException as e:
            print("The Port does not exist", e)
            return 0

        return 1

    def closePort(self) -> None:
        """Closes the port, if possible"""
        try:
            self.writeString('cd')
            self.writeString('bf')
            self.sBus.close()
        except:
            print("Error! Port cannot be closed")

    def deviceOpen(self) -> bool:
        """Returns the state of the connection"""
        return self.sBus.is_open

    # Read-Functions
    def readIntValue(self, address: str = None) -> int:
        """Read a value from an address

        Keyword Arguments:

        address -- The address to read a value from

        Returns the integer value from the teensy or None"""
        value = None
        if (self.deviceOpen() and address != None):
            try:
                self.sBus.write(address.encode())
                time.sleep(0.01)
            except SerialTimeoutException:
                print("Error writing a string to the Teensy")

            try:
                value = int(self.linereader.readline())
            except SerialTimeoutException:
                print("Error while reading value")
                print("{}\t{}".format(address, value))
            except TypeError as e:
                print(e)

        return value

    def readEEPROMValue(self, address: str, mask: str, pos: str) -> int:
        """Reads a value from the EEPROM of the ACS71020

        Keyword Arguments:

        address -- the adress to be read from \n
        mask    -- the mask for the value \n
        pos     -- the position of the LSB"""
        value = -999
        if (self.deviceOpen()):
            try:
                stri = "ev<" + str(address) + " " + str(mask) + " " + str(pos) + ">"
                self.sBus.write(stri.encode())
            except SerialTimeoutException as e:
                print("Timeout while writing String to Teensy")
                return None
            except TypeError as e:
                print(e)
                return None
            try:
                value = int(self.linereader.readline())
            except SerialTimeoutException as e:
                print("Timeout while reading EEPROM value")
                print(stri)
            except TypeError as e:
                print(e)

        return value

    def readLine(self, dec: str = 'utf-8') -> str:
        """Reads a line from the serial bus.

        Keyword Arguments:

        dec -- the encoding as a string(default: utf-8)

        Returns the string or None, if reading failed"""
        k = None
        if (self.deviceOpen()):
            try:
                k = str(self.linereader.readline().decode(dec))
            except SerialException as e:
                print(e)
            except AttributeError as e:
                print(e)
            except SerialTimeoutException:
                print("Timout while reading string. (Serial bus timed out)")

        return k

    # Write Functions
    def writeEEPROMValue(self, address: str, value: int, mask: str, pos: str) -> int:
        """
        Writes a value to the EEPROM of the ACS71020

        Keyword Arguments:

        adress  -- Address of EEPROM-Register \n
        value   -- EEPROM Value \n
        mask    -- Value Mask \n
        pos     -- Position of LSB of value

        Returns 1 if successful, 0 otherwise
        """
        if (self.deviceOpen()):
            try:
                stream = 'ww<' + address + ' ' + str(value) + ' ' + mask + ' ' + str(pos) + '>'
                print(stream)
                self.sBus.write(stream.encode())
            except SerialTimeoutException:
                print("Timeout while writing")
                return 0
            except UnicodeError:
                print("Encoding not successful")
                return 0
        else:
            print("Cannot write: device not open")
            return 0
        return 1

    def writeString(self, string: str) -> None:
        """Writes a string to the serial Bus

        Keyword Arguments:

        string -- the requested string"""
        if (self.deviceOpen()):
            try:
                self.sBus.write(string.encode())
            except SerialTimeoutException:
                print(
                    "Timeout while writing. Not able to reach the teensy or is it busy?")
            except SerialException as e:
                print("Serial Error", e)

    # Getter

    def getPorts(self) -> list[str]:
        """Reads all ports connected to the computer

        Returns a list with all port names, compatible to open with openPort() function"""
        self.port_list = [str(port[0])
                          for port in serial.tools.list_ports.comports()]
        if len(self.port_list) == 0:
            self.port_list.append("No serial port found")

        return self.port_list

    def getCurrentPort(self):
        """Returns the current connected port"""
        return self.currPort

    # Setter

    def setReadTimeout(self, t: int = 2) -> None:
        """Sets the read timeout of the serial bus

        Keyword Arguments:

        t -- integer timeout in seconds (default: 2)"""
        self.readTimeout = t
        self.openPort(self.currPort)
        print("Read Timeout set to ", t, "seconds")

    def setWriteTimeout(self, t: int = 2) -> None:
        """Sets the write timeout of the serial bus

        Keyword arguments:

        t -- integer timeout in seconds (default: 2)"""
        self.writeTimeout = t
        self.openPort(self.currPort)
        print("Write Timeout set to ", t, "seconds")

    # Auxiliary

    def flushBus(self) -> None:
        """Deletes input buffer and reconnects to clear all buffered Data"""
        if self.deviceOpen():
            self.linereader.flush()
            self.sBus.reset_input_buffer()

    def inwaiting(self) -> tuple[int, int]:
        """Returns a tuple of in-waiting in serial bus and buffered data in linereader helperclass

        (in_waiting, buffered)"""
        if self.deviceOpen():
            return (self.sBus.in_waiting, self.linereader.getBufLen())

    def stopReading(self, stop: bool) -> None:
        """Stops reading of linereader helperclass"""
        self.linereader.stopReading(stop)


##############################################################################################################
################                        Readline Helperclass                          ########################
##############################################################################################################


class ReadLine:
    """Linereader Helperclass, which introduces a buffer for incoming values. It is
    faster than the built-in function readline of pyserial."""

    def __init__(self, s: serial.Serial):
        """Inits the helperclass with an empty buffer

        Keyword Arguments:

        s -- the serialBus instance"""
        self.buf = bytearray()
        self.s = s
        self.stopped = False

    def readline(self) -> bytearray:
        """Reads a Line from the Serial bus, without a newline character.
        reads a maximum of 4096 bytes at once.

        Returns a bytearray with the line
        """
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
                        raise SerialTimeoutException

    def getBufLen(self) -> int:
        """Returns the number of bytes in the buffer"""
        return len(self.buf)

    def flush(self) -> None:
        """Empties the buffer"""
        self.buf = bytearray()

    def stopReading(self, stop):
        """Defines if the readline function returns a value when reading a stream """
        self.stopped = stop
