import tkinter as tk
from tkinter import ttk
import _thread
import time

from scopePage import ScopeWindow

import utils


############################################################################################################################
################                        Live Page Class                                             ########################
############################################################################################################################


class LivePage(ttk.Frame):
    """Values page for reading measured values"""

    def __init__(self, parent: ttk.Frame, controller) -> None:
        """
        Init of Page 

        Keyword Arguments: \n
        parent -- Parent widget it is placed within \n
        controller -- controller class
        """
        super().__init__(parent, style="TFrame")
        self.contr = controller
        self.sBus = controller.getSerialBus()
        self.name = "Values"

        self.init = False
        self.precision = 6
        self.continuousReading = False
        self.readDelay = 500  # ms

        self.VolatileDict = {}

        self._guiSetup()

    def _guiSetup(self) -> None:
        """Sets up UI Elements"""
        # Frames for Buttons and Tree
        top_btnfrm = ttk.Frame(self, style="TFrame")
        top_btnfrm.grid_anchor('center')
        top_btnfrm.pack(side=tk.TOP, fill="x")

        self.treeFrame = ttk.Frame(self, style="TFrame")
        self.treeFrame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Fill First Button Frame
        self.get_btn = ttk.Button(top_btnfrm, text="Get Values", state=tk.DISABLED,
                                  command=self._getVolatileValues)
        self.get_single_btn = ttk.Button(top_btnfrm, text="Get Single", state=tk.DISABLED,
                                         command=lambda: self._readValue())
        self.readCont_btn = ttk.Button(top_btnfrm, text="Read Selected Cont.", state=tk.DISABLED,
                                       command=self._startReadContinued)
        self.stopRead_btn = ttk.Button(top_btnfrm, text="Stop", state=tk.DISABLED,
                                       command=self._stopReadContinued)
        self.timeSync_btn = ttk.Button(
            top_btnfrm, text="Time Sync", state=tk.DISABLED, command=self._syncTime)

        self.get_btn.grid(column=0, row=0)
        self.get_single_btn.grid(column=1, row=0)
        self.readCont_btn.grid(column=2, row=0)
        self.stopRead_btn.grid(column=3, row=0)
        self.timeSync_btn.grid(column=0, row=1)

        # Create TreeView
        self.tree = ttk.Treeview(self.treeFrame, style="Treeview")
        self.tree['show'] = 'headings'
        self.tree.pack()

        self._initVolatile()

    def _initVolatile(self) -> None:
        """Initialized the volatile value dictionary, to store all values. \n
            This is read from volatile_cfg.txt
        """
        try:
            d = open("volatile_cfg.txt")
        except OSError as e:
            print(e)
            return

        # Read whole content
        lines = d.readlines()
        d.close()

        self.tree["columns"] = (
            "Field", "Address", "Integer", "Converted", "Unit")
        for e in self.tree["columns"]:
            self.tree.column(e, width=(self.contr.getWidth() //
                                       len(self.tree["columns"])) - 10, stretch=tk.NO)
            self.tree.heading(e, text=e)

        for line in lines[1:-1]:  # Skip first and last line
            tmp = line.split(',')
            self.VolatileDict[tmp[0]] = [tmp[1], int(tmp[2]), "Read Pending", int(
                tmp[4]), int(tmp[5]), int(tmp[6]), tmp[7], float(tmp[8]), tmp[9][:-1]]
            self.tree.insert("", 'end', iid=tmp[0], text=tmp[0])

        self._fillTreeView()
        self.init = True

    def _fillTreeView(self) -> None:
        """Fills the tree view with values from the volatile configuration"""
        for e in self.VolatileDict.keys():
            tmp = self.VolatileDict[e]
            # Structure: [fieldname] = (addr, intvalue,converted,width,frac,sign,code,scaling)
            self.tree.item(e, values=(e, tmp[0], tmp[1], "Read pending", tmp[8]))

    def _readValue(self, field: str = None) -> float:
        """Reads a value from the IC

            Keyword Arguments:\n
            field -- The field selected in the treeview

            Returns the data to the specified field as a float"""
        if (not self.init):
            print("Initializing Memory...")
            self._initVolatile()
        try:
            if field == None:
                field = self.tree.item(self.tree.focus())["text"]
            code = self.sBus.readIntValue(self.VolatileDict[field][6])
        except KeyError:
            print("You must select an item first!")
            return

        self.tree.set(field, "Integer", code)
        tmp = self.VolatileDict[field]
        if tmp[5] == 1:
            data = utils.ConvertSignedFixedPoint(
                code, tmp[4], tmp[3])
        elif tmp[5] == 0:
            data = utils.ConvertUnsignedFixedPoint(
                code, tmp[4], tmp[3])
        data = round(data * tmp[7], self.precision)
        # Calculate RMS values
        if (tmp[6] == 'vr' or tmp[6] == 'vs' or tmp[6] == 'vm'):
            data = round(self._calcVRMS(code), self.precision)
        if (tmp[6] == 'ir' or tmp[6] == 'is' or tmp[6] == 'im'):
            data = round(self._calcIRMS(code), self.precision)
        self.tree.set(field, "Converted", data)
        return data

    def _getVolatileValues(self) -> None:
        """Refreshes the tree view"""
        for e in self.VolatileDict.keys():
            self._readValue(e)

    def _readValuesContinued(self, selection: tuple, log: bool = False) -> None:
        """Reads a selection of values from the IC. If logging is activated, 

            Keyword Arguments:

            selection -- the selection from the treeview\n
            log -- if logging is activated, a file called 'continued_reading.txt' is created"""
        if (log):
            f = open("Continued_reading.txt", "w")
        while (self.continuousReading):
            for e in selection:
                data = self._readValue(e)
                if log:
                    f.write(str(data) + "\n")
            time.sleep(self.readDelay * 10 ** (-3))
        if log:
            f.close()

    def _startReadContinued(self) -> None:
        """Starts a thread to continuously read all values"""
        selection = self.tree.selection()
        if not self.continuousReading and len(selection) > 0:
            self.sBus.writeString("x")
            self.sBus.writeString("cc")
            self.continuousReading = True
            _thread.start_new_thread(self._readValuesContinued, (selection,))
            self.sBus.stopReading(False)

    def _stopReadContinued(self) -> None:
        """Stops the thread of reading values continuously"""
        self.sBus.writeString("x")
        self.sBus.writeString("bf")
        self.continuousReading = False
        self.sBus.stopReading(True)
        self.sBus.flushBus()

    def _syncTime(self) -> None:
        """Time Synchronization with Teensy"""
        t = int(time.time())  # Read current time
        final = t + 3600 - time.timezone  # Calculate integer for time
        self.sBus.writeString("tt<" + str(final) + ">")  # Write the time-value

    def _calcVRMS(self, vrmscode: int) -> float:
        """Calculates the VRMS voltage.

        Keyword Arguments:

        vrmscode -- integer code to be converted

        Returns a float value of the input rms voltage"""
        # 172.37mV expected RMS Voltage
        calib_Code = 20454
        exp_RMS = 230.23 * 3000 / 4003000  # voltage Divider
        # Actual Sensitivity/Measured   Calculated: 8.49*10**-6
        conv_factor = (exp_RMS) / calib_Code
        vrms_rsense = vrmscode * conv_factor  # Volt
        # vrms_input = max_volt * vrmscode / pow(2, 15)
        vrms_input = float(vrms_rsense * (4003000 / 3000))
        return vrms_input

    def _calcIRMS(self, irmscode: int) -> float:
        """Calculates the IRMS voltage.

        Keyword Arguments:

        irmscode -- integer code to be converted

        Returns a float value of the input rms current"""
        i_calib_Code = 257
        conv_factor = 0.2601 / i_calib_Code
        irms = float(irmscode * conv_factor)
        return irms
