import tkinter as tk
from tkinter import ttk
import _thread
import time

from scopePage import ScopeWindow

import utils


############################################################################################################################
################                        Live Page Class                                             ########################
############################################################################################################################


class LivePage(tk.Frame):
    def __init__(self, parent, controller):
        ttk.Frame.__init__(self, parent)
        self.contr = controller
        self.name = "Values"

        self.ratedVoltage = 230.0  # RMS voltage
        self.init = False
        self.precision = 6
        self.continuousReading = False
        self.readDelay = 500  # ms

        self.VolatileDict = {}
        self._guiSetup()
        self.sBus = controller.getSerialBus()

    def _guiSetup(self):
        # Frames for Buttons and Tree
        buttonframe = ttk.Frame(self)
        buttonframe2 = ttk.Frame(self)
        buttonframe3 = ttk.Frame(self)
        self.treeFrame = ttk.Frame(self)

        buttonframe.pack(side=tk.TOP)
        buttonframe2.pack(side=tk.TOP)
        self.treeFrame.pack(fill=tk.BOTH, expand=True)
        buttonframe3.pack(side=tk.TOP)

        # Fill First Button Frame
        self.get_btn = ttk.Button(buttonframe, text="Get Values", state=tk.DISABLED,
                                  command=self._getVolatileValues)
        self.get_btn.pack(side=tk.LEFT)
        self.get_single_btn = ttk.Button(buttonframe, text="Get Single", state=tk.DISABLED,
                                         command=self._getVolatileValue)
        self.get_single_btn.pack(side=tk.LEFT)
        self.readCont_btn = ttk.Button(buttonframe, text="Read Selected Cont.", state=tk.DISABLED,
                                       command=self._startReadContinued)
        self.readCont_btn.pack(side=tk.LEFT)

        self.stopRead_btn = ttk.Button(buttonframe, text="Stop", state=tk.DISABLED,
                                       command=self._stopReadContinued)
        self.stopRead_btn.pack(side=tk.LEFT)

        # Fill Second Button Frame
        self.timeSync_btn = ttk.Button(
            buttonframe2, text="Time Sync", state=tk.DISABLED, command=self._syncTime)
        self.timeSync_btn.pack()

        # Create TreeView
        self.tree = ttk.Treeview(self.treeFrame)
        self.tree['show'] = 'headings'
        self.tree.pack()

        # # Fill Third Button Frame
        # self.openScope_btn = ttk.Button(
        #     buttonframe3, text="Open Waveform View", state=tk.DISABLED, command=self.openScope)
        # self.openScope_btn.pack(pady=5)

        self._initVolatile()

    def _initVolatile(self):
        try:
            d = open("volatile_cfg.txt")
        except OSError as e:
            print(e)
            return

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
            self.VolatileDict[tmp[0]] = [tmp[1], int(tmp[2]), "Read Pending", int(tmp[4]), int(tmp[5]),
                                         int(tmp[6]), tmp[7], float(tmp[8]), tmp[9][:-1]]

        self._fillTreeView()
        self.init = True

    def _readValue(self, field=None):
        if (not self.init):
            print("Volatile Memory is not initialized")
            return
        try:
            if field is None:
                field = self.tree.item(self.tree.focus())["text"]
            code = self.sBus.readIntValue(self.VolatileDict[field][6])
            self.tree.set(field, "Integer", code)
            tmp = self.VolatileDict[field]
            if tmp[5] == 1:
                data = utils.ConvertSignedFixedPoint(
                    code, tmp[4], tmp[3])
            elif tmp[5] == 0:
                data = utils.ConvertUnsignedFixedPoint(
                    code, tmp[4], tmp[3])
            data = round(data * tmp[7], self.precision)
            if (tmp[6] == 'vr' or tmp[6] == 'vs' or tmp[6] == 'vm'):
                data = round(self._calcVRMS(code), self.precision)
            if (tmp[6] == 'ir' or tmp[6] == 'is' or tmp[6] == 'im'):
                data = round(self._calcIRMS(code), self.precision)
            self.tree.set(field, "Converted", data)
        except KeyError:
            print("You must select an item first!")
        return data

    def _getVolatileValues(self):
        for e in self.VolatileDict.keys():
            self._readValue(e)

    def _getVolatileValue(self):
        self._readValue()

    def _readValuesContinued(self, selection):
        f = open("Continued_reading.txt", "w")
        while (self.continuousReading):
            for e in selection:
                data = self._readValue(e)
                f.write(str(data) + "\n")
            time.sleep(self.readDelay * 10 ** (-3))
        f.close()

    def _stopReadContinued(self):
        self.sBus.writeString("x")
        self.sBus.writeString("bf")
        self.continuousReading = False
        self.sBus.stopReading(True)
        self.sBus.flushBus()

    def _startReadContinued(self):
        self.sBus.writeString("x")
        self.sBus.writeString("cc")
        selection = self.tree.selection()
        if (not self.continuousReading):
            self.continuousReading = True
            _thread.start_new_thread(self._readValuesContinued, (selection,))
            self.sBus.stopReading(False)

    def _fillTreeView(self):
        for e in self.VolatileDict.keys():
            tmp = self.VolatileDict[e]
            # Structure: [fieldname] = (addr, intvalue,converted,width,frac,sign,code,scaling)
            self.tree.insert("", "end", e, text=e, values=(
                e, tmp[0], tmp[1], "Read pending", tmp[8]))

    """Time Synchronization with Teensy"""

    def _syncTime(self):
        t = int(time.time())
        final = t + 3600 - time.timezone
        self.sBus.writeString("tt<" + str(final) + ">")
        print("Zeitdaten geschrieben:", final)

    def _calcVRMS(self, vrmscode):
        # 172.37mV expected RMS Voltage (~243mV)
        calib_Code = 20454  # 21280#20411
        exp_RMS = 230.23 * 3000 / 4003000
        # Actual Sensitivity/Measured   Calculated: 8.49*10**-6
        conv_factor = (exp_RMS) / calib_Code
        vrms_rsense = vrmscode * conv_factor  # Volt
        # vrms_input = max_volt * vrmscode / pow(2, 15)
        vrms_input = vrms_rsense * (4003000 / 3000)
        return vrms_input

    def _calcIRMS(self, irmscode):
        i_calib_Code = 257
        conv_factor = 0.2601 / i_calib_Code
        irms = irmscode * conv_factor
        return irms
