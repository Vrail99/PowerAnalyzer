import tkinter as tk
from tkinter import ttk
import utils
import time


############################################################################################################################
################                        Settings Page Class                                             ########################
############################################################################################################################
class SettingsPage(tk.Frame):
    def __init__(self, parent, controller):
        ttk.Frame.__init__(self, parent)
        self.contr = controller
        self.name = "Settings"
        self.writeAdress = tk.StringVar()
        self.sBus = controller.getSerialBus()
        self.EEPROMDict = {}

        self.init = False  # Is the EEPROM initialized?
        self._guiSetup()
        self._initEEPROM()

    # Main Window Setup
    def _guiSetup(self):

        fr1 = ttk.Frame(self)  # Addressframe
        self.adrlab = ttk.Label(fr1, width=20, text="Adress")
        self.adressfield = ttk.Combobox(
            fr1, width=20, textvariable=self.writeAdress)
        self.adressfield['values'] = list(self.EEPROMDict.keys())
        self.initBtn = ttk.Button(
            fr1, text="Initialize", state=tk.DISABLED, width=23, command=self._initEEPROM)

        # grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        self.adrlab.pack(side="left", fill="x", padx=5, expand=True)
        # grid(row=0, column=3, sticky="nsew", padx=5, pady=5)
        self.initBtn.pack(side="right", padx=5)
        # .grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        self.adressfield.pack(side="right", padx=5)

        # Value Settings
        fr2 = ttk.Frame(self)  # Valueframe
        self.vallab = ttk.Label(fr2, width=20, text="Value:")
        self.valuefield = ttk.Entry(fr2, width=22)
        # Readbutton
        self.writeBtn = ttk.Button(
            fr2, text="Write", width=10, state=tk.DISABLED, command=self._writeValue)
        self.readBtn = ttk.Button(
            fr2, text="Read", width=10, state=tk.DISABLED, command=self._readValue)

        self.vallab.pack(side="left", padx=5)
        self.readBtn.pack(side="right", padx=5)
        self.writeBtn.pack(side="right", padx=5)
        self.valuefield.pack(side="right", padx=5)

        fr3 = ttk.Frame(self)  # EEPRROM Value Frame
        self.tree = ttk.Treeview(fr3)
        self.tree.bind('<ButtonRelease-1>', self._treeClickEvent)
        self.tree['show'] = 'headings'
        fr1.pack(side=tk.TOP, fill=tk.X)
        fr2.pack(side=tk.TOP, fill=tk.X)
        fr3.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.tree.pack()

    def _initEEPROM(self):
        if (not self.init):
            try:
                d = open("EEPROM_cfg.txt")
            except:
                print("Error opening file, not writing any value")
                return

            lines = d.readlines()
            d.close()

            # Tree init
            self.tree["columns"] = ("Field", "Address", "Value", "Interval")
            for e in self.tree["columns"]:
                self.tree.column(e, width=(self.contr.getWidth() //
                                           len(self.tree["columns"])) - 3, stretch=tk.NO)
                self.tree.heading(e, text=e)

            #     0       1     2       3     4   5    6
            # Fieldname,Address,Mask,Position,Min,Max,Value

            for line in lines[1:]:
                tmp = line.split(',')
                # Structure: [fieldname] = (addr, mask,pos,min,max,value)
                self.EEPROMDict[tmp[0]] = [tmp[1], tmp[2], int(tmp[3]),
                                           int(tmp[4]), int(tmp[5]), int(tmp[6][:-1])]
            for e in self.EEPROMDict.keys():
                tmp = self.EEPROMDict[e]
                self.tree.insert("", 'end', e, text=e, values=(e, tmp[0], tmp[5],
                                                               "[" + str(tmp[3]) + "," + str(tmp[4]) + "]"))

            self.adressfield["values"] = list(self.EEPROMDict.keys())
            self.adressfield.current(0)
            self.initBtn["text"] = "Get all Values"
            self.init = True
            # Initial read of values only if serial port is connected
            if (self.sBus.deviceOpen()):
                self._getEEPROMValues()
        else:
            self._getEEPROMValues()

    #
    #   Writes a value to an eeprom
    #
    def _writeValue(self):
        try:
            field = self.adressfield.get()
            val = int(self.valuefield.get())
            tpl = self.EEPROMDict[field]
        except:
            print("Value has to be an integer number.")
            return

        if (val < tpl[3] or val > tpl[4]):
            print("Value not in range of [" + str(tpl[3]) +
                  "," + str(tpl[4]) + "], not writing any value")
            return

        adr = tpl[0]
        mask = tpl[1]
        pos = tpl[2]
        self.sBus.writeString("x")
        ret = self.sBus.writeEEPROMValue(adr, val, mask, pos)
        if (ret == 0):  # Writing function could not write a value
            print("Malfunction in EEPROM writing")
            return
        time.sleep(50 / 1000)  # EEPROM-Write waiting time in milliseconds
        self._readValue(field)
        self.sBus.writeString("bf")  # Start Teensy-FFT

    #
    #   Reads an EEPROM-Value and converts it accordingly
    #
    def _readValue(self, field=None):
        if (not self.init):
            print("EEPROM not initialized")
            return
        try:
            if field is None:
                field = self.tree.item(self.tree.focus())["text"]
            adr = self.EEPROMDict[field][0]
            mask = self.EEPROMDict[field][1]
            pos = self.EEPROMDict[field][2]
            data = self.sBus.readEEPROMValue(adr, mask, str(pos))
            #    field, adr, mask, pos, data))
            # See, if a value is in two's complement
            if (self.EEPROMDict[field][3] < 0):
                minval = self.EEPROMDict[field][3]
                data = utils.ConvertTwosComplement(data, len(bin(minval)[3:]))

            self.tree.set(field, "Value", data)

        except KeyError:
            print("You must select an item first!")

    #
    #   Reads all EEPROM-Values
    #
    def _getEEPROMValues(self):
        self.sBus.writeString("x")
        for key, value in self.EEPROMDict.items():
            self._readValue(key)
        self.sBus.writeString("bf")

    # Easy Access helper

    def _treeClickEvent(self, event):
        self.writeAdress.set(self.tree.item(self.tree.focus())["text"])
