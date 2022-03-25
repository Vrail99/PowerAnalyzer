"""MIT License

Copyright (c) 2021 Sebastian Zisch

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import tkinter as tk
from tkinter import ttk
from registerDefs import *
import utils
import time


############################################################################################################################
################                        Settings Page Class                                             ########################
############################################################################################################################
class SettingsPage(ttk.Frame):
    """Settings Page to configure the chip"""

    def __init__(self, parent: ttk.Frame, controller):
        """
        Init of Page 

        Keyword Arguments: \n
        parent -- Parent widget it is placed within \n
        controller -- controller class
        """
        super().__init__(parent, style="TFrame")
        self.contr = controller
        self.name = "Settings"
        self.writeAdress = tk.StringVar()
        self.sBus = controller.getSerialBus()
        self.EEPROMDict = {}

        self.EEPROMDict2 = {"0x0B": reg0x0B(0),
                            "0x0C": reg0x0C(0),
                            "0x0D": reg0x0D(0),
                            "0x0E": reg0x0E(0)
                            }

        self.init = False  # Is the EEPROM initialized?
        self._guiSetup()
        self._initEEPROM_view()

    # Main Window Setup
    def _guiSetup(self) -> None:
        """Sets up UI Elements"""
        buttonFrame = ttk.Frame(self, height=10, style="TFrame")  # Addressframe
        buttonFrame.grid_anchor("center")
        buttonFrame.pack(side=tk.TOP, fill=tk.X)

        self.treeFrame = ttk.Frame(self, style="TFrame")  # EEPRROM Value Frame
        self.treeFrame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.adrlab = ttk.Label(buttonFrame, width=35, style="TLabel", text="Address")
        self.adressfield = ttk.Combobox(
            buttonFrame, style="TCombobox", justify='left', textvariable=self.writeAdress)
        self.adressfield['values'] = list(self.EEPROMDict.keys())
        self.readAllBtn = ttk.Button(buttonFrame, text="Read All", style="TButton",
                                     state=tk.NORMAL, width=10,  command=self._getEEPROMValues)

        self.adrlab.grid(column=0, row=0)  # pack(side=tk.LEFT)
        self.adressfield.grid(column=1, row=0)  # .pack(side=tk.RIGHT)
        self.readAllBtn.grid(column=2, row=0, columnspan=2)

        # Value Settings
        self.vallab = ttk.Label(buttonFrame, width=35, text="Value:")
        self.valuefield = ttk.Entry(buttonFrame, style="TEntry")
        # Readbutton
        self.writeBtn = ttk.Button(
            buttonFrame, text="Write", width=10, style="TButton", state=tk.DISABLED,
            command=self._writeValue)
        self.readBtn = ttk.Button(buttonFrame, text="Read", width=10, style="TButton",
                                  state=tk.DISABLED, command=self._readValue)

        self.vallab.grid(column=0, row=1)  # pack(side=tk.LEFT)
        self.valuefield.grid(column=1, row=1)  # pack(side=tk.LEFT)
        self.readBtn.grid(column=2, row=1)  # pack(side=tk.LEFT)
        self.writeBtn.grid(column=3, row=1)  # pack(side=tk.LEFT)

        self.tree = ttk.Treeview(self.treeFrame, style="Treeview")
        self.tree.bind('<ButtonRelease-1>', self._treeClickEvent)
        self.tree['show'] = 'headings'
        self.tree.pack()

    def _initEEPROM_view(self) -> None:
        """Inits the EEPROM-View, by reading 'EEPROM_cfg.txt' and filling the treeview accordingly"""
        try:
            d = open("EEPROM_cfg.txt")
        except:
            print("Error opening file, cannot initialize")
            return

        lines = d.readlines()
        d.close()

        # Tree init
        self.tree["columns"] = ("Field", "Address", "Value", "Interval")
        for e in self.tree["columns"]:
            self.tree.column(e, width=(self.contr.getWidth() //
                                       len(self.tree["columns"])) - 10, stretch=tk.NO)
            self.tree.heading(e, text=e)

        #     0       1     2       3     4   5    6
        # Fieldname,Address,Mask,Position,Min,Max,Value

        for line in lines[1:]:
            tmp = line.split(',')
            # Structure: [fieldname] = (addr, mask,pos,min,max,value)
            self.EEPROMDict[tmp[0]] = [tmp[1], tmp[2], int(tmp[3]),
                                       int(tmp[4]), int(tmp[5]), int(tmp[6][:-1])]
            self.tree.insert("", 'end', iid=tmp[0], text=tmp[0])

        self.adressfield["values"] = list(self.EEPROMDict.keys())
        self.adressfield.current(0)

        self._fillTreeView()

    def _fillTreeView(self) -> None:
        """Fills the tree view with values from the eeprom configuration"""
        for e in self.EEPROMDict.keys():
            tmp = self.EEPROMDict[e]
            # Structure: [fieldname] = (addr, mask,pos,min,max,value)
            self.tree.item(e, values=(e, tmp[0], "Read pending", '['+str(tmp[3])+','+str(tmp[4])+']'))

    def _treeClickEvent(self, event: tk.Event) -> None:
        """Sets the combobox according to the selected tree

        Keyword Arguments:

        event -- the event which occurred"""
        self.writeAdress.set(self.tree.item(self.tree.focus())["text"])

    def _writeValue(self) -> None:
        """Writes a value to the EEPROM. The value and destination is read from 
        the entry fields. An interval check is done beforehand.

        Uses writeEEPROMValue to write correctly"""
        self.writeBtn["state"] = tk.DISABLED
        try:
            field = self.adressfield.get()
            val = int(self.valuefield.get())
            tpl = self.EEPROMDict[field]
        except KeyError:
            print("Key not in EEPROMDict.")
            return
        except ValueError:
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
        time.sleep(0.5)
        self._readValue(field)  # Returned value from teensy
        self.writeBtn["state"] = tk.NORMAL

    def _readValue(self, field: str = None) -> None:
        """Reads an EEPROM-Value and converts it accordingly.
        If field is left empty, the 
        Keyword Arguments:

        field -- The value to be read, e.g. 'iavgselen'. (default None)
        """
        if (not self.init):
            print("EEPROM not initialized")
            return
        try:
            if field is None:
                field = self.adressfield.get()

            adr = self.EEPROMDict[field][0]
            mask = self.EEPROMDict[field][1]
            pos = self.EEPROMDict[field][2]
            data = self.sBus.readEEPROMValue(adr, mask, str(pos))
            if data is None:
                return
            #    field, adr, mask, pos, data))
            # See, if a value is in two's complement
            if (self.EEPROMDict[field][3] < 0):
                minval = self.EEPROMDict[field][3]
                data = utils.ConvertTwosComplement(data, len(bin(minval)[3:]))

            self.tree.set(field, "Value", data)

        except KeyError:
            print("You must select an item first!")

    def _getEEPROMValues(self) -> None:
        """Inits the EEPROM on the first run and updates all values."""
        if (not self.init):
            self.init = True
            self.writeBtn["state"] = tk.NORMAL

        time.sleep(0.25)
        for key in self.EEPROMDict.keys():
            self._readValue(key)
