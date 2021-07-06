import tkinter as tk
from tkinter import ttk
import time, os

from PIL import Image, ImageTk


############################################################################################################################
################                        Main Page Class                                             ########################
############################################################################################################################


class MainPage(tk.Frame):
    #
    #   Init of Page
    #   @param parent
    #           Parent element it is placed within
    #   @param controller
    #           Controller Class, like a backbone
    #
    def __init__(self, parent, controller):
        ttk.Frame.__init__(self, parent)
        self.contr = controller
        self.sBus = controller._getSerialBus()
        #Styles
        self.style = ttk.Style()
        labelstyle = ttk.Style()
        labelstyle.configure("TLabel", font=('Helvetica', 12))
        titlelabelstyle = ttk.Style()
        titlelabelstyle.configure("Titletext.TLabel", font=('Helvetica', 16))
        buttonstyle = ttk.Style()
        buttonstyle.configure("TButton", font=('Helvetica', 12))
        comboboxstyle = ttk.Style()
        comboboxstyle.configure("TCombobox", font=('Helvetica', 12))
        self._guiSetup()

    # Main Window Setup
    def _guiSetup(self):
        mainframe = ttk.Frame(self)
        mainframe.pack()
        acslabel = ttk.Label(
        mainframe, text="ACS71020 Evaluation Adaptor", style="Titletext.TLabel")
        acslabel.grid(column=0, row=0, columnspan=2)
        image = Image.open(os.getcwd()+"/acspic2.jpg")
        photo = ImageTk.PhotoImage(image)
        label2 = tk.Label(mainframe, image=photo)
        label2.image = photo
        label2.grid(column=0, row=1)

    def _getName(self):
        return self.name
