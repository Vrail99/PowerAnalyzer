import tkinter as tk
from tkinter import ttk
import os
from PIL import Image, ImageTk

############################################################################################################################
################                        Main Page Class                                             ########################
############################################################################################################################


class MainPage(ttk.Frame):
    def __init__(self, parent: ttk.Frame, controller) -> None:
        """
        Init of Page 

        Keyword Arguments: \n
        parent -- Parent widget it is placed within \n
        controller -- controller class
        """
        """This is a very long line. It is used to test whether the autopep8 autoformatting works or not"""
        super().__init__(parent, style="TFrame")
        self.contr = controller
        self.sBus = controller.getSerialBus()
        self.name = "Main"

        self._guiSetup()

    def _guiSetup(self) -> None:
        """GUI Setup with picture and titlelabel"""
        acslabel = ttk.Label(
            self, text="ACS71020 Evaluation Adaptor", style="Titletext.TLabel")
        acslabel.pack()
        image = Image.open(os.getcwd()+"/acspic2.jpg")
        photo = ImageTk.PhotoImage(image)
        label2 = ttk.Label(self, image=photo)
        label2.image = photo
        label2.pack()

    def _getName(self):
        return self.name
