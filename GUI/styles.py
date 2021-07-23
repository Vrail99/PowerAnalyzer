from tkinter import ttk


def initStyles():
    """Configures Styles"""
    bgColor = "#ECECEC"
    baseFont = 'Arial'

    labelstyle = ttk.Style()
    labelstyle.configure("TLabel", font=(
        baseFont, 12), background=bgColor)

    titlelabelstyle = ttk.Style()
    titlelabelstyle.configure("Titletext.TLabel", font=(baseFont, 16))

    buttonstyle = ttk.Style()
    buttonstyle.configure("TButton", font=(baseFont, 12))

    comboboxstyle = ttk.Style()
    comboboxstyle.configure("TCombobox", font=(baseFont, 12))

    framestyle = ttk.Style()
    framestyle.configure("TFrame", borderwidth=0, background=bgColor)

    entrystyle = ttk.Style()
    entrystyle.configure("TEntry", font=(baseFont, 12))

    treestyle = ttk.Style()
    treestyle.configure("Treeview", font=(baseFont, 12))

    radiostyle = ttk.Style()
    radiostyle.configure("TRadiobutton", font=(baseFont, 12))
