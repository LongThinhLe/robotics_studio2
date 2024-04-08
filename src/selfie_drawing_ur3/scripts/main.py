#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
from tkinter.ttk import *


from GUI_lib.GUI_code import SelfieDrawingApp

def main():
    root = tk.Tk()
    SelfieDrawingApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()