from Tkinter import *
# -*- coding: utf8 -*-
# vim:fileencoding=utf8

class App:
    def __init__(self,master):
        frame=Frame(master)
        frame.pack()
        self.qbutt=Button(frame,text="X",command=frame.quit)
        self.qbutt.pack(side=LEFT)
        self.somebutt=Button(frame,text="butt",command=self.saysomething)
        self.somebutt.pack(side=LEFT)
    def saysomething(self):
        print("puncifánk")


root = Tk()
app = App(root)

root.mainloop()
root.destroy()
