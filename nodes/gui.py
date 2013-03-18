#!/usr/bin/env python
import roslib; roslib.load_manifest('kitchencommunication')
import rospy
import time
import os
from std_msgs.msg import String, Int16
# Manifest nicht vergessen!
from std_srvs.srv import Empty
from Tkinter import *
from math import sin
from kitchencommunication.srv import *


class ros_gui:
    pub = rospy.Publisher('gui_log', String)
    pub_startstop = rospy.Publisher('startstop', Int16)

    # Robot running -> status=1, robot stopped -> status=0
    status=1
    # Talk activated -> statusTalk=1, else 0
    statusTalk=1
    # Color light_red
    light_red="#FFAAAA"
    # Trigger variable
    ok_button_pressed = False
    yes_button_pressed = False
    no_button_pressed = False

    def __init__(self):

        top = Tk()

        mainframe = Frame(top)
        mainframe.pack()
        mainframe.master.title("ROS GUI")


        frameTop = Frame(mainframe)
        frameTop.pack(expand=YES, fill=BOTH)
        frameBottom = Frame(top)
        frameBottom.pack(side=BOTTOM, expand=YES, fill=BOTH)

        # Linker Frame
        frameLeft = Frame(frameTop, bd=10)
        frameLeft.pack(side=LEFT, expand=YES, fill=BOTH)

        Label(frameLeft, text="The Robot").pack(side=TOP, fill=X)

        frameRobot = Frame(frameLeft)
        frameRobot.pack(side=TOP, expand=YES, fill=BOTH)

        self.textRobot = Text(frameRobot, width=45, height=15, state=DISABLED)
        self.textRobot.pack(side=LEFT, fill=BOTH)
        scrollVertRobot = Scrollbar(frameRobot, command=self.textRobot.yview)
        scrollVertRobot.pack(side=LEFT, fill=Y)
        self.textRobot.config(yscrollcommand=scrollVertRobot.set,
                              background="light green")


        # Mittlerer Frame
        frameCenter = Frame(frameTop, bd=10)
        frameCenter.pack(side=LEFT, expand=YES, fill=BOTH)

        frameStartStop = Frame(frameCenter)
        frameStartStop.pack(side=TOP, expand=YES, fill=X)

        buttonStop = Button(frameStartStop, text="STOP", fg="red",
                            font=("Helvetica 12 bold"),
                            bd=0, command=self.pushStop)
        buttonStop.bind("<Enter>", lambda e:self.changeStatus(
                "Press STOP to halt all actions"))
        buttonStop.bind("<Leave>", lambda e:self.changeStatus(""))
        buttonStop.pack(side=LEFT, expand=YES, fill=BOTH)

        buttonGo = Button(frameStartStop, text="GO", fg="green",
                          font=("Helvetica 12 bold"),
                          bd=0, command=self.pushGo)
        buttonGo.bind("<Enter>", lambda e:self.changeStatus(
                "Press GO to start the system"))
        buttonGo.bind("<Leave>", lambda e:self.changeStatus(""))
        buttonGo.pack(side=LEFT, expand=YES, fill=BOTH)

        frameTalk = Frame(frameCenter)
        frameTalk.pack(side=TOP)

        self.buttonSpeak = Button(frameTalk, text="TALK!", background=
                                  "light grey", font=('Helvetica', 30))#,
                                  #command=self.pushTalk)
        self.buttonSpeak.bind("<Enter>", lambda e:self.changeStatus(
                    "Blinks when you can communicate with the robot"))
        self.buttonSpeak.bind("<Leave>", lambda e:self.changeStatus(""))
        self.buttonSpeak.pack(side=TOP, expand=YES, fill=BOTH)


        frameYesNo = Frame(frameCenter)
        frameYesNo.pack(side=TOP, expand=YES, fill=X)

        buttonYes = Button(frameYesNo, text="YES", command=self.pushYes)
        #buttonYes.pack(side=LEFT, expand=YES, fill=BOTH)

        buttonNo = Button(frameYesNo, text="NO", command=self.pushNo)
        #buttonNo.pack(side=LEFT, expand=YES, fill=BOTH)


 
        # Rechter Frame
        frameRight = Frame(frameTop, bd=10)
        frameRight.pack(side=LEFT, fill=BOTH)

        labelLog = Label(frameRight, text="Log")
        labelLog.pack(side=TOP, fill=X)

        frameLog = Frame(frameRight)
        frameLog.pack(side=TOP, fill=BOTH)

        self.textLog = Text(frameLog, width=45, height=15, state=DISABLED)
        self.textLog.pack(side=LEFT, fill=BOTH)
        scrollVert = Scrollbar(frameLog, command=self.textLog.yview)
        scrollVert.pack(side=LEFT, fill=Y)
        self.textLog.config(yscrollcommand=scrollVert.set)

        Label(frameRight, text="Command:").pack(side=TOP, fill=X)
        self.entryInput = Entry(frameRight, width=37)
        self.entryInput.bind("<Key>", self.keyPressed)
        self.entryInput.pack(side=TOP)

        frameCommand = Frame(frameRight)
        frameCommand.pack(side=TOP, expand=YES, fill=X)

        buttonOk = Button(frameCommand, text="Ok",
                          command=self.pushOk)
        buttonOk.pack(side=LEFT, expand=YES, fill=X)

        buttonCancel = Button(frameCommand, text="Cancel",
                              command=self.pushCancel)
        buttonCancel.pack(side=LEFT, expand=YES, fill=X)


        # Unterer Frame
        self.labelStatus = Label(frameBottom, text="Robot started.", bd=1,
                            relief=SUNKEN, anchor=W, font=('Helvetica', 8))
        self.labelStatus.pack(side=BOTTOM, fill=X)


        # Schreibe gui_log auf textLog
        rospy.init_node('GUI')
        rospy.Subscriber("gui_log", String, self.receiveLog)
        rospy.Subscriber("robot_response", String, self.receiveResponse)
        rospy.Subscriber("activate_speech", Int16, self.activate_speech)
        rospy.Subscriber("startstop", Int16, self.startstop)

        # Service to receive text input
        srv_rs = rospy.Service('record_text', stt, self.record_text)


        self.buttonSpeak.after(1000, self.update)

        top.mainloop()


    def activate_speech(self, onoff):
        self.statusTalk = onoff.data

    def receiveLog(self, data):
        self.printLog(data.data)

    def receiveResponse(self, data):
        self.textRobot.config(state=NORMAL)
        #self.textRobot.delete(1.0, END)
        self.textRobot.insert(INSERT, data.data+"\n")
        self.textRobot.config(state=DISABLED)

    def printLog(self, txt):
        self.textLog.config(state=NORMAL)
        self.textLog.insert(END, txt+"\n")
        self.textLog.yview(END)
        self.textLog.config(state=DISABLED)

    def changeStatus(self, txt):
        if (txt == ""):
            if (self.status==1):
                txt="Robot is running."
            else:
                txt="Robot stopped"
        self.labelStatus.configure(text=txt)

    def keyPressed(self, event):
        if (event.keysym=="Return"):
            self.pushOk()


    # update status bar
    def update(self):
        if (self.statusTalk == 1 and self.status == 1):
            color = "#%02XFF%02X" % (int(sin(time.clock()*25)*30)+185,
                                    int(sin(time.clock()*25)*30)+175)
        else:
            color = "light grey"
        self.buttonSpeak.configure(background=color)
        self.buttonSpeak.after(10, self.update)


    # Receive text input function
    def record_text(self, x):
        ret = "no"
        while( (not self.ok_button_pressed and not self.yes_button_pressed
                and not self.no_button_pressed) or self.status == 0):
            time.sleep(1)
        if(self.ok_button_pressed):
            self.ok_button_pressed = False
            ret = self.entryInput.get()
            self.pushCancel()
        elif(self.yes_button_pressed):
            self.yes_button_pressed = False
            ret = "yes"
        else:
            self.no_button_pressed = False
        return ret



    def startstop(self, state):
        if(state.data == 1):
            self.status=1
            self.textRobot.configure(background="light green")
        else:
            self.status=0
            self.textRobot.configure(background=self.light_red)


    # Button functions
    def pushGo(self):
        try:
            srv = rospy.ServiceProxy('ki/start_stop', start_stop)
            srv(1)
        except rospy.ServiceException, e:
            print "Exception raised: %s"%e
        self.pub_startstop.publish(1)
        self.printLog(rospy.get_name()+": Start button pressed")
#        self.status=1
#        self.textRobot.configure(background="light green")


    def pushStop(self):
        try:
            srv = rospy.ServiceProxy('ki/start_stop', start_stop)
            srv(0)
        except rospy.ServiceException, e:
            print "Exception raised: %s"%e
        self.pub_startstop.publish(0)
        self.printLog(rospy.get_name()+": Stop button pressed")
#        self.status=0
#        self.textRobot.configure(background=self.light_red)


    def pushYes(self):
        if(self.statusTalk == 1 and self.status == 1):
            self.yes_button_pressed = True

    def pushNo(self):
        if(self.statusTalk == 1 and self.status == 1):
            self.no_button_pressed = True

    def pushOk(self):
        if(self.statusTalk == 1 and self.status == 1):
            self.ok_button_pressed = True

    def pushCancel(self):
        self.entryInput.delete(0, END)



if __name__ == '__main__':
    try:
        g=ros_gui()
    except rospy.ROSInterruptException: pass
