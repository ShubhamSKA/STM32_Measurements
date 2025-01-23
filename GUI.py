    # -*- coding: utf-8 -*-

"""

Created on Mon Mar 4 19:47:18 2024

 

@author: Shubham Saluja Kumar Agarwal

"""

 

# ===============================================

# =========== IMPORT CODE START =================

# ===============================================

from PyQt5 import QtGui, QtWidgets, uic

import sys

import os

from PyQt5.QtWidgets import QMessageBox, QFileDialog

import struct 

import numpy as np

from PyQt5.QtCore import QTimer, QFileInfo

from matplotlib.figure import Figure

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import time

import serial  # pyserial

import serial.tools.list_ports

from serial.tools import list_ports

import pandas as pd

from datetime import datetime

 

MODE = 0

'''

comPortName = "COM8"

print(comPortName)

ser = serial.Serial()

ser.port = comPortName

if(ser.isOpen()== False):

   ser.open()

'''

# ===============================================

# ============ IMPORT CODE END ==================

# ===============================================

 

class Ui(QtWidgets.QMainWindow):

    def __init__(self):

        super(Ui, self).__init__()

        uic.loadUi('Measure.ui', self)      # IMPORT UI FILE

 

        # ===============================================

        # ========== initial Setup CODE START ===========

        # ===============================================

 

        # Block entering characters to lineEdit boxes

        self.onlyNumbers = QtGui.QDoubleValidator()

        self.onlyIntegers = QtGui.QIntValidator()

        self.lineEdit_saveDataInterval.setValidator(self.onlyIntegers)      # Save Data Interval set to INTEGER


      

        # Connect functions to BUTTON CLICK events

        # ----- Main Window

        self.radioButton_saveData.toggled.connect(self.radioButton_saveData_toggled)   

        self.radioButton_xSOC.toggled.connect(self.radioButton_xSOC_toggled)   
        self.radioButton_xTime.toggled.connect(self.radioButton_xTime_toggled)   

        # ----- Square Tap

        self.pushButton_startSquare.clicked.connect(self.pushButton_startSquare_clicked)   


        self.pushButton_abortSquare.clicked.connect(self.pushButton_abortSquare_clicked) 

        self.pushButton_callibrate.clicked.connect(self.pushButton_callibrate_clicked)   

        self.pushButton_check.clicked.connect(self.pushButton_check_clicked)   
        # ----- Parameters Tap

        self.pushButton_selectFile.clicked.connect(self.pushButton_selectFile_clicked)   

        self.pushButton_makeFile.clicked.connect(self.pushButton_makeFile_clicked)   

        self.lineEdit_saveDataInterval.textChanged.connect(self.lineEdit_saveDataInterval_textChanged)

      

        # Set Initial Parameter Values

        # ----- Main Window





        # ---- Set default tab index

        self.tabWidget.setCurrentIndex(0)

        # ----- Square Tap

        self.lineEdit_vin.setText("2.0")                

        self.lineEdit_SOC.setText("20.0")               

        self.lineEdit_Current.setText("1.0")             

        self.lineEdit_Capacity.setText("200") 

      
        # ----- Parameters Tap

      

       

        # Set Plot Canvas       

        self.fig1 = Figure()

        self.frame_1.canvas = FigureCanvas(self.fig1)

        self.frame_1.canvas.axes = self.frame_1.canvas.figure.add_subplot(1,1,1) #, projection='3d')

      

        global axes2

        axes2 = self.frame_1.canvas.axes.twinx()     # second y-axis

      

        layout1 = QtWidgets.QVBoxLayout(self.frame_1)

        layout1.addWidget(self.frame_1.canvas)

        self.fig2 = Figure()

        self.frame_2.canvas = FigureCanvas(self.fig2)

        self.frame_2.canvas.axes = self.frame_2.canvas.figure.add_subplot(1,1,1) #, projection='3d')

        layout2 = QtWidgets.QVBoxLayout(self.frame_2)

        layout2.addWidget(self.frame_2.canvas)

      
        # Enable QTimer and connect timeout event to functions

        self.timer5 = QTimer()

        global total_current

        total_current=0

        global prev_time

        prev_time=datetime.now()

        global current_lim

        current_lim = 2

        self.timer5.timeout.connect(self.pushButton_timer5_run) # use for Square
     

        # Set Menu Action Connections


 

        # ===============================================

        # ========= initial Setup CODE END ==============

        # ===============================================

 

        self.show()

 

############################################################

##### USER DEFINED FUNCTIONS / METHODS:    START ###########

############################################################

# Timer5 Run Code - For Square Mode

    def pushButton_timer5_run(self): # run every 50msec

        # originally defined in "global variable initialization"

        global RV2I                           # just use

        global refTimeStr                     # just use

        global avgLength                      # just use

        global soc, current, capacity

        # originally defined in "Square_Clicked"

        global MODE, Freq, Ampl       # just use

        global saveDataInterval, avgLength    # just use

        global countCVMode, countDisplay, countSaveData  # values change in this function

        global ser   # TO WRITE DAC (& Digital Logic)  # just use

        global charge_start        #markers for MCU to start charging/discharging

        global charge_stop         #markers for MCU to stop charging/discharging

        global cycleStartTime      #the start time of each square cycle



        # Defined/assigned in this function

        global avg_index

        global prev_time

        # ---- get elapsed time

        cycleTime = datetime.now() - cycleStartTime

        cycleElapsed = cycleTime.total_seconds()

        timeDiff = datetime.now() - refTimeStr

        elapsedTime = timeDiff.total_seconds()

        currentSOC = soc + (current*elapsedTime/(36*capacity))
        # Read data from MCU

        # ---- Get vP, vN, vRef, and temperature from Timer1 readings
        data_input = ser.read(8)
        ser.reset_input_buffer()
        ser.flush()
        vref = float(self.lineEdit_vin.text())
        data_int = np.frombuffer(data_input,dtype = np.uint32)
        data_int = list(data_int)
        data_int[1] = struct.unpack('f', struct.pack('I', data_int[1]))[0]
        # print(data_int)

        freq = data_int[0]

        ampl = 3.3*(data_int[1])/4096
        direct_ampl = ampl
        adjust_ampl = vref+(ampl-vref)/12
        print("Measured frequency:",freq,", Adjusted Ampl:", adjust_ampl, ", received ampl:", direct_ampl, ', vref:', vref)

        #print(current)
        

        if avg_index == saveDataInterval:

            avg_index = 0

        Freq[avg_index] = freq

        Ampl[avg_index] = ampl

        change_time = datetime.now() - prev_time

        avg_index += 1

        toPlot = Ampl[avgLength:]
        npVal = np.array(toPlot)
        npVal = (npVal[npVal!=0])
        if (len(npVal)!=0):
            mean_ampl = np.mean(npVal)      # average the latest 10 data
        else:
            mean_ampl = 0

        toPlot = Freq[avgLength:]
        npVal = np.array(toPlot)
        npVal = (npVal[npVal!=0])
        if (len(npVal)!=0):
            mean_freq = np.mean(npVal[npVal!=0])      # average the latest 10 data
        else:
            mean_freq = 0

        countDisplay += 1

        if countDisplay == 2:  # update every 4 * 50ms = 200ms (5/sec)

            data = np.array([mean_ampl, mean_freq, currentSOC])

            self.display_Update(data)

            countDisplay = 0

      

       

        # ---- SAVE DATA if checked

        if self.radioButton_saveData.isChecked():

            countSaveData += 1

            if countSaveData == saveDataInterval:

                data = np.array([mean_ampl, mean_freq, currentSOC])

                self.append_Data(data)

                countSaveData = 0

# SHARED FUNCTIONS

    # -------- Display Update funtion

    def display_Update(self,data):

        global countTime

        # Global variable for graphics

        global lines1, lines4, yData1, yData4

        global axes2

        global soc
      

        # ---- get voltage, current, time, temperature

        countTime += 1

        time = (countTime * 200e-3)     # [sec] Elapsed time: change to MIN later

        amplitude = data[0]     # measured amplitude

        frequency = data[1]               # measured frequency

        currentSOC = data[2]

        # ---- Update Text

        self.label_voltage.setText(str("{:.4}".format(amplitude)))

        self.label_temp.setText(str("{:.2}".format(frequency)))

        self.label_current.setText(str("{:.5}".format(currentSOC)))

        self.label_time.setText(str("{:.2e}".format(time)))

      

        # ---- Update Graph

        # --------- Update yData array

        yData1 = np.append(yData1, amplitude)        # voltage

        yData4 = np.append(yData4, frequency)    # frequency        

        # --------- update lines
        
        # print(newxlims)

        # print(soc, currentSOC)
        if (self.radioButton_xSOC.isChecked()):
            newxlims = np.linspace(soc,currentSOC, len(yData1))
            lines1.set_xdata(newxlims)  # voltage
        else:
            newxlims = np.linspace(0,time, len(yData1))
            lines1.set_xdata(newxlims)  # voltage
            self.frame_1.canvas.axes.set_xlim(0,time)


        if (self.radioButton_xSOC.isChecked()):
            if (currentSOC>=soc):
                if (currentSOC-soc < 0.05):
                    self.frame_1.canvas.axes.set_xlim(soc,soc+0.05)
                else:
                    self.frame_1.canvas.axes.set_xlim(soc,currentSOC)
            else:
                if (currentSOC-soc > -0.05):
                    self.frame_1.canvas.axes.set_xlim(soc,soc-0.05)
                else:
                    self.frame_1.canvas.axes.set_xlim(soc,currentSOC)




        lines1.set_ydata(yData1)

        newxlims = np.linspace(soc,currentSOC, len(yData4))

        if (self.radioButton_xSOC.isChecked()):
            newxlims = np.linspace(soc,currentSOC, len(yData1))
            lines4.set_xdata(newxlims)  # freq
        else:
            newxlims = np.linspace(0,time, len(yData1))
            lines4.set_xdata(newxlims)  # freq
            self.frame_2.canvas.axes.set_xlim(0,time)

        if (self.radioButton_xSOC.isChecked()):
            if (currentSOC>=soc):
                if (currentSOC-soc < 0.05):
                    self.frame_2.canvas.axes.set_xlim(soc,soc+0.05)
                else:
                    self.frame_2.canvas.axes.set_xlim(soc,currentSOC)
            else:
                if (currentSOC-soc > -0.05):
                    self.frame_2.canvas.axes.set_xlim(soc,soc-0.05)
                else:
                    self.frame_2.canvas.axes.set_xlim(soc,currentSOC)

        lines4.set_ydata(yData4)

        if min(yData1) < 0:

            yMin = min(yData1)*1.1

        else:

            yMin = min(yData1)*0.9

        if max(yData1) < 0:

            yMax = max(yData1)*0.9

        else:

            yMax = max(yData1)*1.1
        if yMax==yMin:
        
            yMax+=0.1

        if (yData1.size > 0):
            self.frame_1.canvas.axes.set_ylim(yMin,yMax)


        if min(yData4) < 0:

            yMin = min(yData4)*1.1

        else:

            yMin = min(yData4)*0.9

        if max(yData4) < 0:

            yMax = max(yData4)*0.9

        else:

            yMax = max(yData4)*1.1
        
        if yMin==yMax:
            yMax+=0.1
        
        if (yData4.size >0 ):
            self.frame_2.canvas.axes.set_ylim(yMin,yMax)

        # --------- draw canvas

        self.frame_1.canvas.draw()

        self.frame_2.canvas.draw()

    # -------- Append data funtion

    def append_Data(self,data):

        # Read filename (have checked valid filename in .....)

        fileName = self.lineEdit_fileName.text()

        # Prepare Time to be appended

        now = datetime.now()

        # ----- dd/mm/YY H:M:S:F  (last 6 digits are milliseconds)

        nowStr = now.strftime("%d/%m/%Y %H:%M:%S.%f")

        timeS = nowStr[11:]

        # Append to file

        # ------------ Vbattry, Ibattry, Iset, Temperature, Time

        dataSave = [timeS, data[0], data[1], data[2]]

        dataFrame1 = pd.DataFrame([dataSave])

        try:

            dataFrame1.to_csv(fileName, mode = 'a', header = False, index = False)

        except IOError:

            self.label_message.setText("Cannot append data. IOError.")

    # -------- Check and Activate Serial Port funtion

    def check_Activate_Ser(self):

        global ser


        # ---- Check whether the Charging/Discharging Device is connnected

        # ------------ read all serial COM devices (USB devices)

        all_ports = list_ports.comports()

        # ------------- loop through COM devices, checking vendor and product ids

        usbConnected = False

        for a1, _, a3 in all_ports:

            #print(a1)

            #print(a3)

            if a3.startswith("USB VID:PID=0483:5740") == True:
                usbConnected = True
                comPortName = a1
                print("Selected comport:", comPortName)

        # ------------- If not detected, terminate

        if usbConnected == False:

            return "Fail1"   

        # ---- Open Serial Link (if not open yet) & Reset

        try:

            ser = serial.Serial()       

            #ser.port = self.comboBox_COM.currentText()

            ser.port = comPortName

            ser.set_buffer_size(rx_size = 1280000, tx_size = 1280000)

            ser.parity = serial.PARITY_EVEN

            if(not ser.is_open):

                ser.open()

            ser.reset_input_buffer()

        except:

            return "Fail2"

        return "Success"

    # -------- Initialize Global Variables

    def initialize_Global_Variables(self):

        global t   # temporary

        global countCVMode, iStep, countDisplay, countTime, countSaveData, saveDataInterval, avgLength

        global Freq, Ampl

        global refTimeStr

        global avg_index

      

        avg_index = 0

        t = 0.0                   # GLOBAL : temporary

        countTime = 0             # GLOBAL : to monitor elapsed time

        countDisplay = 0          # GLOBAL -- count how often display data

        countSaveData = 0         # GLOBAL -- count how often save data

        saveDataInterval = int(self.lineEdit_saveDataInterval.text())

        avgLength = saveDataInterval*(-1)

        Freq = [0 for x in range(saveDataInterval)] 

        Ampl = [0 for x in range(saveDataInterval)]

        refTimeStr = datetime.now()

    # -------- Initialize Graphics

    def initialize_Graphics(self):

        # Global variable for graphics

        global lines1, lines2, lines3, lines4, yData1, yData2, yData3, yData4, yDataLength

       

        # ------------ create empty arrays and read manual x-span

        yData1 = np.array([])  # amplitude

        yData4 = np.array([])  # frequency

        # yDataLength = int(float(self.lineEdit_xSpan1.text())/200e-3)

        # ------------ clear all axes

        self.frame_1.canvas.axes.clear()

        self.frame_2.canvas.axes.clear()
                             

        # ------------- set axes titles

        self.frame_1.canvas.axes.set_xlabel('SOC')

        self.frame_1.canvas.axes.set_ylabel('Amplitude [V]')

        # axes2.set_ylabel('Current [A]')

        self.frame_2.canvas.axes.set_xlabel('SOC')

        self.frame_2.canvas.axes.set_ylabel('Frequency [Hz]')

        # ------------- set Figure 1 manual axes limits (will be overrided when auto scale = Yes)

        self.frame_1.canvas.axes.set_xlim(0,100)             # use default = automatic

        # ------------- set Figure 2 manual axes limits (will be overrided when auto scale = Yes)

        self.frame_2.canvas.axes.set_xlim(0,100)              # use default = automatic

        # ------------- Generate lines object (GLOBAL) & draw canvas

        lines1 = self.frame_1.canvas.axes.plot([],[])[0]


        self.frame_1.canvas.draw()

        lines4 = self.frame_2.canvas.axes.plot([],[])[0]

        self.frame_2.canvas.draw()


# Square Charging (discharchig) BUTTON CLICK event functions

    def pushButton_startSquare_clicked(self):

        global MODE

        global charge_start

        global charge_stop

        global cycleStartTime

        global soc, current, capacity, v_init

      

        # # ---- Check and activate serial link

        checkSerial = self.check_Activate_Ser()

        if checkSerial == "Fail1":

             self.label_message.setText("Warning: Microcontroller not connected.")

             QtWidgets.QMessageBox.information(self.pushButton_startSquare, "Warning", "Device not detected")

             return

        elif checkSerial == "Fail2":

             self.label_message.setText("ERROR: Device detected but cannot open serial link.")

             QtWidgets.QMessageBox.information(self.pushButton_startSquare, "Warning", "Cannot open serial link")

             return

      

        # Read Current, Voltage, and Time Set Data Values


        v_init = float(self.lineEdit_vin.text())         # GLOBAL [V initial]

        soc = float(self.lineEdit_SOC.text())             # GLOBAL [%]

        current = float(self.lineEdit_Current.text())             # GLOBAL [A]

        capacity = float(self.lineEdit_Capacity.text())           # GLOBAL [Ah]

        cycleStartTime = datetime.now()       # to start the cycle when start button is clicked.

      

        if self.radioButton_isCharging.isChecked():

            if current < 0: 
                current*=-1

        else:
            if current > 0:
                current*=-1

      

        # Check input errors

        # ---- Check |i1| <= 7A OR |i2| <= 7A

        if (v_init > 3.3) or (v_init < 0):

            self.label_message.setText("ERROR: V-init is too big or too small.")

            QtWidgets.QMessageBox.information(self.pushButton_startCharge, "Warning", "V-init is too big or too small")

            return

        if soc < 0 or soc>100:

            self.label_message.setText("ERROR: SOC is too big or too small.")

            QtWidgets.QMessageBox.information(self.pushButton_startCharge, "Warning", "SOC is too big or too small")

            return
      
        if capacity < 0:

            self.label_message.setText("ERROR: Capacity is too small.")

            QtWidgets.QMessageBox.information(self.pushButton_startCharge, "Warning", "Capacity is too small")

            return

        # Disable other Taps

        for i in range(0,2):

            if i != 0: self.tabWidget.setTabEnabled(i,False)

      

        # Initialize global variables

        MODE = 3        # GLOBAL Square Mode

        self.initialize_Global_Variables()

        v_send = int((v_init/3.3)*4096)

        # send = [v_send]

        # send_int = np.uint8(send)

        send_int = v_send.to_bytes(2, byteorder='little')

        # for _ in range(5):
        # print("sent:",ser.write(send_int),"bytes")
        ser.write(send_int)
            # time.sleep(0.1)

        time.sleep(3)

        # Initialize graphics

        self.initialize_Graphics()

      

        # Start Timers & Display message

        self.label_message.setText("Measuring has started.")

        self.timer5.start(50)      # TIMER2 INTERVAL [ms] 

        # FOR ACTUAL IMPLEMENTATION

        # ---- send i1Set, i2Set, td, tr, ton, tf, tp, tstop, MODE to FW

        # ---- make the FW perform the timer1 and timer5 functions

    def pushButton_abortSquare_clicked(self):

        global MODE

        global charge_stop

        global charge_start

      

        # initialise stop variable, marker sent to MCU to stop charging/discharging


        if MODE == 3:    # process not completed yet --> abort

            # ---- Stop timers

            self.timer5.stop()

            self.label_message.setText("Measuring has stopped.")

            # ---- ENable all Taps

            for i in range(0,2): self.tabWidget.setTabEnabled(i,True)

        elif MODE == 0:

            self.label_message.setText("Cannot abort. Measuring has already been aborted, or not started.")

        MODE = 0  # waiting mode

    def pushButton_callibrate_clicked(self):
        if MODE == 0:
            global ser

            serial_activate = self.check_Activate_Ser()

            if serial_activate == "Fail1":

                self.label_message.setText("Warning: Microcontroller not connected.")

                QtWidgets.QMessageBox.information(self.pushButton_startSquare, "Warning", "Device not detected")

                return

            elif serial_activate == "Fail2":

                self.label_message.setText("ERROR: Device detected but cannot open serial link.")

                QtWidgets.QMessageBox.information(self.pushButton_startSquare, "Warning", "Cannot open serial link")

                return
            
            value = 4100

            value = value.to_bytes(2, byteorder='little')

            ser.write(value)
            values = [0,0]
            while values[0]==0:
                current_adc = ser.read(8)
                ser.reset_input_buffer()
                ser.flush()
                values = np.frombuffer(current_adc,dtype = np.uint32)
                values2 = list(values)
                values2[1] = struct.unpack('f', struct.pack('I', values2[1]))[0]
                values2[0] = values2[0]/4096*3.3
                values2[1] = values2[1]/4096*3.3
            self.label_out_voltage.setText("Output Voltage: "+ str(round(values2[1],3)))
            self.lineEdit_vin.setText(str(values2[0]))
            ser.close()

    def pushButton_check_clicked(self):
        if MODE==0:
            global ser

            serial_activate = self.check_Activate_Ser()

            if serial_activate == "Fail1":

                self.label_message.setText("Warning: Microcontroller not connected.")

                QtWidgets.QMessageBox.information(self.pushButton_startSquare, "Warning", "Device not detected")

                return

            elif serial_activate == "Fail2":

                self.label_message.setText("ERROR: Device detected but cannot open serial link.")

                QtWidgets.QMessageBox.information(self.pushButton_startSquare, "Warning", "Cannot open serial link")

                return
            voltage = int(((float(self.lineEdit_vin.text()))/3.3)*4095)

            if voltage > 4095:
                self.label_message.setText("ERROR: Voltage too high.")
                ser.close()
                return

            value = 4200 + voltage

            value = value.to_bytes(2, byteorder='little')

            ser.write(value)

            current_adc = ser.read(8)
            ser.reset_input_buffer()
            ser.flush()
            values = np.frombuffer(current_adc,dtype = np.uint32)
            values2 = list(values)
            values2[1] = struct.unpack('f', struct.pack('I', values2[1]))[0]
            values2[0] = values2[0]/4096*3.3
            values2[1] = values2[1]/4096*3.3
            self.label_out_voltage.setText("Output Voltage: "+ str(round(values2[1],3)))
            ser.close()


# Parameters

    def pushButton_selectFile_clicked(self):

        oldFileName = self.lineEdit_fileName.text()

        # Open file selection dialog window and get filename

        filename,_ = QFileDialog.getOpenFileName(None,"Open","","CSV Files (*.csv);;All Files (*)")

        if filename == "":   # "Cancel" button clicked

            self.lineEdit_fileName.setText(oldFileName)

            message = "File selection canceled."

        else:                # "Open" button is clicked (Update filename but do not open file)

            self.lineEdit_fileName.setText(filename)

            message = "A file selected. Data will be saved to the selected file."

        # Update status message

        self.label_message.setText(message)


    def pushButton_makeFile_clicked(self):

        # Read text in the filename textbox       

        filename = self.lineEdit_fileName.text()

        # First check if file selected

            # if filename == "Filename":

        if filename.startswith("Filename not"):

            message = "Please put filename first."   # Status message to ask for selecting file

            QtWidgets.QMessageBox.information(self.pushButton_makeFile, "Warning", "File name error")

        elif (filename[-3:] != "csv") and (filename[-3:] != "CSV") :

            message = "Please use CSV file extension."   # Status message to ask for selecting file

            QtWidgets.QMessageBox.information(self.pushButton_makeFile, "Warning", "File extension error")

            # Check if file exist

        elif QFileInfo(filename).exists():          # File exist, a warning message show up

            QtWidgets.QMessageBox.information(self.pushButton_makeFile, "Warning", "File already exist")

            message = "File already exists. Make file process canceled."

        else:

            dialog = QMessageBox(self.pushButton_makeFile)   # Message to confirm to create a new file

            dialog.setText("File will be created.")

            dialog.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)      # Set "Ok" and "cancel" button

            if dialog.exec() == QMessageBox.Ok:         # if "Ok" button pressed, new file created

                new_file = open(filename, 'w')

                message = "The selected file was successfully created."

            else:               # Otherwise, process cancel

                message = "Make file process canceled."

        # Update status message

        self.label_message.setText(message)


    def lineEdit_saveDataInterval_textChanged(self):
        if (self.lineEdit_saveDataInterval.text() != ''):
            interval = int(self.lineEdit_saveDataInterval.text())
        else:
            interval = 10

        self.label_52.setText("x 1s = " + str(interval) + " s")

      

# Main Window

# ---- X Span number changed Event Function

    def xSpan1_changed(self):

        self.lineEdit_xSpan2.setText(self.lineEdit_xSpan1.text())

        dataPoints = int(float(self.lineEdit_xSpan1.text())/200e-3)

        dataPointUpdate = "[sec] " + str(dataPoints) + " data points"

        self.label_27.setText(dataPointUpdate)

        self.label_45.setText(dataPointUpdate) 

# ---- Save Data Radiobutton Toggled Event Function

    def radioButton_saveData_toggled(self):

        if self.radioButton_saveData.isChecked():

            # ------------ check "filename valid?"

            filename = self.lineEdit_fileName.text()

            if filename.startswith("Filename not"):

                message = "Please put filename first."   # Status message to ask for selecting file

                QtWidgets.QMessageBox.information(self.pushButton_makeFile, "Warning", "File name error")

                self.radioButton_saveData.setChecked(False)

            elif (filename[-3:] != "csv") and (filename[-3:] != "CSV") :

                message = "Please use CSV file extension."   # Status message to ask for selecting file

                QtWidgets.QMessageBox.information(self.pushButton_makeFile, "Warning", "File extension error")

                self.radioButton_saveData.setChecked(False)

            elif QFileInfo(filename).exists():          # File exist, a warning message show up

                if os.stat(filename).st_size != 0:

                    QtWidgets.QMessageBox.information(self.pushButton_makeFile, "Warning", "File not empty. Data will be appended")

                    message = "The selected CSV file is not empty. Data will be appended to the selected File."

                else:

                    message = "Data will be saved to the selected file."

            self.label_message.setText(message)   

    def radioButton_xSOC_toggled(self):
        if (self.radioButton_xSOC.isChecked() and self.radioButton_xTime.isChecked()):
            self.radioButton_xTime.setChecked(False)
        elif (not self.radioButton_xSOC.isChecked() and not self.radioButton_xTime.isChecked()):
            self.radioButton_xSOC.setChecked(True)


    def radioButton_xTime_toggled(self):
        if (self.radioButton_xSOC.isChecked() and self.radioButton_xTime.isChecked()):
            self.radioButton_xSOC.setChecked(False)
        elif (not self.radioButton_xSOC.isChecked() and not self.radioButton_xTime.isChecked()):
            self.radioButton_xTime.setChecked(True)


# -------- Exit by clicking Windows X button

    def closeEvent(self, event):   # close by clicking "X" button

        global ser

        self.timer5.stop()

        try: ser
        except: ser = None
        if ser is not None:

            if ser.is_open:

                ser.close()

        event.accept()

############################################################

##### USER DEFINED FUNCTIONS / METHODS:    END #############

############################################################

 

app = QtWidgets.QApplication(sys.argv)

window = Ui()

app.exec_()
