import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque
import time
import serial.tools.list_ports
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
import os
import datetime
import smtplib
from email.message import EmailMessage

#import atexit


##from multiprocessing import Queue, Process
from multiprocess import Queue, Process

##from Queue import Empty

emptyODRatioOD=1
emptyODRatioLevel=1
maxLen=100

class SerialCom:
    def __init__(self, data_queue, msg_queue):
        self.data_q = data_queue
        self.msg_q = msg_queue

    ##def setup_serial(self):
        #ports = list(serial.tools.list_ports.comports()) # Detect arduino serial por

        #p=[port for port in ports if port[2] != 'n/a']
        #SERIAL=p[0][0]
        #SERIAL = ports[0].device####################3
    #SERIAL='/dev/ttyACM0' ###########################################
    #SERIAL='/dev/tty.usbmodem14201' ###########################################

        #self.serial_port = SERIAL
        #print('Reading from serial port %s...' % SERIAL)


        #time.sleep(2) #wait for initialization

    def arduino(self):
        ##self.setup_serial()
        while True:
            if ser.inWaiting():
                line = ser.readline().decode('utf-8').strip()
                print(line)
                s = line.split()
                if int(float(s[0])) == -1:
                    # message
                    self.msg_q.put((int(s[1]), ' '.join(s[2:])))
                else:
                    data = [float(val) for val in s]
                    self.data_q.put(data)

# plot class
class AnalogPlot:
    # constr
    def __init__(self, data_queue, msg_queue, fig):
        # email set up
        self._destinations = ["rdfliu@uchicago.edu"]
        self._machine_name = "Flu2"
    
        # open serial port
        self.NumberOfData=29
        self.fig = fig
        self.data_q = data_queue
        self.msg_q = msg_queue

        self.databuf=[deque([0.0]*maxLen) for i in range(self.NumberOfData)]

        self.t0 = time.time()			
        self.t = deque([0.0]*maxLen)
        
        self.SensitivitySetTo0 = deque([0.0]*maxLen)
        self.SenV0 = deque([0.0]*maxLen)
        self.ExcitationSetTo0 = deque([0.0]*maxLen)
        self.ExcitationOutput0 = deque([0.0]*maxLen)
        self.ExcitationRFRead0 = deque([0.0]*maxLen)
        self.Target_fluoro0 = deque([0.0]*maxLen)
        self.PMTRead0 = deque([0.0]*maxLen)

        self.SensitivitySetTo1 = deque([0.0]*maxLen)
        self.SenV1 = deque([0.0]*maxLen)
        self.ExcitationSetTo1 = deque([0.0]*maxLen)
        self.ExcitationOutput1 = deque([0.0]*maxLen)
        self.ExcitationRFRead1 = deque([0.0]*maxLen)
        self.Target_fluoro1 = deque([0.0]*maxLen)
        self.PMTRead1 = deque([0.0]*maxLen)

        self.target_OD = deque([0.0]*maxLen)
        self.ODRead = deque([0.0]*maxLen)
        self.temperature_SetTo = deque([0.0]*maxLen)
        self.thermocouple_celcius = deque([0.0]*maxLen)
        self.ODLEDStatus = deque([0.0]*maxLen)
        self.StirStatus = deque([0.0]*maxLen)
        self.HeaterStatus = deque([0.0]*maxLen)
        self.PumpInStatus = deque([0.0]*maxLen)
        self.PumpOutStatus = deque([0.0]*maxLen)
        self.ODLEDrf = deque([0.0]*maxLen)
        self.LevelLEDStatus = deque([0.0]*maxLen)
        self.LiquidLevel = deque([0.0]*maxLen)
        self.LevelODLEDrf = deque([0.0]*maxLen)
        self.ODRatio = deque([0.0]*maxLen)
        self.LevelRatio = deque([0.0]*maxLen)



        self.timestr = time.strftime("%YY%mM%dD_%HH%MM%S")
        fileDir = os.path.join(os.path.dirname(os.path.realpath('__file__')),os.pardir)
        if not os.path.exists(fileDir + '/Experiments/' + self.timestr + '/raw_data'):
            os.makedirs(fileDir + '/Experiments/' + self.timestr + '/raw_data')
        self.record = open(fileDir + '/Experiments/' + self.timestr + '/raw_data/' + self.timestr + '.txt', 'w')
        self.record.write('time'+"\t" +'TempSt'+"\t" +'TCouple'+"\t" 
        +'Sens0St'+"\t"+'Sens0Op'+"\t" +'Exci0St'+"\t" +'Exci0Op'+"\t" +'Exci0Rf'+"\t"+'Target0'+"\t" +'PMT0rds'+"\t"
        +'Sens1St'+"\t"+'Sens1Op'+"\t" +'Exci1St'+"\t" +'Exci1Op'+"\t" +'Exci1Rf'+"\t"+'Target1'+"\t" +'PMT1rds'+"\t"
        +'ODSET'+"\t"+'ODrds'+"\t"+'ODLED'+"\t"+'Stir'+"\t"+'Heater'+"\t"+'PumpIn'+"\t"
        +'PumpOut'+"\t"+'ODLEDrf'+"\t"+'LevelLED'+"\t"+'Level'+"\t"+'LevelLEDrf'+"\t"
        +'ODratio'+"\t"+'Levelratio'+"\n")
        self.record.flush()


        grid = plt.GridSpec(19, 1, wspace=0.05, hspace=0.06,bottom=0.08,top=0.95,left= 0.1,right=0.9)
        self.ax1 = self.fig.add_subplot(grid[:3, 0])#OD ratio
        self.ax2 = self.fig.add_subplot(grid[3, 0])# OD set, OD Read, OD ref
        self.ax3 = self.fig.add_subplot(grid[4:7, 0])# PMT0read
        self.ax4 = self.fig.add_subplot(grid[7, 0])#PMT0SenseSet,PMT0SensePin
        self.ax5 = self.fig.add_subplot(grid[8, 0])#Excite0set,Excite0Pin Excite0Ref
        self.ax6 = self.fig.add_subplot(grid[9:12, 0])# PMT1read
        self.ax7 = self.fig.add_subplot(grid[12, 0])#PMT1SenseSet,PMT1SensePin
        self.ax8 = self.fig.add_subplot(grid[13, 0])#Excite1set,Excite1Pin Excite1Ref
        self.ax9 = self.fig.add_subplot(grid[14:16, 0])# TempSet, TempRead
        self.ax10 = self.fig.add_subplot(grid[16, 0])#LevelRatio
        self.ax11 = self.fig.add_subplot(grid[17, 0])#Level, Level LEDrf
        self.ax12 = self.fig.add_subplot(grid[18, 0])#Heater,Stir,Pumpin,Pumpout


        self.ax1.set_ylim(0, 1)
        self.ax1.set_xlim(0, 100)
        self.ax2.set_ylim(0, 1100)
        self.ax2.set_xlim(0, 100)
        self.ax2.set_yticks([500,1000])

        self.ax3.set_ylim(0, 1100)
        self.ax3.set_xlim(0, 100)
        self.ax4.set_xlim(0, 100)
        self.ax4.set_ylim(0, 1.5)
        self.ax4.set_yticks([0.5,1.1])
        self.ax4.set_ylabel("0.5-1.1"+"\n"+"Volt", color="black")
        self.ax5.set_xlim(0, 100)
        self.ax5.set_ylim(0, 1100)
        self.ax5.set_yticks([500,1000])

        self.ax6.set_ylim(0, 1100)
        self.ax6.set_xlim(0, 100)
        self.ax7.set_xlim(0, 100)
        self.ax7.set_ylim(0, 1.5)
        self.ax7.set_yticks([0.5,1.1])
        self.ax7.set_ylabel("0.5-1.1"+"\n"+"Volt", color="black")
        self.ax8.set_xlim(0, 100)
        self.ax8.set_ylim(0, 1100)
        self.ax8.set_yticks([500,1000])


        self.ax9.set_ylim(20, 46)
        self.ax9.set_yticks(range(31,45, 3))
        self.ax9.set_xlim(0, 100)
        self.ax9.set_ylabel("Degree", color="r")
        self.ax9.tick_params(axis="y", labelcolor="r")
        self.ax10.set_ylim(0, 1)
        self.ax10.set_xlim(0, 100)
        self.ax11.set_xlim(0, 100)
        self.ax11.set_ylim(0, 1100)
        self.ax11.set_yticks([500,1000])
        self.ax12.set_xlim(0, 100)
        self.ax12.set_ylim(0, 1.9)
        self.ax12.set_yticklabels([])
        self.ax12.set_ylabel("ON/OFF", color="black")


        self.ODRatio, = self.ax1.plot([], [],'orange', label='ODRatio')
        self.ax1.legend(loc='upper right',ncol=1,fontsize = 'xx-small', frameon=False)  

        self.target_OD, = self.ax2.plot([], [],'darkred', label='ODSET')
        self.ODRead, = self.ax2.plot([], [],'orange', label='ODRead')
        self.ODLEDrf, = self.ax2.plot([], [],'black', label='ODLEDrf')
        self.ax2.legend(loc='upper right',ncol=3,fontsize = 'xx-small',frameon=False)  
        
        self.Target_fluoro0, = self.ax3.plot([], [],'greenyellow', label='Fluoro0SET')
        self.PMTRead0, = self.ax3.plot([], [],'green', label='PMT0read')


        self.ax3.legend(loc='upper right',fontsize = 'xx-small',frameon=False)  
        self.SensitivitySetTo0, = self.ax4.plot([], [],'darkgreen', label='PMT0SenseSet')
        self.SenV0, = self.ax4.plot([], [],'greenyellow', label='PMT0SensePin')
        self.ax4.legend(loc='upper right',ncol=2,fontsize = 'xx-small',frameon=False)
        self.ExcitationSetTo0, = self.ax5.plot([], [],'darkblue', label='Excite0Set')
        self.ExcitationOutput0, = self.ax5.plot([], [],'cyan', label='Excite0Pin')
        self.ExcitationRFRead0, = self.ax5.plot([], [],'blue', label='Excite0rf')
        self.ax5.legend(loc='upper right',ncol=3,fontsize = 'xx-small',frameon=False)  
        
        self.Target_fluoro1, = self.ax6.plot([], [],'pink', label='Fluoro1SET')
        self.PMTRead1, = self.ax6.plot([], [],'red', label='PMT1read')
        self.ax6.legend(loc='upper right',fontsize = 'xx-small',frameon=False)  
        self.SensitivitySetTo1, = self.ax7.plot([], [],'darkred', label='PMT1SenseSet')
        self.SenV1, = self.ax7.plot([], [],'pink', label='PMT1SensePin')
        self.ax7.legend(loc='upper right',ncol=2,fontsize = 'xx-small',frameon=False)
        self.ExcitationSetTo1, = self.ax8.plot([], [],'orange', label='Excite1Set')
        self.ExcitationOutput1, = self.ax8.plot([], [],'brown', label='Excite1Pin')
        self.ExcitationRFRead1, = self.ax8.plot([], [],'yellow', label='Excite1rf')
        self.ax8.legend(loc='upper right',ncol=3,fontsize = 'xx-small',frameon=False)  


        self.temperature_SetTo, = self.ax9.plot([], [],'darkred', label='TempSet')
        self.thermocouple_celcius, = self.ax9.plot([], [],'red', label='TempRead')
        self.ax9.legend(loc='upper right',ncol=2,fontsize = 'xx-small',frameon=False)  

        self.LevelRatio, = self.ax10.plot([], [],'blue', label='LevelRatio')
        self.ax10.legend(loc='upper right',ncol=1,fontsize = 'xx-small',frameon=False)  

        self.LiquidLevel, = self.ax11.plot([], [],'blue', label='Level')
        self.LevelODLEDrf, = self.ax11.plot([], [],'black', label='LevelLEDrf')
        self.ax11.legend(loc='upper right',ncol=2,fontsize = 'xx-small',frameon=False)  

        self.ODLEDStatus, = self.ax12.plot([], [],'orange', label='ODLED')
        self.LevelLEDStatus, = self.ax12.plot([], [],'darkred', label='LevelLED')
        self.HeaterStatus, = self.ax12.plot([], [],'red', label='Heater')
        self.StirStatus, = self.ax12.plot([], [],'greenyellow', label='Stir')
        self.PumpInStatus, = self.ax12.plot([], [],'blue', label='PumpIn')
        self.PumpOutStatus, = self.ax12.plot([], [],'darkblue', label='PumpOut')
        self.ax12.legend(loc='upper right',ncol=6,fontsize = 'xx-small',frameon=False)  


        self.ax1.set_xticklabels([])
        self.ax2.set_xticklabels([])
        self.ax3.set_xticklabels([])
        self.ax4.set_xticklabels([])
        self.ax5.set_xticklabels([])
        self.ax6.set_xticklabels([])
        self.ax7.set_xticklabels([])
        self.ax8.set_xticklabels([])
        self.ax9.set_xticklabels([])
        self.ax10.set_xticklabels([])
        self.ax11.set_xticklabels([])
        self.ax12.set_xticklabels([])


  # add to buffer
    def addToBuf(self, buf, val):
        if len(buf) < maxLen:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)

    # add data
    def add(self, data):
        assert(len(data) == self.NumberOfData-2)
        t=time.time() - self.t0
        self.addToBuf(self.t, t)

        for i in range (self.NumberOfData-2):
            self.addToBuf(self.databuf[i], data[i])

        if data[23] > 0 :
            data27=float(data[15]/data[23])
            data27=data27/emptyODRatioOD
        else: 
            data27=0
        
        self.addToBuf(self.databuf[27], data27)


        if data[26] > 0 :
            data28=float(data[25]/data[26])
            data28=data28/emptyODRatioLevel
        else: 
            data28=0

        self.addToBuf(self.databuf[28], data28)


        self.write_to_file(t ,data[0], data[1], 
                        data[2], data[3], data[4], 
                        data[5], data[6], data[7], 
                        data[8], data[9], data[10], 
                        data[11], data[12], data[13], 
                        data[14], data[15], data[16], 
                        data[17], data[18], data[19], 
                        data[20], data[21], data[22], 
                        data[23], data[24], data[25], data[26], 
                        data27, data28)
        
        # TODO: how this program works right now is that
        # a subprocess continues to listen to the serial port for new data
        # which are pushed into the data queue/message queue
        # every time the plot is updated by update() (every 80 millisec currently)
        # it invokes the add() function which writes new data into a file
        # but ideally new data and new messages should be handled as soon as possible
        # which triggers the downstream plotting and emailing functions
        
    
    def flush_data_q(self):
        while not self.data_q.empty():
            ##self.data_q.get()
            data2=self.data_q.get(False)

    # update plotaddToBuf
    def update(self, frameNum):
        
        while not self.msg_q.empty():
            err_flag, msg = msg_q.get()
            send_email(self._destinations, self._machine_name, err_flag, msg)

        while not self.data_q.empty():
            data = self.data_q.get(False)          
            if(len(data) == self.NumberOfData-2):

                self.add(data)
                
                self.SensitivitySetTo0.set_data(range(maxLen), self.databuf[0])
                self.SenV0.set_data(range(maxLen), self.databuf[1])
                self.ExcitationSetTo0.set_data(range(maxLen), self.databuf[2])
                self.ExcitationOutput0.set_data(range(maxLen), self.databuf[3])
                self.ExcitationRFRead0.set_data(range(maxLen), self.databuf[4])
                self.Target_fluoro0.set_data(range(maxLen), self.databuf[5])
                self.PMTRead0.set_data(range(maxLen), self.databuf[6])
                
                self.SensitivitySetTo1.set_data(range(maxLen), self.databuf[7])
                self.SenV1.set_data(range(maxLen), self.databuf[8])
                self.ExcitationSetTo1.set_data(range(maxLen), self.databuf[9])
                self.ExcitationOutput1.set_data(range(maxLen), self.databuf[10])
                self.ExcitationRFRead1.set_data(range(maxLen), self.databuf[11])
                self.Target_fluoro1.set_data(range(maxLen), self.databuf[12])
                self.PMTRead1.set_data(range(maxLen), self.databuf[13])
                
                self.target_OD.set_data(range(maxLen), self.databuf[14])
                self.ODRead.set_data(range(maxLen), self.databuf[15])
                self.temperature_SetTo.set_data(range(maxLen), self.databuf[16])
                self.thermocouple_celcius.set_data(range(maxLen), self.databuf[17])
                self.ODLEDStatus.set_data(range(maxLen), self.databuf[18])
                self.StirStatus.set_data(range(maxLen), self.databuf[19])
                self.HeaterStatus.set_data(range(maxLen), self.databuf[20])
                self.PumpInStatus.set_data(range(maxLen), self.databuf[21])
                self.PumpOutStatus.set_data(range(maxLen), self.databuf[22])
                self.ODLEDrf.set_data(range(maxLen), self.databuf[23])
                self.LevelLEDStatus.set_data(range(maxLen), self.databuf[24])
                self.LiquidLevel.set_data(range(maxLen), self.databuf[25])
                self.LevelODLEDrf.set_data(range(maxLen), self.databuf[26])
                self.ODRatio.set_data(range(maxLen), self.databuf[27])
                self.LevelRatio.set_data(range(maxLen), self.databuf[28])


        #print (".")
        data=None

    def write_to_file(self, t, 
                        Sense0SetTo, Sense0Output, Excite0SetTo, Excite0Output, Excite0Ref, PMT0Target, PMT0reads, 
                        Sense1SetTo, Sense1Output, Excite1SetTo, Excite1Output, Excite1Ref, PMT1Target,  PMT1reads,
                        ODSET, ODreads,
                        TSetTo, TCouple,
                        ODLED, Stir, Heater, PumpIn, PumpOut, ODLEDrf, LevelLED, Level, LevelLEDrf, ODratio, Levelratio):
            
        self.record.write(str(t)+"\t"
        +str(TSetTo)+"\t"+str(TCouple)+"\t"
        +str(Sense0SetTo)+"\t"+str(Sense0Output)+"\t"
        +str(Excite0SetTo)+"\t"+str(Excite0Output)+"\t"+str(Excite0Ref)+"\t"+str(PMT0Target)+"\t"+str(PMT0reads)+"\t"
        +str(Sense1SetTo)+"\t"+str(Sense1Output)+"\t"
        +str(Excite1SetTo)+"\t"+str(Excite1Output)+"\t"+str(Excite1Ref)+"\t"+str(PMT1Target)+"\t"+str(PMT1reads)+"\t"

        +str(ODSET)+"\t"+str(ODreads)+"\t"
        +str(ODLED)+"\t"+str(Stir)+"\t"+str(Heater)+"\t"
        +str(PumpIn)+"\t"+str(PumpOut)+"\t"
        +str(ODLEDrf)+"\t"
        +str(LevelLED)+"\t"+str(Level)+"\t"+str(LevelLEDrf)+"\t"+str(ODratio)+"\t"+str(Levelratio)+"\n")
        
        self.record.flush()


        print ('time: '+str("%.1f"%t)+"\t"
        +'TempSt: '+str("%.2f"%TSetTo)+"\t"
        +'TCouple: '+str("%.2f"%TCouple)+"\t"
        +'Sens0St: '+str(str(Sense0SetTo))+"\t"
        +'Sens0Op: '+str(Sense0Output)+"\t"
        +'Exci0St: '+str(str(Excite0SetTo))+"\t"
        +'Exci0Op: '+str(Excite0Output)+"\t"
        +'Exci0Rf: '+str(Excite0Ref)+"\t"
        +'PMT0Target: '+str(PMT0Target)+"\t"
        +'PMT0rds: '+str("%.2f"%PMT0reads)+"\t"

        +'Sens1St: '+str(str(Sense1SetTo))+"\t"
        +'Sens1Op: '+str(Sense1Output)+"\t"
        +'Exci1St: '+str(str(Excite1SetTo))+"\t"
        +'Exci1Op: '+str(Excite1Output)+"\t"
        +'Exci1Rf: '+str(Excite1Ref)+"\t"
        +'PMT1Target: '+str(PMT1Target)+"\t"
        +'PMT1rds: '+str("%.2f"%PMT1reads)+"\t"

        +'ODSET: '+str(ODSET)+"\t"
        +'ODrds: '+str("%.2f"%ODreads)+"\t"
        +'ODLED: '+str(ODLED)+"\t"
        +'Stir: '+str(Stir)+"\t"
        +'Heater: '+str(Heater)+"\t"
        +'PumpIn: '+str(PumpIn)+"\t"
        +'PumpOut: '+str(PumpOut)+"\t"
        +'ODLEDrf: '+str(ODLEDrf)+"\t"
        +'LevelLED: '+str(LevelLED)+"\t"
        +'Level: '+str(Level)+"\t"
        +'LevelLEDrf: '+str(LevelLEDrf)+"\t"
        +'ODratio: '+str(ODratio)+"\t"
        +'Levelratio: '+str(Levelratio)+"\n")

    # clean up
    def close(self):
        # close serial
        ser.flush()
        ser.close()
        
def send_email(destinations, machine_name, err_flag, msg):
    SENDER = "rainl199922@gmail.com"
    SMTP_SERVER = "smtp.gmail.com"
    SMTP_PORT = 587    # TLS
    PASSWORD = "jvnnztbhhbbchwhe"
    
    err_dict = { 0: "Info", 1: "Warning", 2: "Error" }
    
    subject = f"[{err_dict[err_flag]}] {machine_name}"
    
    email = EmailMessage()
    email["Subject"] = subject
    email["From"] = SENDER
    email["To"] = ','.join(destinations)
    email["Cc"] = SENDER    # cc to the lab email
    email.set_content(msg)
    
    try:
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.ehlo()
            server.starttls()
            server.ehlo()
            server.login(SENDER, PASSWORD)
            server.send_message(email, to_addrs=destinations + [SENDER])
    except Exception as err:
        print("Error sending email:", err)
    

# main() function
if __name__ == '__main__':
    

    ports = list(serial.tools.list_ports.comports()) # Detect arduino serial por
    p=[port for port in ports if port[2] != 'n/a']
    SERIAL=p[0][0]
    serial_port = SERIAL
    print('Reading from serial port %s...' % SERIAL)
    ser = serial.Serial(SERIAL, 9600)

    time.sleep(2) #wait for initialization



    data_q = Queue() #queue that holds all data from arduino
    msg_q = Queue()
    
    serialcom = SerialCom(data_q, msg_q)
    arduino_process = Process(target = serialcom.arduino, args = ())
    arduino_process.start()

    # set up animation
    fig = plt.figure(facecolor='white',figsize=(20,9))
    #ax = plt.axes(xlim=(0, 100), ylim=(0, 1023))


    # plot parameters
    analogPlot = AnalogPlot(data_q, msg_q, fig)

    print('plotting data...')
                    
    anim = animation.FuncAnimation(fig, analogPlot.update, interval=80)

    # show plot
    plt.show()
    
    # clean up
    print('exiting.')
    analogPlot.close()

  
