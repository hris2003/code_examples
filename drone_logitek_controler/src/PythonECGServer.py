'''
Created on Apr 8, 2013

@author: meka
'''
import sys, struct, array, serial
from bluetooth import *
import time as Time
import threading, random
import socket
import Tkinter
from Tkinter import *
from PythonGSRServer import PythonGSRServer
from threading import Timer

p1_10_56 = 0.0373
p2_10_56 = -24.9915
p1_56_220 = 0.0054
p2_56_220 = -3.5194
p1_220_680 = 0.0015
p2_220_680 = -1.0163
p1_680_47 = 0.00045580
p2_680_47 = -0.3014

      
class PythonECGServer(threading.Thread):
    def __init__(self, verbose, real, folder):
        threading.Thread.__init__(self)
	self.folder = folder;
        self.file = str(self.folder) + "/ECG_{0}.txt".format(Time.asctime())
        self.keepRunning = True
        self.lall = []
        self.rall = []
        self.verbose = verbose;
        self.real = real#real data reception from bluetooth device
        if self.real:
            self.bluetoothSockInit()
        if self.verbose:
            self.tcpSockInit()
            
        pass
    
    def wait_for_ack(self):
        ddata = ""
        ack = struct.pack('B', 0xff) 
        while ddata != ack:
            ddata = self.ser.recv(1)
        return
        
    def bluetoothSockInit(self):
        port = 1;
        host = "00:06:66:42:20:81"
        
#        self.btSock = BluetoothSocket( RFCOMM )
#        self.btSock.connect((host, port))
        
        self.ser = BluetoothSocket( RFCOMM )
        self.ser.connect((host, port))

        # send the set sensors command 
        self.ser.send(struct.pack('BBB', 0x08, 0x10, 0x00))    # ecg only
        self.wait_for_ack()

        # send the set sampling rate command
        self.ser.send(struct.pack('BB', 0x05, 0x14))           # 51.2Hz
        self.wait_for_ack()


        # send start streaming command
        self.ser.send(struct.pack('B', 0x07))
        self.wait_for_ack()
        
    def tcpSockInit(self):
   
        HOST, PORT = "localhost", 8887
        # Create a socket (SOCK_STREAM means a TCP socket)
        self.tcpSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcpSock.connect((HOST, PORT))
                   
    def genECGValue(self):
        j = 0
        try:
            while self.keepRunning:
#                if j == 10: self.keepRunning = False
                j+= 1
                
                la = random.randint(3600, 4700)
                ra = random.randint(3600, 4700)
#                ddata = self.btSock.recv(1024)
#                
#                numbytes = len(ddata)
#                if numbytes <= 0: break
                if (self.verbose):
                    self.tcpSock.sendall(str(la) + "\t"+ str(ra) + "\n")
                
                
                Time.sleep(0.02)
                
        except:# KeyboardInterrupt:
            pass
        if self.verbose:
            self.tcpSock.close()
        print
        print "genECG All done"  
 
    def getSensedValue(self):#get ECG value

        
        ddata = ""
        numbytes = 0
        
#        framesize = 80 
        framesize = 7  #default. i.e. Packet type (1), TimeStamp (2), ECG (2*2)
        f = open(self.file, "w")
        num = 0
        j = 0
        try:
            while self.keepRunning:
                j+= 1
                
                while numbytes < framesize:
                    ddata += self.ser.recv(framesize)
                    numbytes = len(ddata)
#                ddata = self.btSock.recv(1024)
#                
#                numbytes = len(ddata)
#                if numbytes <= 0: break
                
                data = ddata[0:framesize]
                ddata = ddata[framesize:]
                numbytes = len(ddata)
                #print data
                t = Time.asctime()
                packettype = struct.unpack('B', data[0:1])
                #(timestamp, ecg_ra_ll, ecg_la_ll) = struct.unpack('HHH', data[1:framesize])
                (timestamp, ecgra, ecgla) = struct.unpack('HHH', data[1:framesize])
                self.lall.insert(0, ecgla);
                self.rall.insert(0, ecgra);
                #print "%05d: %04d\t%04d" % (timestamp, (ecgla), (ecgra))
                f.write('{0}\t{1}\t{2}\t{3}\n'.format(t, timestamp, ecgla, ecgra))
                if self.verbose:
            
                    self.tcpSock.sendall(str(ecgla) + "\t"+ str(ecgra) + "\n")
                sys.stdout.flush()
                
#                for i in range(0,framesize,4):
#                    gsr = struct.unpack('I', data[i:i+4])
#                    val = int(gsr[0])
#                    f.write('{0}\t{1}\n'.format(t, val))
#                    print '({0:10} {1:10})'.format( val, self.convert2GSR(p1_10_56, p2_10_56, val))
##                    print gsr
#                    self.tcpSock.sendall(str(val) + "\n")
#                    sys.stdout.flush()
#                    num += 1
#                    if num == 5:
#                        #print
#                        num = 0
                #print "still in the while"
        except:# KeyboardInterrupt:
            print "Unexpected error:", sys.exc_info()[0]
            pass
        #self.btSock.close()
        # send stop streaming command
        self.ser.send(struct.pack('B', 0x20));
        f.close()
        self.wait_for_ack()
        self.ser.close()
        print " close the serial port"
        if self.verbose:
            self.tcpSock.close()
        #print
        #print "All done"
    
    def setKeepRunning(self, val):
        self.keepRunning = val
   
    def run(self):
        if (self.real):
            self.getSensedValue()
        else:
            self.genECGValue()
    


class GUI_GSR_ECG(Tkinter.Tk):
    def __init__(self, parent, folder):
        Tkinter.Tk.__init__(self, parent )
        self.parent = parent;
        self.myPyGSRServer = None
        self.initGUI();
	self.is_started = False;
	self.folder = folder;
        
    def initGUI(self):
        self.grid()
        
        r = 0
        self.startGSRBtn = Tkinter.Button(self, text="Start GSR", command=self.startGSRRecording)
        self.startGSRBtn.grid(column=0, row = r)
        self.stopGSRBtn = Tkinter.Button(self, text="Stop GSR", command=self.stopGSRRecording)
        self.stopGSRBtn.grid(column=1, row = r)
        
        r +=1
        self.quitBtn = Tkinter.Button(self, text="Quit", command=self.destroy)
        self.quitBtn.grid(column=1, row = r, columnspan=2)
    def toggleGSRRecording(self):
	self.is_started = not self.is_started;
	if self.is_started:
		self.startGSRRecording();
	else:
		self.stopGSRRecording();

    def startGSRRecording(self):
        #Start the GRS reading
        #If GSRThread is already running, then abort!
#         if self.myPyGSRServer is not None:
#             print "GSRThread is already running"
#             
#             return;
        
        self.myPyGSRServer = PythonGSRServer(True, True, self.folder)
        self.myPyGSRServer.start()
        
        self.myPyECGServer = PythonECGServer(True, True, self.folder)
        self.myPyECGServer.start()
    
        
    def stopGSRRecording(self):
        if self.myPyGSRServer is not None:
            self.myPyGSRServer.setKeepRunning(False)
            
            self.myPyECGServer.setKeepRunning(False)
            return;
        print "Nothing to stop"
        self.myPyGSRServer = None
        return;
    
    

        
if __name__ == '__main__':
    
    gui = GUI_GSR_ECG(None, "data")
    gui.title("my GUI_GSR_ECG")
    gui.mainloop()
    
    print "Finished!!!"
    
