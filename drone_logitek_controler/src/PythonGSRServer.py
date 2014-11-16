'''
Created on Apr 8, 2013

@author: meka
'''
#import sys, struct, array, serial
from bluetooth import *
import time as Time
import threading, random
import socket

p1_10_56 = 0.0373
p2_10_56 = -24.9915
p1_56_220 = 0.0054
p2_56_220 = -3.5194
p1_220_680 = 0.0015
p2_220_680 = -1.0163
p1_680_47 = 0.00045580
p2_680_47 = -0.3014


       
class PythonGSRServer(threading.Thread):
    def __init__(self, verbose, real, folder):
        threading.Thread.__init__(self)
	self.folder = folder;
        self.file = str(self.folder) + "/GSR_{0}.txt".format(Time.asctime())
        self.GSRVals = [];
        self.keepRunning = True
        self.verbose = verbose;
        
        self.real = real;
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
        host = "00:06:66:42:20:2D"
        
#        self.btSock = BluetoothSocket( RFCOMM )
#        self.btSock.connect((host, port))
        
        self.ser = BluetoothSocket( RFCOMM )
        self.ser.connect((host, port))

        # send the set sensors command 
        self.ser.send(struct.pack('BBB', 0x08, 0x04, 0x00))    # gsr only
        self.wait_for_ack()

        # send the set sampling rate command
        self.ser.send(struct.pack('BB', 0x05, 0x64))           # 10.24Hz
        self.wait_for_ack()

        # send the set gsr range command
        self.ser.send(struct.pack('BB', 0x21, 0x04))           # autorange 
        #   ser.write(struct.pack('BB', 0x21, 0x03))
        self.wait_for_ack()

        # send start streaming command
        self.ser.send(struct.pack('B', 0x07))
        self.wait_for_ack()
        
    def tcpSockInit(self):
   
        HOST, PORT = "localhost", 8888
        # Create a socket (SOCK_STREAM means a TCP socket)
        self.tcpSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcpSock.connect((HOST, PORT))
                   
    def generateGSRValue(self):
        j = 0
        try:
            while self.keepRunning:
#                if j == 10: self.keepRunning = False
                j+= 1
                
                val = random.randint(1600, 2700)
                rang = 0
#                ddata = self.btSock.recv(1024)
#                
#                numbytes = len(ddata)
#                if numbytes <= 0: break
                if (self.verbose):
                    self.tcpSock.sendall(str(val) + "\t"+ str(rang) + "\n")
                
                #print val
                Time.sleep(0.1)
                
        except:# KeyboardInterrupt:
            pass
        if self.verbose:
            self.tcpSock.close()
        print
        print "genGSR All done"       
        
    def getSensedValue(self):

        
        ddata = ""
        numbytes = 0
        
#        framesize = 80 
        framesize = 5  #default. i.e. Packet type (1), TimeStamp (2), GSR (2)
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
                (timestamp, gsr) = struct.unpack('HH', data[1:framesize])
                #print "%05d: %04d\t%02d" % (timestamp, (gsr&0x3FFF), ((gsr&0xC000)>>14))
                val = gsr&0x3FFF
                rang = (gsr&0xC000)>>14
                f.write('{0}\t{1}\t{2}\t{3}\t{4}\n'.format(t, timestamp, val, rang, self.convert2GSR(val, rang)));
                self.GSRVals.insert(0, self.convert2GSR(val, rang));
                
                if self.verbose:
                    self.tcpSock.sendall(str(val) + "\t"+ str(rang) + "\n")
                    
                sys.stdout.flush()
                
#                for i in range(0,framesize,4):
#                    gsr = struct.unpack('I', data[i:i+4])
#                    val = int(gsr[0])
#                    f.write('{0}\t{1}\n'.format(t, val))
#                    #print '({0:10} {1:10})'.format( val, self.convert2GSR(p1_10_56, p2_10_56, val))
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
        
    def convert2GSR(self, adc, rang):
        '''GSR conductance reponse'''
        p1 = -1
        p2 = -1
        
        if rang == 0:
            p1 = p1_10_56;
            p2 = p2_10_56;
            
        if rang == 1:
            p1 = p1_56_220;
            p2 = p2_56_220;
            
        if rang == 2:
            p1 = p1_220_680;
            p2 = p2_220_680;
            
        if rang == 3:
            p1 = p1_680_47;
            p2 = p2_680_47;
            
        
        d = adc*p1 + p2

        return d 
    
    def run(self):
        if self.real:
            self.getSensedValue();
        else:
            self.generateGSRValue();
    
    
if __name__ == '__main__':
    
    myPyGSRServer = PythonGSRServer(True, True, "data")
#    myPyGSRServer.getGSRValue()
    myPyGSRServer.start()

    print "Finished!!!"
    
