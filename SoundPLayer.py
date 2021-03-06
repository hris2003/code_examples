'''
Created on Oct 4, 2013

@author: meka
'''
'''
Created on Jul 1, 2013

@author: meka
'''
#!usr/bin/env python
# -*- coding: utf-16 -*-

from os import path
import os

#os.environ['http_proxy']=''
import urllib, urllib2
from threading import Thread
import time

class SoundPlayClient(Thread):
    '''
    classdocs
    '''

    def __init__(self):
        '''
        Constructor
        '''
        Thread.__init__(self, verbose=False)
        self.langs =["en", "fr"]
        self.langId = 0
        self.sentence = ""
        self.keepRunning = True
        self.sentenAvail = False;
        self.recordedSound = []
        self.getRecords()
        self.pitch = 1.2;
        
    def setLang(self, lanId):
        self.langId = lanId;
        
    def setPitch(self, v):
        self.pitch = v;
        
    def getRecords(self):
        '''to get records from indexes.txt file for recorded sentences'''
        recordFile = "../data/sound/indexes.txt"
        try:
            self.indexfile = open(recordFile, "r+");
        except IOError:
            d = os.path.dirname(recordFile)
            if not os.path.exists(d):
                os.makedirs(d)
            self.indexfile = open(recordFile, "wr+");
            
        for line in self.indexfile:
            #print line,
            if line[0] == ("#"):
                continue;
            self.recordedSound.append(line[:-1])
            
        
    def getMp3(self, msg):
        
        '''get mp3 filename of the input msg. If not exist then call get_tts_mp3 to create one'''
        folder = "../data/sound/{}.flac"
        idx = [self.recordedSound.index(x) for x in self.recordedSound if (x.lower() == str(msg).lower())]
        print idx
        
        if len(idx) > 0:
            print "exist: {}".format(idx[0])
            return folder.format(idx[0])
        else:
            filename = folder.format(len(self.recordedSound))
            self.get_tts_mp3(self.langId, msg, filename);
            self.recordedSound.append(msg)
            self.indexfile.write(msg)
            self.indexfile.write("\n")
            print "not yet exist. New one is: {}".format(msg)
            return filename
        
    def say(self, msg, immediate=False):
        if len(msg.strip()) == 0:
            print "nothing to say"
            return;
        if immediate:
            self.playSound(self.getMp3(' '.join(msg.split())))
            return;
        
        self.sentence = ' '.join(msg.split())
        self.sentenAvail = True;
        

    def get_tts_mp3(self, langId, sent, fname=None ):
        print "Retrieving .mp3 for sentence: %s" % sent
        baseurl  = "http://translate.google.com/translate_tts"
        values   = { 'q': sent, 'tl': self.langs[langId] }
        data     = urllib.urlencode(values)
        request  = urllib2.Request(baseurl, data)
        request.add_header("User-Agent", "Mozilla/5.0 (X11; U; Linux i686) Gecko/20071127 Firefox/2.0.0.11" )
        response = urllib2.urlopen(request)
        if( fname==None ):
            fname = "_".join(sent.split())
        ofp = open(fname,"wb")
        
        ofp.write(response.read())
        print "Saved to file: %s" % fname
        
        
        #os.system("mplayer {0} -af extrastereo=0 &".format(video_path))
        
        return 
    
    def playSound(self, filename):
        if filename is None:
            print "nothing to play"
            return;
        rate = 1.2 if self.langId == 0 else 1.4
        

        vlc_path = 'cvlc'
        proc = "gst-launch-0.10 filesrc"
        video_path = '{0}'.format(path.abspath(filename))
        option = "!  audioconvert ! audioresample ! autoaudiosink"
        #option = "!  audioconvert ! audioresample ! alsasink"
        
        
        #subprocess.call([vlc_path, video_path, '--play-and-exit', '--rate={0}'.format(rate), '&'])
        #os.system("{0} {1} --play-and-exit --rate={2} --quiet".format(vlc_path,video_path, rate))
        os.system("{} location={} ! decodebin ! audioconvert ! pitch pitch={}  tempo={} {} &".format(proc,video_path, self.pitch, rate, option))
        #os.system("{} location={} ! decodebin !audioconvert ! pitch pitch={}  tempo={}  &".format(proc,video_path, pitch, rate, option))
        
        
    def utter(self):
        
        self.playSound(self.getMp3(self.sentence));
    
    def stop(self):
        self.keepRunning = False;
        self.indexfile.close()
        
        
    def run(self):
        while self.keepRunning:
            if self.sentenAvail:
                self.utter();
                self.sentence=""
                self.sentenAvail = False
            
        print "finished"
        return

if __name__ == '__main__':
    player1 = SoundPlayClient()
    player1.setLang(0)
    print time.time();
    # player1.start()
    player1.say("   Hello beautiful, can I help you?  ", True)
    print time.time();
    time.sleep(1)
    player1.setLang(1)
    # player1.start()
    msg =" Bonjour cheri. Comment vas tu ?  "
    print time.time();
    player1.say(msg, True)
       
    time.sleep(1)
    print time.time();
    player1.stop();
    
    print "done"
    print time.time();