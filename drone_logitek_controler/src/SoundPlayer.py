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
        Thread.__init__(self)
        self.langs =["en", "fr"]
        self.langId = 0
        self.sentence = ""
        self.keepRunning = True
        self.sentenAvail = False;
        self.recordedSound = []
        self.getRecords();
	self.pitch = 1.2;
        
        
    def setLang(self, lanId):
        self.langId = lanId;
        
    def setPitch(self, v):
	pass;
        #self.pitch = v;
        
    def getRecords(self):
        '''to get records from indexes.txt file for recorded sentences'''
        recordFile = os.getcwd() +"/sound_data/indexes.txt"
        try:
            self.indexfile = open(recordFile, "r+");
        except IOError:
            d = os.path.dirname(recordFile)
            if not os.path.exists(d):
                os.makedirs(d)
            self.indexfile = open(recordFile, "wr+");
            
        for line in self.indexfile:
            if line[0] == ("#"):
                continue;
            self.recordedSound.append(line[:-1])
            
        
    def getMp3(self, msg):
        
        '''get mp3 filename of the input msg. If not exist then call get_tts_mp3 to create one'''
        folder = os.getcwd() +"/sound_data/{}.flac"
        idx = [self.recordedSound.index(x) for x in self.recordedSound if (x.lower() == str(msg).lower())]
        #print idx: list of sentences that march the requested msg
        
        if len(idx) > 0:
            #print "exist: {}".format(idx[0])
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
        ss = sent.split(".")
        if len(ss) == 1:
            ss.append("")
        for s in ss[:-1]:
            #print "s: ",s
            values   = { 'q': s, 'tl': self.langs[langId] }
            data     = urllib.urlencode(values)
            request  = urllib2.Request(baseurl, data)
            request.add_header("User-Agent", "Mozilla/5.0 (X11; U; Linux i686) Gecko/20071127 Firefox/2.0.0.11" )
            try:
                response = urllib2.urlopen(request)
            except urllib2.HTTPError, e:
                print "HTTPError, no result returned ", e
                return False
            if( fname==None ):
                fname = "_".join(sent.split())
            ofp = open(fname,"ab")
            
            ofp.write(response.read())
        print "Saved to file: %s" % os.path.abspath(fname)
        
        
        #os.system("mplayer {0} -af extrastereo=0 &".format(video_path))
        
        return True

    def getTime(self):
	return time.time();

    def playSound(self, filename):
        if filename is None:
            print "nothing to play"
            return;
        rate = 1.4 if self.langId == 0 else 1.4
        

        vlc_path = 'cvlc'
        proc = "gst-launch-0.10 filesrc"
        video_path = '{0}'.format(path.abspath(filename))
        option = "!  audioconvert ! audioresample ! autoaudiosink"
        #option = "!  audioconvert ! audioresample ! alsasink"
        
        
        #subprocess.call([vlc_path, video_path, '--play-and-exit', '--rate={0}'.format(rate), '&'])
        #os.system("{0} {1} --play-and-exit --rate={2} --play-and-exit".format(vlc_path,video_path, rate))
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

    import sys
    if len(sys.argv) < 2:
        print "Wrong number of arguments!";
        print "Usage: python SoundPlayer.py <Text_File_Name> [lang]";
        print "Usage: lang: 0(English), 1(French)";
        print "Stopped!";
        exit(1);
    f = open(sys.argv[1]);
    player1 = SoundPlayClient()
    player1.setLang(0)
    if len(sys.argv) > 2:
          player1.setLang(int(sys.argv[2]))
    for line in f:
        player1.getMp3(line[:-1])
    
    f.close()

       
    print "done"
