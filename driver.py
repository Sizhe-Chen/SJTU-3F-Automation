#!/usr/bin/python
# -*- coding: utf-8 -*-

import threading
import socket, json, time
import traceback

class driver:
    __sock = None
    __dst = None
    __conf = {}
    __keepRunning = True
    __recv_t = None
    __uid = 0
    
    def __init__(self):
        print ("----------Driver No.%d Init----------" % id(self))
        try: 
            self.__dst = ('127.0.0.1', 61551)

            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.__sock.settimeout(1.0)
            self.__recv_t = threading.Thread(target=self.recv_thread)
            self.__recv_t.start()
        except:
            traceback.print_exc() 
            print ("----------Driver Init Failed----------")
            return
        print ("----------Driver Init Done----------")

    def __del__(self):
        # self.close()
        print ("\n----------Driver No.%d End----------" % id(self))

    def __launch(self):
        self.__sock.sendto(json.dumps(self.__conf).encode('utf-8'), self.__dst)
        self.__conf = {}
    
    def __setMotor(self, motor):
        # motor: from -1 to 1
        self.__conf['sm'] = round(max(min(motor, 1), -1), 3)

    def __setServo(self, steer):
        # steer: from -1 to 1
        self.__conf['ss'] = round(max(min(steer, 1), -1), 3)

    def __setDist(self, dt):
        # distance: from 0 to 0xffffffff
        self.__conf['sd'] = dt

    def __setMode(self, md):
        if md == 'speed':
            self.__conf['so'] = 1
        if md == 'distance':
            self.__conf['so'] = 3
        if md == 'stop':
            self.__conf['so'] = 0

    def __getMode(self):
        self.__conf['ro'] = 0

    def __getSensor(self):
        self.__conf['rs'] = 0

    def setStatus(self, **dt):
        self.__conf['uid'] = round(time.time(), 5)
        if 'motor' in dt:
            self.__setMotor(dt['motor'])
        if 'servo' in dt:
            self.__setServo(dt['servo'])
        if 'dist' in dt:
            self.__setDist(dt['dist'])
        if 'mode' in dt:
            self.__setMode(dt['mode'])
        self.__launch()

    def getStatus(self, **dt):
        self.__conf['uid'] = round(time.time(), 5)
        if 'mode' in dt:
            self.__getMode()
        if 'sensor' in dt:
            self.__getSensor()
            self.__launch()

    def close(self):
        self.__keepRunning = False
        self.__recv_t.join()
        self.__sock.close()
    
    def heartBeat(self):
        self.setStatus()

    def parse_feedback(self, js):
        try:
            obj = json.loads(js.decode('utf-8'))
            """
            if not 'uid' in obj:
                print("Uid is needed")
                return 2
            elif self.__uid > obj['uid']:
                print("Uid %0.3f is behind %0.3f, drop %s." % (obj['uid'], self.__uid, js))
            else:
                self.__uid = float(obj['uid']
            """
            return 0
        except:
            traceback.print_exc()
            return 1

    def recv_thread(self):
        while (self.__keepRunning):
            try:
                res = self.__sock.recvfrom(1024)
                print("recv: %s, from %s:%d" % (res[0], res[1][0], res[1][1]))
                if self.parse_feedback(res[0]):
                    print("Error feedback: %s" % res[0])
            except socket.timeout as e:
                pass
        
        

if __name__=='__main__':
    print ("----------Cyber Car Driver----------")
    print ("Usage: import driver.py and using driver() to start.")
    print ("------------------------------")

