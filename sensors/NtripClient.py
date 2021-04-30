#!/usr/bin/python -u
"""
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.
"""

import socket
import sys
import datetime
import base64
import time
from optparse import OptionParser


version=0.2
useragent="NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor=2 # How much the sleep time increases with each failed attempt
maxReconnect=1
maxReconnectTime=1200
sleepTime=1 # So the first one is 1 second
maxConnectTime=0


class NtripClient(object):
    def __init__(self,
                 buffer=50,
                 user="",
                 out=sys.stdout,
                 port=80,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=57,
                 lon=12,
                 height=1212,
                 ssl=False,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 ):
        self.buffer=buffer
        self.user=base64.b64encode(user.encode())
        self.out=out
        self.port=port
        self.caster=caster
        self.mountpoint=mountpoint
        self.lat = lat
        self.lon = lon
        self.setPosition(lat, lon)
        self.height=height
        self.verbose=verbose
        self.ssl=ssl
        self.host=host
        self.UDP_Port=UDP_Port
        self.V2=V2
        self.headerFile=headerFile
        self.headerOutput=headerOutput
        self.maxConnectTime=maxConnectTime

        self.socket=None

        if UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.bind(('', 0))
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        else:
            self.UDP_socket=None

        reconnectTry = 1
        if maxConnectTime > 0:
            EndConnect = datetime.timedelta(seconds=maxConnectTime)
        try:
            while reconnectTry <= maxReconnect:
                found_header = False
                if self.verbose:
                    sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry, maxReconnect))

                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.ssl:
                    self.socket = ssl.wrap_socket(self.socket)

                error_indicator = self.socket.connect_ex((self.caster, self.port))

                if error_indicator == 0:
                    print("Connected to %s:%d" % (self.caster, self.port) + "\n")
                    print("")
                    sleepTime = 1
                    connectTime = datetime.datetime.now()

                    error_indicator = self.socket.connect_ex((self.caster, self.port))
                    self.socket.settimeout(10)
                    self.socket.sendall(self.getMountPointString().encode())
                    # now = datetime.datetime.utcnow()
                    # test_str = "GET /MSM_GNSS HTTP/1.1\r\nUser-Agent: NTRIP sNTRIP/2.12.00n\r\nAuthorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=\r\n\r\n"
                    # self.socket.sendall(test_str)

                    while not found_header:
                        casterResponse = self.socket.recv(4096)  # All the data
                        header_lines = casterResponse.decode().split("\r\n")

                        for line in header_lines:
                            if line == "":
                                if not found_header:
                                    found_header = True
                                    if self.verbose:
                                        sys.stderr.write("End Of Header" + "\n")
                            else:
                                if self.verbose:
                                    sys.stderr.write("Header: " + line + "\n")
                            if self.headerOutput:
                                self.headerFile.write(line + "\n")

                        for line in header_lines:
                            if line.find("SOURCETABLE") >= 0:
                                sys.stderr.write("Mount point does not exist")
                                sys.exit(1)
                            elif line.find("401 Unauthorized") >= 0:
                                sys.stderr.write("Unauthorized request\n")
                                sys.exit(1)
                            elif line.find("404 Not Found") >= 0:
                                sys.stderr.write("Mount Point does not exist\n")
                                sys.exit(2)
                            elif line.find("ICY 200 OK") >= 0:
                                # Request was valid
                                sys.stderr.write("RECEIVED \"ICY 200 OK\" from NTRIP caster, reading correction data ...\n")
                                self.socket.sendall(self.getGGAString(self.lat,self.lon).encode())
                            elif line.find("HTTP/1.0 200 OK") >= 0:
                                # Request was valid
                                sys.stderr.write("RECEIVED \"HTTP/1.0 200 OK\" from NTRIP caster, reading correction data ...\n")
                                self.socket.sendall(self.getGGAString(self.lat,self.lon).encode())
                            elif line.find("HTTP/1.1 200 OK") >= 0:
                                # Request was valid
                                sys.stderr.write("RECEIVED \"HTTP/1.1 200 OK\" from NTRIP caster, reading correction data ...\n")
                                self.socket.sendall(self.getGGAString(self.lat,self.lon).encode())

                    if reconnectTry < maxReconnect:
                        sys.stderr.write("%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= factor

                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime

                    reconnectTry += 1
                else:
                    self.socket = None
                    if self.verbose:
                        print("Error indicator: ", error_indicator)

                    if reconnectTry < maxReconnect:
                        sys.stderr.write("%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= factor
                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime
                    reconnectTry += 1

        except KeyboardInterrupt:
            if self.socket:
                self.socket.close()

    def setPosition(self, lat, lon):
        self.flagN="N"
        self.flagE="E"
        if lon>180:
            lon=(lon-360)*-1
            self.flagE="W"
        elif (lon<0 and lon>= -180):
            lon=lon*-1
            self.flagE="W"
        elif lon<-180:
            lon=lon+360
            self.flagE="E"
        else:
            self.lon=lon
        if lat<0:
            lat=lat*-1
            self.flagN="S"
        self.lonDeg=int(lon)
        self.latDeg=int(lat)
        self.lonMin=(lon-self.lonDeg)*60
        self.latMin=(lat-self.latDeg)*60

    def getMountPointString(self):
        mountPointString = "GET /%s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (self.mountpoint, useragent, self.user.decode())
        if self.host or self.V2:
           hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
           mountPointString+=hostString
        if self.V2:
           mountPointString+="Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString+="\r\n"
        if self.verbose:
           print(mountPointString)
        return mountPointString

    def getGGAString(self,lat,lon):
        now = datetime.datetime.utcnow()
        ggaString= "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % (now.hour,now.minute,now.second,self.latDeg,self.latMin,self.flagN,self.lonDeg,self.lonMin,self.flagE,self.height)
        # ggaString = "GPGGA,%02d%02d%04.2f,5700.00000000,N,01200.00000000,E,4,10,1.0,0.000,M,0.0,M,," % (now.hour, now.minute, now.second)
        # ggaString = "GPGGA,%02d%02d%04.2f,%5.8f,N,%5.8f,E,4,10,1.0,0.000,M,0.0,M,," % (now.hour, now.minute, now.second,lat,lon)
        # print("NTRIP GGA lat : %.8f ; lon : %.8f" % (lat,lon))
        checksum = self.calcultateCheckSum(ggaString)
        if self.verbose:
            print("$%s*%s\r\n" % (ggaString, checksum))
        return "$%s*%s\r\n" % (ggaString, checksum)

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def readData(self,lat,lon):
        try:
            self.socket.sendall(self.getGGAString(lat, lon).encode())
            data=self.socket.recv(1028)#(self.buffer)
            # print("Received %d bytes of data" % (len(data)))
            # self.out.write(data)

            if self.UDP_socket:
                self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))
                # print(datetime.datetime.now()-connectTime)
            return data
        except socket.timeout:
            if self.verbose:
                sys.stderr.write('Connection TimedOut\n')
                sys.stderr.write('Closing Connection\n')
            data=False
            if self.socket:
                self.socket.close()
                self.socket = None
        except socket.error:
            if self.verbose:
                sys.stderr.write('Connection Error\n')
                sys.stderr.write('Closing Connection\n')
            data=False
            if self.socket:
                self.socket.close()
                self.socket = None
        except KeyboardInterrupt:
            if self.verbose :
                sys.stderr.write('Closing Connection\n')
            if self.socket:
                sys.stderr.write('Closing Connection\n')
                self.socket.close()
                self.socket = None

    def reconnectNTRIPSocket(self):
        reconnectTry = 1
        if maxConnectTime > 0:
            EndConnect = datetime.timedelta(seconds=maxConnectTime)
        try:
            while reconnectTry <= maxReconnect:
                found_header = False
                if self.verbose:
                    sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry, maxReconnect))

                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.ssl:
                    self.socket = ssl.wrap_socket(self.socket)

                error_indicator = self.socket.connect_ex((self.caster, self.port))

                if error_indicator == 0:
                    print("Connected to %s:%d" % (self.caster, self.port) + "\n")
                    print("")
                    sleepTime = 1
                    connectTime = datetime.datetime.now()

                    error_indicator = self.socket.connect_ex((self.caster, self.port))
                    self.socket.settimeout(10)
                    self.socket.sendall(self.getMountPointString().encode())
                    # now = datetime.datetime.utcnow()
                    # test_str = "GET /MSM_GNSS HTTP/1.1\r\nUser-Agent: NTRIP sNTRIP/2.12.00n\r\nAuthorization: Basic Q2hhbG1lcnNFMlJUSzo4ODU1MTE=\r\n\r\n"
                    # self.socket.sendall(test_str)

                    while not found_header:
                        casterResponse = self.socket.recv(4096)  # All the data
                        header_lines = casterResponse.decode().split("\r\n")

                        for line in header_lines:
                            if line == "":
                                if not found_header:
                                    found_header = True
                                    if self.verbose:
                                        sys.stderr.write("End Of Header" + "\n")
                            else:
                                if self.verbose:
                                    sys.stderr.write("Header: " + line + "\n")
                            if self.headerOutput:
                                self.headerFile.write(line + "\n")

                        for line in header_lines:
                            if line.find("SOURCETABLE") >= 0:
                                sys.stderr.write("Mount point does not exist")
                                sys.exit(1)
                            elif line.find("401 Unauthorized") >= 0:
                                sys.stderr.write("Unauthorized request\n")
                                sys.exit(1)
                            elif line.find("404 Not Found") >= 0:
                                sys.stderr.write("Mount Point does not exist\n")
                                sys.exit(2)
                            elif line.find("ICY 200 OK") >= 0:
                                # Request was valid
                                sys.stderr.write("RECEIVED \"ICY 200 OK\" from NTRIP caster, reading correction data ...\n")
                                self.socket.sendall(self.getGGAString(self.lat,self.lon).encode())
                            elif line.find("HTTP/1.0 200 OK") >= 0:
                                # Request was valid
                                sys.stderr.write("RECEIVED \"HTTP/1.0 200 OK\" from NTRIP caster, reading correction data ...\n")
                                self.socket.sendall(self.getGGAString(self.lat,self.lon).encode())
                            elif line.find("HTTP/1.1 200 OK") >= 0:
                                # Request was valid
                                sys.stderr.write("RECEIVED \"HTTP/1.1 200 OK\" from NTRIP caster, reading correction data ...\n")
                                self.socket.sendall(self.getGGAString(self.lat,self.lon).encode())

                    if reconnectTry < maxReconnect:
                        sys.stderr.write("%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= factor

                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime

                    reconnectTry += 1
                else:
                    self.socket = None
                    if self.verbose:
                        print("Error indicator: ", error_indicator)

                    if reconnectTry < maxReconnect:
                        sys.stderr.write("%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= factor
                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime
                    reconnectTry += 1

        except KeyboardInterrupt:
            if self.socket:
                self.socket.close()