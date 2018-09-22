#!/usr/bin/env python
#
# Copyright (C) 2018 Freek van Tienen <freek.v.tienen@gmail.com>
#               2018 Dennis Wijngaarden
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#
"""
Receive asterix events over UDP
"""

import sys
from os import path, getenv
import threading
import socket
import asterix
import time
import logging
from sets import Set
import math
import numpy as np

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprz_math import geodetic

# Asterix receiver over UDP
class AsterixReceiver(threading.Thread):

    def __init__(self, ltp_def):
        """
        Initialize the Asterix receiver thread
        """
        threading.Thread.__init__(self)

        # Initialize the variables
        self.name = "AsterixReceiver" 
        self.deamon = True
        self.events = Set([])
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ltp_def = ltp_def
        
        # Create the socket for communication
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1.0)
        self.sock.bind(('', 45454))

    def run(self):
        """
        Asterix packet receiver over UDP
        """
        logging.info("Starting " + self.name)
        
        # Main loop for receiving packets until stop is given
        while not self.stop_event.is_set():

            # Receive and parse an asterix packets
            try:
                asterix_packet = self.sock.recv(10240)
            except socket.timeout:
                continue
            packets = asterix.parse(asterix_packet)
            
            # For each received asterix packet
            logging.debug("Parsing %d asterix packets", len(packets))
            for p in packets:
                new_t = AsterixEvent(p, self.ltp_def)

                # Check if it already existed then update else add
                try:
                    self.lock.acquire()
                    found = False
                    for t in self.events:
                        if t == new_t:
                            t.update(new_t, self.ltp_def)
                            found = True
                        
                    if not found:
                        self.events.add(new_t)
                finally:
                    self.lock.release()

            # Check if we need to remove any events
            try:
                self.lock.acquire()
                events_copy = self.events.copy()
                for t in events_copy:
                    if t.timed_out():
                        self.events.discard(t)
            finally:
                self.lock.release()

        logging.info("Stopping " + self.name)

    def get_events(self):
        """
        Get the current events in a thread safe way
        """
        events = Set([])
        try:
            self.lock.acquire()
            events = self.events.copy()
        finally:
            self.lock.release()
        return events

    def stop(self):
        """
        Thread safe stopping the asterix receiver
        """
        self.stop_event.set()
        self.sock.close()
        

# Asterix event object received through the asterix protocol
class AsterixEvent(object):
    TIMEOUT = 5         # Timeout in seconds
    FT_TO_M = 0.3048    # Convert Feet to Meter

    # Type of asterix traffic
    OTHER_AIR_TRAFFIC = 0
    LOCALIZED_WEATHER = 1
    MIGRATORY_BIRD = 2
    BIRD_OF_PREY = 3

    def __init__(self, asterix_packet, ltp_def):
        """
        Create a new event from received asterix packet
        """
        self.id = asterix_packet['I040']['TrkN']['val'] 
        self.tot = asterix_packet['I070']['ToT']['val'] # [s]
        self.lat = asterix_packet['I105']['Lat']['val'] / 180. * math.pi # [deg]
        self.lon = asterix_packet['I105']['Lon']['val'] / 180. * math.pi # [deg]
        self.alt = asterix_packet['I130']['Alt']['val'] * AsterixEvent.FT_TO_M # [m]
        self.roc = asterix_packet['I220']['RoC']['val'] * AsterixEvent.FT_TO_M # [m]
        self.tot_lst = [self.tot]
        self.lla_lst = [geodetic.LlaCoor_f(self.lat, self.lon, self.alt)]
        self.enu_lst = [geodetic.LlaCoor_f(self.lat/180*math.pi, self.lon/180*math.pi, 0).to_enu(ltp_def)]
        self.gspeed = 0.
        self.velocity = 0.
        self.hdg = 0.
        self.time = time.time()

    def update(self, other, ltp_def):
        """
        Update the current object with information from the new other object
        """
        assert self.id == other.id
        self.tot = other.tot
        self.lat = other.lat
        self.lon = other.lon
        self.alt = other.alt
        self.roc = other.roc
        self.tot_lst = [self.tot_lst[-1], other.tot_lst[-1]]
        self.lla_lst = [self.lla_lst[-1], other.lla_lst[-1]]
        self.enu_lst = [self.enu_lst[-1], geodetic.LlaCoor_f(self.lat/180*math.pi, self.lon/180*math.pi, self.alt).to_enu(ltp_def)]
        dt = (self.ToT_lst[1] - self.ToT_lst[0])
        dx = (self.enu_lst[1].x - self.enu_lst[0].x)
        dy = (self.enu_lst[1].y - self.enu_lst[0].y) 
        self.gspeed = {'east': dx/dt , 'north' : dy/dt }
        self.velocity = np.sqrt(self.gspeed['east']**2 + self.gspeed['north']**2)
        self.hdg = np.rad2deg(np.arctan2(self.gspeed['east'], self.gspeed['north'])) % 360.
        self.time = time.time()
        
    def timed_out(self):
        """
        Whether we haven't received an update for a while and exceeded the TIMEOUT
        """
        return ((time.time() - self.time) > AsterixEvent.TIMEOUT)

    def get_type(self):
        """
        Return the type of the traffic event
        """
        # Other Air Traffic
        if self.id < 20000:
            return AsterixEvent.OTHER_AIR_TRAFFIC
        # Localised Weather Events
        elif self.id >= 20000 and self.id < 30000: 
            return AsterixEvent.LOCALIZED_WEATHER
        # Animal Migratory Bird
        elif self.id >= 30000 and self.id < 40000:
            return AsterixEvent.MIGRATORY_BIRD
        # Animal Bird of prey
        else:
            return AsterixEvent.BIRD_OF_PREY

    def get_radius(self):
        """
        Return the radius of the asterix event
        """
        ev_type = self.get_type()
        if ev_type == AsterixEvent.OTHER_AIR_TRAFFIC:
            return 300.
        elif ev_type == AsterixEvent.LOCALIZED_WEATHER:
            return 150. + 150. * self.alt/3000.
        elif ev_type == AsterixEvent.MIGRATORY_BIRD:
            return 100.
        else:
            return 200.

    def get_lla(self):
        """
        Return the last lat, long, alt position
        """
        return self.lla_lst[-1]
        
    def get_enu(self):
        """
        Return the last east, north, up position
        """
        return self.enu_lst[-1]
        
    def get_gspeed(self):
        """
        Return the ground speed vector
        """
        return self.gspeed
        
    def get_velocity(self):
        """
        Return the gspeed magnitude
        """
        return self.velocity

    def __eq__(self, other):
        """
        Equivalent if the id is similar
        """
        return self.id == other.id
