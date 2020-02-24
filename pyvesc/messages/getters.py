from __future__ import absolute_import
from pyvesc.messages.base import VESCMessage

class GetValues(object):
    __metaclass__ = VESCMessage
    u""" Gets internal sensor data
    """
    id = 4

    fields = [
            (u'temp_mos1', u'h', 10),
            (u'temp_mos2', u'h', 10),
            (u'temp_mos3', u'h', 10),
            (u'temp_mos4', u'h', 10),
            (u'temp_mos5', u'h', 10),
            (u'temp_mos6', u'h', 10),
            (u'temp_pcb',  u'h', 10),
            (u'current_motor', u'i', 100),
            (u'current_in',  u'i', 100),
            (u'duty_now',    u'h', 1000),
            (u'rpm',         u'i', 1),
            (u'v_in',        u'h', 10),
            (u'amp_hours',   u'i', 10000),
            (u'amp_hours_charged', u'i', 10000),
            (u'watt_hours',  u'i', 10000),
            (u'watt_hours_charged', u'i', 10000),
            (u'tachometer', u'i', 1),
            (u'tachometer_abs', u'i', 1),
            (u'mc_fault_code', u'c')
    ]


class GetRotorPosition(object):
    __metaclass__ = VESCMessage
    u""" Gets rotor position data
    
    Must be set to DISP_POS_MODE_ENCODER or DISP_POS_MODE_PID_POS (Mode 3 or 
    Mode 4). This is set by SetRotorPositionMode (id=21).
    """
    id = 21

    fields = [
            (u'rotor_pos', u'i', 100000)
    ]
