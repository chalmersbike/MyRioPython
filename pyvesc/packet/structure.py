from __future__ import absolute_import
import collections
import struct
from pyvesc.packet.exceptions import *
from PyCRC.CRCCCITT import CRCCCITT


class Header(collections.namedtuple(u'Header', [u'payload_index', u'payload_length'])):
    u"""
    Tuple to help with packing and unpacking the header of a VESC packet.
    """
    @staticmethod
    def generate(payload):
        u"""
        Creates a Header for the given payload.
        :param payload: byte string representation of payload.
        :return: Header object.
        """
        payload_length = len(payload)
        if payload_length < 256:
            payload_index = 0x2
        elif payload_length < 65536:
            payload_index = 0x3
        else:
            raise InvalidPayload(u"Invalid payload size. Payload must be less than 65536 bytes.")
        return Header(payload_index, payload_length)

    @staticmethod
    def parse(buffer):
        u"""
        Creates a Header by parsing the given buffer.
        :param buffer: buffer object.
        :return: Header object.
        """
        return Header._make(struct.unpack_from(Header.fmt(buffer[0]), buffer, 0))

    @staticmethod
    def fmt(start_byte):
        u"""
        Format characters of the header packet.
        :param start_byte: The first byte in the buffer.
        :return: The character format of the packet header.
        """
        if start_byte is 0x2:
            return u'>BB'
        elif start_byte is 0x3:
            return u'>BH'
        else:
            raise CorruptPacket(u"Invalid start byte: %u" % start_byte)


class Footer(collections.namedtuple(u'Footer', [u'crc', u'terminator'])):
    u"""
    Footer of a VESC packet.
    """
    TERMINATOR = 0x3 # Terminator character

    @staticmethod
    def parse(buffer, header):
        return Footer._make(struct.unpack_from(Footer.fmt(), buffer, header.payload_index + header.payload_length))

    @staticmethod
    def generate(payload):
        crc = CRCCCITT().calculate(payload)
        terminator = Footer.TERMINATOR
        return Footer(crc, terminator)

    @staticmethod
    def fmt():
        u"""
        Format of the footer.
        :return: Character format of the footer.
        """
        return u'>HB'
