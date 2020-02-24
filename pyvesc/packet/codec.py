from __future__ import absolute_import
from pyvesc.packet.structure import *
from pyvesc.packet.exceptions import *
from PyCRC.CRCCCITT import CRCCCITT


class UnpackerBase(object):
    u"""
    Helper methods for both stateless and stated unpacking.
    """
    @staticmethod
    def _unpack_header(buffer):
        u"""
        Attempt to unpack a header from the buffer.
        :param buffer: buffer object.
        :return: Header object if successful, None otherwise.
        """
        if len(buffer) == 0:
            return None
        fmt = Header.fmt(buffer[0])
        if len(buffer) >= struct.calcsize(fmt):
            try:
                header = Header.parse(buffer)
                return header
            except struct.error:
                raise CorruptPacket(u"Unable to parse header: %s" % buffer)
        else:
            return None

    @staticmethod
    def _unpack_footer(buffer, header):
        u"""
        Unpack the footer. Parse must be valid.
        :param buffer: buffer object.
        :param header: Header object for current packet.
        :return: Footer object.
        """
        try:
            footer = Footer.parse(buffer, header)
            return footer
        except struct.error:
            raise CorruptPacket(u"Unable to parse footer: %s" % buffer)

    @staticmethod
    def _next_possible_packet_index(buffer):
        u"""
        Tries to find the next possible start byte of a packet in a buffer. Typically called after a corruption has been
        detected.
        :param buffer: buffer object.
        :return: Index of next valid start byte. Returns -1 if no valid start bytes are found.
        """
        if len(buffer) < 2: # too short to find next
            return -1
        next_short_sb = buffer[1:].find('\x02')
        next_long_sb= buffer[1:].find('\x03')
        possible_index = []
        if next_short_sb >= 0: # exclude index zero's as we know the current first packet is corrupt
            possible_index.append(next_short_sb + 1) # +1 because we want found from second byte
        if next_long_sb >= 0:
            possible_index.append(next_long_sb + 1)
        if possible_index == []:
            return -1
        else:
            return min(possible_index)

    @staticmethod
    def _consume_after_corruption_detected(buffer):
        u"""
        Gives the number of bytes in the buffer to consume after a corrupt packet was detected.
        :param buffer: buffer object
        :return: Number of bytes to consume in the buffer.
        """
        next_index = UnpackerBase._next_possible_packet_index(buffer)
        if next_index == -1: # no valid start byte was found
            return len(buffer) # consume entire buffer
        else:
            return next_index # consume up to next index

    @staticmethod
    def _packet_size(header):
        return struct.calcsize(Header.fmt(header.payload_index)) + header.payload_length + struct.calcsize(Footer.fmt())

    @staticmethod
    def _packet_parsable(buffer, header):
        u"""
        Checks if an entire packet is parsable.
        :param buffer: buffer object
        :param header: Header object
        :return: True if the current packet is parsable, False otherwise.
        """
        frame_size = UnpackerBase._packet_size(header)
        return len(buffer) >= frame_size

    @staticmethod
    def _unpack_payload(buffer, header):
        u"""
        Unpacks the payload of the packet.
        :param buffer: buffer object
        :param header: Header object
        :return: byte string of the payload
        """
        footer_index = header.payload_index + header.payload_length
        return str(buffer[header.payload_index:footer_index])

    @staticmethod
    def _validate_payload(payload, footer):
        u"""
        Validates the payload using the footer. CorruptPacket is raised if the payload is corrupt or the terminator is
        not correct.
        :param payload: byte string
        :param footer: Footer object
        :return: void
        """
        if CRCCCITT().calculate(payload) != footer.crc:
            raise CorruptPacket(u"Invalid checksum value.")
        if footer.terminator is not Footer.TERMINATOR:
            raise CorruptPacket(u"Invalid terminator: %u" % footer.terminator)
        return

    @staticmethod
    def _unpack(buffer, header, errors, recovery_mode=False):
        u"""
        Attempt to parse a packet from the buffer.
        :param buffer: buffer object
        :param errors: specifies error handling scheme. see codec error handling schemes
        :return: (1) Packet if parse was successful, None otherwise, (2) Length consumed of buffer
        """
        while True:
            try:
                # if we were not given a header then try to parse one
                if header is None:
                    header = UnpackerBase._unpack_header(buffer)
                if header is None:
                    # buffer is too short to parse a header
                    if recovery_mode:
                        return Stateless._recovery_recurse(buffer, header, errors, False)
                    else:
                        return None, 0
                # check if a packet is parsable
                if UnpackerBase._packet_parsable(buffer, header) is False:
                    # buffer is too short to parse the rest of the packet
                    if recovery_mode:
                        return Stateless._recovery_recurse(buffer, header, errors, False)
                    else:
                        return None, 0
                # parse the packet
                payload = UnpackerBase._unpack_payload(buffer, header)
                footer = UnpackerBase._unpack_footer(buffer, header)
                # validate the payload
                UnpackerBase._validate_payload(payload, footer)
                # clean header as we wont need it again
                consumed = UnpackerBase._packet_size(header)
                header = None
                return payload, consumed
            except CorruptPacket, corrupt_packet:
                if errors is u'ignore':
                    # find the next possible start byte in the buffer
                    return Stateless._recovery_recurse(buffer, header, errors, True)
                elif errors is u'strict':
                    raise corrupt_packet

    @staticmethod
    def _recovery_recurse(buffer, header, errors, consume_on_not_recovered):
        header = None  # clean header
        next_sb = UnpackerBase._next_possible_packet_index(buffer)
        if next_sb == -1:  # no valid start byte in buffer. consume entire buffer
            if consume_on_not_recovered:
                return None, len(buffer)
            else:
                return None, 0
        else:
            payload, consumed = UnpackerBase._unpack(buffer[next_sb:], header, errors, True)
            if payload is None:
                # failed to recover
                if consume_on_not_recovered:
                    return payload, consumed + next_sb
                else:
                    return payload, consumed
            else:
                # recovery was successful
                return payload, consumed + next_sb



class PackerBase(object):
    u"""
    Packing is the same for stated and stateless. Therefore its implemented in this base class.
    """
    @staticmethod
    def _pack(payload):
        u"""
        Packs a payload.
        :param payload: byte string of payload
        :return: byte string of packed packet
        """
        if payload == '':
            raise InvalidPayload(u"Empty payload")
        # get header/footer tuples
        header = Header.generate(payload)
        footer = Footer.generate(payload)
        # serialize tuples
        header = struct.pack(Header.fmt(header.payload_index), *header)
        footer = struct.pack(Footer.fmt(), *footer)
        return header + payload + footer


class Stateless(UnpackerBase, PackerBase):
    u"""
    Statelessly pack and unpack VESC packets.
    """
    @staticmethod
    def unpack(buffer, errors=u'ignore'):
        u"""
        Attempt to parse a packet from the buffer.
        :param buffer: buffer object
        :param errors: specifies error handling scheme. see codec error handling schemes
        :return: (1) Packet if parse was successful, None otherwise, (2) Length consumed of buffer
        """
        return Stateless._unpack(buffer, None, errors)

    @staticmethod
    def pack(payload):
        u"""
        See PackerBase.pack
        """
        return Stateless._pack(payload)

def frame(bytestring):
    return Stateless.pack(bytestring)

def unframe(buffer, errors=u'ignore'):
    return Stateless.unpack(buffer, errors)
