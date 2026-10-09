"""Strict Modbus RTU FC03 feedback decoding with externally verified mapping.

The ZLAC8030D Version 1.00 user manual establishes Modbus RTU, but does NOT
publish the feedback register table/units. No 8015D register map is substituted.
FC03 responses do not contain their register address: a matched request is
mandatory, including for passive captures. This module never opens a device.
"""

from dataclasses import dataclass
import math
import struct


class FeedbackError(ValueError):
    pass


def integer(value, name, lower=0, upper=None):
    if isinstance(value, bool) or not isinstance(value, int) or value < lower or (upper is not None and value > upper):
        raise FeedbackError(f'{name} must be integer in [{lower}, {upper}]')
    return value


def crc16(payload):
    crc = 0xffff
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return struct.pack('<H', crc)


def checked_frame(frame):
    frame = bytes(frame)
    if not 5 <= len(frame) <= 256 or crc16(frame[:-2]) != frame[-2:]:
        raise FeedbackError('invalid RTU length or CRC')
    return frame


def read_request(slave, address, count):
    """Build FC03 only; caller cannot select a write function code."""
    integer(slave, 'slave', 1, 247)
    integer(address, 'address', 0, 65535)
    integer(count, 'count', 1, 125)
    if address + count > 65536:
        raise FeedbackError('register range overflow')
    payload = struct.pack('>BBHH', slave, 3, address, count)
    return payload + crc16(payload)


def parse_exchange(request, response):
    request, response = checked_frame(request), checked_frame(response)
    if len(request) != 8 or request[1] != 3 or not 1 <= request[0] <= 247:
        raise FeedbackError('only unicast FC03 read requests are accepted')
    slave, _, address, count = struct.unpack('>BBHH', request[:-2])
    if request != read_request(slave, address, count):
        raise FeedbackError('invalid FC03 request')
    if response[0] != slave:
        raise FeedbackError('response slave mismatch')
    if response[1] == 0x83:
        raise FeedbackError(f'Modbus read exception {response[2]}')
    if response[1] != 3 or response[2] != count * 2 or len(response) != 5 + count * 2:
        raise FeedbackError('response function/byte count mismatch')
    return slave, address, struct.unpack('>' + 'H' * count, response[3:-2])


@dataclass(frozen=True)
class Field:
    address: int
    words: int
    signed: bool
    word_order: str
    scale: float
    unit: str

    @classmethod
    def from_dict(cls, data):
        result = cls(**data)
        integer(result.address, 'field address', 0, 65535)
        if result.words not in (1, 2) or isinstance(result.words, bool) or result.address + result.words > 65536:
            raise FeedbackError('field must contain one or two registers')
        if type(result.signed) is not bool or result.word_order not in ('big', 'little'):
            raise FeedbackError('explicit signedness and word_order required')
        if not isinstance(result.scale, (float, int)) or isinstance(result.scale, bool) or not math.isfinite(result.scale) or result.scale <= 0:
            raise FeedbackError('positive finite unit scale required')
        if result.unit not in ('motor_rpm', 'motor_counts'):
            raise FeedbackError('unknown feedback unit')
        if result.unit == 'motor_counts' and result.scale != 1:
            raise FeedbackError('position counts must remain exact integers; scale must be one')
        return result

    def decode(self, start, values):
        offset = self.address - start
        if offset < 0 or offset + self.words > len(values):
            raise FeedbackError('reply does not contain both complete wheel fields')
        words = values[offset:offset + self.words]
        if self.word_order == 'little':
            words = tuple(reversed(words))
        value = int.from_bytes(b''.join(struct.pack('>H', w) for w in words), 'big', signed=self.signed)
        return value if self.unit == 'motor_counts' else value * self.scale


class FeedbackProfile:
    def __init__(self, config):
        if config.get('protocol_verified') is not True or not config.get('protocol_evidence'):
            raise FeedbackError('confirmed model-specific protocol evidence is required')
        if config.get('model') != 'ZLAC8030D':
            raise FeedbackError('only an explicitly confirmed ZLAC8030D profile is accepted')
        self.slave = integer(config.get('slave_id'), 'slave_id', 1, 247)
        self.device_id = config.get('device_id')
        if not isinstance(self.device_id, str) or not self.device_id:
            raise FeedbackError('unique physical device_id required')
        self.fields = {name: Field.from_dict(value) for name, value in config.get('fields', {}).items()}
        if set(self.fields) not in ({'left_velocity', 'right_velocity'}, {'left_position', 'right_position'},
                                    {'left_position', 'right_position', 'left_velocity', 'right_velocity'}):
            raise FeedbackError('matching left/right position and/or velocity fields required')
        for name, field in self.fields.items():
            if field.unit != ('motor_rpm' if name.endswith('velocity') else 'motor_counts'):
                raise FeedbackError('field/unit mismatch')
        if 'left_position' in self.fields and self.fields['left_position'].words != self.fields['right_position'].words:
            raise FeedbackError('left/right count widths must match')
        self.allowed_reads = tuple((integer(x[0], 'read start', 0, 65535), integer(x[1], 'read count', 1, 125))
                                   for x in config.get('allowed_reads', []))
        if not self.allowed_reads:
            raise FeedbackError('explicit read-only register whitelist required')
        for address, count in self.allowed_reads:
            read_request(self.slave, address, count)

    def decode(self, record):
        if record.get('schema') != 'wc_wheel_feedback_v1' or record.get('device_id') != self.device_id:
            raise FeedbackError('schema or physical device identity mismatch')
        try:
            request, response = bytes.fromhex(record['request_hex']), bytes.fromhex(record['response_hex'])
        except (KeyError, TypeError, ValueError) as error:
            raise FeedbackError('matched request/response hex required') from error
        slave, start, words = parse_exchange(request, response)
        if slave != self.slave or (start, len(words)) not in self.allowed_reads:
            raise FeedbackError('exchange outside verified read-only whitelist')
        values = {name: field.decode(start, words) for name, field in self.fields.items()}
        return values

    def validated_query(self, request):
        frame = checked_frame(request)
        if len(frame) != 8 or frame[1] != 3:
            raise FeedbackError('only FC03 queries allowed')
        slave, _, address, count = struct.unpack('>BBHH', frame[:-2])
        if slave != self.slave or (address, count) not in self.allowed_reads or frame != read_request(slave, address, count):
            raise FeedbackError('query outside verified whitelist')
        return frame


class ReadOnlyFeedbackClient:
    """Optional query client for an already leased transport; disabled by default.

    Transmission is limited to a configured, evidence-backed FC03 whitelist.
    The ROS node never constructs this client. No retries or reconnects occur.
    Transport must supply write(bytes), read(count); no motor-control API exists.
    """
    def __init__(self, profile, transport, *, allow_read_queries=False):
        self.profile, self.transport, self.allow = profile, transport, allow_read_queries

    def read_feedback(self, address, count):
        if self.allow is not True:
            raise FeedbackError('active read queries disabled')
        request = self.profile.validated_query(read_request(self.profile.slave, address, count))
        if self.transport.write(request) != len(request):
            raise FeedbackError('short FC03 request transmission; no retry')
        response = bytearray()
        expected = 5 + count * 2
        while len(response) < expected:
            chunk = self.transport.read(expected - len(response))
            if not chunk:
                raise FeedbackError('read timeout/EOF; no retry')
            response.extend(chunk)
            if len(response) >= 2 and response[1] == 0x83:
                expected = 5
            if len(response) > expected:
                raise FeedbackError('response overflow')
        parse_exchange(request, response)
        return request, bytes(response)
