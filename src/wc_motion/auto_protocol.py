"""Messages of the automatic velocity channel (docs/architecture/velocity_channel.md, sections 3 and 9).

No ROS and no devices here: only the JSON documents and their validation. Field lists as agreed with window D on
2026-11-09 (D.md 18:45, velocity_channel.md 9.2).

- gate command: the safety gate's decision of every cycle on ``/wc_nav/gate_command`` (``std_msgs/String`` JSON).
  The base bridge accepts nothing else from the navigation side. ``gate_id`` is generated when the gate process
  starts; a new ``gate_id`` or a ``seq`` that does not increase is a new source and ends automatic control.
- hello / decision: base bridge -> wheel serial owner, JSON lines over the session's ``auto.sock``.
- status: wheel serial owner -> base bridge over the same socket, republished on ``/wc_nav/base_status``.
"""
import json
import math
import re

GATE_COMMAND_SCHEMA = 'wc_gate_command_v1'
HELLO_SCHEMA = 'wc_auto_bridge_hello_v1'
STATUS_SCHEMA = 'wc_base_status_v1'
GATE_ACTIVE = 'ACTIVE'
GATE_ESTOP = 'ESTOP_LATCHED'
GATE_LIMIT_FIELDS = ('max_linear', 'max_angular', 'linear_ceiling', 'allow_reverse')
DECISION_STALE_S = 0.3      # a gate decision older than this is not used to move
STATUS_PERIOD_S = 0.1       # owner -> bridge status period
STATUS_STALE_S = 0.3        # bridge treats an older owner status as missing
MAX_LINE_BYTES = 4096
_STATE = re.compile(r'[A-Z][A-Z0-9_]{0,47}')
_SHA256 = re.compile(r'[0-9a-f]{64}')
_IDENTITY = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}')


class ProtocolError(ValueError):
    """A message that must not be acted upon."""


def _reject_constant(value):
    raise ProtocolError('non-finite JSON constant ' + value)


def decode_line(line):
    """One JSON object from one line; NaN/Infinity, duplicates and oversize lines are rejected."""
    if isinstance(line, bytes):
        if len(line) > MAX_LINE_BYTES:
            raise ProtocolError('message line exceeds bound')
        try:
            line = line.decode('utf-8')
        except UnicodeDecodeError as error:
            raise ProtocolError('message is not UTF-8') from error
    if not isinstance(line, str) or len(line.encode('utf-8')) > MAX_LINE_BYTES:
        raise ProtocolError('message line exceeds bound')

    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ProtocolError('duplicate key ' + key)
            value[key] = item
        return value
    try:
        value = json.loads(line, parse_constant=_reject_constant, object_pairs_hook=unique)
    except ValueError as error:
        raise ProtocolError('invalid JSON: ' + str(error)) from error
    if not isinstance(value, dict):
        raise ProtocolError('message must be a JSON object')
    return value


def encode_line(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')


def number(value, name, low=-math.inf, high=math.inf):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or not low <= value <= high:
        raise ProtocolError('%s must be a finite number in [%s, %s]' % (name, low, high))
    return float(value)


def integer(value, name, low=0, high=2 ** 63 - 1):
    if type(value) is not int or not low <= value <= high:
        raise ProtocolError('%s must be an integer in [%s, %s]' % (name, low, high))
    return value


def identity(value, name='identity'):
    if not isinstance(value, str) or not _IDENTITY.fullmatch(value):
        raise ProtocolError('invalid ' + name)
    return value


def session_identity(value):
    return identity(value, 'session identity')


def _limits(value):
    """max_linear and max_angular are the gate's configured limits; linear_ceiling is the linear limit in force now
    (lower while the lateral slow-down applies); allow_reverse must be reported explicitly."""
    if not isinstance(value, dict) or set(value) != set(GATE_LIMIT_FIELDS):
        raise ProtocolError('limits must give exactly ' + ', '.join(GATE_LIMIT_FIELDS))
    result = {'max_linear': number(value['max_linear'], 'limits.max_linear', 0.0, 5.0),
              'max_angular': number(value['max_angular'], 'limits.max_angular', 0.0, 5.0),
              'linear_ceiling': number(value['linear_ceiling'], 'limits.linear_ceiling', 0.0, 5.0)}
    if result['linear_ceiling'] > result['max_linear'] + 1e-9:
        raise ProtocolError('limits.linear_ceiling must not exceed limits.max_linear')
    if type(value['allow_reverse']) is not bool:
        raise ProtocolError('limits.allow_reverse must be a boolean')
    result['allow_reverse'] = value['allow_reverse']
    return result


def _reasons(value):
    if not isinstance(value, list) or len(value) > 16 or \
            any(not isinstance(item, str) or not 0 < len(item) <= 80 for item in value):
        raise ProtocolError('reasons must be a short list of short strings')
    return list(value)


def _decision_fields(value, prefix=''):
    state = value.get('state')
    if not isinstance(state, str) or not _STATE.fullmatch(state):
        raise ProtocolError('invalid gate state')
    sha = value.get('config_sha256')
    if sha is not None and (not isinstance(sha, str) or not _SHA256.fullmatch(sha)):
        raise ProtocolError('config_sha256 must be 64 lowercase hex characters or null')
    return {'gate_id': identity(value.get('gate_id'), 'gate_id'), 'state': state,
            'reasons': _reasons(value.get('reasons')),
            'linear': number(value.get('linear'), prefix + 'linear', -10.0, 10.0),
            'angular': number(value.get('angular'), prefix + 'angular', -10.0, 10.0),
            'limits': _limits(value.get('limits')), 'config_sha256': sha}


GATE_COMMAND_FIELDS = ('schema', 'gate_id', 'seq', 'stamp_ns', 'mono_ns', 'state', 'reasons', 'linear', 'angular',
                       'limits', 'config_sha256')


def parse_gate_command(text):
    """The safety gate's decision. Unknown states are kept; only ``ACTIVE`` may carry a nonzero velocity."""
    value = decode_line(text)
    if value.get('schema') != GATE_COMMAND_SCHEMA or set(value) != set(GATE_COMMAND_FIELDS):
        raise ProtocolError('gate command must be %s with exactly %s' % (GATE_COMMAND_SCHEMA, list(GATE_COMMAND_FIELDS)))
    return {'seq': integer(value['seq'], 'seq'), 'stamp_ns': integer(value['stamp_ns'], 'stamp_ns'),
            'mono_ns': integer(value['mono_ns'], 'mono_ns', 1), **_decision_fields(value)}


def hello_packet(session_id, bridge_id):
    return {'type': 'hello', 'schema': HELLO_SCHEMA, 'session_id': session_identity(session_id),
            'bridge_id': session_identity(bridge_id)}


def check_hello(value, session_id):
    if not isinstance(value, dict) or set(value) != {'type', 'schema', 'session_id', 'bridge_id'} or \
            value.get('type') != 'hello' or value.get('schema') != HELLO_SCHEMA or value.get('session_id') != session_id:
        raise ProtocolError('base bridge hello with this session identity required')
    return session_identity(value['bridge_id'])


def decision_packet(gate, session_id, sequence, received_mono_ns):
    """What the bridge forwards for one gate command: the decision plus the bridge's own sequence and receipt time."""
    return {'type': 'decision', 'session_id': session_identity(session_id), 'sequence': integer(sequence, 'sequence', 1),
            'received_mono_ns': integer(received_mono_ns, 'received_mono_ns', 1), 'gate_id': gate['gate_id'],
            'gate_seq': gate['seq'], 'gate_mono_ns': gate['mono_ns'], 'state': gate['state'],
            'reasons': list(gate['reasons']), 'linear': gate['linear'], 'angular': gate['angular'],
            'limits': dict(gate['limits']), 'config_sha256': gate['config_sha256']}


DECISION_FIELDS = ('type', 'session_id', 'sequence', 'received_mono_ns', 'gate_id', 'gate_seq', 'gate_mono_ns',
                   'state', 'reasons', 'linear', 'angular', 'limits', 'config_sha256')


def parse_decision(value, session_id, last_sequence):
    """Owner side. The bridge sequence must increase strictly; nothing is inferred from a malformed packet."""
    if not isinstance(value, dict) or set(value) != set(DECISION_FIELDS) or value.get('type') != 'decision':
        raise ProtocolError('decision packet must have exactly %s' % list(DECISION_FIELDS))
    if value['session_id'] != session_id:
        raise ProtocolError('decision for another session')
    sequence = integer(value['sequence'], 'sequence', 1)
    if sequence <= last_sequence:
        raise ProtocolError('decision sequence must increase')
    return {'sequence': sequence, 'received_mono_ns': integer(value['received_mono_ns'], 'received_mono_ns', 1),
            'gate_seq': integer(value['gate_seq'], 'gate_seq'), 'gate_mono_ns': integer(value['gate_mono_ns'], 'gate_mono_ns', 1),
            **_decision_fields(value)}


def parse_status(value, session_id):
    """Bridge side: the owner's status line. Only the fields the bridge acts on are checked strictly."""
    if not isinstance(value, dict) or value.get('type') != 'status' or value.get('schema') != STATUS_SCHEMA or \
            value.get('session_id') != session_id:
        raise ProtocolError('owner status of this session required')
    for name in ('dead_man', 'estop'):
        if value.get(name) not in (True, False, None):
            raise ProtocolError(name + ' must be true, false or null')
    if value.get('source') not in ('NONE', 'MANUAL', 'AUTO'):
        raise ProtocolError('unknown control source')
    if not isinstance(value.get('auto_state'), str):
        raise ProtocolError('auto_state must be a string')
    return value
