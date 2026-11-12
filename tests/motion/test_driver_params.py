"""Driver parameter writes into RAM over a socket pair that holds register words; never a real device."""
import datetime
import json
from pathlib import Path
import socket
import struct
import threading

import pytest

from wc_motion import driver_params as dp
from wc_motion import driver_readback as dr
from wc_motion.protocol import FeedbackError, crc16, read_request

CONFIG_PATH = Path(__file__).resolve().parents[2]/'config/wheel_feedback_current.json'
DEVICE_ID = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))['device_id']
TODAY = datetime.date(2026, 11, 12)
APPROVED = {dp.APPROVAL_ENV: 'APPROVED_BY_USER_20261112'}


def frame(*parts):
    payload = bytes(parts)
    return payload + crc16(payload)


def memory(**changes):
    """Both motor blocks alike, then ``changes`` (register name as r2064 etc. -> word)."""
    words = {0x2030 + offset: 100 + offset for offset in range(17)}
    words.update({0x2060 + offset: 100 + offset for offset in range(17)})
    words.update({0x2080 + offset: 300 + offset // 2 for offset in range(8)})
    words.update({0x20AB: 0, 0x20AC: 0})
    words.update({int(name[1:], 16): value for name, value in changes.items()})
    return words


class RegisterDriver:
    """Far end of a socket pair: FC03 reads the words, FC06 stores and echoes (or refuses, or ignores)."""
    def __init__(self, words, *, refuse=(), ignore=(), silent=()):
        self.words, self.refuse, self.ignore, self.silent = dict(words), set(refuse), set(ignore), set(silent)
        self.near, self.far = socket.socketpair()
        self.requests = []
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        while True:
            try:
                request = self.far.recv(64)
            except OSError:
                return
            if not request:
                return
            self.requests.append(request)
            start, value = struct.unpack('>HH', request[2:6])
            if request[1] == 3:
                data = b''.join(struct.pack('>H', self.words.get(start + i, 0)) for i in range(value))
                self.far.sendall(frame(1, 3, len(data), *data))
            elif start in self.refuse:
                self.far.sendall(frame(1, 0x86, 3))
            elif start in self.silent:
                self.words[start] = value          # applied, but the echo never comes
            else:
                if start not in self.ignore:
                    self.words[start] = value
                self.far.sendall(request)

    def writes(self):
        return [struct.unpack('>HH', request[2:6]) for request in self.requests if request[1] == 6]

    def close(self):
        self.near.close()
        self.thread.join(timeout=2)
        self.far.close()


class FakeLease:
    def __init__(self, config, run_root, *, driver=None, lock=None):
        self.config, self.driver, self.lock = config, driver, lock
        self.fd, self.failed, self.host_configuration, self.closed = None, False, None, False

    def open(self):
        self.fd, self.host_configuration = self.driver.near.fileno(), {'device': 'socketpair'}
        return self

    def close(self):
        self.fd, self.closed = None, True


@pytest.fixture
def bench(tmp_path, monkeypatch):
    """main() against a RegisterDriver: bench(words, *argv, answer=True, **driver options) -> (code, result, driver)."""
    folders, drivers = [], []
    def folder(root, label):
        path = tmp_path/('run%d_%s' % (len(folders), label))
        path.mkdir()
        folders.append(path)
        return path
    monkeypatch.setattr(dp, 'report_folder', folder)
    monkeypatch.setattr(dr, 'REQUEST_GAP_S', 0)
    lock = (tmp_path/'owner.lock').open('w+', encoding='utf-8')
    asked = []

    def run(words, *argv, answer=True, environ=APPROVED, **options):
        driver = RegisterDriver(words, **options)
        drivers.append(driver)
        monkeypatch.setattr(dp, 'FeedbackSerialLease', lambda *args: FakeLease(*args, driver=driver, lock=lock))
        def confirm(prompt):
            asked.append(prompt)
            return answer
        try:
            code = dp.main([argv[0], '--project-root', str(tmp_path), '--config', str(CONFIG_PATH), *argv[1:]],
                           environ=environ, today=TODAY, confirm=confirm)
        finally:
            driver.close()
        result = json.loads((folders[-1]/'result.json').read_text(encoding='utf-8')) if folders else None
        return code, result, driver
    run.folders, run.asked = folders, asked
    yield run
    lock.close()


def test_only_the_listed_per_wheel_registers_are_writable_and_never_2010h():
    assert sorted(dp.REGISTERS) == [0x2033, 0x2034, 0x2037, 0x203C, 0x203D, 0x2063, 0x2064, 0x2067, 0x206C,
                                    0x206D, 0x2080, 0x2081, 0x2082, 0x2083]
    assert dp.write_request(0x206D, 113) == frame(1, 6, 0x20, 0x6D, 0, 113)
    assert dp.checked_write(dp.write_request(0x2034, 500)) == (0x2034, 500)
    # 2010h saves to EEPROM; 2031h/2061h are the hall offsets; 2088h the speed command; 200Eh the control word
    for register in (0x2010, 0x2031, 0x2061, 0x2088, 0x200E, 0x2084):
        with pytest.raises(FeedbackError, match='not a writable'):
            dp.write_request(register, 1)
    for refused in (frame(1, 6, 0x20, 0x10, 0, 1), frame(2, 6, 0x20, 0x34, 0, 1), read_request(1, 0x2034, 1),
                    frame(1, 0x10, 0x20, 0x34, 0, 1, 2, 0, 1)):
        with pytest.raises(FeedbackError):
            dp.checked_write(refused)
    with pytest.raises(FeedbackError):
        dp.write_request(0x2034, 0x10000)


def test_plan_copies_only_differing_items_and_keeps_given_values_near_the_read():
    before = memory(r2064=60, r206D=300, r2083=310)
    plan = dp.plan_writes(before, copy='left-to-right')
    assert [(s['register'], s['before'], s['value'], s['wheel']) for s in plan] == [
        ('2064h', 60, 104, 'right'), ('206Dh', 300, 113, 'right'), ('2083h', 310, 301, 'right')]
    assert plan[0]['why'] == 'copy from the left wheel'
    assert [s['register'] for s in dp.plan_writes(before, copy='right-to-left', items=['maximum_current'])] == ['2034h']
    assert dp.plan_writes(memory(), copy='left-to-right') == []
    given = dp.plan_writes(before, sets=['left.velocity_loop_ki=90', 'right.velocity_loop_kp=112'],
                           reason='less integral windup on the slow turn')
    assert [(s['register'], s['value'], s['why']) for s in given] == [
        ('203Dh', 90, 'less integral windup on the slow turn')]       # 206Ch already reads 112
    with pytest.raises(FeedbackError, match='0.5-1.5 times'):
        dp.plan_writes(before, sets=['right.maximum_current=200'], reason='more torque')
    with pytest.raises(FeedbackError, match='needs --reason'):
        dp.plan_writes(before, sets=['right.maximum_current=70'])
    with pytest.raises(FeedbackError, match='twice'):
        dp.plan_writes(before, copy='left-to-right', sets=['right.maximum_current=70'], reason='x')
    with pytest.raises(FeedbackError):
        dp.parse_set('right.hall_offset=3')
    with pytest.raises(FeedbackError):
        dp.plan_writes(before, copy='left-to-right', items=['hall_offset'])


def test_revert_plan_puts_back_only_what_still_holds_the_written_value():
    previous = {'schema': dp.SCHEMA, 'mode': 'write', 'writes': [
        dict(item='maximum_current', wheel='right', register='2064h', before=60, value=104, transmitted_bytes=8),
        dict(item='velocity_loop_ki', wheel='right', register='206Dh', before=300, value=113, transmitted_bytes=8),
        dict(item='deceleration_time', wheel='right', register='2083h', before=310, value=301, transmitted_bytes=0)]}
    now = memory(r206D=300)          # 2064h still holds the write, 206Dh is back already (power cycle)
    assert [(s['register'], s['before'], s['value']) for s in dp.revert_plan(previous, now, 'run1')] == [
        ('2064h', 104, 60)]
    with pytest.raises(FeedbackError, match='2064h is 77'):
        dp.revert_plan(previous, memory(r2064=77, r206D=300))
    with pytest.raises(FeedbackError, match='write run'):
        dp.revert_plan({'schema': dp.SCHEMA, 'mode': 'revert'}, now)


def test_write_reads_still_wheels_asks_writes_fc06_and_reads_back(bench, capsys):
    code, result, driver = bench(memory(r2064=60, r206D=300), 'write', '--label', 'right_like_left',
                                 '--copy', 'left-to-right')
    assert code == 0 and result['status'] == 'COMPLETE' and result['confirmed'] and len(bench.asked) == 1
    assert driver.writes() == [(0x2064, 104), (0x206D, 113)]
    assert driver.words[0x2064] == 104 and driver.words[0x206D] == 113
    # reads before (three parameter blocks and the speed), the writes, the blocks again; nothing else
    starts = [(request[1], struct.unpack('>H', request[2:4])[0]) for request in driver.requests]
    assert starts == [(3, 0x2030), (3, 0x2060), (3, 0x2080), (3, 0x20AB), (6, 0x2064), (6, 0x206D),
                      (3, 0x2030), (3, 0x2060), (3, 0x2080)]
    assert result['verification'] == [] and result['function_codes_sent'] == [3, 6]
    assert result['parameter_writes_sent'] == 2 and result['eeprom_save_written'] is False
    assert result['speed_words'] == [0, 0] and result['device_id'] == DEVICE_ID
    assert [row['status'] for row in result['writes']] == ['ACKED', 'ACKED']
    printed = capsys.readouterr().out
    assert 'right maximum current' in printed and '60 -> 104' in printed
    assert 'every write reads back, nothing else changed' in printed
    assert 'undo: WC_DRIVER_WRITE_APPROVAL=APPROVED_BY_USER_20261112 python3 -m wc_motion.driver_params revert' \
        in printed


def test_nothing_is_written_without_yes_while_a_wheel_turns_or_when_nothing_differs(bench):
    code, result, driver = bench(memory(r2064=60), 'write', '--label', 'x', '--copy', 'left-to-right',
                                 answer=False)
    assert code == 1 and result['status'] == 'NOT_CONFIRMED' and driver.writes() == []
    assert result['function_codes_sent'] == [3] and driver.words[0x2064] == 60
    asked = len(bench.asked)
    code, result, driver = bench(memory(r2064=60, r20AC=0xFFFB), 'write', '--label', 'x', '--copy', 'left-to-right')
    assert code == 1 and result['status'] == 'NOT_STANDING_STILL' and result['speed_words'] == [0, -5]
    assert driver.writes() == [] and len(bench.asked) == asked     # not even asked
    code, result, driver = bench(memory(), 'write', '--label', 'x', '--copy', 'left-to-right')
    assert code == 0 and result['status'] == 'NOTHING_TO_WRITE' and driver.writes() == []
    code, result, driver = bench(memory(), 'write', '--label', 'x', '--set', 'right.maximum_current=500',
                                 '--reason', 'more torque')
    assert code == 1 and result['status'] == 'PLAN_REFUSED' and '0.5-1.5 times' in result['error']


def test_a_refused_write_stops_the_rest_and_a_write_that_does_not_hold_fails_the_check(bench):
    code, result, driver = bench(memory(r2064=60, r206D=300, r2083=310), 'write', '--label', 'x',
                                 '--copy', 'left-to-right', refuse={0x206D})
    assert code == 1 and result['status'] == 'WRITE_FAILED' and 'exception 3' in result['error']
    assert [row['status'] for row in result['writes']] == ['ACKED', 'MODBUS_EXCEPTION', 'NOT_SENT']
    assert driver.writes() == [(0x2064, 104), (0x206D, 113)] and result['verification'] == []
    code, result, driver = bench(memory(r2064=60), 'write', '--label', 'x', '--copy', 'left-to-right',
                                 ignore={0x2064})
    assert code == 1 and result['status'] == 'VERIFY_FAILED'
    assert result['verification'] == ['2064h reads 60, expected 104']
    # no answer at all: the line is not used again, and revert still knows the write was sent
    code, result, driver = bench(memory(r2064=60, r206D=300), 'write', '--label', 'x', '--copy', 'left-to-right',
                                 silent={0x2064})
    assert code == 1 and result['status'] == 'WRITE_FAILED' and 'line not used again' in result['error']
    assert [row['status'] for row in result['writes']] == ['TRANSPORT_FAILED', 'NOT_SENT']
    assert driver.requests[-1][1] == 6 and 'after' not in result and result['parameter_writes_sent'] == 1
    assert [(s['register'], s['value']) for s in dp.revert_plan(result, driver.words)] == [('2064h', 60)]


def test_revert_writes_back_the_values_read_before_that_run(bench, capsys):
    code, result, driver = bench(memory(r2064=60, r206D=300), 'write', '--label', 'right_like_left',
                                 '--copy', 'left-to-right')
    assert code == 0 and 'undo:' in capsys.readouterr().out
    run = bench.folders[-1]
    code, result, driver = bench(driver.words, 'revert', '--run', str(run))
    assert code == 0 and result['status'] == 'COMPLETE' and result['mode'] == 'revert'
    assert result['label'] == 'revert_right_like_left' and result['request'] == {'run': str(run)}
    assert driver.writes() == [(0x2064, 60), (0x206D, 300)] and result['verification'] == []
    assert 'undo:' not in capsys.readouterr().out
    code, result, driver = bench(memory(r2064=77, r206D=113), 'revert', '--run', str(run/'result.json'))
    assert code == 1 and result['status'] == 'PLAN_REFUSED' and '2064h is 77' in result['error']
    assert driver.writes() == []


def test_no_approval_bad_options_and_plan_from_a_readback_never_open_the_port(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(dp, 'FeedbackSerialLease', lambda *args: pytest.fail('serial lease constructed'))
    monkeypatch.setattr(dp, 'report_folder', lambda *args: pytest.fail('report folder created'))
    base = ['write', '--project-root', str(tmp_path), '--config', str(CONFIG_PATH), '--label', 'x']
    assert dp.main(base + ['--copy', 'left-to-right'], environ={}, today=TODAY) == 2
    assert dp.main(base + ['--copy', 'left-to-right'], environ={dp.APPROVAL_ENV: 'APPROVED_BY_USER_20261111'},
                   today=TODAY) == 2
    assert 'APPROVED_BY_USER_20261112' in capsys.readouterr().out
    for bad in ([], ['--set', 'right.maximum_current=70'], ['--items', 'maximum_current'],
                ['--copy', 'left-to-right', '--set', 'right.hall_offset=1', '--reason', 'x']):
        with pytest.raises(SystemExit):
            dp.main(base + bad, environ=APPROVED, today=TODAY)
    words = memory(r2064=60)
    reads = [{'start_register': '%04Xh' % start, 'status': 'WORDS',
              'words_u16': [words[start + i] for i in range(count)]} for start, count, _ in dp.PARAMETER_READS]
    (tmp_path/'readback').mkdir()
    (tmp_path/'readback/result.json').write_text(json.dumps({'schema': dr.SCHEMA, 'device_id': DEVICE_ID,
                                                             'reads': reads}), encoding='utf-8')
    assert dp.main(base + ['--copy', 'left-to-right', '--plan-from', str(tmp_path/'readback')], environ={},
                   today=TODAY) == 0
    printed = capsys.readouterr().out
    assert '60 -> 104' in printed and '1 writes; nothing sent' in printed
    assert dp.main(base + ['--set', 'right.maximum_current=500', '--reason', 'x', '--plan-from',
                           str(tmp_path/'readback')], environ={}, today=TODAY) == 2
