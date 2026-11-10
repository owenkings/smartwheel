"""Read-only driver readback over socket pairs and a Linux PTY; never a real device."""
import datetime
import json
import os
from pathlib import Path
import socket
import struct
import sys
import threading
from types import SimpleNamespace
import pytest
from wc_motion import driver_readback as dr
from wc_motion import feedback_transport as ft
from wc_motion.protocol import FeedbackError, crc16, read_request

CONFIG_PATH = Path(__file__).resolve().parents[2]/'config/wheel_feedback_current.json'
TODAY = datetime.date(2026, 11, 11)
APPROVED = {dr.APPROVAL_ENV: 'APPROVED_BY_USER_20261111'}


def words(*values, slave=1):
    payload = bytes((slave, 3, 2*len(values)))+b''.join(struct.pack('>H', value) for value in values)
    return payload+crc16(payload)


def exception(code, slave=1):
    payload = bytes((slave, 0x83, code))
    return payload+crc16(payload)


def answers_for(entries, special=None):
    special = special or {}
    return [special.get(start, words(*range(start & 0xff, (start & 0xff)+count))) for start, count, _, _ in entries]


class Driver:
    """Far end of a socket pair: one scripted answer per request; None stays silent."""
    def __init__(self, answers, *, before=b''):
        self.near, self.far = socket.socketpair()
        self.answers, self.requests = list(answers), []
        if before:
            self.far.sendall(before)
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
            answer = self.answers.pop(0) if self.answers else None
            if answer is not None:
                self.far.sendall(answer)

    def lease(self):
        return SimpleNamespace(fd=self.near.fileno(), failed=False)

    def close(self):
        self.near.close()
        self.thread.join(timeout=2)
        self.far.close()


def test_whitelist_is_the_design_list_and_only_exact_fc03_reads_of_slave_1_pass():
    entries = dr.plan()
    assert [(start, count) for start, count, _, _ in entries] == [
        (0x2000, 1), (0x2003, 1), (0x2007, 1), (0x2008, 1), (0x2011, 1), (0x2016, 3),
        (0x2084, 1), (0x2085, 1), (0x20A0, 1), (0x20A5, 1), (0x20A6, 1)]
    assert all(request == read_request(1, start, count) and dr.whitelisted(request) == (start, count)
               for start, count, _, request in entries)
    write = bytes((1, 6, 0x20, 0x0e, 0, 8)); write += crc16(write)
    for refused in (read_request(1, 0x200e, 1), read_request(2, 0x2000, 1), read_request(1, 0x2016, 1),
                    read_request(1, 0x20ab, 2), write):
        with pytest.raises(FeedbackError):
            dr.whitelisted(refused)
    with pytest.raises(FeedbackError, match='200Eh'):
        dr.plan({0x2003, 0x200e})
    assert [entry[0] for entry in dr.plan({0x20a6, 0x2003})] == [0x2003, 0x20a6]


def test_reads_words_records_exceptions_and_sends_each_whitelisted_read_once():
    entries = dr.plan()
    driver = Driver(answers_for(entries, {0x2007: exception(2)}))
    lease, reads = driver.lease(), []
    try:
        assert dr.read_all(lease, entries, .2, reads, sleep=lambda _: None) is None
    finally:
        driver.close()
    assert driver.requests == [entry[3] for entry in entries] and not lease.failed
    by_start = {row['start_register']: row for row in reads}
    assert by_start['2007h']['status'] == 'MODBUS_EXCEPTION' and by_start['2007h']['exception_code'] == 2
    assert by_start['2016h']['words_u16'] == [0x16, 0x17, 0x18] and by_start['2016h']['words_hex'] == ['0016', '0017', '0018']
    assert all(row['status'] == 'WORDS' for name, row in by_start.items() if name != '2007h')
    assert all(row['transmitted_bytes'] == 8 for row in reads)


@pytest.mark.parametrize('second, match', [
    (None, 'timeout'),
    (words(7)[:-1]+bytes((words(7)[-1] ^ 0xff,)), 'CRC'),
    (words(7, slave=2), 'slave'),
    (words(7)+b'\x01', 'overflow'),
    (words(7)[:4], 'timeout'),
])
def test_transport_failure_stops_the_run_without_retry(second, match):
    entries = dr.plan()
    driver = Driver([words(1), second])
    lease, reads = driver.lease(), []
    try:
        error = dr.read_all(lease, entries, .05, reads, sleep=lambda _: None)
    finally:
        driver.close()
    assert error.startswith('2003h') and match in error and lease.failed
    assert [row['status'] for row in reads] == ['WORDS', 'TRANSPORT_FAILED'] + ['NOT_READ']*9
    assert driver.requests == [entries[0][3], entries[1][3]]


def test_unsolicited_bytes_fail_before_anything_is_sent():
    driver = Driver([], before=b'\x01\x03')
    lease, reads = driver.lease(), []
    try:
        for _ in range(100):  # the pending bytes must already be readable
            if dr.select.select([lease.fd], [], [], .01)[0]:
                break
        error = dr.read_all(lease, dr.plan({0x2003}), .2, reads, sleep=lambda _: None)
    finally:
        driver.close()
    assert 'unsolicited' in error and reads[0]['transmitted_bytes'] == 0 and driver.requests == []


@pytest.fixture
def no_port(monkeypatch):
    monkeypatch.setattr(dr, 'FeedbackSerialLease', lambda *args: pytest.fail('serial lease constructed'))
    monkeypatch.setattr(dr, 'report_folder', lambda *args: pytest.fail('report folder created'))


def cli(root, *extra, environ=None):
    return dr.main(['--project-root', str(root), '--label', 'estop_released', '--config', str(CONFIG_PATH), *extra],
                   environ={} if environ is None else environ, today=TODAY)


def test_dry_run_missing_or_stale_approval_and_other_registers_never_open_the_port(tmp_path, no_port, capsys):
    assert cli(tmp_path, '--dry-run') == 0
    printed = capsys.readouterr().out
    assert '01 03 20 03 00 01' in printed and 'nothing sent' in printed and printed.count(' x') == 11
    assert cli(tmp_path) == 2
    assert cli(tmp_path, environ={dr.APPROVAL_ENV: 'APPROVED_BY_USER_20261110'}) == 2
    assert 'APPROVED_BY_USER_20261111' in capsys.readouterr().out
    with pytest.raises(FeedbackError, match='200Eh'):
        cli(tmp_path, '--registers', '200E', environ=APPROVED)
    with pytest.raises(SystemExit):
        dr.main(['--project-root', str(tmp_path), '--label', '../x', '--config', str(CONFIG_PATH)])


class FakeLease:
    def __init__(self, config, run_root, *, driver=None, refusal=None, lock=None):
        self.config, self.run_root, self.driver, self.refusal, self.lock = config, run_root, driver, refusal, lock
        self.fd, self.failed, self.host_configuration, self.closed = None, False, None, False

    def open(self):
        if self.refusal:
            raise FeedbackError(self.refusal)
        self.fd, self.host_configuration = self.driver.near.fileno(), {'device': 'socketpair'}
        return self

    def close(self):
        self.fd, self.closed = None, True


def test_cli_writes_a_result_for_a_refused_port_and_for_a_complete_run(tmp_path, monkeypatch):
    folders = []
    def folder(root, label):
        path = tmp_path/('run%d_%s' % (len(folders), label))
        path.mkdir()
        folders.append(path)
        return path
    monkeypatch.setattr(dr, 'report_folder', folder)
    monkeypatch.setattr(dr, 'REQUEST_GAP_S', 0)
    leases = []
    monkeypatch.setattr(dr, 'FeedbackSerialLease', lambda *args: leases.append(
        FakeLease(*args, refusal='serial owner appeared; panel holds the port')) or leases[-1])
    assert cli(tmp_path, environ=APPROVED) == 1
    refused = json.loads((folders[0]/'result.json').read_text(encoding='utf-8'))
    assert refused['status'] == 'PORT_REFUSED' and 'panel holds' in refused['error']
    assert refused['reads'] == [] and refused['function_codes_sent'] == [] and leases[0].closed
    entries = dr.plan()
    driver = Driver(answers_for(entries, {0x20a6: exception(2)}))
    lock = (tmp_path/'owner.lock').open('w+', encoding='utf-8')
    monkeypatch.setattr(dr, 'FeedbackSerialLease', lambda *args: leases.append(
        FakeLease(*args, driver=driver, lock=lock)) or leases[-1])
    try:
        assert cli(tmp_path, environ=APPROVED) == 0
    finally:
        driver.close()
    result = json.loads((folders[1]/'result.json').read_text(encoding='utf-8'))
    assert result['status'] == 'COMPLETE' and result['error'] is None and result['approval'] == 'APPROVED_BY_USER_20261111'
    assert result['function_codes_sent'] == [3] and result['control_transmissions'] == 0 and len(result['reads']) == 11
    assert result['reads'][-1]['status'] == 'MODBUS_EXCEPTION' and result['device_id'] == leases[1].config['device_id']
    assert driver.requests == [entry[3] for entry in entries] and leases[1].closed
    lock.seek(0)
    note = json.loads(lock.read())
    assert note['purpose'] == 'driver_readback_fc03_only' and len(note['request_hex']) == 11
    lock.close()


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux PTY/termios acceptance, no real serial device')
def test_linux_lease_carries_only_the_selected_fc03_reads(tmp_path):
    import pty
    import select
    config = {**json.loads(CONFIG_PATH.read_text(encoding='utf-8')), 'startup_settle_s': 0}
    master, slave = pty.openpty()
    device = Path(os.ttyname(slave))
    identity = device.stat()
    lease = ft.FeedbackSerialLease(config, tmp_path, identity_checker=lambda _: (device, identity),
                                   occupancy_checker=lambda _: None, exclusive_checker=lambda _: None)
    entries = dr.plan({0x2003, 0x2016})
    seen = []
    def reply():
        for answer in (words(0x0005), words(1, 2, 3)):
            if select.select([master], [], [], 1)[0]:
                seen.append(os.read(master, 64))
                os.write(master, answer)
    try:
        lease.open()
        assert not select.select([master], [], [], .03)[0], 'opening wrote to the motor bus'
        thread = threading.Thread(target=reply)
        thread.start()
        reads = []
        assert dr.read_all(lease, entries, .2, reads, sleep=lambda _: None) is None
        thread.join(timeout=2)
        assert seen == [entry[3] for entry in entries]
        assert [row['words_u16'] for row in reads] == [[5], [1, 2, 3]]
        lease.close()
        assert not select.select([master], [], [], .03)[0], 'closing wrote to the motor bus'
    finally:
        lease.close()
        os.close(master)
        os.close(slave)
