"""Actual identity snapshot via inert descriptor/udev substitutes; no hardware."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_runtime import ultrasonic_capture as module


@pytest.fixture
def identity_context(monkeypatch):
    config=dict(device='/dev/synthetic_alias',expected_by_id='/dev/serial/by-id/synthetic',
                expected_usb_path='synthetic-port',usb_vid='1a86',usb_pid='7523')
    descriptor=SimpleNamespace(st_rdev=123,st_dev=456,st_ino=789)
    properties={'ID_VENDOR_ID':'1a86','ID_MODEL_ID':'7523','ID_PATH':'synthetic-port'}
    monkeypatch.setattr(module,'identity',lambda value:(Path('/dev/synthetic_target'),descriptor))
    monkeypatch.setattr(module.os,'fstat',lambda fd:descriptor)
    calls=[]
    def enumeration(command,**kwargs):
        calls.append(command)
        return '\n'.join(key+'='+value for key,value in properties.items())
    monkeypatch.setattr(module.subprocess,'check_output',enumeration)
    return config,SimpleNamespace(fd=99),descriptor,properties,calls


def test_snapshot_uses_current_udev_values_and_open_descriptor(identity_context):
    config,lease,descriptor,properties,calls=identity_context
    result=module.observed_identity(config,lease,'synthetic')
    assert calls==[['udevadm','info','--query=property','--name='+str(Path('/dev/synthetic_target'))]]
    assert result['udev_properties']==properties
    assert result['opened_identity']==vars(descriptor)
    assert result['resolved_device']==str(Path('/dev/synthetic_target'))
    assert result['verified_monotonic_ns']>0 and result['verified_wall_ns']>0
    assert result['session_id']=='synthetic' and result['physical_role_status']=='UNVERIFIED'
    assert result['evidence_basis']=='CURRENT_UDEV_AND_OPEN_DESCRIPTOR_IDENTITY'


@pytest.mark.parametrize('property_name',['ID_VENDOR_ID','ID_MODEL_ID','ID_PATH'])
def test_identity_change_during_snapshot_is_rejected(identity_context,property_name):
    config,lease,descriptor,properties,calls=identity_context
    properties[property_name]='foreign'
    with pytest.raises(RuntimeError,match='MISMATCH'):module.observed_identity(config,lease,'synthetic')


def test_open_descriptor_change_is_rejected_before_enum(identity_context,monkeypatch):
    config,lease,descriptor,properties,calls=identity_context
    monkeypatch.setattr(module.os,'fstat',lambda fd:SimpleNamespace(st_rdev=999,st_dev=456,st_ino=789))
    with pytest.raises(RuntimeError,match='opened identity changed'):module.observed_identity(config,lease,'synthetic')
    assert not calls


def test_closed_lease_cannot_self_certify_identity_from_config(identity_context):
    config,lease,*_=identity_context
    lease.fd=None
    with pytest.raises(RuntimeError,match='open unique lease'):module.observed_identity(config,lease,'synthetic')
