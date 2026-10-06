#!/usr/bin/env python3
"""Read process identities and device owners; never signal or open a sensor."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from wc_runtime.cli import ROOT, RUN, target
from wc_runtime.component import process
from wc_runtime.storage_policy import StoragePolicy


def main():
    target()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    storage=StoragePolicy(ROOT)
    args.output=storage.resolve(args.output)
    if not args.output.is_relative_to(storage.resolve('reports')) or args.output.exists():
        raise ValueError('new project report path required')
    records=[]
    for path in sorted((RUN/'sessions').glob('*/*/manifest.json')):
        data=json.loads(path.read_text())
        ids=[{'pid':data.get('supervisor_pid'),'start_ticks':data.get('supervisor_start_ticks')},*data.get('children',[])]
        for item in ids:
            if not item.get('pid'):continue
            current=process(item['pid'])
            if current and current.start_ticks==item.get('start_ticks') and current.state!='Z':
                records.append({'manifest':str(path.relative_to(ROOT)),'pid':current.pid,'state':current.state})
    owners={}
    paths=[Path('/dev/smartwheel_h30_imu'),Path('/dev/smartwheel_zlac8030')]
    paths+=sorted(Path('/dev/v4l/by-path').glob('platform-3610000.usb-usb-0:3.*:1.0-video-index0'))
    for path in paths:
        if not path.exists():owners[str(path)]={'state':'MISSING'};continue
        result=subprocess.run(['fuser',str(path.resolve())],capture_output=True,text=True,timeout=5)
        owners[str(path)]={'exit_code':result.returncode,'pids':result.stdout.strip(),'stderr':result.stderr.strip()}
    data={'observed_ns':time.time_ns(),'host':os.uname().nodename,'root':str(ROOT),
        'registered_live_processes':records,'device_owners':owners,
        'udp_7687':subprocess.check_output(['ss','-H','-lun','sport = :7687'],text=True).strip(),
        'processes_signalled':False,'devices_opened':False,'motor_commands_sent':False}
    storage.check()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(data,indent=2))
    print(json.dumps(data,indent=2))


if __name__=='__main__':main()
