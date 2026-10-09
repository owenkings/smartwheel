"""Exercise actual event-mode main loop using inert ROS and serial doubles."""
import sys
from types import SimpleNamespace as NS
import pytest

from test_device_timing import packet, message, write_profile
from wc_imu import ros_node as imu


@pytest.mark.parametrize('late_reset',[False,True])
def test_event_wait_and_bounded_quarantine_preserve_batches_without_late_reset(tmp_path,monkeypatch,late_reset):
    policy,_=write_profile(tmp_path)
    state=NS(ok=True,nodes=[],events=[],closed=False,reads=0,clock=1000000000,journal=[])
    class Publisher:
        def __init__(self): self.messages=[]
        def publish(self,msg): self.messages.append(msg)
    class Node:
        def __init__(self,name):
            self.publishers={};self.timers=[];state.nodes.append(self)
        def get_parameter(self,name): return NS(value=False)
        def create_publisher(self,kind,name,qos):
            pub=Publisher();self.publishers[name]=pub;return pub
        def create_timer(self,seconds,callback): self.timers.append(seconds)
        def get_logger(self): return NS(error=lambda msg:state.events.append(('error',msg)))
        def get_clock(self): return NS(now=lambda:NS(to_msg=lambda:NS(sec=0,nanosec=0)))
        def destroy_node(self): pass
    class Serial:
        def __init__(self,*args): pass
        def open(self): return self
        def wait_readable(self,timeout):
            assert 0 < timeout <= .01
            state.events.append('wait');return True
        def read_available(self):
            state.events.append('read')
            values=[(0,packet(1,601,200)+packet(1000000,1000600,201)),
                (249000000,packet(1005000,1005600,202)),
                (250000000,packet(1010000,1010600,203)+packet(1015000,1015600,204)),
                (255000000,packet(0,600,205))]
            offset,data=values[state.reads];state.reads+=1;state.clock=1000000000+offset
            return data
        def close(self): state.closed=True
    def spin_once(node,**kwargs):
        assert kwargs['timeout_sec']==0
        state.events.append('spin')
        if state.reads==3 and not late_reset: node.done=True
    class Journal:
        def __init__(self,*args): pass
        def append(self,row): state.journal.append(row)
        def close(self,**kwargs): state.journal_error=kwargs['source_error']
    modules={
        'rclpy':NS(init=lambda **kw:None,ok=lambda:state.ok,spin_once=spin_once,shutdown=lambda:None),
        'rclpy.node':NS(Node=Node),'rclpy.qos':NS(qos_profile_sensor_data='SYN'),
        'rclpy.duration':NS(Duration=NS),'rclpy.signals':NS(SignalHandlerOptions=NS(NO='NO')),
        'sensor_msgs.msg':NS(Imu=object),
        'diagnostic_msgs.msg':NS(DiagnosticArray=lambda:NS(header=NS(stamp=None,frame_id='')),
            DiagnosticStatus=type('Status',(NS,),{'ERROR':2,'WARN':1}),KeyValue=NS),
        'wc_interfaces.msg':NS(H30Frame=message),
    }
    for key,value in modules.items(): monkeypatch.setitem(sys.modules,key,value)
    monkeypatch.setattr(imu,'ReadOnlySerialLease',Serial)
    from wc_runtime import source_archive
    monkeypatch.setattr(source_archive,'SourceJournal',Journal)
    monkeypatch.setattr(imu.time,'monotonic_ns',lambda:state.clock)
    monkeypatch.setattr(imu.time,'time_ns',lambda:1700000000000000000+state.clock)
    monkeypatch.setattr(imu.os,'write',lambda *args:(_ for _ in ()).throw(AssertionError('device writes forbidden')))
    arguments=['--device','/dev/SYN','--expected-by-id','/dev/serial/by-id/usb-SYN',
        '--hardware-serial','SYN','--sensor-id','H30-SYN','--session-id','SYN','--run-root',str(tmp_path),
        '--read-mode','event','--timing-profile',str(policy),'--duration','0','--stale-timeout','0',
        '--journal-dir',str(tmp_path/'journal')]
    assert imu.main(arguments)==(2 if late_reset else 0)
    assert state.events[:9]==['wait','read','spin']*3 and state.closed
    node=state.nodes[0]
    assert node.timers==[.25]
    frames=node.publishers['/wc_mapping/imu/source_frame'].messages
    assert len(frames)==2
    assert frames[0].host_monotonic_ns==frames[1].host_monotonic_ns
    assert [f.sample_timestamp_raw for f in frames]==[1010000,1015000]
    assert [f.frame_sequence for f in frames]==[1,2]
    assert frames[0].device_timestamp_unit=='us' and frames[0].time_source=='arrival_only'
    quarantine=[row for row in state.journal if row['event']=='startup_quarantine']
    assert len(quarantine)==3 and [row['tid'] for row in quarantine]==[200,201,202]
    assert [row['event'] for row in state.journal].count('byte_batch')==(4 if late_reset else 3)
    assert bool(state.journal_error)==late_reset
    if late_reset:
        assert len([row for row in state.journal if row['event']=='device_time_fault'])==1
        assert node.device_time.accepted==2  # The mid-stream reset is never quarantined or restarted.
