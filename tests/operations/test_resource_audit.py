"""Resource attribution boundaries; no native process or hardware starts."""
from wc_runtime.resource_audit import delta,proc_snapshot

def reading(parent_cpu,children_cpu,worker=None,peak=100):
    return {'parent':dict(python_process_cpu_s=parent_cpu,rss_bytes=80,peak_rss_bytes=peak),
            'reaped_children_cpu_cumulative_s':children_cpu,'rl_worker':worker}

def test_resource_deltas_keep_parent_live_worker_and_reaped_children_separate():
    before=reading(10.,2.,dict(pid=4,cpu_s=5.,rss_bytes=20,peak_rss_bytes=30))
    after=reading(12.,3.,dict(pid=4,cpu_s=5.5,rss_bytes=25,peak_rss_bytes=35),peak=150)
    result=delta(before,after)
    assert result['python_parent_cpu_delta_s']==2.
    assert result['rl_worker_cpu_delta_s']==.5
    assert result['reaped_children_cpu_delta_s']==1.
    assert result['parent_lifetime_peak_rss_bytes']==150
    assert 'NOT a per-cell peak' in result['parent_peak_scope']
    assert 'parent only' in result['python_parent_cpu_scope']

def test_new_worker_uses_lifetime_counter_and_unavailable_memory_stays_unknown():
    result=delta(reading(1.,None),reading(1.2,None,dict(pid=8,cpu_s=.4,rss_bytes=None,peak_rss_bytes=None)))
    assert result['rl_worker_cpu_delta_s']==.4 and 'created in cell' in result['rl_worker_cpu_scope']
    assert result['reaped_children_cpu_delta_s'] is None
    missing=proc_snapshot(100,proc_root='/nonexistent-resource-audit-fixture')
    assert missing['status']=='UNAVAILABLE' and missing['rss_bytes'] is None
