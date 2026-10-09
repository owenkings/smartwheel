"""Process-scoped resource evidence; never label parent CPU as full algorithm CPU."""
import os
from pathlib import Path
import sys
import time

def proc_snapshot(pid,proc_root=Path('/proc')):
    """Linux process CPU/RSS counters, including the persistent actual RL worker."""
    try:
        directory=Path(proc_root)/str(pid)
        fields=(directory/'stat').read_text().rpartition(')')[2].split()
        if len(fields)<13: raise ValueError('short process stat')
        ticks=os.sysconf('SC_CLK_TCK')
        status={line.split(':',1)[0]:line.split(':',1)[1].strip()
                for line in (directory/'status').read_text().splitlines() if ':' in line}
        def memory(name):
            value=status.get(name)
            return int(value.split()[0])*1024 if value and value.split()[1]=='kB' else None
        return {'status':'AVAILABLE','pid':pid,'cpu_s':(int(fields[11])+int(fields[12]))/ticks,
                'rss_bytes':memory('VmRSS'),'peak_rss_bytes':memory('VmHWM'),
                'peak_scope':'cumulative since this process started; not a per-cell peak'}
    except (OSError,ValueError,IndexError,AttributeError) as error:
        return {'status':'UNAVAILABLE','pid':pid,'reason':str(error),'cpu_s':None,'rss_bytes':None,'peak_rss_bytes':None}

def snapshot():
    parent=proc_snapshot(os.getpid()); parent['python_process_cpu_s']=time.process_time()
    children=None
    try:
        import resource
        child=resource.getrusage(resource.RUSAGE_CHILDREN)
        children=child.ru_utime+child.ru_stime
        usage=resource.getrusage(resource.RUSAGE_SELF)
        parent['peak_rss_bytes']=int(usage.ru_maxrss)*(1 if sys.platform=='darwin' else 1024)
        parent['peak_scope']='RUSAGE_SELF lifetime cumulative peak; not a per-cell peak'
    except ImportError:
        # Optional Windows evidence; psutil is never required or installed here.
        try:
            import psutil
            memory=psutil.Process().memory_info()
            parent['rss_bytes']=memory.rss; parent['peak_rss_bytes']=getattr(memory,'peak_wset',None)
            parent['peak_scope']='process lifetime working-set peak when available; not per-cell'
        except (ImportError,OSError): pass
    module=sys.modules.get('wc_runtime.robot_localization_provider')
    worker=getattr(module,'_worker',None) if module else None
    process=getattr(worker,'process',None)
    native=proc_snapshot(process.pid) if process is not None and process.poll() is None else None
    return {'parent':parent,'rl_worker':native,'reaped_children_cpu_cumulative_s':children}

def delta(before,after):
    parent_cpu=max(0.,after['parent']['python_process_cpu_s']-before['parent']['python_process_cpu_s'])
    a,b=before.get('rl_worker'),after.get('rl_worker')
    worker_cpu=None; worker_scope='NO_LIVE_RL_WORKER'
    if b and b.get('cpu_s') is not None:
        same=a and a.get('pid')==b.get('pid') and a.get('cpu_s') is not None
        worker_cpu=max(0.,b['cpu_s']-a['cpu_s']) if same else b['cpu_s']
        worker_scope='same worker process counter delta' if same else 'worker created in cell; CPU since worker startup'
    child_cpu=None
    if before['reaped_children_cpu_cumulative_s'] is not None and after['reaped_children_cpu_cumulative_s'] is not None:
        child_cpu=max(0.,after['reaped_children_cpu_cumulative_s']-before['reaped_children_cpu_cumulative_s'])
    return {'python_parent_cpu_delta_s':parent_cpu,
            'python_parent_cpu_scope':'parent only; excludes live native worker and RTABMap processes',
            'parent_current_rss_before_bytes':before['parent'].get('rss_bytes'),
            'parent_current_rss_after_bytes':after['parent'].get('rss_bytes'),
            'parent_lifetime_peak_rss_bytes':after['parent'].get('peak_rss_bytes'),
            'parent_peak_scope':'cumulative parent process since startup; NOT a per-cell peak',
            'rl_worker_cpu_delta_s':worker_cpu,'rl_worker_cpu_scope':worker_scope,
            'rl_worker_after':b,'reaped_children_cpu_delta_s':child_cpu,
            'reaped_children_scope':'RUSAGE_CHILDREN delta; live persistent workers excluded',
            'availability':'process snapshots only; no unmeasured total algorithm CPU claim'}
