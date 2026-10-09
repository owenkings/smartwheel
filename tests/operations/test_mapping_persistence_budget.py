"""Persistence deadline propagation; synthetic only, no devices or ROS graph."""
import pytest

from wc_runtime import mapping_controller as controller, mapping_shutdown as budget, supervisor, component
from test_mapping_controller import setup


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_every_mapping_storage_process_gets_the_extended_stop_wait(setup, mode):
    request = dict(setup, mode=mode)
    config = controller.configuration(request)
    plan = controller.plan(request, config)
    required = {'wc_runtime.mapping_input', 'wc_runtime.mapping_prior', 'wc_runtime.mapping_monitor',
                'wc_runtime.mapping_wheel', 'wc_runtime.mapping_cameras'}
    observed = set()
    for argv in plan['commands']:
        for module in required.intersection(argv):
            observed.add(module)
            assert float(argv[argv.index('--close-timeout-s')+1]) == 90.
    assert observed == required
    assert config['shutdown_persistence']['persistence_close_s'] == 90.
    assert plan['duration_s'] == 0  # No acquisition deadline; closing still has a persistence budget.
    shutdown = supervisor.shutdown_policy(plan)
    assert 9 + budget.PERSISTENCE_CLOSE_S < shutdown['sigint_grace_s']
    assert budget.NATIVE_SIGTERM_S + 5 < shutdown['sigint_grace_s']
    assert shutdown['sigint_grace_s'] + component.TERM_GRACE_S + component.KILL_GRACE_S < shutdown['component_wait_s']
    assert shutdown['final_manifest_wait_s'] == 30
    assert shutdown['cli_stop_wait_s'] == 166


def test_recording_tools_keep_their_previous_default_stop_policy():
    assert component.DEFAULT_SIGINT_GRACE_S == 5
    assert component.RECORD_SIGINT_GRACE_S == 30
    assert supervisor.shutdown_policy({'role': 'record'})['cli_stop_wait_s'] == 47


@pytest.mark.parametrize('bad', [0, -.1, 90.1, float('nan'), float('inf'), True, None, 'invalid'])
def test_invalid_extended_wait_rejected(bad):
    with pytest.raises(ValueError):
        budget.persistence_wait(bad)


def test_explicit_async_journal_wait_reaches_the_writer_without_changing_default(setup):
    from wc_motion.feedback_transport import create_journal
    from pathlib import Path
    root = Path(setup['project_root'])
    for name, kwargs, expected in [('mapping', {'close_timeout_s': 90}, 90), ('legacy', {}, 10)]:
        journal = create_journal(root/(name+'.jsonl'), asynchronous=True, **kwargs)
        waits = []
        join = journal.thread.join
        def observe_join(timeout):
            waits.append(timeout)
            return join(timeout)
        journal.thread.join = observe_join
        journal({'event': 'SYNTHETIC_FILE_ONLY'})
        journal.close()
        assert waits == [expected]
        assert journal.status()['final_fsync_complete'] is True
