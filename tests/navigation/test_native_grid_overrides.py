import copy
import pytest
from wc_runtime.compare_native import with_offline_grid_overrides


def specification():
    return {'nodes': [{'parameters': [{'Grid/3D': 'true', 'Grid/RayTracing': 'false', 'Grid/NormalsSegmentation': 'true'}]}]}


def test_explicit_grid_experiment_is_local_and_has_provenance():
    source = specification()
    before = copy.deepcopy(source)
    result, provenance = with_offline_grid_overrides(source, {'Grid/3D': 'false', 'Grid/RayTracing': 'true'})
    assert source == before
    assert result['nodes'][0]['parameters'][0]['Grid/RayTracing'] == 'true'
    assert provenance['previous']['Grid/RayTracing'] == 'false'
    assert provenance['requested']['Grid/3D'] == 'false'
    assert provenance['physical_navigation_validated'] is False


@pytest.mark.parametrize('value', [
    {'Icp/PointToPlane': 'false'}, {'database_path': '/tmp/other.db'},
    {'Grid/RayTracing': True}, {'Grid/3D': 'False'}, {'Grid/3D': 0}, [],
])
def test_rebuild_cannot_change_registration_or_arbitrary_native_settings(value):
    with pytest.raises(ValueError):
        with_offline_grid_overrides(specification(), value)


def test_default_preserves_all_parameters():
    source = specification()
    result, provenance = with_offline_grid_overrides(source)
    assert result == source
    assert result is not source
    assert provenance['requested'] == {}
