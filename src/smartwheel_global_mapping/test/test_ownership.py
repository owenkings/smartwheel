import pytest

from smartwheel_global_mapping.ownership import map_tf_owner, validate_tf_owners


def test_each_backend_has_one_distinct_map_tf_owner():
    assert map_tf_owner("rtabmap") == "rtabmap"
    assert map_tf_owner("slam_toolbox") == "slam_toolbox"
    validate_tf_owners([map_tf_owner("rtabmap")])
    validate_tf_owners([map_tf_owner("slam_toolbox")])


def test_duplicate_or_missing_map_tf_owner_is_rejected():
    with pytest.raises(ValueError, match="exactly one"):
        validate_tf_owners([])
    with pytest.raises(ValueError, match="exactly one"):
        validate_tf_owners(["rtabmap", "slam_toolbox"])

