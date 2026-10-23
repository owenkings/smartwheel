import pytest
from wc_panel.ui_common import result_label

@pytest.mark.parametrize('group',list('ABCDEF'))
def test_letter_group_is_first_and_recording_method_remain_visible(group):
    row=dict(name='fusion_batch / '+group+' · 官方 / filtered',group_id=group,
             dataset='/media/nvidia/tiany-USB/wheelchair/data/experiments/zoulang',path='/analysis/run/cell')
    assert result_label(row)==group+' · zoulang · 官方 / filtered'
