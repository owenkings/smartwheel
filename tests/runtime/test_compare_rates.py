"""Offline all-pairs option keeps native ceilings and rejects invalid rates."""
import pytest

from wc_runtime.mapping_compare import validate_comparison_rates


@pytest.mark.parametrize('rates,native', [([0],1), ([0],10), ([5,10,0],5), ([10],10), ([5],1)])
def test_frontend_zero_is_not_zero_native_capacity(rates,native):
    validate_comparison_rates(rates,native)


@pytest.mark.parametrize('rates,native', [([],1), ([-1],1), ([11],1), ([float('nan')],1),
    ([float('inf')],1), ([0],0), ([0],-1), ([0],11), ([0],float('nan')),
    ([0],float('inf')), ([0,5,10],6), ([5],10)])
def test_invalid_rates_and_native_above_capped_cell_are_rejected(rates,native):
    with pytest.raises(ValueError):
        validate_comparison_rates(rates,native)
