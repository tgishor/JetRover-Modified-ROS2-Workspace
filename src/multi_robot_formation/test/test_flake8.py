# Copyright 2024 Multi-Robot Formation
#
# Licensed under the MIT License

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Test source code for compliance with flake8."""
    rc, errors = main_with_errors(argv=['.'])
    assert rc == 0, f'Found {len(errors)} lint errors:\n' + '\n'.join(errors)
