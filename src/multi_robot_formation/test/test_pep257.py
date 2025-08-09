# Copyright 2024 Multi-Robot Formation
#
# Licensed under the MIT License

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Test source code for compliance with pep257."""
    rc = main(argv=['.', 'test'])
    assert rc == 0
