"""Board startup smoke test; run with pytest-embedded and a connected ESP32-S3."""

import pytest
from pytest_embedded_idf.dut import IdfDut


@pytest.mark.generic
@pytest.mark.esp32s3
def test_app_startup(dut: IdfDut) -> None:
    dut.expect("ServoTorqueTesting started")
    dut.expect("PCA9685 ready; all channels OFF")
    dut.expect("16-channel motion started")
