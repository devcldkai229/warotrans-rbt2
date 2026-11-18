from warotrans_rbt002_hardware.imu_convert import GyroBiasEstimator


def test_bias_learned_only_while_still_and_subtracted():
    b = GyroBiasEstimator()
    t = 0.0
    b.on_wheels([0.0] * 4, t)
    for _ in range(300):              # 6 s still at 50 Hz
        t += 0.02
        out = b.apply((0.001, -0.002, 0.0034), t)
    assert abs(b.bias[2] - 0.0034) < 1e-6 and abs(out[2]) < 1e-6
    b.on_wheels([0.5, 0.5, -0.5, -0.5], t)  # moving: bias frozen, real rate passes through
    t += 0.02
    out = b.apply((0.0, 0.0, 0.6034), t)
    assert abs(out[2] - 0.6) < 1e-6 and abs(b.bias[2] - 0.0034) < 1e-6


def test_no_learning_before_settle_or_with_a_large_rate():
    b = GyroBiasEstimator(settle_s=0.5)
    b.on_wheels([0.0] * 4, 0.0)
    b.apply((0.0, 0.0, 0.01), 0.2)    # not settled yet
    assert b.samples == 0
    b.apply((0.0, 0.0, 0.3), 1.0)     # turning by hand / pushed: not a bias
    assert b.samples == 0
