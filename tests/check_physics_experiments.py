#!/usr/bin/env python3
"""Check experimental physics against explicit mathematical references."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys


def run(executable, scenario, variant, substeps=5):
    result = subprocess.run([str(executable), '--scenario', scenario, '--variant', variant,
                             '--substeps', str(substeps)], text=True, capture_output=True,
                            env=dict(os.environ, QT_QPA_PLATFORM='offscreen'), timeout=30)
    data = json.loads(result.stdout)
    if result.returncode or not data['success']:
        raise AssertionError(data.get('error', result.stderr))
    return data


def main():
    executable = Path(sys.argv[1]).resolve()
    for variant in ['inertia-z', 'system']:
        for scenario in ['torque-chassis', 'torque-wheel']:
            data = run(executable, scenario, variant)
            error = data['metrics']['max_relative_angular_velocity_error']
            assert error < 1e-10, (scenario, error)
            print(f'PASS {variant} {scenario}: torque/inertia relative error {error:.3g}')
    for variant in ['ball-native', 'system']:
        for substeps in [5, 15, 30]:
            data = run(executable, 'ball-flight', variant, substeps)
            metrics = data['metrics']
            assert metrics['flight_horizontal_error_m'] < 1e-10, metrics
            # Semi-implicit Euler's constant-gravity position error.
            expected_error = 0.5*data['parameters']['gravity_m_s2']*data['duration_s']/(60*substeps)
            assert math.isclose(metrics['flight_vertical_error_m'], expected_error, abs_tol=1e-9), metrics
            print(f'PASS {variant} flight, {substeps} substeps: analytic integration error {expected_error:.6g} m')
    for variant, scale in [('ball-native-half', 0.5), ('ball-native', 1), ('ball-native-double', 2), ('system', 1)]:
        data = run(executable, 'ball-rolling', variant)
        metrics = data['metrics']
        expected = (-data['parameters']['ball_rolling_deceleration_m_s2'] if variant == 'system'
                    else scale*metrics['expected_matched_rolling_acceleration_m_s2'])
        assert math.isclose(metrics['mean_rolling_acceleration_m_s2'], expected, rel_tol=1e-6), metrics
        print(f'PASS {variant}: steady rolling acceleration {expected:.6g} m/s^2')
    for variant in ['ball-native', 'system']:
        data = run(executable, 'ball-spin', variant)
        metrics = data['metrics']
        assert 0 <= metrics['ball_stop_time_s']-metrics['expected_native_spin_stop_s'] <= 1/60, metrics
        print(f'PASS {variant} spin: stop time agrees within one recorded tick')
        data = run(executable, 'ball-drop', variant, 30)
        time = data['trajectory'][18][0]
        gravity = data['parameters']['gravity_m_s2']
        expected_z = 1-0.5*gravity*time*time-0.5*gravity*time/(60*30)
        assert math.isclose(data['trajectory'][18][3], expected_z, abs_tol=1e-9), data['trajectory'][18]
        print(f'PASS {variant} fine-step drop: release from rest follows gravity before ground impact')


if __name__ == '__main__':
    main()
