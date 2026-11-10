"""ABS/ASR-Ersatzmodell: reduziert nur, greift nur über dem Zielschlupf, Ratenbegrenzung, ZOH-Identität."""

import unittest
from dataclasses import replace
from torquevector.control.slip_control import SlipControlSettings, SlipController, slip_control_metrics
from torquevector.domain.controls import TorqueControls
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.simulation.sampled import SampledSettings, simulate_sampled

ON = SlipControlSettings(
    abs_enabled=True,
    asr_enabled=True,
    period_s=0.005,
    abs_target_slip=0.1,
    asr_target_slip=0.1,
    kp=4.0,
    ki_per_s=60.0,
    torque_rate_nm_s=1e9,
)
BRAKE = TorqueControls(0.0, (0.0,) * 4, (600.0, 600.0, 400.0, 400.0), 800.0)
DRIVE = TorqueControls(0.0, (500.0,) * 4, (0.0,) * 4, 800.0)


class SlipControllerLogic(unittest.TestCase):
    def test_no_intervention_below_target(self):
        ctl = SlipController(ON)
        for k in range(50):
            out = ctl(k * 0.005, (-0.05, -0.02, 0.0, -0.09), BRAKE)
        self.assertEqual(out.brake_nm, BRAKE.brake_nm)
        self.assertEqual(slip_control_metrics(ctl.log)['max_reduction'], [0.0] * 4)

    def test_abs_reduces_only_overbraked_wheel(self):
        ctl = SlipController(ON)
        for k in range(20):
            out = ctl(k * 0.005, (-0.3, -0.02, -0.02, -0.02), BRAKE)
        self.assertLess(out.brake_nm[0], BRAKE.brake_nm[0])
        self.assertEqual(out.brake_nm[1:], BRAKE.brake_nm[1:])
        self.assertEqual(out.drive_nm, BRAKE.drive_nm)

    def test_asr_reduces_spinning_wheel_and_never_increases(self):
        ctl = SlipController(ON)
        for k in range(40):
            out = ctl(k * 0.005, (0.05, 0.4, 0.05, 0.05), DRIVE)
            for applied, requested in zip(out.drive_nm, DRIVE.drive_nm):
                self.assertLessEqual(applied, requested + 1e-12)
        self.assertLess(out.drive_nm[1], DRIVE.drive_nm[1])

    def test_release_when_slip_recovers(self):
        ctl = SlipController(ON)
        for k in range(20):
            ctl(k * 0.005, (-0.3, 0.0, 0.0, 0.0), BRAKE)
        reduced = ctl.reduction[0]
        for k in range(20, 200):
            out = ctl(k * 0.005, (-0.02, 0.0, 0.0, 0.0), BRAKE)
        self.assertGreater(reduced, 0.0)
        self.assertEqual(ctl.reduction[0], 0.0)
        self.assertEqual(out.brake_nm, BRAKE.brake_nm)

    def test_disabled_function_is_identity(self):
        ctl = SlipController(replace(ON, abs_enabled=False))
        for k in range(20):
            out = ctl(k * 0.005, (-0.4, -0.4, -0.4, -0.4), BRAKE)
        self.assertEqual(out.brake_nm, BRAKE.brake_nm)

    def test_torque_rate_limited(self):
        ctl = SlipController(replace(ON, torque_rate_nm_s=1000.0))
        ctl(0.0, (0.0,) * 4, BRAKE)
        out = ctl(0.005, (-0.5, 0.0, 0.0, 0.0), replace(BRAKE, brake_nm=(0.0, 600.0, 400.0, 400.0)))
        self.assertAlmostEqual(out.brake_nm[0], 600.0 - 1000.0 * 0.005, 9)

    def test_invalid_settings_rejected(self):
        for bad in (
            dict(period_s=0.0),
            dict(period_s=0.1),
            dict(abs_target_slip=0.6),
            dict(kp=-1.0),
            dict(torque_rate_nm_s=0.0),
        ):
            with self.assertRaises(ValueError):
                replace(ON, **bad)


class SampledLoopIdentity(unittest.TestCase):
    def test_pass_through_inner_loop_is_bit_identical(self):
        car, tyre = VehicleParameters(), TyreParameters()
        settings = SampledSettings(period_s=0.04, max_step_s=0.02)
        controls = TorqueControls(2.0, (100.0, 100.0, 150.0, 150.0), (0.0,) * 4, 800.0)
        base = simulate_sampled(car, tyre, controls, speed_ms=15.0, duration_s=0.6, settings=settings)
        inner = simulate_sampled(
            car,
            tyre,
            controls,
            speed_ms=15.0,
            duration_s=0.6,
            settings=settings,
            slip_controller=lambda t, k, held: held,
            slip_period_s=0.04,
        )
        self.assertEqual(base, inner)

    def test_slip_controller_requires_valid_period(self):
        with self.assertRaises(ValueError):
            simulate_sampled(
                VehicleParameters(),
                TyreParameters(),
                TorqueControls(),
                duration_s=0.1,
                slip_controller=lambda t, k, held: held,
                slip_period_s=None,
            )
