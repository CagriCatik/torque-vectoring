"""Q06 – Radrotation und Momentenantrieb."""

import ast
import inspect
import unittest
from dataclasses import fields, replace

import numpy as np

from quality.common import run_case, runtime
from torquevector.domain.controls import TorqueControls
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.domain.states import VehicleState
from torquevector.domain.wheels import WHEEL_ORDER


class Q06WheelRotation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.car, cls.tyre, cls.wheel, cls.model = rt.vehicle, rt.tyre, rt.wheel, rt.tyre_model

    def state_vector(self, vx=18.0, slip=0.0):
        return np.array([0.0, 0.0, 0.0, vx, 0.0, 0.0] + [(1 + slip) * vx / self.wheel.radius_m] * 4)

    def test_Q06_1_wheel_equation_with_signs(self):
        """Q06.1 (kritisch): Iw*omega_dot = T_drive − T_brake − R_eff*Fx je Rad, Vorzeichen/Einheiten dokumentiert."""
        from torquevector.physics.wheel_rotation import derivative, forces_at

        ctrl = TorqueControls(1.0, (15.0, 25.0, 140.0, 60.0), (0.0, 10.0, 0.0, 40.0))
        for slip in (0.0, 0.03, -0.04):
            x = self.state_vector(slip=slip)
            d = derivative(1.0, x, self.car, self.tyre, ctrl, self.wheel, tyre_model=self.model, load_solver='root')
            f, _ = forces_at(
                VehicleState(vx_ms=18.0),
                tuple(x[6:10]),
                1.0,
                self.car,
                self.tyre,
                ctrl,
                self.wheel,
                self.model,
                None,
                'root',
            )
            for i, n in enumerate(WHEEL_ORDER):
                rhs = ctrl.drive_nm[i] - ctrl.brake_nm[i] - self.wheel.radius_m * f.wheels[n].fx_n
                self.assertAlmostEqual(self.wheel.inertia_kgm2 * d[6 + i], rhs, 9)

    def test_Q06_2_wheel_speeds_are_states_slip_is_not_an_input(self):
        """Q06.2 (kritisch): vier omega_i sind dynamische Zustände; Schlupf ist keine primäre Stellgröße."""
        from torquevector.domain import controls
        from torquevector.simulation import sampled

        self.assertEqual(
            {f.name for f in fields(TorqueControls)}, {'steer_deg', 'drive_nm', 'brake_nm', 'torque_limit_nm'}
        )
        tree = ast.parse(inspect.getsource(sampled))
        names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        self.assertNotIn('DriverControls', names)  # getaktete Simulation kennt keine Schlupfvorgabe
        seq = run_case(None, 'lqr', (('scenario.simulation_duration_s', 2.0),))[2]
        omega = np.array([s.omega_rad_s for s in seq])
        self.assertEqual(omega.shape[1], 4)
        self.assertGreater(np.ptp(omega[:, 0] - omega[:, 1]), 0)  # Räder drehen individuell
        kappa = np.array([s.kappa for s in seq])
        self.assertGreater(np.ptp(kappa), 0)  # Schlupf entsteht aus der Dynamik

    def test_Q06_3_torque_limits_and_power_signs(self):
        """Q06.3: Antriebs-/Bremsmomentgrenzen, Verzögerung und Leistungsvorzeichen korrekt erfasst."""
        from torquevector.evaluation.diagnostics import power_w
        from torquevector.simulation.continuous import simulate_torque

        for bad in (
            dict(drive_nm=(0.0, 0.0, 801.0, 0.0)),
            dict(brake_nm=(0.0, 0.0, -1.0, 0.0)),
            dict(brake_nm=(0.0, 900.0, 0.0, 0.0)),
        ):
            with self.assertRaises(ValueError):
                TorqueControls(**bad)
        drive = TorqueControls(0.0, (0.0, 0.0, 150.0, 150.0))
        brake = TorqueControls(0.0, (0.0,) * 4, (300.0, 300.0, 200.0, 200.0))
        for ctrl, sign in ((drive, 1), (brake, -1)):
            seq = simulate_torque(self.car, self.tyre, ctrl, self.wheel, duration_s=1.0, tyre_model=self.model)
            p = [power_w(s, self.car, self.wheel, ctrl) for s in seq[1:]]
            self.assertTrue(all(x['brake_w'] >= 0 for x in p))  # Bremse entzieht Leistung
            self.assertTrue(all(np.sign(x['drive_w']) in (0, 1) for x in p))
            self.assertEqual(np.sign(seq[-1].state.vx_ms - seq[0].state.vx_ms), sign)

    def test_Q06_4_torque_step_slip_build_up_and_lock_up(self):
        """Q06.4: Momentensprung, Schlupfaufbau und Radblockieren mit unabhängigen Grenzfällen."""
        from torquevector.physics.wheel_rotation import derivative
        from torquevector.simulation.continuous import simulate_torque

        step = TorqueControls(0.0, (0.0, 0.0, 400.0, 400.0))
        d = derivative(0.0, self.state_vector(), self.car, self.tyre, step, self.wheel, tyre_model=self.model)
        self.assertAlmostEqual(d[8], 400.0 / self.wheel.inertia_kgm2, 9)  # bei kappa = 0: Fx = 0 ⇒ omega_dot = T/Iw
        seq = simulate_torque(
            self.car, self.tyre, step, self.wheel, duration_s=0.3, output_dt=0.005, tyre_model=self.model
        )
        kappa = [s.kappa[2] for s in seq]
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(kappa[:10], kappa[1:11])))  # monotoner Aufbau
        quasi = seq[-1].forces.wheels['RL'].fx_n * self.wheel.radius_m
        self.assertAlmostEqual(quasi, 400.0, delta=20.0)  # quasistationär T ≈ R·Fx
        wet = replace(self.tyre, mu=0.3)  # Haftgrenze R·µ·Fz ≈ 380 Nm < 800 Nm
        with self.assertRaisesRegex(OutsideEnvelopeError, 'Slip'):  # Überbremsen ⇒ Blockieren erkannt
            simulate_torque(
                self.car,
                wet,
                TorqueControls(0.0, (0.0,) * 4, (800.0,) * 4),
                self.wheel,
                duration_s=2.0,
                tyre_model=self.model,
            )

    def test_Q06_5_energy_balance_within_two_percent(self):
        """Q06.5: Leistungs-/Energiebilanz Antrieb, Rotation, Fahrzeug inkl. Verluste im Testfenster <= 2 %."""
        from torquevector.evaluation.diagnostics import audit_energy
        from torquevector.simulation.continuous import simulate_torque

        for ctrl in (
            TorqueControls(1.5, (0.0, 0.0, 120.0, 100.0)),
            TorqueControls(0.0, (0.0,) * 4, (150.0, 150.0, 80.0, 80.0)),
            TorqueControls(3.0, (0.0,) * 4),
        ):
            seq = simulate_torque(
                self.car, self.tyre, ctrl, self.wheel, duration_s=2.0, output_dt=0.005, tyre_model=self.model
            )
            audit = audit_energy(seq, self.car, self.wheel, ctrl)
            with self.subTest(ctrl=ctrl):
                self.assertLessEqual(audit['relative_residual'], 0.02)


if __name__ == '__main__':
    unittest.main()
