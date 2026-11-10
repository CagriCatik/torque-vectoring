"""Erzeugt tests/fixtures/road_multi_mu_reference.json (eingefrorene Multi-mu-Referenz, nur bewusst neu erzeugen)."""

import json, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from integration.road.helpers import run  # noqa: E402

CASES = [
    ('straight_multi_mu', 'off'),
    ('straight_multi_mu', 'lqr'),
    ('straight_split_mu_braking', 'off'),
    ('straight_split_mu_acceleration', 'off'),
    ('mixed_mu_lane_change', 'lqr'),
]
KEYS = ('yaw_rmse_rads', 'cross_rmse_m', 'max_abs_beta_deg', 'max_tyre_utilization', 'final_speed_ms', 'max_abs_slip')
rows = []
for scenario, kind in CASES:
    _, entry, seq, _, _ = run(scenario, kind)
    rows.append(
        {
            'scenario': scenario,
            'controller': kind,
            'status': entry['status'],
            'metrics': {k: entry['metrics'][k] for k in KEYS},
            'final_x_m': seq[-1].state.x_m,
            'final_y_m': seq[-1].state.y_m,
        }
    )
(ROOT / 'tests/fixtures/road_multi_mu_reference.json').write_text(json.dumps(rows, indent=2) + '\n')
print(json.dumps(rows, indent=1))
