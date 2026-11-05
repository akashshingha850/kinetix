# acquisition/config

Settings for `kinetix run`, merged in this order (later wins; loader: `kinetix/config.py`, `load_config`):

```
common.yaml  <  object.yaml | room.yaml  <  --config files  <  key=value on the command line
```

| File | Role |
|---|---|
| `common.yaml` | Shared by every scene: camera, coverage model, rig (tolerances, timeouts, bag), planner limits. |
| `object.yaml` | Loaded when the scene has `kind: object`: ROI box scanned from close up (ring-stack candidates, NBV, baselines). |
| `room.yaml` | Loaded when the scene has `kind: room`: grid candidates, `tour`, and the `explore` pass. Only the keys that differ from `common.yaml`. |
| `experiments/*.yaml` | Optional overlays, one hypothesis each, used with `--config` or `scripts/room_scan.sh --experiment NAME`. Each sets `experiment:` so `python -m acquisition.compare` can label the run. |

Experiments: `close`, `dense`, `sparse`, `fine_grid`, `slow` (change one planner/rig knob) and `traditional` (the `--explore` baseline arm, `explore.reuse_views: false`).

## Rules

- A key lives in one file. `room.yaml` and `object.yaml` repeat nothing from `common.yaml`.
- A new knob goes in the file matching where it applies, with a comment; do not hard-code a default in Python.
- A one-off change does not need a file: `kinetix run ... planner.budget=24`. Add an experiment file only for a named, repeated comparison.
- Units: metres, seconds, degrees (radians only inside the code).
