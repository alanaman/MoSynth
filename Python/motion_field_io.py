"""
On-disk format for the trained motion field value function.

Two artefacts come out of training:

  * `<name>.mffield.npz` -- the value function itself, written into
    StreamingAssets so it ships with the player.
  * `<name>.mftables.npz` -- the precomputed transition tables, a rebuild cache
    only. It is ~15 MB and useless at runtime, so it lives in `Library/` where
    Unity never imports it and no build ever picks it up.

Both carry a signature of the pose database they were derived from. Every index
in the value function is a row of that database, so loading a value function
against a re-extracted `.mmpose` would silently address the wrong poses. The
legacy notebook guarded this with a states_count assert alone, which happily
accepts a database that was regenerated with the same frame count but different
content -- hence the content hashes here.
"""

from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass

import numpy as np

SCHEMA_VERSION = 1

# `delta_yaw` is read straight off the root quaternion as 2*atan2(q.y, q.w).
# That is exact rather than an approximation because PoseExtractor builds the
# SimulationBone rotation as LookRotation(hipsForward_projected, up), making
# every root rotation in the database a pure yaw. Recorded in the file so a
# future non-yaw root cannot silently reuse a value function trained under this
# assumption.
DELTA_YAW_CONVENTION = 'root_quat_pure_yaw_xyzw'

# Which database state an action is tugged toward. 'emphasized' follows the
# paper's reference implementation: the neighbour that the action emphasises.
# 'nearest' always tugs toward the closest state, which makes the candidate
# actions nearly indistinguishable.
TUG_MODE = 'emphasized'


def file_sha1(path: str) -> str:
    h = hashlib.sha1()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def database_signature(data_dir: str, db_name: str) -> dict:
    """Content hashes identifying the pose database the field was built from."""
    return {
        'pose_db_name': db_name,
        'pose_db_sha1': file_sha1(os.path.join(data_dir, f'{db_name}.mmpose')),
        'skeleton_sha1': file_sha1(os.path.join(data_dir, f'{db_name}.mmskeleton')),
    }


# Keys that must agree between the trained file and the runtime asking for it.
# Anything outside this list is informational.
_GATE_KEYS = (
    'schema_version',
    'delta_yaw_convention',
    'pose_db_sha1',
    'skeleton_sha1',
    'states_count',
    'bone_count',
    'k_neighbors',
    'tug_ratio',
    'tug_mode',
    'pos_weight',
    'vel_weight',
    'frame_time',
)

_FLOAT_GATE_TOLERANCE = 1e-6


@dataclass
class ValueFunctionData:
    """A loaded and validated value function."""
    values: np.ndarray          # (states, thetas) float32
    scores: np.ndarray          # (epochs, 3) float32 -- Bellman residual min/max/mean
    thetas: np.ndarray          # (thetas,) float32
    theta_count: int
    theta_spacing: float
    gamma: float
    k_neighbors: int
    tug_ratio: float
    tug_mode: str
    pos_weight: float
    vel_weight: float
    frame_time: float
    states_count: int
    bone_count: int

    @property
    def final_residual(self) -> float:
        return float(self.scores[-1, 2]) if self.scores.size else float('nan')


def save_value_function(out_path: str, values: np.ndarray, scores: np.ndarray,
                        params: dict, signature: dict) -> None:
    """
    Write `<name>.mffield.npz`.

    `params` carries the hyperparameters the runtime must match plus the purely
    informational ones; `signature` is the output of `database_signature`.
    """
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

    theta_count = int(params['theta_count'])
    payload = {
        'schema_version': np.int64(SCHEMA_VERSION),
        'value_function': np.ascontiguousarray(values, dtype=np.float32),
        'scores': np.ascontiguousarray(scores, dtype=np.float32),
        'thetas': theta_grid(theta_count),
        'theta_count': np.int64(theta_count),
        'theta_spacing': np.float32(2.0 * np.pi / theta_count),
        'states_count': np.int64(params['states_count']),
        'bone_count': np.int64(params['bone_count']),
        'frame_time': np.float32(params['frame_time']),
        'pos_weight': np.float32(params['pos_weight']),
        'vel_weight': np.float32(params['vel_weight']),
        'feature_shape': np.asarray(params['feature_shape'], dtype=np.int64),
        'k_neighbors': np.int64(params['k_neighbors']),
        'tug_ratio': np.float32(params['tug_ratio']),
        'tug_mode': np.str_(params.get('tug_mode', TUG_MODE)),
        'delta_yaw_convention': np.str_(DELTA_YAW_CONVENTION),
        'gamma': np.float32(params['gamma']),
        'epochs': np.int64(params['epochs']),
        'config_json': np.str_(json.dumps(params, sort_keys=True, default=str)),
    }
    payload.update({k: np.str_(v) for k, v in signature.items()})

    np.savez(out_path, **payload)


def theta_grid(theta_count: int) -> np.ndarray:
    """The task-parameter grid: `theta_count` headings spanning [-pi, pi)."""
    return np.linspace(-np.pi, np.pi, theta_count + 1, dtype=np.float32)[:theta_count]


def _mismatch(loaded, expected) -> bool:
    if isinstance(expected, float) or isinstance(loaded, float):
        return abs(float(loaded) - float(expected)) > _FLOAT_GATE_TOLERANCE
    return str(loaded) != str(expected)


def load_value_function(path: str, expect: dict | None = None,
                        log=print) -> ValueFunctionData | None:
    """
    Load and validate a `.mffield.npz`.

    Returns None -- never raises -- when the file is missing, unreadable or does
    not match `expect`. Callers run inside `Py.GIL()` from Unity, where an
    exception surfaces as an opaque managed error, and a stale value function
    should degrade to greedy control rather than take the player down.
    """
    if not path or not os.path.isfile(path):
        return None

    try:
        with np.load(path, allow_pickle=False) as f:
            data = {k: f[k] for k in f.files}
    except Exception as exc:  # noqa: BLE001 - report and fall back
        log(f'[MotionField] could not read value function {path}: {exc}')
        return None

    def scalar(key):
        return data[key].item() if key in data else None

    if scalar('schema_version') != SCHEMA_VERSION:
        log(f'[MotionField] value function {path} has schema version '
            f'{scalar("schema_version")}, expected {SCHEMA_VERSION}.')
        return None

    for key, expected in (expect or {}).items():
        if key not in _GATE_KEYS or expected is None:
            continue
        if key not in data:
            log(f'[MotionField] value function {path} is missing "{key}".')
            return None
        if _mismatch(scalar(key), expected):
            log(f'[MotionField] value function {path} is stale: {key} is '
                f'{scalar(key)!r} but the runtime expects {expected!r}. '
                f'Retrain the motion field.')
            return None

    return ValueFunctionData(
        values=np.ascontiguousarray(data['value_function'], dtype=np.float32),
        scores=np.ascontiguousarray(data['scores'], dtype=np.float32),
        thetas=np.ascontiguousarray(data['thetas'], dtype=np.float32),
        theta_count=int(scalar('theta_count')),
        theta_spacing=float(scalar('theta_spacing')),
        gamma=float(scalar('gamma')),
        k_neighbors=int(scalar('k_neighbors')),
        tug_ratio=float(scalar('tug_ratio')),
        tug_mode=str(scalar('tug_mode')),
        pos_weight=float(scalar('pos_weight')),
        vel_weight=float(scalar('vel_weight')),
        frame_time=float(scalar('frame_time')),
        states_count=int(scalar('states_count')),
        bone_count=int(scalar('bone_count')),
    )


def save_tables(path: str, delta_yaw: np.ndarray, value_indices: np.ndarray,
                value_weights: np.ndarray, params: dict, signature: dict) -> None:
    """Write the `<name>.mftables.npz` rebuild cache."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    payload = {
        'schema_version': np.int64(SCHEMA_VERSION),
        'delta_yaw': np.ascontiguousarray(delta_yaw, dtype=np.float32),
        # int32, not the legacy int16: JLData has ~19k states and would
        # silently wrap around.
        'value_indices': np.ascontiguousarray(value_indices, dtype=np.int32),
        'value_weights': np.ascontiguousarray(value_weights, dtype=np.float32),
        'tables_json': np.str_(json.dumps(params, sort_keys=True, default=str)),
    }
    payload.update({k: np.str_(v) for k, v in signature.items()})
    np.savez(path, **payload)


def load_tables(path: str, params: dict, signature: dict, log=print):
    """
    Load the transition-table cache, or None if absent or stale.

    Staleness is decided on the database hashes plus every parameter that feeds
    the precompute (k, tug ratio, feature weights, frame time).
    """
    if not path or not os.path.isfile(path):
        return None
    try:
        with np.load(path, allow_pickle=False) as f:
            data = {k: f[k] for k in f.files}
    except Exception as exc:  # noqa: BLE001
        log(f'[MotionField] could not read table cache {path}: {exc}')
        return None

    if data.get('schema_version', np.int64(-1)).item() != SCHEMA_VERSION:
        return None
    for key, expected in signature.items():
        if str(data.get(key, '')) != str(expected):
            return None
    if str(data.get('tables_json', '')) != json.dumps(params, sort_keys=True, default=str):
        return None

    return (np.asarray(data['delta_yaw'], dtype=np.float32),
            np.asarray(data['value_indices'], dtype=np.int32),
            np.asarray(data['value_weights'], dtype=np.float32))
