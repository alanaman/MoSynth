"""
On-disk format for the trained motion field value function.

Training produces one artefact: `<name>.mffield.npz`, the value function itself,
written into StreamingAssets so it ships with the player. The transition tables
it is fitted on are intermediate and rebuilt on every train.

Deciding whether a value function still matches the config that will run it is
Unity's job, not this module's: `MotionFieldConfig.hasTrained` is set by the
Train button and cleared the moment a field is edited, so the answer is known in
the inspector rather than at load time with the character already in play mode.
The loader here therefore takes whatever file it is handed.

The parameters and database hashes are still written into the npz. Nothing reads
them back: they are provenance, so a value function found on its own is still
answerable for what produced it.
"""

from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass

import numpy as np

# Which database state an action is tugged toward. 'emphasized' follows the
# paper's reference implementation: the neighbour that the action emphasises.
# 'nearest' always tugs toward the closest state, which makes the candidate
# actions nearly indistinguishable.
TUG_MODE = 'emphasized'

# Semantic version of the similarity feature itself, for changes no parameter
# expresses. Bumped to 2 when `build_motion_states` switched the metric FK to
# the rest-pose hips offset -- same weights, different feature, so anything
# trained before the change is worthless.
#
# Bumping it does not invalidate anything by itself: Unity clears `hasTrained`
# when a config field is edited, and a change in here moves no config field.
# Editing the metric means retraining every field by hand, and this number is
# what makes it obvious afterwards which files were left behind.
METRIC_VERSION = 2

# Signature of an all-ones per-bone weight table. Spelled out rather than hashed
# so a file written before per-bone weights existed -- which had uniform weights
# by definition -- still matches a runtime that has not set any.
UNIFORM_BONE_WEIGHTS = 'uniform'


def bone_weights_signature(bone_weights) -> str:
    """
    Stable identifier for a per-bone weight table.

    Bone weights change the similarity metric, so they change which neighbours a
    state has, so they invalidate every transition and value trained under the
    old table. Hashed rather than compared elementwise because the embedding's
    staleness check stores one scalar per key and a 23x2 table is not a scalar.
    """
    if bone_weights is None:
        return UNIFORM_BONE_WEIGHTS

    table = np.ascontiguousarray(bone_weights, dtype=np.float32)
    if table.size == 0 or np.all(np.abs(table - 1.0) <= 1e-6):
        return UNIFORM_BONE_WEIGHTS
    return hashlib.sha1(table.tobytes()).hexdigest()


def load_bone_weights_file(path: str | None):
    """
    Read a `{"JointName": [pos, vel], ...}` JSON file for the CLIs.

    Unity passes the same mapping straight across PythonNET, so this exists only
    to give the headless entry points parity with the Editor buttons.
    """
    if not path:
        return None
    with open(path, 'r', encoding='utf-8') as handle:
        return json.load(handle)


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
        'bone_weights_sha1': np.str_(params.get('bone_weights_sha1', UNIFORM_BONE_WEIGHTS)),
        'metric_version': np.int64(params.get('metric_version', METRIC_VERSION)),
        'locomotion_factor': np.float32(params['locomotion_factor']),
        'locomotion_speed_threshold': np.float32(params['locomotion_speed_threshold']),
        'feature_shape': np.asarray(params['feature_shape'], dtype=np.int64),
        'k_neighbors': np.int64(params['k_neighbors']),
        'tug_ratio': np.float32(params['tug_ratio']),
        'tug_mode': np.str_(params.get('tug_mode', TUG_MODE)),
        'gamma': np.float32(params['gamma']),
        'epochs': np.int64(params['epochs']),
        'config_json': np.str_(json.dumps(params, sort_keys=True, default=str)),
    }
    payload.update({k: np.str_(v) for k, v in signature.items()})

    np.savez(out_path, **payload)


def theta_grid(theta_count: int) -> np.ndarray:
    """The task-parameter grid: `theta_count` headings spanning [-pi, pi)."""
    return np.linspace(-np.pi, np.pi, theta_count + 1, dtype=np.float32)[:theta_count]


def load_value_function(path: str, log=print) -> ValueFunctionData | None:
    """
    Load a `.mffield.npz`.

    Whether this file belongs with the config about to use it is settled in
    Unity before the call; here a missing or unreadable file is the only reason
    to decline. Returns None rather than raising either way -- callers run inside
    `Py.GIL()` from Unity, where an exception surfaces as an opaque managed
    error, and an unusable value function should degrade to greedy control
    rather than take the player down.
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
        return data[key].item()

    try:
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
    except KeyError as exc:
        # A file written by an older layout. Nothing checks a format version, so
        # the missing key is the symptom that surfaces first.
        log(f'[MotionField] value function {path} is missing {exc}')
        return None
