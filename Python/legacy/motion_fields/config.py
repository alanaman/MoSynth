"""Configuration for the Motion Fields pipeline.

All values here mirror the notebook
``Motion Fields For Interactive Character Animation.ipynb`` (base variant).
Paths are anchored on the repository root so the pipeline can be run from any
working directory.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# motion_fields/config.py -> motion_fields -> AnimationPapers -> labs -> repo root
PACKAGE_DIR = Path(__file__).resolve().parent
PAPER_DIR = PACKAGE_DIR.parent
REPO_ROOT = PACKAGE_DIR.parents[2]

BVH_DIR = REPO_ROOT / 'resources' / 'lafan1' / 'BVH'

# The 16 clips loaded by the notebook, in order. Indices into this list are what
# ``walk_ranges`` / ``jog_ranges`` refer to.
BVH_FILES = [
    'walk1_subject1.bvh',   # 0
    'walk1_subject2.bvh',   # 1
    'walk1_subject5.bvh',   # 2
    'walk2_subject1.bvh',   # 3
    'walk2_subject3.bvh',   # 4
    'walk2_subject4.bvh',   # 5
    'walk3_subject1.bvh',   # 6
    'walk3_subject2.bvh',   # 7
    'walk3_subject3.bvh',   # 8
    'walk3_subject4.bvh',   # 9
    'walk3_subject5.bvh',   # 10
    'walk4_subject1.bvh',   # 11
    'run1_subject2.bvh',    # 12
    'run1_subject5.bvh',    # 13
    'run2_subject1.bvh',    # 14
    'run2_subject4.bvh',    # 15
]

# (animation index, start frame, end frame) -- the notebook's add_states() calls.
WALK_RANGES = [(2, 100, 2800)]
JOG_RANGES = [
    (15, 1200, 1800),
    (15, 3450, 3860),
    (14, 180, 800),
    (13, 200, 2300),
]

# Per-bone weights used by the distance metric. 23 entries, one per bone.
METRIC_WEIGHTS = [0, .3, .1, .1, .5, .01, .01, .01, .01, .01, .01,
                  .01, .01, .01, .01, .2, .5, 1, 1, .2, .5, 1, 1]

# Defined in the notebook but never consumed by build_distance_metric().
# Kept verbatim so the port stays faithful; it is dead weight today.
METRIC_VELOCITY_WEIGHTS = [1, .8, .5, .1, .1, .5, .1, 0, 0, 0, 0, 0, 0, 0, 0,
                           1.2, 1.5, 2, 0, 1.2, 1.5, 2, 0]


@dataclass
class Config:
    """Every knob that affects the pipeline output."""

    # --- data ---
    character_usd: str = 'AnimLabSimpleMale.usd'
    bvh_files: list = field(default_factory=lambda: list(BVH_FILES))
    walk_ranges: list = field(default_factory=lambda: list(WALK_RANGES))
    jog_ranges: list = field(default_factory=lambda: list(JOG_RANGES))
    contact_velfactor: float = 0.1

    # --- similarity metric ---
    metric_weights: list = field(default_factory=lambda: list(METRIC_WEIGHTS))
    metric_velocity_weights: list = field(
        default_factory=lambda: list(METRIC_VELOCITY_WEIGHTS))

    # --- motion field integration ---
    k_neighbors: int = 15
    tug_ratio: float = 0.1
    tug_ratio_learning: float = 0.1

    # --- value function training ---
    theta_count: int = 17
    epochs: int = 300
    gamma: float = 0.99
    locomotion_factor: float = 1.0

    # --- runtime ---
    device: str = 'auto'          # auto | cuda | cpu
    query_chunk: int = 256        # k-NN query batch size (memory bound)
    max_states: int = 0           # 0 = use every state; >0 truncates (smoke test)

    # ------------------------------------------------------------------
    @property
    def bvh_paths(self):
        return [BVH_DIR / name for name in self.bvh_files]

    @property
    def thetas(self):
        return np.linspace(-np.pi, np.pi, self.theta_count + 1)[:self.theta_count]

    @property
    def theta_spacing(self):
        return 2.0 * np.pi / self.theta_count

    @property
    def _suffix(self):
        return f'_s{self.max_states}' if self.max_states else ''

    @property
    def states_cache_path(self):
        # Not suffixed: states are always built in full, then truncated in memory.
        return PAPER_DIR / 'motion_fields_states_base.npz'

    @property
    def tables_cache_path(self):
        return PAPER_DIR / f'motion_fields_tables_base{self._suffix}.npz'

    @property
    def value_function_path(self):
        return PAPER_DIR / f'motion_fields_value_function_base{self._suffix}.npz'

    # ------------------------------------------------------------------
    def states_signature(self):
        """Canonical JSON of everything the state database depends on."""
        return json.dumps({
            'character_usd': self.character_usd,
            'bvh_files': self.bvh_files,
            'walk_ranges': self.walk_ranges,
            'jog_ranges': self.jog_ranges,
            'contact_velfactor': self.contact_velfactor,
            'metric_weights': self.metric_weights,
        }, sort_keys=True)

    def tables_signature(self):
        """Canonical JSON of everything the transition tables depend on."""
        return json.dumps({
            'states': self.states_signature(),
            'k_neighbors': self.k_neighbors,
            'tug_ratio_learning': self.tug_ratio_learning,
            'max_states': self.max_states,
        }, sort_keys=True)

    def training_signature(self):
        return json.dumps({
            'tables': self.tables_signature(),
            'theta_count': self.theta_count,
            'epochs': self.epochs,
            'gamma': self.gamma,
            'locomotion_factor': self.locomotion_factor,
        }, sort_keys=True)
