"""Motion state database.

Loads the LAFAN1 clips, retargets them onto the AnimLab character, and turns
selected frame ranges into motion states ``m = (x, v)`` plus the "next" velocity
``y`` and foot contact flags.
"""

import time
from dataclasses import dataclass

import numpy as np
import ipyanimlab as lab

from .metric import build_metric_matrix, init_metric
from .pose import init_pose_module, pose_pack, pose_subtract, pose_unpack


@dataclass
class Dataset:
    states_x: np.ndarray          # (N, bones+2, 4) pose
    states_v: np.ndarray          # (N, bones+2, 4) velocity
    states_y: np.ndarray          # (N, bones+2, 4) next velocity
    states_c: np.ndarray          # (N, 2) left/right foot contact
    end_of_walk_ids: int          # states below this index are walking
    bones: list
    parents: np.ndarray
    default_skeleton_p: np.ndarray
    metric_matrix: np.ndarray     # (N, bones*2, 3)

    @property
    def states_count(self):
        return self.states_x.shape[0]

    @property
    def bone_count(self):
        return self.states_x.shape[1] - 2


def load_animations(cfg):
    """Import every configured BVH, retargeted onto the character skeleton."""
    # The Viewer is created purely to obtain the character asset that AnimMapper
    # needs -- it is never displayed and no GL commands are ever flushed.
    viewer = lab.Viewer(move_speed=5, width=1280, height=720)
    character = viewer.import_usd_asset(cfg.character_usd)

    animmap = lab.AnimMapper(character, keep_translation=False, root_motion=True,
                             match_effectors=True, local_offsets={'Hips': [0, 2, 0]})

    animations = []
    for path in cfg.bvh_paths:
        if not path.exists():
            raise FileNotFoundError(f'BVH not found: {path}')
        animations.append(lab.import_bvh(str(path), anim_mapper=animmap))

    return character, animations


def extract_contacts(animations, parents, bones, velfactor):
    left, right = bones.index('LeftToe'), bones.index('RightToe')
    contacts = []
    for anim in animations:
        _, p = lab.utils.quat_fk(anim.quats, anim.pos, parents)
        contacts.append(lab.utils.extract_feet_contacts(p, left, right, velfactor))
    return contacts


def _add_states(out, cursor, quats, pos, lcontact, rcontact):
    """Append one state per frame triplet (a, b, c) of the given range."""
    states_x, states_v, states_y, states_c = out

    for i in range(quats.shape[0] - 2):
        a = pose_pack(pos[i, 0, :].copy(), pos[i, 1, :].copy(), quats[i, ...].copy())
        b = pose_pack(pos[i + 1, 0, :].copy(), pos[i + 1, 1, :].copy(), quats[i + 1, ...].copy())
        c = pose_pack(pos[i + 2, 0, :].copy(), pos[i + 2, 1, :].copy(), quats[i + 2, ...].copy())

        y = pose_subtract(c, b)
        v = pose_subtract(b, a)

        # States are stored root-relative: origin at zero, facing identity.
        a = pose_unpack(a)
        a.root[:] = 0
        a.quats[0, :] = [1, 0, 0, 0]
        a = pose_pack(a.root, a.hips, a.quats)

        states_x[cursor, :, :] = a
        states_v[cursor, :, :] = v
        states_y[cursor, :, :] = y
        states_c[cursor, 0] = lcontact[i]
        states_c[cursor, 1] = rcontact[i]
        cursor += 1

    return cursor


def build_states(cfg, animations, contacts, bone_count):
    ranges = [('walk', r) for r in cfg.walk_ranges] + [('jog', r) for r in cfg.jog_ranges]
    total = sum(max(0, end - start - 2) for _, (_, start, end) in ranges)

    shape = (total, bone_count + 2, 4)
    out = (np.zeros(shape, dtype=np.float32),
           np.zeros(shape, dtype=np.float32),
           np.zeros(shape, dtype=np.float32),
           np.zeros((total, 2), dtype=np.bool_))

    cursor = 0
    end_of_walk_ids = 0
    for group, (anim_id, start, end) in ranges:
        timings = slice(start, end)
        anim = animations[anim_id]
        cursor = _add_states(out, cursor, anim.quats[timings], anim.pos[timings],
                             contacts[anim_id][0][timings], contacts[anim_id][1][timings])
        if group == 'walk':
            end_of_walk_ids = cursor

    return out, end_of_walk_ids


def build(cfg):
    """Full rebuild: BVH -> states -> metric matrix."""
    print('  loading animations...')
    t0 = time.time()
    _, animations = load_animations(cfg)
    bones = list(animations[0].bones)
    parents = np.asarray(animations[0].parents)
    bone_count = len(bones)
    print(f'    {len(animations)} clips, {bone_count} bones ({time.time() - t0:.1f}s)')

    default_skeleton_p = animations[0].pos[0, ...].copy()
    default_skeleton_p[0] = 0

    init_pose_module(bone_count, default_skeleton_p)
    init_metric(bone_count, parents, default_skeleton_p, cfg.metric_weights)

    print('  extracting foot contacts...')
    contacts = extract_contacts(animations, parents, bones, cfg.contact_velfactor)

    print('  building motion states...')
    (states_x, states_v, states_y, states_c), end_of_walk_ids = build_states(
        cfg, animations, contacts, bone_count)
    print(f'    {states_x.shape[0]} states (end_of_walk_ids={end_of_walk_ids})')

    print('  building metric matrix...')
    metric_matrix = build_metric_matrix(states_x, states_v)

    return Dataset(states_x=states_x, states_v=states_v, states_y=states_y,
                   states_c=states_c, end_of_walk_ids=end_of_walk_ids, bones=bones,
                   parents=parents, default_skeleton_p=default_skeleton_p,
                   metric_matrix=metric_matrix)


def save(dataset, path, signature):
    np.savez(path,
             states_x=dataset.states_x,
             states_v=dataset.states_v,
             states_y=dataset.states_y,
             states_c=dataset.states_c,
             metric_matrix=dataset.metric_matrix,
             end_of_walk_ids=dataset.end_of_walk_ids,
             bones=np.array(dataset.bones),
             parents=dataset.parents,
             default_skeleton_p=dataset.default_skeleton_p,
             config_json=signature)


def load(path, cfg):
    """Return the cached Dataset, or None if the cache is absent or stale."""
    if not path.exists():
        print(f'  cache miss: {path.name} does not exist')
        return None

    with np.load(path, allow_pickle=False) as data:
        if str(data['config_json']) != cfg.states_signature():
            print(f'  cache stale: {path.name} was built with a different config')
            return None

        bones = [str(b) for b in data['bones']]
        dataset = Dataset(
            states_x=data['states_x'], states_v=data['states_v'],
            states_y=data['states_y'], states_c=data['states_c'],
            end_of_walk_ids=int(data['end_of_walk_ids']), bones=bones,
            parents=data['parents'], default_skeleton_p=data['default_skeleton_p'],
            metric_matrix=data['metric_matrix'])

    # The metric weights are part of the cache signature, so the config's copy is
    # the one that produced the cached metric matrix.
    init_pose_module(dataset.bone_count, dataset.default_skeleton_p)
    init_metric(dataset.bone_count, dataset.parents, dataset.default_skeleton_p,
                cfg.metric_weights)
    return dataset


def load_or_build(cfg, force=False):
    """Load the state database from cache, rebuilding it when needed."""
    signature = cfg.states_signature()
    dataset = None if force else load(cfg.states_cache_path, cfg)

    if dataset is None:
        dataset = build(cfg)
        print(f'  writing {cfg.states_cache_path.name}...')
        save(dataset, cfg.states_cache_path, signature)
    else:
        print(f'  loaded {dataset.states_count} states from {cfg.states_cache_path.name}')

    return dataset


def truncate(dataset, max_states):
    """Shrink the database for fast smoke runs."""
    if not max_states or max_states >= dataset.states_count:
        return dataset

    n = int(max_states)
    print(f'  truncating state database to {n} states')
    return Dataset(
        states_x=dataset.states_x[:n], states_v=dataset.states_v[:n],
        states_y=dataset.states_y[:n], states_c=dataset.states_c[:n],
        end_of_walk_ids=min(dataset.end_of_walk_ids, n), bones=dataset.bones,
        parents=dataset.parents, default_skeleton_p=dataset.default_skeleton_p,
        metric_matrix=dataset.metric_matrix[:n])
