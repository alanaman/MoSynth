"""Entry point for the Motion Fields pipeline.

Runs, in order:

  1. build (or load) the motion state database and its metric matrix
  2. build the k-NN index
  3. recompute the transition tables if the cache is missing or stale
  4. train the walk and jog value functions
  5. write the value functions to disk
"""

import argparse
import time

import numpy as np

from . import dataset as dataset_mod
from . import precompute, train
from .config import Config
from .knn import KnnIndex, resolve_device


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog='motion_fields',
        description='Train motion field value functions (Lee et al. 2010).')
    p.add_argument('--force-states', action='store_true',
                   help='rebuild the state database even if the cache is valid')
    p.add_argument('--force-tables', action='store_true',
                   help='rebuild the transition tables even if the cache is valid')
    p.add_argument('--epochs', type=int, help='value iteration epochs')
    p.add_argument('--k-neighbors', type=int, help='neighbours per state')
    p.add_argument('--theta-count', type=int, help='goal direction bins')
    p.add_argument('--gamma', type=float, help='discount factor')
    p.add_argument('--device', choices=['auto', 'cuda', 'cpu'], default='auto')
    p.add_argument('--query-chunk', type=int, help='k-NN query batch size')
    p.add_argument('--max-states', type=int, default=0,
                   help='truncate the state database (fast smoke test)')
    p.add_argument('--out', help='output path for the value function .npz')
    return p.parse_args(argv)


def build_config(args):
    cfg = Config()
    for name in ('epochs', 'k_neighbors', 'theta_count', 'gamma', 'query_chunk',
                 'max_states', 'device'):
        value = getattr(args, name, None)
        if value is not None:
            setattr(cfg, name, value)
    return cfg


def main(argv=None):
    args = parse_args(argv)
    cfg = build_config(args)
    started = time.time()

    print('=' * 70)
    print('Motion Fields for Interactive Character Animation')
    print('=' * 70)

    print('\n[1/5] Motion state database')
    ds = dataset_mod.load_or_build(cfg, force=args.force_states)
    ds = dataset_mod.truncate(ds, cfg.max_states)
    print(f'  {ds.states_count} states, {ds.bone_count} bones, '
          f'end_of_walk_ids={ds.end_of_walk_ids}')

    print('\n[2/5] k-NN index')
    device = resolve_device(cfg.device)
    print(f'  device: {device}')
    knn = KnnIndex(ds.metric_matrix, device, cfg.query_chunk)

    print('\n[3/5] Transition tables')
    tables = precompute.load_or_build(cfg, ds, knn, force=args.force_tables)

    print('\n[4/5] Value function training')
    walk, walk_scores, jog, jog_scores = train.train_all(tables, ds, cfg, device)

    for name, scores in (('walk', walk_scores), ('jog', jog_scores)):
        final = scores[-1]
        print(f'  {name}: final Bellman residual '
              f'min={final[0]:.6f} max={final[1]:.6f} mean={final[2]:.6f}')

    print('\n[5/5] Saving')
    out_path = cfg.value_function_path if args.out is None else args.out
    train.save(out_path, cfg, ds, walk, walk_scores, jog, jog_scores)
    print(f'  value function written to {out_path}')
    print(f'  walk {walk.shape} finite={np.isfinite(walk).all()}  '
          f'jog {jog.shape} finite={np.isfinite(jog).all()}')

    print(f'\nTotal time: {time.time() - started:.1f}s')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
