"""Fitted value function training.

One scalar V(s) is stored per (database state, goal direction) pair. Values are
refined by repeated Bellman backups:

    V(m) <- max_a [ R(s, a) + gamma * sum_j w_j V(s'_j) ]

Because the successor value is read through k-NN interpolation weights, the
discretely-stored function behaves as a continuous one over the motion field.
The goal direction theta is discretised too, and interpolated linearly between
the two adjacent grid angles.
"""

import time

import numpy as np
import torch

from . import progress


def train_value_function(tables, dataset, cfg, device, is_walking, factor=1.0):
    """Returns (V, scores) where V is (states, thetas) and scores is (epochs, 3)."""
    states_count = dataset.states_count
    k = cfg.k_neighbors
    theta_count = cfg.theta_count
    pi = torch.pi

    scores = torch.zeros([cfg.epochs, 3])
    V = torch.zeros([states_count, theta_count], device=device)

    # Row 2 of a packed pose is the root quaternion.
    root_quat = torch.tensor(tables.states_x[:, :, 2, :], device=device)
    value_indices = torch.tensor(tables.value_indices, device=device).to(torch.long)
    value_weights = torch.tensor(tables.value_weights, device=device)
    theta_grid = torch.tensor(cfg.thetas, device=device)

    # Yaw change produced by each action, extracted from the root quaternion.
    delta_orientation = torch.atan2(
        2. * root_quat[..., 0] * root_quat[..., 2],
        1.0 - 2. * root_quat[..., 2] ** 2.)
    del root_quat

    # Reward bonus for staying in the requested locomotion type.
    state_score = torch.zeros([states_count], device=device)
    if is_walking:
        state_score[:dataset.end_of_walk_ids] = 1
    else:
        state_score[dataset.end_of_walk_ids:] = 1

    # Goal angle after taking each action, per state / action / goal.
    expand_theta_grid = theta_grid.view(1, 1, theta_count)
    expand_delta_orientation = delta_orientation.view(states_count, k, 1)
    next_orientation = expand_theta_grid + expand_delta_orientation
    next_orientation = (next_orientation + pi) % (2.0 * pi) - pi

    rewards = -torch.abs(next_orientation) + state_score.unsqueeze(-1).unsqueeze(-1) * factor

    # Accessors for the theta interpolation.
    step = (2 * pi) / theta_count
    coord = (next_orientation + pi) / step
    i0 = torch.floor(coord).to(torch.long) % theta_count
    i1 = (i0 + 1) % theta_count
    alpha = coord - torch.floor(coord)

    i0_exp = i0.unsqueeze(2).expand(-1, -1, k, -1)
    i1_exp = i1.unsqueeze(2).expand(-1, -1, k, -1)

    step = progress.interval(25, cfg.epochs, fraction=4)
    t0 = time.time()
    for epoch in range(cfg.epochs):
        V_neighbors = V[value_indices]

        V0 = torch.gather(V_neighbors, 3, i0_exp)
        V1 = torch.gather(V_neighbors, 3, i1_exp)

        # Task interpolation, then motion interpolation.
        V_task = (1.0 - alpha).unsqueeze(2) * V0 + alpha.unsqueeze(2) * V1
        exp_next_V = (V_task * value_weights.unsqueeze(-1)).sum(dim=2)

        # Bellman backup, then max over actions.
        Q = rewards + cfg.gamma * exp_next_V
        V_prime = Q.max(dim=1).values

        bellman_residual = torch.abs(V - V_prime)
        scores[epoch, 0] = bellman_residual.min()
        scores[epoch, 1] = bellman_residual.max()
        scores[epoch, 2] = bellman_residual.mean()
        V = V_prime

        if (epoch + 1) % step == 0 or epoch == cfg.epochs - 1:
            progress.line(f'    epoch {epoch + 1}/{cfg.epochs}  '
                          f'residual mean={scores[epoch, 2]:.6f} '
                          f'max={scores[epoch, 1]:.6f}')

    progress.end()
    print(f'    done in {time.time() - t0:.1f}s')
    return V.cpu().numpy(), scores.numpy()


def train_all(tables, dataset, cfg, device):
    print('  training walk value function...')
    walk, walk_scores = train_value_function(
        tables, dataset, cfg, device, is_walking=True, factor=cfg.locomotion_factor)

    print('  training jog value function...')
    jog, jog_scores = train_value_function(
        tables, dataset, cfg, device, is_walking=False, factor=cfg.locomotion_factor)

    return walk, walk_scores, jog, jog_scores


def save(path, cfg, dataset, walk, walk_scores, jog, jog_scores):
    np.savez(path,
             value_function_walk=walk,
             value_function_jog=jog,
             scores_walk=walk_scores,
             scores_jog=jog_scores,
             thetas=cfg.thetas,
             theta_count=cfg.theta_count,
             theta_spacing=cfg.theta_spacing,
             states_count=dataset.states_count,
             end_of_walk_ids=dataset.end_of_walk_ids,
             k_neighbors=cfg.k_neighbors,
             gamma=cfg.gamma,
             epochs=cfg.epochs,
             tug_ratio_learning=cfg.tug_ratio_learning,
             bones=np.array(dataset.bones),
             parents=dataset.parents,
             config_json=cfg.training_signature())
