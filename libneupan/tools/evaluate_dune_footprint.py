"""Compare a trained DUNE checkpoint's distances with a rectangle's true distance.

Usage: python evaluate_dune_footprint.py CHECKPOINT.pth LENGTH WIDTH
"""

import argparse

import numpy as np
import torch
from neupan.blocks import ObsPointNet


def evaluate(model, points, length, width):
    with torch.no_grad():
        mu = model(torch.as_tensor(points, dtype=torch.float32)).numpy()
    G = np.array([[0, -length], [width, 0], [0, length], [-width, 0]])
    h = length * width / 2
    predicted = np.sum(mu * (points @ G.T - h), axis=1)
    exact = np.linalg.norm(
        np.maximum(np.abs(points) - [length / 2, width / 2], 0), axis=1
    )
    error = np.abs(predicted - exact)
    return np.mean(error), np.percentile(error, 95), np.max(error)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint")
    parser.add_argument("length", type=float)
    parser.add_argument("width", type=float)
    args = parser.parse_args()

    model = ObsPointNet(2, 4)
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    model.eval()

    rng = np.random.default_rng(92551)
    scan = rng.uniform(-6, 6, size=(20000, 2))
    near = rng.uniform(-1, 1, size=(20000, 2))
    side = rng.integers(0, 4, size=20000)
    along = rng.uniform(-1, 1, size=20000)
    clearance = rng.uniform(0, 0.2, size=20000)
    edge = np.empty((20000, 2))
    edge[:, 0] = np.where(
        side < 2, (2 * side - 1) * (args.length / 2 + clearance),
        along * args.length / 2,
    )
    edge[:, 1] = np.where(
        side >= 2, (2 * side - 5) * (args.width / 2 + clearance),
        along * args.width / 2,
    )

    for name, points in (("scan [-6,6] m", scan),
                         ("near [-1,1] m", near),
                         ("within 0.2 m of sides", edge)):
        mean, p95, worst = evaluate(model, points, args.length, args.width)
        print(f"{name}: MAE={mean:.6f} m, p95={p95:.6f} m, max={worst:.6f} m")
