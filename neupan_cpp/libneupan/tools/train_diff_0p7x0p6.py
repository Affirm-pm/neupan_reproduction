"""Train the simulator's wheel-inclusive DUNE model with upstream NeuPAN.

Run with an environment containing the upstream ``neupan`` Python package:

    python train_diff_0p7x0p6.py --init-checkpoint OLD_MODEL.pth --epochs 1500

Checkpoints and training results are written to ``model/diff_0p7x0p6/``
beside this script. Export the final checkpoint with export_dune_weights.py.
"""

import argparse
from pathlib import Path

import numpy as np
import torch
from neupan import neupan


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--init-checkpoint", type=Path,
                        help="Optional DUNE weights used as initialization")
    parser.add_argument("--epochs", type=int,
                        help="Override the epoch count in the YAML config")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    config = Path(__file__).with_name("dune_train_diff_0p7x0p6.yaml")
    planner = neupan.init_from_yaml(str(config))
    if args.init_checkpoint:
        weights = torch.load(args.init_checkpoint, map_location="cpu")
        planner.pan.dune_layer.model.load_state_dict(weights)
        print(f"Initialized from {args.init_checkpoint}", flush=True)
    if args.epochs is not None:
        if args.epochs < 1:
            parser.error("--epochs must be at least 1")
        planner.dune_train_kwargs["epoch"] = args.epochs
    planner.train_dune()
