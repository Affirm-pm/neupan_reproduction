# Wheel-inclusive simulator DUNE model

The robot's body is 0.60 × 0.40 m. Its four wheels extend the XY bounds to
0.70 × 0.60 m, centered on `base_link`. The trained footprint is this enclosing
rectangle; its corners include some empty space between the wheels.

`model_1500.pth` was trained on 100,000 new geometry labels sampled in
`[-10, 10] × [-10, 10]` m. It was initialized from the earlier 0.60 × 0.40 m
checkpoint at `/home/affirm/NeuPAN/example/dune_train/model/diff_0p6x0p4/model_5000.pth`
and trained for 1,500 epochs with seed 42. Run these commands from
`/home/affirm/FineNav-Engine_ws/src/neupan_cpp`:

```bash
/home/affirm/miniconda3/envs/neupan/bin/python \
  libneupan/tools/train_diff_0p7x0p6.py \
  --init-checkpoint /home/affirm/NeuPAN/example/dune_train/model/diff_0p6x0p4/model_5000.pth \
  --epochs 1500
```

The final validation distance MSE was `3.70e-7 m²`. On 20,000 independently
sampled points per region, `evaluate_dune_footprint.py` measured:

| Region | Mean absolute error | 95th percentile |
| --- | ---: | ---: |
| `[-6, 6] × [-6, 6]` m | 0.35 mm | 1.07 mm |
| `[-1, 1] × [-1, 1]` m | 0.70 mm | 1.78 mm |
| Within 0.20 m outside the sides | 0.91 mm | 2.03 mm |

`libneupan/models/diff_0p7x0p6.bin` is the exported C++ runtime model. The
export command was:

```bash
/home/affirm/miniconda3/envs/neupan/bin/python \
  libneupan/tools/export_dune_weights.py \
  libneupan/tools/model/diff_0p7x0p6/model_1500.pth \
  libneupan/models/diff_0p7x0p6.bin 0.7 0.6
```
