#!/usr/bin/env python3
"""Train upstream ObsPointNet for a centred rectangular differential robot.

Rectangle projection provides the exact solution to upstream DUNETrain's dual
problem outside the footprint (and mu=0 is optimal inside). This avoids invoking
ECOS separately for every point. No environment/trajectory training is needed.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import torch


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--upstream', type=Path, help='Optional upstream NeuPAN checkout; defaults to the bundled ObsPointNet')
    p.add_argument('--cpp', type=Path, help='libneupan source; defaults to the bundled neupan_cpp_ws')
    p.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'models')
    p.add_argument('--length', type=float, default=0.4)
    p.add_argument('--width', type=float, default=0.4)
    p.add_argument('--steps', type=int, default=16000)
    a = p.parse_args()
    if a.cpp is None:
        a.cpp = next((parent/'neupan_cpp_ws/src/neupan_cpp/libneupan'
                      for parent in Path(__file__).resolve().parents
                      if (parent/'neupan_cpp_ws/src/neupan_cpp/libneupan/CMakeLists.txt').is_file()), None)
        if a.cpp is None:
            p.error('Cannot find bundled libneupan; specify --cpp /path/to/libneupan')
    assert a.length > 0 and a.width > 0 and a.steps > 0
    torch.set_num_threads(2)
    torch.manual_seed(42)
    np.random.seed(42)
    architecture = (a.upstream/'neupan/blocks/obs_point_net.py' if a.upstream else
                    Path(__file__).resolve().with_name('obs_point_net.py'))
    model = module('obs_point_net', architecture).ObsPointNet()
    sys.path.insert(0, str(a.cpp/'tools'))
    from nptf import write_nptf
    from export_dune_weights import rectangle_gh
    G0, h0 = rectangle_gh(a.length, a.width)
    G, h = torch.tensor(G0, dtype=torch.float32), torch.tensor(h0[:,0], dtype=torch.float32)
    half = torch.tensor([a.length/2, a.width/2])

    def labels(x):
        delta = x - torch.clamp(x, -half, half)
        distance = delta.norm(dim=1)
        normal = delta / distance.clamp_min(1e-9)[:,None]
        mu = torch.stack([(-normal[:,1]).clamp_min(0)/a.length,
                          normal[:,0].clamp_min(0)/a.width,
                          normal[:,1].clamp_min(0)/a.length,
                          (-normal[:,0]).clamp_min(0)/a.width], dim=1)
        return mu, distance

    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    a.output.mkdir(parents=True, exist_ok=True)
    # Mix near-boundary examples with the full 6 m local sensing range.
    for step in range(a.steps):
        x = (torch.rand(1024,2)*2-1)*torch.where(torch.rand(1024,1)<0.7, 0.8, 6.0)
        mu, distance = labels(x)
        out = model(x)
        dual = (out*(x@G.T-h)).sum(1)
        normal = out@G
        loss = (out-mu).square().mean() + (dual-distance).square().mean() + (normal-mu@G).square().mean()
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if step == int(a.steps*0.7):
            for group in optimizer.param_groups: group['lr'] *= 0.2
        if step % 1000 == 0: print(f'step={step} loss={loss.item():.6f}', flush=True)
    model.eval()
    with torch.no_grad():
        x=(torch.rand(50000,2)*2-1)*torch.where(torch.rand(50000,1)<0.7,0.8,6.0)
        mu,distance=labels(x)
        out=model(x)
        error=((out*(x@G.T-h)).sum(1)-distance).abs()
        near=error[distance<0.3]
        report={'length':a.length,'width':a.width,'height':0.5,'wheelbase':0.0,
                'seed':42,'steps':a.steps,'validation_samples':len(x),
                'distance_mae_m':error.mean().item(), 'distance_p99_m':torch.quantile(error,0.99).item(),
                'near_distance_mae_m':near.mean().item(),'near_distance_p99_m':torch.quantile(near,0.99).item(),
                'near_distance_max_m':near.max().item(),
                'label_method':'exact rectangle projection; upstream dual objective',
                'upstream_architecture_sha256':hashlib.sha256(architecture.read_bytes()).hexdigest()}
    torch.save(model.state_dict(),a.output/'rectangle.pth')
    records=[]
    for i,layer in enumerate(model.MLP):
        prefix=f'L{i:02d}'
        if isinstance(layer,torch.nn.Linear):
            records.extend([(prefix+'.linear.weight',layer.weight.detach().numpy()),(prefix+'.linear.bias',layer.bias.detach().numpy())])
        elif isinstance(layer,torch.nn.LayerNorm):
            records.extend([(prefix+'.ln.gamma',layer.weight.detach().numpy()),(prefix+'.ln.beta',layer.bias.detach().numpy())])
        elif isinstance(layer,torch.nn.Tanh): records.append((prefix+'.tanh',np.zeros((0,0),np.float32)))
        elif isinstance(layer,torch.nn.ReLU): records.append((prefix+'.relu',np.zeros((0,0),np.float32)))
        else: raise TypeError(type(layer))
    records.extend([('meta.G',G0),('meta.h',h0)])
    output=a.output/'diff_0p4.bin'
    write_nptf(output,records)
    write_nptf(a.output/'validation.nptf', [('points', x[:256].numpy().T),
                                            ('mu', out[:256].numpy().T)])
    report['model_sha256']=hashlib.sha256(output.read_bytes()).hexdigest()
    (a.output/'training_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if report['near_distance_p99_m'] > 0.05:
        raise RuntimeError('Model near-field error exceeds 5 cm; increase training before deployment')

if __name__=='__main__': main()
