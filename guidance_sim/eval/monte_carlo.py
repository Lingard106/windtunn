# -*- coding: utf-8 -*-
"""Monte Carlo 打靶 — 初始扰动撒点 → 批量闭环 → CEP 统计。"""
import numpy as np
from eval.run_mission import run_mission, default_cfg


def monte_carlo(n=20, seed0=42, verbose=True):
    rng = np.random.default_rng(seed0)
    misses, hits = [], 0
    for i in range(n):
        cfg = default_cfg()
        cfg["p0"] = (np.array(cfg["p0"]) + [0, 0, rng.uniform(-3, 3)]).tolist()
        cfg["v0"] = (np.array(cfg["v0"]) + [rng.uniform(-2, 2),
                     rng.uniform(-1, 1), 0]).tolist()
        cfg["rpy0"][2] += rng.uniform(-0.09, 0.09)
        out = run_mission(cfg=cfg, seed=seed0 + i)
        misses.append(out["miss"] if out["miss"] is not None else 99.0)
        hits += int(out["hit"])
        if verbose:
            print(f"[{i + 1}/{n}] hit={out['hit']} miss={misses[-1]:.2f} "
                  f"reason={out['reason']}")
    misses = np.array(misses)
    print(f"=== MC SUMMARY: {hits}/{n} hit ({100 * hits / n:.0f}%), "
          f"CEP50={np.percentile(misses, 50):.2f}m, CEP90={np.percentile(misses, 90):.2f}m")
    return misses
