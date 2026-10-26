"""Offline safety preflight on the original Hybrid V2 round-2 reference."""
import argparse
import json
from pathlib import Path
from .scenario import reference_inputs
from .regression_b import load_controls
from .safety import SafetySettings, certify


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--operational-margin', type=float, default=0.05)
    p.add_argument('--max-speed', type=float, default=1.20)
    p.add_argument('--max-accel', type=float, default=1.00)
    p.add_argument('--time-safety-factor', type=float, default=1.08)
    args = p.parse_args()
    paths, points, obstacles = reference_inputs()
    controls = load_controls(args.reference_dir / 'controles_slsqp_refinados.npz')
    settings = SafetySettings(args.operational_margin, args.max_speed,
                              args.max_accel, args.time_safety_factor)
    try:
        report = certify(controls, points, paths, obstacles, settings=settings)
    except ValueError as exc:
        report = {'certified':False, 'reason':str(exc)}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['certified'] else 2


if __name__=='__main__':
    raise SystemExit(main())
