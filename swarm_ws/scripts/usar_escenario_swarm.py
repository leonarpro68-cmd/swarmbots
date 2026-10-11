#!/usr/bin/env python3
"""Ejecuta colocación o misión usando el mismo JSON del escenario aleatorio."""
from __future__ import annotations
import argparse
import sys
import colocar_basuras_depositos as place
import grafo_basuras_dinamico as dyn
from escenario_aleatorio_swarm import load_scenario


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('action', choices=['place', 'dry-run', 'mission'])
    args, extra = ap.parse_known_args()
    data, elements = load_scenario(args.scenario)
    # Ambos módulos importan ELEMENTS como constante; deben apuntar al mismo escenario.
    place.ELEMENTS = elements
    dyn.ELEMENTS = elements
    print(f'Escenario compartido: {args.scenario}; semilla={data["seed"]}; acción={args.action}', flush=True)
    if args.action in ('place', 'dry-run'):
        sys.argv = [sys.argv[0]] + (['--dry-run'] if args.action == 'dry-run' else []) + extra
        place.main()
    else:
        import cbs_mision_completa_ros as mission
        sys.argv = [sys.argv[0]] + extra
        mission.main()


if __name__ == '__main__':
    main()
