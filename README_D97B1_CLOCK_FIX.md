# D.9.7B.1 — Diagnóstico y corrección de recepción /clock

- Explicit QoS for /clock: BEST_EFFORT, VOLATILE, KEEP_LAST(10).
- DDS discovery grace period up to 5 seconds when fewer than 2 distinct clock values received; configurable with `--clock-grace-seconds`.
- JSON now includes `/clock` publisher discovery and first/last timestamps.
- Does not bypass any safety decision, never sends nonzero speed.

Install on host from ~/swarmbots: `unzip -o ~/Descargas/swarm_D97B1_fix_clock_qos.zip -d ~/swarmbots`

Inside container, test:
`python3 -m unittest discover -s scripts/tests -p test_d97b1_zero_actuator.py -v`

Then **preflight only**:
`python3 scripts/d97b1_zero_actuator.py --warmup-seconds 3 --clock-grace-seconds 8 --output scripts/resultados/d9_live/d97b1_precheck_qos.json`

If PASS_PREFLIGHT_NO_PUBLISHERS, with all robots stationary and no other controllers:
`python3 scripts/d97b1_zero_actuator.py --send-zero --warmup-seconds 3 --clock-grace-seconds 8 --output scripts/resultados/d9_live/d97b1_zero_qos.json`
