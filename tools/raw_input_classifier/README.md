# raw_input_classifier — learned anti-cheat adversary (A/B benchmark)

A PyTorch 2D-CNN that tries to tell **human** mouse input from **firmware-driven**
(aimbot/script) input over windows of raw HID reports — the "Raw Input
Classification" threat. Run it to measure whether the firmware's humanization
defeats a real learned classifier.

**Read the AUC the right way:** AUC **0.5 = the CNN is guessing = firmware WINS**.
AUC **1.0 = firmware fully exposed**. The per-channel ablation names which feature
leaks most.

## Layers
- `humanization_analyze.py` (sibling) — Tier-1 statistical, stdlib, always-on.
- this tool — Tier-2 learned, torch, on-demand. Complementary, not a replacement.

## Install
    pip install -e tools/raw_input_classifier[torch]      # adds torch + numpy
Capture-only (no training) needs no extras beyond stdlib (+pyserial for --port).

## Use
    # 1. capture labeled corpora (host-side; same schema upgrades to device 1 MHz t_us later)
    python -m raw_input_classifier capture --port /dev/ttys00N --label human     --source passthrough        --out human_A.jsonl
    python -m raw_input_classifier capture --port /dev/ttys00N --label synthetic --source firmware_humanized --out fw.jsonl

    # 2. verdict
    python -m raw_input_classifier train   --data 'corpus/*.jsonl' --encoder time
    python -m raw_input_classifier compare-encoders --data 'corpus/*.jsonl'

## Honesty gate
Below `--min-windows` (default 500) the tool prints `INSUFFICIENT DATA` instead of a
number — it will not report separability before a real corpus exists. There is no
synthetic-corpus fallback by design.

## Limitations (v1)
- Capture is host-clock timestamped; on-device 1 MHz `t_us` exfil is a future upgrade
  (same schema, no trainer change).
- `eval`/`compare-encoders` re-fit on the supplied corpus; frozen `--ckpt` persistence
  through the CLI is a small follow-up (the `BaseCNN.save/load` layer already supports it).
