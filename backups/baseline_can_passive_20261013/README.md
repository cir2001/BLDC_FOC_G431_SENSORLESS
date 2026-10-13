# CAN passive baseline — 2026-10-13

CAN_PHASE1_PASSIVE_TEST=1. User verified STOP heartbeat, repeated sequence timeout, RUN target write, watchdog latch, STOP recovery, and MOE=0.

Evidence: blocked RUN rx_rpm=700 target=600 writes=4818 watchdog=1; recovered target=700 writes=5014 then 5168 watchdog=0; final writes=5613 connected=0 watchdog=1, unchanged across three reads.

These are user-provided summaries, not captured raw trace files. Running-motor link-loss protection remains untested.

Check: `python backups/baseline_can_passive_20261013/restore.py --check`
Restore: `python backups/baseline_can_passive_20261013/restore.py --apply`
Restore saves current files into a pre_restore backup first, then restores the entire source snapshot (including removing extra project files). It does not flash. Rebuild before flashing. Firmware hashes are in firmware_sha256.json.
