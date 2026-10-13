# Soft-current experimental baseline — 2026-10-13

50 RPM/s; soft envelope ON 3.6A/OFF 3.3A; hard trip 4.5A.
User test: peak 3.660A, soft entries 5, 600 RPM last-window error +1.29%, return 500 error -2.41%. Dynamics not fully converged. Watchdog software off upper bound 50002us.
Source snapshot includes tested 8/10/10-second script. Firmware artifacts are existing locally built can_active_500 outputs, not read back from user's board. Original supplied log: user_run_log.txt.

Check: python backups/baseline_can_softcurrent_20261013/restore.py --check
Restore: python backups/baseline_can_softcurrent_20261013/restore.py --apply
Restore saves current project first, restores snapshot files and removes extra project files. It does not flash; rebuild before flashing.
