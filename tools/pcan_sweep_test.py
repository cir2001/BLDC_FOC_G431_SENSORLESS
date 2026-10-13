#!/usr/bin/env python3
"""Active 500/600 RPM three-round repeat test. Requires 0x301/0x302/0x303/0x304 firmware.
Reset board before use, close other CAN transmitters, secure propeller-free motor.
Install: py -m pip install "python-can[pcan]"
"""
import argparse
import csv
import datetime
import json
from pathlib import Path
import struct
import time

# Repeated dynamic validation; calibrated feedforward firmware unchanged.
STAGES = ([('hold_500', 500, 10.0)]
          + [(f'round_{i}_{rpm}', rpm, 10.0)
             for i in range(1, 4) for rpm in (600, 500)]
          + [('final_hold_500', 500, 15.0)])


def summarize(samples, target, duration):
    tail = [x for x in samples if x[0] >= duration - 2.0]
    good = [x[1] for x in tail if x[2]]
    values = [x[1] for x in samples if x[2]]
    return dict(target=target, samples=len(samples), tail_samples=len(tail),
                tail_valid_fraction=sum(x[2] for x in tail)/len(tail) if tail else None,
                tail_mean_rpm=sum(good)/len(good) if good else None,
                tail_mean_error_pct=(sum(good)/len(good)-target)/target*100 if good else None,
                tail_max_abs_error_pct=max(abs(x-target)/target*100 for x in good) if good else None,
                peak_above_command_rpm=max(0, max(values)-target) if values else None,
                peak_below_command_rpm=max(0, target-min(values)) if values else None)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--channel', default='PCAN_USBBUS1')
    p.add_argument('--confirm-motor-test', action='store_true')
    p.add_argument('--output', default='can_sweep_logs')
    args = p.parse_args()
    if not args.confirm_motor_test:
        p.error('Requires --confirm-motor-test; this test starts the motor')
    import can
    folder = Path(args.output)/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    report = {'stages': [], 'outcome': 'incomplete',
              'notes': 'RPM is estimated; 0x301/302 asynchronous. Command excursions include target ramp. Vbus fixed. No phase-current peak or trim saturation in CAN.'}
    seq = tx = rx = 0
    last_send = None
    max_gap = 0.0
    telem = diag = control = None
    control_at = -1e9
    soft = None
    soft_at = -1e9
    telem_at = diag_at = -1e9
    timer = None
    bus = None
    start = time.perf_counter()
    try:
        import sys
        if sys.platform == 'win32':
            import ctypes
            candidate = ctypes.WinDLL('winmm')
            if candidate.timeBeginPeriod(1) == 0:
                timer = candidate
        bus = can.Bus(interface='pcan', channel=args.channel, bitrate=1_000_000,
                      fd=False, receive_own_messages=False, ignore_config=True)
        with (folder/'frames.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['host_s', 'stage', 'direction', 'id', 'data_hex', 'adapter_timestamp'])

            def send(mode, rpm, stage):
                nonlocal seq, tx, last_send, max_gap
                payload = struct.pack('<BBhI', mode, seq, rpm, 0)
                bus.send(can.Message(arbitration_id=0x201, is_extended_id=False, data=payload), timeout=.005)
                now = time.perf_counter()
                if last_send is not None:
                    max_gap = max(max_gap, now-last_send)
                last_send = now
                writer.writerow([now-start, stage, 'TX', '201', payload.hex(), ''])
                seq = (seq+1) & 255
                tx += 1

            def phase(name, rpm, seconds, mode=1, require_closed=False, await_closed=False):
                nonlocal telem, diag, control, telem_at, diag_at, control_at, soft, soft_at, rx
                begun = time.perf_counter()
                next_tx = begun
                next_print = begun
                samples = []
                controls = []
                soft_samples = []
                print(f'\n{name}: target={rpm}, max={seconds}s', flush=True)
                while time.perf_counter()-begun < seconds:
                    now = time.perf_counter()
                    if mode is not None and now >= next_tx:
                        send(mode, rpm, name)
                        next_tx = time.perf_counter()+.010
                    msg = bus.recv(timeout=.002)
                    now = time.perf_counter()
                    if msg:
                        writer.writerow([now-start, name, 'RX', f'{msg.arbitration_id:X}', msg.data.hex(), msg.timestamp])
                        if msg.is_error_frame:
                            raise RuntimeError('CAN error frame')
                        if not msg.is_extended_id and not msg.is_remote_frame and not msg.is_fd and len(msg.data)==8:
                            if msg.arbitration_id == 0x304:
                                envelope, bits, version, entries = struct.unpack('<HBBI', msg.data)
                                if version != 1:
                                    raise RuntimeError('Unsupported soft-current telemetry')
                                soft = (envelope/1000, bool(bits & 4), entries)
                                soft_at = now
                                if bits & 1:
                                    soft_samples.append(soft)
                            elif msg.arbitration_id == 0x303:
                                ramp, trim, base, bits, version = struct.unpack('<HhHBB', msg.data)
                                if version != 1:
                                    raise RuntimeError('Unsupported control telemetry version')
                                control = (ramp/10, trim/1e6, base/1e5, bits)
                                control_at = now
                                if bits & 1:
                                    controls.append((now-begun, *control))
                            elif msg.arbitration_id == 0x302:
                                diag = struct.unpack('<BBHI', msg.data)
                                diag_at = now
                            elif msg.arbitration_id == 0x301:
                                telem = struct.unpack('<hhHBB', msg.data)
                                telem_at = now
                                rx += 1
                                valid = diag is not None and now-diag_at < .1 and bool(diag[0]&2) and telem[3]==7
                                samples.append((now-begun, telem[0], valid))
                    fresh = now-diag_at < .1 and now-telem_at < .1
                    if now-begun > .5 and not fresh:
                        raise RuntimeError('Telemetry missing/stale >100ms')
                    if require_closed and now-begun > .5 and now-control_at > .1:
                        raise RuntimeError('0x303 missing/stale; update firmware')
                    if require_closed and now-begun > .5 and now-soft_at > .1:
                        raise RuntimeError('0x304 missing/stale; update soft-current firmware')
                    if fresh:
                        flags, state, target, off_us = diag
                        if flags & 64:
                            raise RuntimeError('Passive firmware detected; active sweep refused')
                        if telem[4] or state >= 11:
                            raise RuntimeError(f'Motor fault: status={state}, fault={telem[4]}')
                        if mode == 1 and now-begun > .1 and flags & 8:
                            raise RuntimeError('Watchdog latched during RUN')
                        if require_closed and (state != 7 or flags & 3 != 3):
                            raise RuntimeError('Unexpected loss of CLOSED/MOE/valid; no restart')
                        if await_closed and state == 7 and flags & 3 == 3:
                            return
                        if await_closed and now-begun > .5 and state in (8, 9, 10):
                            raise RuntimeError('Unexpected stop during startup; no restart')
                    if now >= next_print:
                        print(f'{now-start:6.1f}s TX={tx} RX={rx} telemetry={telem} diag={diag} control={control} soft={soft}', flush=True)
                        next_print = now+1
                if await_closed:
                    raise RuntimeError('No closed loop within 15s')
                if require_closed:
                    stats = summarize(samples, rpm, seconds)
                    tail_controls = [c for c in controls if c[0] >= seconds-2]
                    stats.update(control_samples=len(controls),
                        trim_sat_fraction=sum(bool(c[4]&2) for c in controls)/len(controls) if controls else None,
                        max_abs_trim=max(abs(c[2]) for c in controls) if controls else None,
                        tail_ramp_mean_rpm=sum(c[1] for c in tail_controls)/len(tail_controls) if tail_controls else None,
                        tail_ramp_at_command_fraction=sum(abs(c[1]-rpm)<=.2 for c in tail_controls)/len(tail_controls) if tail_controls else None)
                    stats.update(soft_active_fraction=sum(x[1] for x in soft_samples)/len(soft_samples) if soft_samples else None,
                        phase_envelope_max_a=max(x[0] for x in soft_samples) if soft_samples else None,
                        soft_entries_end=soft_samples[-1][2] if soft_samples else None)
                    stats['stage'] = name
                    report['stages'].append(stats)
                    print('STATS', json.dumps(stats), flush=True)

            try:
                phase('initial_STOP', 0, 1, mode=0)
                phase('startup_500', 500, 15, await_closed=True)
                for name, target, duration in STAGES:
                    phase(name, target, duration, require_closed=True)
                last_send = None  # Intentional silence is not scheduling jitter.
                phase('link_loss', 500, 5, mode=None)
                flags, state, target, off_us = diag
                report['last_diag'] = dict(flags=flags, state=state, target=target, off_upper_us=off_us)
                if flags & 1 or not flags & 8 or not flags & 16 or not flags & 32 or state != 10:
                    raise RuntimeError('Final timeout/MOE/STOPPED evidence incomplete')
                report['outcome'] = 'completed; review tracking metrics (not automatic performance acceptance)'
            except BaseException:
                # Best-effort fresh STOP before closing bus. No automatic restart.
                for _ in range(10):
                    try:
                        send(0, 0, 'abort_STOP')
                        time.sleep(.01)
                    except Exception:
                        break
                raise
    except KeyboardInterrupt:
        report['outcome'] = 'interrupted; best-effort STOP sent'
    except Exception as exc:
        report['outcome'] = 'failed: '+str(exc)
    finally:
        if bus is not None:
            bus.shutdown()
        if timer is not None:
            timer.timeEndPeriod(1)
        report.update(tx=tx, rx_301=rx, max_host_send_gap_ms=max_gap*1000)
        (folder/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(report['outcome'])
        print('Logs:', folder.resolve())
    return 0 if report['outcome'].startswith('completed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
