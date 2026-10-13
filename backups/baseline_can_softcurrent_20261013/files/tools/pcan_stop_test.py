#!/usr/bin/env python3
"""PCAN classic CAN tests: STOP heartbeat, repeated sequence, or passive RUN.

Windows: py -m pip install "python-can[pcan]"
         py tools/pcan_stop_test.py
Requires the PCAN-Basic runtime matching Python's architecture.
Reset the board before each run; disable all other command transmitters.
"""
import argparse
import math
import struct
import sys
import time


def recovery_phase(elapsed):
    """Fixed 25s sequence; None means intentional silence, not a command."""
    for end, label, mode, rpm in [
        (0.5, 'STOP_INIT', 0, 0),
        (5.0, 'RUN_600: C0 -> target=600 watchdog=0 writes increasing', 1, 600),
        (8.0, 'SILENCE: expect timeout; C0 -> connected=0 watchdog=1', None, 0),
        (16.0, 'BLOCKED_RUN_700: C0 twice -> rx_rpm=700 target=600 writes frozen watchdog=1', 1, 700),
        (17.0, 'STOP_CLEAR: clear latch with fresh STOP frames', 0, 0),
        (25.0, 'RECOVERED_RUN_700: C0 -> target=700 writes increasing watchdog=0', 1, 700),
    ]:
        if elapsed < end:
            return end, label, mode, rpm
    return 27.0, 'FINAL_SILENCE: expect second timeout; C0 -> watchdog=1 MOE=0', None, 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', default='PCAN_USBBUS1')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='seconds transmitting commands (default: 10)')
    parser.add_argument('--start-seq', type=int, default=0,
                        help='first sequence 0..255; reset board before testing')
    parser.add_argument('--test', choices=['heartbeat', 'repeat', 'passive-run', 'recovery', 'active-stop', 'active-timeout'],
                        default='heartbeat')
    parser.add_argument('--confirm-passive-firmware', action='store_true',
                        help='confirm flashed firmware has CAN_PHASE1_PASSIVE_TEST=1')
    parser.add_argument('--rpm', type=int, choices=[500, 600, 700], default=600)
    parser.add_argument('--confirm-motor-test', action='store_true',
                        help='explicitly allow fixed 500 RPM RUN commands to active firmware')
    args = parser.parse_args()
    active_test = args.test in ('active-stop', 'active-timeout')
    if active_test and not args.confirm_motor_test:
        parser.error('Active test can spin motor; requires --confirm-motor-test')
    if args.test in ('passive-run', 'recovery') and not args.confirm_passive_firmware:
        parser.error('RUN requires --confirm-passive-firmware; active firmware could start motor')
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    if not 0 <= args.start_seq <= 255:
        parser.error('--start-seq must be 0..255')
    try:
        import can
    except ImportError:
        print('Install dependency: py -m pip install "python-can[pcan]"')
        return 1

    tx = rx = rx_after = errors = 0
    latest = None
    diag = None
    diag_time = None
    last_send = None
    max_gap = 0.0
    timer_active = False
    winmm = None
    try:
        # Improve Windows timer resolution; this is still not real-time scheduling.
        if sys.platform == 'win32':
            import ctypes
            winmm = ctypes.WinDLL('winmm')
            timer_active = winmm.timeBeginPeriod(1) == 0
        with can.Bus(interface='pcan', channel=args.channel, bitrate=1_000_000,
                     fd=False, receive_own_messages=False,
                     ignore_config=True) as bus:
            print(f'Classic CAN 1 Mbit/s; test={args.test}; TX=0x201; RX=0x301')
            if args.test == 'repeat':
                print('Fixed STOP sequence: expect ONE UART timeout DURING transmission.')
            elif args.test == 'passive-run':
                print(f'First 0.5s: STOP; then RUN target={args.rpm} RPM with fresh sequence.')
                print('Use UART C0 to read target/writes; 0x301 has no target field.')
                print('Firmware MUST remain passive. This script cannot read the firmware gate.')
            if args.test == 'recovery':
                print('Recovery test: fixed 25s TX/silence schedule + 2s final RX; --duration/--rpm ignored.')
                print('PASSIVE firmware required. Use UART C0 at phase prompts to verify actual state.')
            print('Close other command transmitters. Reset board before each run.')
            print('UART is required to verify watchdog; telemetry does not acknowledge sequence.')
            if active_test:
                print('ACTIVE MOTOR TEST: fixed 500 RPM. STOP 1s -> RUN 14s -> STOP or silence.')
                print('Use only with secured propeller-free motor. Close passive RUN/recovery scripts.')
            start = time.perf_counter()
            stop_tx = start + (17.0 if args.test == 'active-stop' else 15.0 if active_test else 25.0 if args.test == 'recovery' else args.duration)
            finish = stop_tx + (5.0 if active_test else 2.0)
            next_tx = start
            next_report = start + 1.0
            seq = args.start_seq
            announced = False
            phase_label = None
            active_run_sent = False
            seen_closed = False
            while True:
                now = time.perf_counter()
                if now >= finish:
                    break
                phase_end, label, phase_mode, phase_rpm = recovery_phase(now - start)
                if args.test == 'recovery' and label != phase_label:
                    print(f'\n[{now-start:.1f}s] {label}', flush=True)
                    phase_label = label
                    if phase_mode is None:
                        last_send = None  # Exclude planned silence from host jitter metric.
                if now >= stop_tx and not announced:
                    print(f'\nTX stopped. RX continues for {5 if active_test else 2}s.')
                    if args.test == 'repeat':
                        print('Timeout should already be latched; expect NO second message.')
                    else:
                        print('Expect ONE UART watchdog message after last fresh command.')
                    announced = True
                tx_enabled = now < stop_tx and (args.test != 'recovery' or phase_mode is not None)
                if tx_enabled and now >= next_tx:
                    mode = 1 if args.test == 'passive-run' and now - start >= 0.5 else 0
                    rpm = args.rpm if mode else 0
                    if args.test == 'recovery':
                        mode, rpm = phase_mode, phase_rpm
                    if active_test:
                        mode = 1 if 1.0 <= now - start < 15.0 else 0
                        rpm = 500 if mode else 0
                    message = can.Message(arbitration_id=0x201,
                                          is_extended_id=False, is_fd=False,
                                          data=struct.pack('<BBhI', mode, seq, rpm, 0))
                    bus.send(message, timeout=0.005)
                    sent_at = time.perf_counter()
                    if last_send is not None:
                        max_gap = max(max_gap, sent_at - last_send)
                    last_send = sent_at
                    tx += 1
                    if active_test and mode == 1:
                        active_run_sent = True
                    if args.test != 'repeat':
                        seq = (seq + 1) & 0xFF
                    # Do not burst missed frames after a scheduling stall.
                    next_tx = sent_at + 0.010
                deadline = min(next_tx, stop_tx) if tx_enabled else finish
                if args.test == 'recovery':
                    deadline = min(deadline, start + phase_end)
                timeout = max(0.0, min(0.005, deadline - time.perf_counter()))
                msg = bus.recv(timeout=timeout)
                if msg is not None:
                    if msg.is_error_frame:
                        errors += 1
                    elif (msg.arbitration_id == 0x302 and not msg.is_extended_id
                          and not msg.is_remote_frame and not msg.is_fd and len(msg.data) == 8):
                        diag = struct.unpack('<BBHI', msg.data)
                        diag_time = time.perf_counter()
                    elif (msg.arbitration_id == 0x301 and not msg.is_extended_id
                          and not msg.is_remote_frame and not msg.is_fd
                          and len(msg.data) == 8):
                        latest = struct.unpack('<hhHBB', msg.data)
                        rx += 1
                        if active_test and active_run_sent and time.perf_counter() - start < 15.0:
                            if latest[3] == 7:
                                seen_closed = True
                            if latest[4] != 0 or latest[3] >= 11 or (seen_closed and latest[3] >= 8):
                                raise RuntimeError('Motor stopped/faulted during RUN; aborting further RUN requests')
                        if time.perf_counter() >= stop_tx:
                            rx_after += 1
                now = time.perf_counter()
                if now >= next_report:
                    detail = 'no telemetry'
                    if latest is not None:
                        rpm, iq_ma, vbus_mv, status, fault = latest
                        detail = (f'RPM={rpm} Iq={iq_ma}mA Vbus_field={vbus_mv}mV '
                                  f'status={status} fault={fault}')
                    if diag is not None and now - diag_time < 0.1:
                        flags, state, target, off_us = diag
                        detail += (f' | MOE={flags & 1} valid={(flags >> 1) & 1}'
                                   f' connected={(flags >> 2) & 1} watchdog={(flags >> 3) & 1}'
                                   f' live_state={state} target={target}'
                                   f' timing_valid={(flags >> 4) & 1}'
                                   f' moe_before={(flags >> 5) & 1} off_upper_us={off_us}')
                        if not flags & 2:
                            detail += ' [RPM/Iq held; NOT live measurements]'
                    else:
                        detail += ' [0x302 missing/stale: validity unknown]'
                    print(f'{now-start:5.1f}s TX={tx} RX={rx} {detail}')
                    next_report = now + 1.0
            print(f'\nDone: TX={tx}, RX={rx}, RX_after_stop={rx_after}, error_frames={errors}')
            print(f'Max host send gap: {max_gap*1000:.2f}ms (not bus delivery timing)')
            if max_gap >= 0.050:
                print('WARNING: host scheduling gap >=50ms; may trigger watchdog during TX.')
            print('Vbus field is currently fixed at 12000mV by firmware, not measured.')
            if args.test == 'recovery':
                print('Verify C0 evidence: blocked RUN keeps target/writes unchanged; STOP then RUN restores writes.')
                print('Expect TWO timeouts: first planned silence and final silence. No automatic PASS from CAN RX.')
            elif args.test == 'repeat':
                print('Check UART: one timeout during fixed-sequence TX, none extra after stopping.')
            else:
                print('Check UART: no timeout during fresh frames; one after TX stops.')
            if args.test == 'passive-run':
                print('Target write requires UART C0 or debugger readback.')
            if active_test and not seen_closed:
                print('FAIL: no CLOSED (status=7) telemetry observed; motor run not validated.')
                return 3
            if rx == 0:
                print('FAIL: no 0x301 telemetry received.')
                return 2
            # Reception alone cannot prove the MCU accepted commands or timed out.
            return 0
    except KeyboardInterrupt:
        print('\nInterrupted; STOP transmission ended.')
        return 130
    except Exception as exc:
        print(f'PCAN test failed: {exc}', file=sys.stderr)
        print('Check PCAN-Basic DLL/driver, Python bitness, channel and adapter access.',
              file=sys.stderr)
        return 1
    finally:
        if timer_active:
            winmm.timeEndPeriod(1)


if __name__ == '__main__':
    sys.exit(main())
