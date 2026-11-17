#!/usr/bin/env python3
# Copyright 2026 ROBOTSIX
# SPDX-License-Identifier: Apache-2.0
"""Check GUIDED takeoff and LAND on an already running ArduCopter SITL."""

import argparse
import time

from pymavlink import mavutil


def check_flight(connection, altitude, timeout):
    """Use normal arming checks, verify altitude, then land and await disarm."""
    client = mavutil.mavlink_connection(connection, source_system=254)
    armed = False
    landed = False
    try:
        if client.wait_heartbeat(timeout=15) is None:
            raise TimeoutError('No ArduCopter heartbeat')
        client.mav.request_data_stream_send(
            client.target_system, client.target_component, 0, 10, 1)
        deadline = time.monotonic() + timeout
        last_attempt = 0.0
        last_heartbeat = 0.0
        high_since = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_heartbeat >= 1:
                client.mav.heartbeat_send(6, 8, 0, 0, 0)
                last_heartbeat = now
            if not armed and now - last_attempt >= 5:
                client.set_mode('GUIDED')
                client.mav.command_long_send(
                    client.target_system, client.target_component,
                    400, 0, 1, 0, 0, 0, 0, 0, 0)
                last_attempt = now
            message = client.recv_match(blocking=True, timeout=0.5)
            if message is None:
                continue
            kind = message.get_type()
            if kind == 'STATUSTEXT':
                print(message.text, flush=True)
            if kind == 'HEARTBEAT' and message.base_mode & 128 and not armed:
                armed = True
                print('Armed; requesting takeoff', flush=True)
                client.mav.command_long_send(
                    client.target_system, client.target_component,
                    22, 0, 0, 0, 0, 0, 0, 0, altitude)
            if kind == 'GLOBAL_POSITION_INT':
                above = armed and message.relative_alt / 1000 >= altitude - 0.5
                high_since = (high_since or now) if above else None
                if high_since is not None and now - high_since >= 3:
                    print('Takeoff verified; landing', flush=True)
                    break
        else:
            raise TimeoutError('Takeoff altitude was not maintained for three seconds')
        client.set_mode('LAND')
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            client.mav.heartbeat_send(6, 8, 0, 0, 0)
            message = client.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if message is not None and not message.base_mode & 128:
                landed = True
                print('Landing and disarm verified', flush=True)
                return
        raise TimeoutError('Landing disarm was not observed')
    finally:
        if armed and not landed:
            client.set_mode('LAND')
        client.close()


def main():
    """Parse the connection of a separately launched simulation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='tcp:127.0.0.1:5760')
    parser.add_argument('--altitude', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=240.0)
    args = parser.parse_args()
    if args.altitude <= 0.5 or args.timeout <= 0:
        parser.error('altitude must exceed 0.5 m and timeout must be positive')
    check_flight(args.connection, args.altitude, args.timeout)


if __name__ == '__main__':
    main()
