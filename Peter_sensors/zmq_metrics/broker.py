#!/usr/bin/env python3
"""
broker.py — ZMQ XPUB/XSUB message broker.

Run this once before starting any node:
    python broker.py

All publishers connect their PUB socket to port 5555.
All subscribers connect their SUB socket to port 5556.
The broker forwards everything between them.
"""

import signal
import sys
import zmq

from topics import BROKER_FRONTEND, BROKER_BACKEND


def main():
    ctx = zmq.Context()

    frontend = ctx.socket(zmq.XSUB)   # publishers connect here
    frontend.bind(BROKER_FRONTEND)

    backend = ctx.socket(zmq.XPUB)    # subscribers connect here
    backend.bind(BROKER_BACKEND)

    print(f"[broker] XSUB listening on {BROKER_FRONTEND}")
    print(f"[broker] XPUB listening on {BROKER_BACKEND}")
    print("[broker] Ready — Ctrl-C to stop")

    def _shutdown(sig, frame):
        print("\n[broker] Shutting down...")
        frontend.close()
        backend.close()
        ctx.term()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    zmq.proxy(frontend, backend)


if __name__ == "__main__":
    main()
