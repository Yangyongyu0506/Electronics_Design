#!/usr/bin/env python3
"""Live viewer for the green-mask binary stream emitted by cam-ball-track.

The ESP32 prints one line per decoded frame:

    BIN <width> <height> <seq> <base64-bitmap>

where the bitmap packs one bit per pixel (MSB first, row-major):
1 = green (threshold pass), 0 = background.  This tool reads the UART
console, ignores all other log lines, and renders the mask in a tkinter
window.

Usage:
    python3 mask_viewer.py -p /dev/ttyUSB0 -b 115200 -s 8
"""

import argparse
import base64
import queue
import re
import sys
import threading
import time

import tkinter as tk

try:
    import serial
except ImportError:
    serial = None

LINE_RE = re.compile(rb"BIN (\d+) (\d+) (\d+) ([A-Za-z0-9+/=]+)")


def serial_reader(port, baud, frame_queue):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"opened {port} @ {baud}")
            break
        except Exception as exc:
            print(f"cannot open {port}: {exc}; retrying in 2 s")
            time.sleep(2)

    buf = b""
    while True:
        try:
            chunk = ser.read(4096)
        except Exception as exc:
            print(f"serial read error: {exc}")
            time.sleep(1)
            continue
        if not chunk:
            continue

        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            match = LINE_RE.search(line)
            if not match:
                continue
            width = int(match.group(1))
            height = int(match.group(2))
            seq = int(match.group(3))
            try:
                data = base64.b64decode(match.group(4))
            except Exception:
                continue
            if len(data) != (width * height + 7) // 8:
                continue
            frame_queue.put((width, height, seq, data))

        if len(buf) > 65536:
            buf = buf[-16384:]


class MaskViewer:
    def __init__(self, root, frame_queue, scale):
        self.root = root
        self.frame_queue = frame_queue
        self.scale = scale
        self.width = 0
        self.height = 0
        self.last_seq = -1
        self.frames = 0
        self.t0 = time.monotonic()
        self.canvas = tk.Canvas(root, bg="black", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        root.title("cam-ball-track green mask")
        root.after(30, self.poll)

    def poll(self):
        try:
            while True:
                width, height, seq, data = self.frame_queue.get_nowait()
                self.render(width, height, data)
        except queue.Empty:
            pass
        self.root.after(30, self.poll)

    def render(self, width, height, data):
        rows = []
        for y in range(height):
            row = []
            for x in range(width):
                bit = (data[(y * width + x) >> 3] >> (7 - (x & 7))) & 1
                row.append("#00FF00" if bit else "#000000")
            rows.append("{" + " ".join(row) + "}")

        image = tk.PhotoImage(width=width, height=height)
        image.put(" ".join(rows))
        if self.scale > 1:
            image = image.zoom(self.scale)
        self.canvas.create_image(0, 0, anchor="nw", image=image)
        self.canvas.image = image

        self.frames += 1
        elapsed = time.monotonic() - self.t0
        if elapsed >= 1.0:
            fps = self.frames / elapsed
            self.root.title(f"cam-ball-track green mask  {width}x{height}  {fps:.1f} fps")
            self.frames = 0
            self.t0 = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description="cam-ball-track green-mask viewer")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-s", "--scale", type=int, default=8)
    args = parser.parse_args()

    if serial is None:
        sys.exit("pyserial is not installed: pip install pyserial")

    root = tk.Tk()
    frame_queue = queue.Queue()
    MaskViewer(root, frame_queue, args.scale)
    threading.Thread(target=serial_reader, args=(args.port, args.baud, frame_queue),
                     daemon=True).start()
    root.mainloop()


if __name__ == "__main__":
    main()
