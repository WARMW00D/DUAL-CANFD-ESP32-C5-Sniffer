#!/usr/bin/env python3
import argparse, os, struct, sys
from datetime import datetime, timezone
import heatshrink2

HEADER_MAGIC = b"C5CAN\0\0\0"
HS_WINDOW_SZ2 = 10
HS_LOOKAHEAD_SZ2 = 5

def decompress_hs_tolerant(raw):
    try:
        return heatshrink2.decompress(raw, window_sz2=HS_WINDOW_SZ2, lookahead_sz2=HS_LOOKAHEAD_SZ2)
    except Exception:
        # Если поток оборван, пробуем отрезать байты с конца, чтобы убрать неполные данные
        for cut in range(1, min(64, len(raw))):
            try:
                return heatshrink2.decompress(raw[:-cut], window_sz2=HS_WINDOW_SZ2, lookahead_sz2=HS_LOOKAHEAD_SZ2)
            except Exception:
                pass
        # Если не вышло, возвращаем пустоту
        return b""

def read_buses(data, n_bus):
    buses = {}
    # C5CAN v1 has a fixed 58-byte header and two 14-byte bus records.
    # Reserved bytes 3..5 now store FD mode and data bitrate (kbit/s).
    if len(data) >= 58 and data[:8] == HEADER_MAGIC and data[8] == 1 and n_bus == 2:
        for i in range(n_bus):
            off = 22 + i * 14
            index, nominal, mode, data_rate = struct.unpack_from('<BHBH', data, off)
            name = data[off + 6:off + 14].split(b'\0', 1)[0].decode('utf-8', errors='replace')
            buses[index] = (name or f'CAN{index}', nominal, mode, data_rate)
        return buses
    for i in range(n_bus):
        pattern = f"CAN{i}".encode()
        pos = data.find(pattern, 0, min(len(data), 200))
        if pos < 0: buses[i] = (f"CAN{i}", None); continue
        name = ""; bitrate = None
        for k in range(pos, min(pos + 8, len(data))):
            b = data[k]
            if b == 0 or not (32 <= b < 127): break
            name += chr(b)
        for off in range(max(0, pos - 20), pos):
            if off + 2 > len(data): break
            val = data[off] | (data[off + 1] << 8)
            if val in (100, 125, 250, 500, 800, 1000): bitrate = val; break
        buses[i] = (name, bitrate)
    return buses

def find_frame_start(data, n_bus=2, header_size=58):
    def score_offset(offset, n_frames=10):
        if offset >= len(data): return 0
        pos = offset; ok = 0
        for _ in range(n_frames):
            if pos + 10 > len(data): break
            bus_fl = data[pos + 4]
            if bus_fl & 0x80:
                if pos + 6 > len(data): break
                plen = data[pos + 5]
                if plen > 64: break
                pos += 6 + plen; ok += 1; continue
            if (bus_fl & 0x07) >= n_bus: break
            dlc = data[pos + 9]
            if dlc > 64: break
            pos += 10 + dlc; ok += 1
        return ok

    if score_offset(header_size) >= 5: return header_size
    best_offset, best_score = None, 0
    for offset in range(max(0, header_size - 10), min(len(data), header_size + 20)):
        score = score_offset(offset)
        if score > best_score: best_score, best_offset = score, offset
    if best_score < 5:
        for offset in range(30, min(len(data), 200)):
            score = score_offset(offset)
            if score > best_score: best_score, best_offset = score, offset
    return best_offset if best_score >= 5 else None

def iter_frames(data, start):
    i, n = start, len(data)
    while i + 5 <= n:
        ts = struct.unpack('<I', data[i:i + 4])[0]
        bus_fl = data[i + 4]
        if bus_fl & 0x80:
            if i + 6 > n: break
            plen = data[i + 5]
            if i + 6 + plen > n: break
            yield ('svc', ts, bus_fl & 0x7F, data[i + 6:i + 6 + plen])
            i += 6 + plen
        else:
            if i + 10 > n: break
            bus = bus_fl & 0x07
            is_ext = bool(bus_fl & 0x08); is_rtr = bool(bus_fl & 0x10)
            is_fdf = bool(bus_fl & 0x20); is_brs = bool(bus_fl & 0x40)
            id_ = struct.unpack('<I', data[i + 5:i + 9])[0]
            dlc = data[i + 9]
            if i + 10 + dlc > n: break
            yield ('frame', ts, bus, is_ext, is_rtr, is_fdf, is_brs, id_, data[i + 10:i + 10 + dlc])
            i += 10 + dlc

def fmt_line(ts_str, bus_tag, is_ext, is_rtr, is_fdf, is_brs, id_, fdata):
    parts = [ts_str]
    if bus_tag: parts.append(bus_tag)
    parts.append(f"ID:0x{id_:08X}" if is_ext else f"ID:0x{id_:03X}")
    if is_rtr: parts.append(f"RTR DL:{len(fdata)}")
    else:
        parts.append(f"D:{' '.join(f'{b:02X}' for b in fdata)}")
        if is_fdf: parts.append("[FD/BRS]" if is_brs else "[FD]")
    return " ".join(parts)

def unpack_one(path, args):
    try:
        raw = open(path, 'rb').read()
    except FileNotFoundError:
        return 0, None

    if path.lower().endswith('.hs'):
        data = decompress_hs_tolerant(raw)
    else:
        data = raw

    if not data.startswith(HEADER_MAGIC):
        sys.stderr.write(f"skip: bad magic {data[:8]!r}: {path}\n")
        return 0, None

    n_bus = data[21] if len(data) > 21 else 0
    buses = read_buses(data, n_bus)
    for i, bus in buses.items():
        if len(bus) == 4 and bus[2]:
            sys.stderr.write(f"CAN{i} ({bus[0]}): FD nominal={bus[1]} data={bus[3]} kbit/s\n")
    # Fixed framing also supports files containing just one CAN/FD frame.
    frame_start = 58 if len(data) >= 58 and data[8] == 1 and n_bus == 2 else find_frame_start(data, n_bus=n_bus)
    if frame_start is None:
        sys.stderr.write(f"skip: no frame data: {path}\n")
        return 0, None

    anchor_ts, anchor_epoch = 0, 0
    if frame_start >= 8:
        try:
            anchor_ts, anchor_epoch = struct.unpack('<II', data[frame_start - 8:frame_start])
        except Exception:
            pass

    bus_names = {i: b[0] for i, b in buses.items()}
    main_out, out_files = None, {}
    stem = os.path.splitext(path)[0]

    if args.output:
        main_out = open(args.output, 'w', encoding='utf-8')
        out_path = args.output
    elif args.merge:
        out_path = stem + '.txt'
        main_out = open(out_path, 'w', encoding='utf-8')
    else:
        for i in range(n_bus):
            nm = bus_names.get(i, f"B{i}")
            nm = ''.join(c if c.isalnum() or c in '-_' else '_' for c in nm)
            out_files[i] = open(f"{stem}_{nm}.txt", 'w', encoding='utf-8')
        out_path = f"{stem}_<BUS>.txt"

    n_frames = 0
    for rec in iter_frames(data, frame_start):
        if rec[0] == 'svc':
            _, ts, rtype, payload = rec
            if rtype == 0 and len(payload) == 4:
                anchor_ts, anchor_epoch = ts, struct.unpack('<I', payload)[0]
            continue

        _, ts, bus, is_ext, is_rtr, is_fdf, is_brs, id_, fdata = rec
        if args.bus is not None and bus != args.bus: continue
        if args.id is not None and id_ != args.id: continue

        delta = (ts - anchor_ts) & 0xFFFFFFFF
        if delta > 0x80000000: delta = ((ts + 0x100000000) - anchor_ts) & 0xFFFFFFFF
        wall = anchor_epoch + delta // 1_000_000
        ms = (delta % 1_000_000) // 1000
        try:
            dt = datetime.fromtimestamp(wall, tz=timezone.utc)
            ts_str = dt.strftime('%Y-%m-%d %H:%M:%S') + f".{ms:03d}"
        except Exception:
            ts_str = f"ts={ts}.{ms:03d}"

        tag = f"[{bus_names.get(bus, f'B{bus}')}]" if args.bus_tag else None
        line = fmt_line(ts_str, tag, is_ext, is_rtr, is_fdf, is_brs, id_, fdata) + "\n"
        n_frames += 1
        if main_out: main_out.write(line)
        if bus in out_files: out_files[bus].write(line)

    if main_out: main_out.close()
    for f in out_files.values(): f.close()
    return n_frames, out_path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("-o", "--output")
    ap.add_argument("--dir", action="store_true")
    ap.add_argument("--merge", action="store_true")
    ap.add_argument("--bus", type=int)
    ap.add_argument("--id", type=lambda s: int(s, 0))
    ap.add_argument("--bus-tag", action="store_true")
    args = ap.parse_args()

    if args.dir:
        files = [os.path.join(r, n) for r, _, ns in os.walk(args.input) for n in sorted(ns) if n.endswith('.hs')]
        for p in files:
            n, out = unpack_one(p, args)
            if out: print(f"  {os.path.basename(p)} -> {out} ({n} frames)")
    else:
        n, out = unpack_one(args.input, args)
        if out: sys.stderr.write(f"\n{n} frames written to {out}\n")

if __name__ == "__main__":
    main()

