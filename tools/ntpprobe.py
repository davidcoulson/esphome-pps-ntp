#!/usr/bin/env python3
"""Query an NTP server repeatedly and summarise what it says and how quickly it answers.

usage: ntpprobe.py HOST [COUNT] [INTERVAL_S]
Round-trip time and the server's own processing time (transmit - receive timestamp) need no trusted
clock on this side, so they are a fair way to compare transports or firmware versions.
"""
import socket, struct, sys, time, statistics

host = sys.argv[1]
count = int(sys.argv[2]) if len(sys.argv) > 2 else 50
interval = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
NTP_DELTA = 2208988800

def ts(b):
    s, f = struct.unpack("!II", b)
    return s - NTP_DELTA + f / 2**32

# Works for names, IPv4 and IPv6 literals (link-local needs a zone: fe80::1%en0)
family, _, _, _, dest = socket.getaddrinfo(host, 123, type=socket.SOCK_DGRAM)[0]
sock = socket.socket(family, socket.SOCK_DGRAM)
sock.settimeout(1.0)
rtts, procs, offsets, lost, last = [], [], [], 0, None
for _ in range(count):
    t1 = time.time()
    pkt = b"\x23" + bytes(39) + struct.pack("!II", int(t1) + NTP_DELTA, int((t1 % 1) * 2**32))
    p1 = time.perf_counter()
    sock.sendto(pkt, dest)
    try:
        data, _ = sock.recvfrom(128)
    except socket.timeout:
        lost += 1
        continue
    p4 = time.perf_counter()
    t4 = t1 + (p4 - p1)
    t2, t3 = ts(data[32:40]), ts(data[40:48])
    rtts.append((p4 - p1) * 1e6)
    procs.append((t3 - t2) * 1e6)
    offsets.append(((t2 - t1) + (t3 - t4)) / 2)
    last = data
    time.sleep(interval)

if last is None:
    print(f"{host}: no replies ({lost} lost)")
    sys.exit(1)
li, vn, mode = last[0] >> 6, (last[0] >> 3) & 7, last[0] & 7
prec = struct.unpack("b", last[3:4])[0]
disp = struct.unpack("!I", last[8:12])[0] / 65536 * 1e6
refid = last[12:16].rstrip(b"\0").decode("ascii", "replace")
print(f"{host}: LI={li} VN={vn} mode={mode} stratum={last[1]} precision=2^{prec} refid={refid} root_dispersion={disp:.0f}us")
print(f"  server time: {time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(ts(last[40:48])))} UTC, offset from this machine {statistics.median(offsets):+.3f} s")
best = min(range(len(rtts)), key=lambda i: rtts[i])
print(f"  offset at the lowest-delay sample (least path asymmetry): {offsets[best]*1000:+.2f} ms  [RTT {rtts[best]:.0f} us]")
q = lambda v, p: sorted(v)[min(len(v) - 1, int(len(v) * p))]
print(f"  replies {len(rtts)}/{count}  RTT us: min {min(rtts):.0f}  median {statistics.median(rtts):.0f}  p95 {q(rtts, .95):.0f}  max {max(rtts):.0f}")
print(f"  server processing (tx-rx) us: min {min(procs):.0f}  median {statistics.median(procs):.0f}  p95 {q(procs, .95):.0f}  max {max(procs):.0f}")
