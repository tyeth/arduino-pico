"""HIL rig on the S31 bridge (192.168.1.102): Feather RP2040 USB Host at 1-1.4.4 (flashed and logged through the
bridge HTTP API), Jumperless V5 imported into WSL over USB/IP (S2 EN = row 30, switched to GND via jl_wsl.py).

usage: python rig_s31.py <uf2> <log.txt> [max_s=1500] [pre_s=1] [swap_off_s=2]
  - flashes the UF2 (bridge enters the UF2 bootloader itself), power-cycles the S2 for 5 s for a clean start,
  - follows the Feather's serial, and on every "ENTER " line waits pre_s then power-cycles the S2 (swap_off_s off),
  - stops 2 s after a line containing "fix DONE" (or after max_s).
"""
import json
import os
import subprocess
import sys
import time
import urllib.request

BRIDGE = "http://192.168.1.102"
FEATHER = "1-1.4.4"
WSL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wsl")


def api(method, path, body=None, raw=None, timeout=30):
    data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
    hdr = {"Content-Type": "application/octet-stream" if raw is not None else "application/json"}
    req = urllib.request.Request(BRIDGE + path, data=data, headers=hdr, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def jl(code, wait=1.5):
    wd = subprocess.run(["wsl.exe", "-e", "wslpath", WSL_DIR.replace("\\", "/")], capture_output=True, text=True).stdout.strip()
    r = subprocess.run(["wsl.exe", "-e", "bash", "-lc", f'cd "{wd}" && python3 jl_wsl.py \'{code}\' {wait}'],
                       capture_output=True, text=True, timeout=60)
    return [l for l in r.stdout.replace("\0", "").splitlines() if l and not l.startswith(">")]


def en_cycle(off_s):
    a = jl('fast_connect(30, GND); print("S2 EN", is_connected(30, GND))')
    time.sleep(off_s)
    b = jl('fast_disconnect(30, GND); print("S2 EN", is_connected(30, GND))')
    return a + b


def flash(uf2):
    name = "rig.uf2"
    with open(uf2, "rb") as f:
        api("PUT", f"/api/rescue/files/{name}", raw=f.read(), timeout=120)
    api("POST", f"/api/usb/devices/{FEATHER}/release", {})
    job = api("POST", f"/api/usb/devices/{FEATHER}/flash/uf2", {"file": name, "enter_bootloader": True})
    jid = job.get("job") if isinstance(job.get("job"), int) else job.get("id", "latest")
    t = time.time()
    while time.time() - t < 120:
        time.sleep(2)
        j = api("GET", f"/api/rescue/jobs/{jid}")
        j = j.get("job", j)
        if j.get("state") in ("done", "failed", "error", "cancelled"):
            return j
    return {"state": "timeout"}


def wait_feather(t_s=30):
    t = time.time()
    while time.time() - t < t_s:
        devs = api("GET", "/api/usb/devices")["devices"]
        if any(d["busid"] == FEATHER and d["vid"] == "239a" and d["pid"] == "cafe" for d in devs):
            return True
        time.sleep(1)
    return False


def main():
    uf2, logp = sys.argv[1], sys.argv[2]
    max_s = float(sys.argv[3]) if len(sys.argv) > 3 else 1500
    pre_s = float(sys.argv[4]) if len(sys.argv) > 4 else 1
    off_s = float(sys.argv[5]) if len(sys.argv) > 5 else 2
    j = flash(uf2)
    print("flash:", j.get("state"), j.get("result") or j.get("error") or "", flush=True)
    print("feather back:", wait_feather(), flush=True)
    print("clean start:", en_cycle(5), flush=True)
    st = api("POST", f"/api/usb/devices/{FEATHER}/capture/start", {"baud": 115200, "dtr": True})
    cur = st["cursors"][0] if st.get("cursors") else 0
    t0 = time.time(); done_at = None; buf = ""
    with open(logp, "w", encoding="utf-8") as log:
        while time.time() - t0 < max_s and not (done_at and time.time() > done_at):
            d = api("GET", f"/api/usb/devices/{FEATHER}/capture?port=0&since={cur}&max=16384&format=lines")
            cur = d.get("next", cur)
            buf += d.get("text") or ""
            *lines, buf = buf.split("\n")
            for l in lines:
                l = l.rstrip("\r")
                if not l.strip():
                    continue
                log.write(f"{time.time() - t0:8.3f} {l}\n"); log.flush()
                if "ENTER " in l:
                    time.sleep(pre_s)
                    r = en_cycle(off_s)
                    log.write(f"{time.time() - t0:8.3f} [rig] swapped S2: {r}\n"); log.flush()
                    print(f"{time.strftime('%T')} swap after: {l.strip()[:90]}", flush=True)
                if "fix DONE" in l and not done_at:
                    done_at = time.time() + 2
            time.sleep(0.5)
    api("POST", f"/api/usb/devices/{FEATHER}/release", {})
    print("log:", logp, "done" if done_at else "timeout", flush=True)


if __name__ == "__main__":
    main()
