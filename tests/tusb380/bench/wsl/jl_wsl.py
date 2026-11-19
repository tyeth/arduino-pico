"""Send one MicroPython one-liner to the Jumperless main terminal (WSL /dev/ttyACM0 via USB/IP) as '>code'."""
import re, sys, time, serial
def jl(code, wait=1.5, dev="/dev/ttyACM0"):
    s = serial.Serial(dev, 115200, timeout=0.2)
    s.dtr = True; s.rts = True
    time.sleep(0.3); s.reset_input_buffer()
    s.write((">" + code + "\r").encode())
    out = b""; t = time.time() + wait
    while time.time() < t:
        out += s.read(4096)
    s.close()
    txt = re.sub(r"[^\x20-\x7e\n]", "", re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", out.decode(errors="replace")))
    flat = code.replace(" ", "")
    return [l.strip() for l in txt.split("\n") if l.strip() and not (l.strip().startswith(">") and flat.startswith(l.strip().lstrip(">").replace(" ", "")))]
if __name__ == "__main__":
    print("\n".join(jl(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1.5, sys.argv[3] if len(sys.argv) > 3 else "/dev/ttyACM0")))
