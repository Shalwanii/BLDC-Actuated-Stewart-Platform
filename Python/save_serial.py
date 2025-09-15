import sys, time, csv, os
import serial
import serial.tools.list_ports as lp
from datetime import datetime

BAUD = 115200

def pick_port():
    ports = list(lp.comports())
    if not ports:
        print("No serial ports found."); sys.exit(1)
    # Try to auto-pick an Arduino/CH340
    for p in ports:
        desc = f"{p.device} - {p.description}".lower()
        if "arduino" in desc or "ch340" in desc or "wch" in desc or "usb serial" in desc:
            return p.device
    # Fallback: first one
    print("Auto-pick failed; using first port. Available:")
    for p in ports: print(" ", p.device, "-", p.description)
    return ports[0].device

def main():
    port = pick_port() if len(sys.argv)<2 else sys.argv[1]
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    outname = f"run_{ts}.csv"
    print(f"Opening {port} @ {BAUD} → {outname}")
    # Open with DTR True so UNO resets and we capture from beginning
    with serial.Serial(port, BAUD, timeout=1, write_timeout=1, dsrdtr=False, rtscts=False) as ser, \
         open(outname, "w", newline="", buffering=1) as f:
        # If your sketch prints a CSV header, we just pass it through.
        # Otherwise you can uncomment a custom header:
        # f.write("time_ms,raw0,tgt0,pwm0,raw1,tgt1,pwm1\n")
        try:
            while True:
                line = ser.readline()
                if not line:
                    continue
                try:
                    s = line.decode(errors="ignore")
                except:
                    continue
                # Save everything; if you only want CSV-looking lines, require commas:
                # if s.count(",") < 2: continue
                f.write(s)
                sys.stdout.write(s)
        except KeyboardInterrupt:
            print("\nStopped. Saved:", outname)

if __name__ == "__main__":
    main()
