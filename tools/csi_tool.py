"""
Captura y visualización de CSI del ESP32-C5 (firmware sample_project).

Uso:
  Capturar (guarda CSV y grafica en vivo):
      python tools/csi_tool.py capture --port COM3 --out datos/vacio_01.csv
  Capturar sin gráfica (más liviano), durante 60 s:
      python tools/csi_tool.py capture --port COM3 --out datos/vacio_01.csv --no-plot --seconds 60
  Graficar un CSV ya guardado:
      python tools/csi_tool.py plot datos/vacio_01.csv

Requisitos:  pip install pyserial numpy matplotlib
IMPORTANTE: cerrar el "idf.py monitor" antes; solo un programa puede abrir COM3.
"""
import argparse
import collections
import csv
import os
import sys
import threading
import time

import numpy as np

HEADER = ["type", "seq", "mac", "rssi", "rate", "noise_floor", "channel", "secondary_channel",
          "local_timestamp", "sig_len", "rx_format", "len", "first_word_invalid", "data", "pc_time"]
COL = {name: i for i, name in enumerate(HEADER)}


def parse_row(row):
    """Devuelve (rssi, amplitudes por subportadora) o None si la fila no es válida."""
    try:
        raw = np.array([int(v) for v in row[COL["data"]].strip("[]").split(",")], dtype=np.float32)
        if len(raw) != int(row[COL["len"]]) or len(raw) % 2:
            return None
        im, re = raw[0::2], raw[1::2]          # el ESP32 entrega pares [Im, Re]
        amp = np.sqrt(im ** 2 + re ** 2)
        if row[COL["first_word_invalid"]] == "1":
            amp[:2] = np.nan                   # 4 primeros bytes inválidos (limitación HW)
        return int(row[COL["rssi"]]), amp
    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------- captura

def open_serial(port):
    import serial
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 921600                      # irrelevante en USB nativo, necesario para pyserial
    ser.timeout = 1
    ser.dtr = False                            # evita resetear la placa al abrir el puerto
    ser.rts = False
    ser.open()
    return ser


def capture(args):
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    ser = open_serial(args.port)
    window = collections.deque(maxlen=args.window)    # últimas N muestras para la gráfica
    stats = {"n": 0, "stop": False}
    t_end = time.time() + args.seconds if args.seconds else None

    def reader():
        with open(args.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(HEADER)
            while not stats["stop"]:
                if t_end and time.time() > t_end:
                    break
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line.startswith("CSI_DATA"):
                    if line:
                        print("[ESP]", line)          # logs del firmware (conexión, avisos)
                    continue
                row = next(csv.reader([line]))
                if len(row) != len(HEADER) - 1:
                    continue                          # línea cortada
                row.append(f"{time.time():.6f}")
                w.writerow(row)
                stats["n"] += 1
                parsed = parse_row(row)
                if parsed:
                    window.append(parsed)
                if stats["n"] % 100 == 0:
                    f.flush()
                    print(f"\r{stats['n']} muestras guardadas en {args.out}", end="", flush=True)
        stats["stop"] = True

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    try:
        if args.no_plot:
            while not stats["stop"]:
                time.sleep(0.5)
        else:
            live_plot(window, stats)
    except KeyboardInterrupt:
        pass
    stats["stop"] = True
    th.join(timeout=2)
    ser.close()
    print(f"\nListo: {stats['n']} muestras en {args.out}")


def live_plot(window, stats):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, (ax_hm, ax_ts) = plt.subplots(2, 1, figsize=(11, 7), gridspec_kw={"height_ratios": [2, 1]})
    fig.suptitle("CSI en vivo (Ctrl+C en la terminal o cerrar la ventana para terminar)")
    ax_rssi = ax_ts.twinx()

    def update(_):
        if stats["stop"]:
            plt.close(fig)
            return
        data = list(window)
        if len(data) < 2:
            return
        n_sc = collections.Counter(len(a) for _, a in data).most_common(1)[0][0]
        data = [(r, a) for r, a in data if len(a) == n_sc]   # un solo formato por gráfica
        amps = np.array([a for _, a in data])
        rssi = np.array([r for r, _ in data])

        ax_hm.clear()
        ax_hm.imshow(amps.T, aspect="auto", origin="lower", cmap="viridis", interpolation="nearest")
        ax_hm.set_ylabel(f"subportadora (0..{n_sc - 1})")
        ax_hm.set_title("Amplitud |H| por subportadora (eje x = muestras recientes)")

        ax_ts.clear()
        ax_ts.plot(np.nanmean(amps, axis=1), label="amplitud media")
        ax_ts.set_ylabel("amplitud media")
        ax_ts.set_xlabel("muestra")
        ax_rssi.clear()
        ax_rssi.plot(rssi, color="tab:red", alpha=0.6)
        ax_rssi.set_ylabel("RSSI (dBm)", color="tab:red")

    _anim = FuncAnimation(fig, update, interval=300, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------- gráfica offline

def plot_file(args):
    import matplotlib.pyplot as plt

    rows = []
    with open(args.file, newline="") as f:
        for row in csv.reader(f):
            if row and row[0] == "CSI_DATA":
                p = parse_row(row)
                if p:
                    rows.append((float(row[COL["pc_time"]]), *p))
    if not rows:
        sys.exit("No hay filas CSI_DATA válidas en el archivo")

    n_sc = collections.Counter(len(a) for _, _, a in rows).most_common(1)[0][0]
    rows = [r for r in rows if len(r[2]) == n_sc]
    t = np.array([r[0] for r in rows]); t -= t[0]
    rssi = np.array([r[1] for r in rows])
    amps = np.array([r[2] for r in rows])
    print(f"{len(rows)} muestras, {n_sc} subportadoras, {t[-1]:.1f} s, ~{len(rows) / max(t[-1], 1e-9):.0f} muestras/s")

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].imshow(amps.T, aspect="auto", origin="lower", cmap="viridis",
                   extent=[t[0], t[-1], 0, n_sc])
    axes[0].set_ylabel("subportadora")
    axes[0].set_title(f"Amplitud CSI — {os.path.basename(args.file)}")
    axes[1].plot(t, np.nanmean(amps, axis=1))
    axes[1].set_ylabel("amplitud media")
    axes[2].plot(t, rssi, color="tab:red")
    axes[2].set_ylabel("RSSI (dBm)")
    axes[2].set_xlabel("tiempo (s)")
    plt.tight_layout()
    plt.show()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="leer COM, guardar CSV y graficar en vivo")
    c.add_argument("--port", default="COM3")
    c.add_argument("--out", default=f"datos/csi_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    c.add_argument("--seconds", type=float, default=0, help="duración (0 = hasta Ctrl+C)")
    c.add_argument("--window", type=int, default=500, help="muestras visibles en la gráfica en vivo")
    c.add_argument("--no-plot", action="store_true")
    c.set_defaults(func=capture)

    g = sub.add_parser("plot", help="graficar un CSV guardado")
    g.add_argument("file")
    g.set_defaults(func=plot_file)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
