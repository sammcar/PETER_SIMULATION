#!/usr/bin/env python3
import os, mmap, struct, time, threading
import numpy as np
import matplotlib.pyplot as plt

# ========= Config =========
NEURON_SHM_PATH = "/dev/shm/neuron_activity"
NEURON_COUNT    = 85
NEURON_SHM_SIZE = 8 + NEURON_COUNT * 4   # 8 bytes seq + 85 float32
FREQ_HZ         = 10.0                   # expected sampling frequency
MAX_HISTORY     = 3000                   # ~5 minutes at 10 Hz (3000 samples)

# ======== NEW: IMU SHM (stuck, flat, inclined) ========
IMU_SHM_PATH    = "/dev/shm/imu_activity"
IMU_COUNT       = 3                      # [stuck, flat, inclined]
IMU_SHM_SIZE    = 8 + IMU_COUNT * 4
IMU_FREQ_HZ     = 10.0                   # sampling frequency for IMU stream
IMU_LABELS      = ["Inclined", "Flat", "Stuck"]

# ======== Plot toggles (turn ON/OFF modules) ========
PLOT_BASAL_GANGLIA = False
PLOT_LOCOMOTION    = True
PLOT_GAIT_DECISION = True
PLOT_LIDAR_NETWORK = False
PLOT_IMU           = True   # <— toggle IMU raster (stuck/flat/inclined)

# ======== Sections (like your ROS plotter) ========
#  0.. 2  GPi   (3)
#  3.. 5  GPe   (3)
#  6.. 8  STN   (3)
#  9..11  STR   (3)   → 12
# 12..31  z     (20)  → 32
# 32..36  lidar (5)   → 37
# 37..52  INPUT/total_activations (16) → 53
# 53..68  Response (16) → 69
# 69..84  Aux      (16) → 85

SECTS = {
    "Basal Ganglia": [
        (0,  3, ['R','G','B'], "GPi",  True),
        (3,  6, ['R','G','B'], "GPe",  True),
        (6,  9, ['R','G','B'], "STN",  True),
        (9, 12, ['R','G','B'], "STR",  True),
    ],
    "Locomotion": [
        (15, 26, [f"X{i}" for i in range(3, 14)], "Locomotion 1", True),
        (29, 30, ["X17"], "Locomotion 2", True),
    ],
    "Gait Decision": [
        (12, 17, [f"X{i}" for i in range(0, 5)], "Decision 1", True),
        (26, 29, [f"X{i}" for i in range(14, 17)], "Decision 2", True),
    ],
    "Lidar Network": [
        (32, 37, [f"L{i}" for i in range(5)], "Lidar", True),
        (37, 53, [str(i) for i in range(1,17)], "Input", True),
        (53, 69, [str(i) for i in range(1,17)], "Response", True),
        (69, 85, [str(i) for i in range(1,17)], "Auxiliary", True),
    ],
}

# ======== SHM readers with seqlock ========
class ShmReader:
    """Generic seqlock reader for a fixed-length float32 vector."""
    def __init__(self, path: str, count: int):
        size = 8 + count * 4
        if not os.path.exists(path):
            raise FileNotFoundError(f"{path} does not exist. Publish first from the producer.")
        self.count = count
        self.fd = os.open(path, os.O_RDONLY)
        self.mm = mmap.mmap(self.fd, size, access=mmap.ACCESS_READ)

    def close(self):
        try:
            if self.mm: self.mm.close()
        finally:
            try:
                if self.fd: os.close(self.fd)
            except Exception:
                pass

    def read_once(self) -> np.ndarray:
        while True:
            seq1 = struct.unpack_from("Q", self.mm, 0)[0]
            if seq1 % 2 == 1:
                time.sleep(0.001); continue
            buf = self.mm[8:8 + self.count * 4]
            seq2 = struct.unpack_from("Q", self.mm, 0)[0]
            if seq1 == seq2 and seq2 % 2 == 0:
                return np.frombuffer(buf, dtype=np.float32).copy()

class History:
    def __init__(self, cols: int, max_len=MAX_HISTORY):
        self.max_len = max_len
        self.cols = cols
        self.data = np.zeros((self.cols, 0), dtype=np.float32)
        self.lock = threading.Lock()

    def push(self, vec: np.ndarray):
        with self.lock:
            vec = vec.reshape(-1, 1)  # (cols,1)
            self.data = np.hstack([self.data, vec])
            if self.data.shape[1] > self.max_len:
                self.data = self.data[:, -self.max_len:]

    def snapshot(self) -> np.ndarray:
        with self.lock:
            return self.data.copy()

def sampler(reader: ShmReader, hist: History, stop_evt: threading.Event, freq_hz: float):
    period = 1.0 / freq_hz
    next_t = time.time()
    while not stop_evt.is_set():
        try:
            v = reader.read_once()
            hist.push(v)
        except Exception:
            pass
        next_t += period
        time.sleep(max(0.0, next_t - time.time()))

# ======== Plot ========
def plot_raster(neuron_hist: History, imu_hist: History | None):
    data = neuron_hist.snapshot()
    if data.size == 0 or data.shape[1] < 2:
        print("Not enough neuron data to plot yet.")
        return

    num_samples = data.shape[1]
    time_axis = np.arange(num_samples) / FREQ_HZ

    def plot_module(submods, module_title):
        fig, axs = plt.subplots(len(submods), 1, figsize=(10, 2.5*len(submods)))
        fig.suptitle(module_title)
        if len(submods) == 1:
            axs = [axs]

        for i, (start, end, labels, name, reverse) in enumerate(submods):
            raster = data[start:end, :]
            if reverse:
                raster = raster[::-1, :]
                ylabels = labels[::-1] if labels else [str(k) for k in range(end-start)][::-1]
            else:
                ylabels = labels if labels else [str(k) for k in range(end-start)]

            im = axs[i].imshow(
                raster,
                aspect='auto',
                cmap='gist_yarg',
                interpolation='nearest',
                vmin=0.0,
                vmax=max(1e-6, np.max(raster)),
                extent=[time_axis[0], time_axis[-1], 0, raster.shape[0]],
            )
            axs[i].set_yticks(range(len(ylabels)))
            axs[i].set_yticklabels(ylabels)
            axs[i].set_title(name, fontsize=10)
            axs[i].set_ylabel("Neuron")
            axs[i].set_xlabel("Time (s)")
            fig.colorbar(im, ax=axs[i], orientation='vertical')
        plt.tight_layout()

    # Draw groups (respect toggles)
    if PLOT_BASAL_GANGLIA:
        plot_module(SECTS["Basal Ganglia"], "Basal Ganglia Module")
    if PLOT_LOCOMOTION:
        plot_module(SECTS["Locomotion"], "Locomotion Module")
    if PLOT_GAIT_DECISION:
        plot_module(SECTS["Gait Decision"], "Gait Decision Module")
    if PLOT_LIDAR_NETWORK:
        plot_module(SECTS["Lidar Network"], "Lidar Network Module")

    # ======== NEW: IMU raster (stuck/flat/inclined) ========
    if PLOT_IMU and imu_hist is not None:
        imu_data = imu_hist.snapshot()  # shape (3, T_imu)
        if imu_data.size >= 6 and imu_data.shape[1] >= 2:
            t_imu = np.arange(imu_data.shape[1]) / IMU_FREQ_HZ
            fig, ax = plt.subplots(1, 1, figsize=(10, 3.0))
            fig.suptitle("IMU (Stuck / Flat / Inclined)")
            # normalize time axis to seconds on extent
            im = ax.imshow(
                imu_data,
                aspect='auto',
                cmap='gist_yarg',
                interpolation='nearest',
                vmin=0.0,
                vmax=1.0,
                extent=[t_imu[0], t_imu[-1], 0, imu_data.shape[0]],
            )
            ax.set_yticks(range(IMU_COUNT))
            ax.set_yticklabels(IMU_LABELS)
            ax.set_xlabel("Time (s)")
            ax.set_title("IMU States (0–1)")
            fig.colorbar(im, ax=ax, orientation='vertical')
            plt.tight_layout()
        else:
            print("IMU data not available yet (or too short).")

    plt.show()

# ======== Main ========
def main():
    neuron_reader = imu_reader = None
    try:
        neuron_reader = ShmReader(NEURON_SHM_PATH, NEURON_COUNT)
        neuron_hist = History(NEURON_COUNT)
        stop_evt = threading.Event()
        th_neuron = threading.Thread(target=sampler, args=(neuron_reader, neuron_hist, stop_evt, FREQ_HZ), daemon=True)
        th_neuron.start()

        # Optional IMU reader
        imu_hist = None
        if PLOT_IMU:
            try:
                imu_reader = ShmReader(IMU_SHM_PATH, IMU_COUNT)
                imu_hist = History(IMU_COUNT)
                th_imu = threading.Thread(target=sampler, args=(imu_reader, imu_hist, stop_evt, IMU_FREQ_HZ), daemon=True)
                th_imu.start()
                print(f"📟 Reading {IMU_SHM_PATH} (stuck/flat/inclined) at {IMU_FREQ_HZ:.1f} Hz")
            except FileNotFoundError:
                print(f"⚠️ {IMU_SHM_PATH} not found. IMU raster will be skipped.")
                imu_hist = None

        print("🧠 Reading /dev/shm/neuron_activity at {:.1f} Hz.".format(FREQ_HZ))
        print("Press Enter to show plots. Ctrl+C to exit.")

        while True:
            input()
            plot_raster(neuron_hist, imu_hist)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_evt.set()
        except Exception:
            pass
        if neuron_reader: neuron_reader.close()
        if imu_reader: imu_reader.close()
        print("Bye.")

if __name__ == "__main__":
    main()
