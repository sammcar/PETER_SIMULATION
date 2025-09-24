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

# ======== SHM reader with seqlock ========
class NeuronShmReader:
    def __init__(self, path=NEURON_SHM_PATH, size=NEURON_SHM_SIZE):
        if not os.path.exists(path):
            raise FileNotFoundError(f"{path} does not exist. Publish first from the producer.")
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
        """Read a consistent snapshot (85 float32)."""
        while True:
            seq1 = struct.unpack_from("Q", self.mm, 0)[0]
            if seq1 % 2 == 1:
                time.sleep(0.001); continue  # being written
            buf = self.mm[8:8 + NEURON_COUNT * 4]
            seq2 = struct.unpack_from("Q", self.mm, 0)[0]
            if seq1 == seq2 and seq2 % 2 == 0:
                return np.frombuffer(buf, dtype=np.float32).copy()

# ======== History buffer ========
class History:
    def __init__(self, max_len=MAX_HISTORY):
        self.max_len = max_len
        self.cols = NEURON_COUNT
        self.data = np.zeros((self.cols, 0), dtype=np.float32)
        self.lock = threading.Lock()

    def push(self, vec85: np.ndarray):
        with self.lock:
            vec85 = vec85.reshape(-1, 1)  # (85,1)
            self.data = np.hstack([self.data, vec85])
            if self.data.shape[1] > self.max_len:
                self.data = self.data[:, -self.max_len:]

    def snapshot(self) -> np.ndarray:
        with self.lock:
            return self.data.copy()

# ======== Sampling thread ========
def sampler(reader: NeuronShmReader, hist: History, stop_evt: threading.Event):
    period = 1.0 / FREQ_HZ
    next_t = time.time()
    while not stop_evt.is_set():
        try:
            v = reader.read_once()
            hist.push(v)
        except Exception:
            # avoid a temporary failure killing the thread
            pass
        next_t += period
        time.sleep(max(0.0, next_t - time.time()))

# ======== Plot ========
def plot_raster(hist: History):
    data = hist.snapshot()
    if data.size == 0 or data.shape[1] < 2:
        print("Not enough data to plot yet.")
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

    # Draw groups
    plot_module(SECTS["Basal Ganglia"],   "Basal Ganglia Module")
    plot_module(SECTS["Locomotion"],      "Locomotion Module")
    plot_module(SECTS["Gait Decision"],   "Gait Decision Module")
    plot_module(SECTS["Lidar Network"],   "Lidar Network Module")
    plt.show()

# ======== Main ========
def main():
    reader = None
    try:
        reader = NeuronShmReader()
        hist = History()
        stop_evt = threading.Event()
        th = threading.Thread(target=sampler, args=(reader, hist, stop_evt), daemon=True)
        th.start()

        print("🧠 Reading /dev/shm/neuron_activity at {:.1f} Hz.".format(FREQ_HZ))
        print("Press Enter to show raster. Ctrl+C to exit.")

        while True:
            input()
            plot_raster(hist)

    except KeyboardInterrupt:
        pass
    finally:
        if reader: reader.close()
        print("Bye.")

if __name__ == "__main__":
    main()
