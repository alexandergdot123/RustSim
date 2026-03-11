import os
import numpy as np
import matplotlib.pyplot as plt
import imageio.v2 as imageio

IN_DIR = "../rust_rt_arch_sim/sim_logs"
OUT_DIR = "sim_viz"
os.makedirs(OUT_DIR, exist_ok=True)

def render_video(name, fps=30, vmin=None, vmax=None):
    arr = np.load(os.path.join(IN_DIR, f"{name}.npy"))  # shape [T,Y,X]
    T, Y, X = arr.shape
    path = os.path.join(OUT_DIR, f"{name}.mp4")

    # global scaling looks best for comparing frames
    if vmin is None: vmin = float(arr.min())
    if vmax is None: vmax = float(arr.max())

    writer = imageio.get_writer(path, fps=fps)
    for t in range(T):
        fig, ax = plt.subplots()
        im = ax.imshow(arr[t], vmin=vmin, vmax=vmax)
        ax.set_title(f"{name} t={t}")
        plt.colorbar(im, ax=ax)
        fig.canvas.draw()
        frame = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        writer.append_data(frame)
        plt.close(fig)
    writer.close()
    print("wrote", path)

def render_scalar(name):
    arr = np.load(os.path.join(IN_DIR, f"{name}.npy"))  # [Y,X]
    fig, ax = plt.subplots()
    im = ax.imshow(arr)
    ax.set_title(name)
    plt.colorbar(im, ax=ax)
    out = os.path.join(OUT_DIR, f"{name}.png")
    plt.savefig(out, dpi=200)
    plt.close(fig)
    print("wrote", out)

metrics = [
    "up_noc_util","up_noc_congestion",
    "down_noc_util","down_noc_congestion",
    "left_noc_util","left_noc_congestion",
    "right_noc_util","right_noc_congestion",
    "mailbox_congestion","core_busy"
]

for m in metrics:
    render_video(m, fps=30)

for s in ["bytes_read_close","bytes_read_far","bytes_wrote_close","bytes_wrote_far","flits_sent","flits_received"]:
    render_scalar(s)
