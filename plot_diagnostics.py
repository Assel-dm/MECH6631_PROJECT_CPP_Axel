import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path):
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def col(rows, name, default=float('nan')):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(name, default)))
        except Exception:
            out.append(default)
    return out


def save(fig, out):
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    print('saved', out)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', default='run_diagnostics.csv')
    ap.add_argument('--prefix', default=None)
    args = ap.parse_args()

    rows = read_csv(args.csv)
    if not rows:
        print('No rows found')
        return

    csv_path = Path(args.csv)
    prefix = args.prefix or csv_path.with_suffix('').name
    t = col(rows, 't')
    t0 = t[0]
    t = [x - t0 for x in t]

    fig = plt.figure(figsize=(10, 5))
    plt.plot(t, col(rows, 'pwm_left'), label='Left command')
    plt.plot(t, col(rows, 'pwm_right'), label='Right command')
    plt.step(t, col(rows, 'laser'), where='post', label='Laser')
    plt.xlabel('Time [s]')
    plt.ylabel('Normalized command')
    plt.title('Motor commands and laser')
    plt.grid(True, alpha=0.3)
    plt.legend()
    save(fig, f'{prefix}_commands.png')

    fig = plt.figure(figsize=(10, 7))
    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(t, col(rows, 'enemy_dist_px'), label='Enemy distance')
    ax1.plot(t, col(rows, 'nearest_obstacle_dist_px'), label='Nearest obstacle')
    ax1.set_ylabel('Distance [px]')
    ax1.set_title('Combat geometry')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax2 = plt.subplot(2, 1, 2, sharex=ax1)
    ax2.plot(t, col(rows, 'enemy_bearing_deg'), label='Enemy bearing')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Bearing [deg]')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    save(fig, f'{prefix}_geometry.png')

    fig = plt.figure(figsize=(10, 7))
    ax1 = plt.subplot(2, 1, 1)
    ax1.step(t, col(rows, 'offense_mode'), where='post', label='Offense mode')
    ax1.step(t, col(rows, 'boundary_guard'), where='post', label='Boundary guard')
    ax1.set_ylabel('State')
    ax1.set_title('Strategy state')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax2 = plt.subplot(2, 1, 2, sharex=ax1)
    ax2.plot(t, col(rows, 'num_tracks'), label='Tracks')
    ax2.plot(t, col(rows, 'num_obstacles'), label='Obstacles')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Count')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    save(fig, f'{prefix}_state.png')

    fig = plt.figure(figsize=(10, 5))
    plt.plot(t, col(rows, 'fps'), label='FPS')
    plt.xlabel('Time [s]')
    plt.ylabel('FPS')
    plt.title('Runtime performance')
    plt.grid(True, alpha=0.3)
    plt.legend()
    save(fig, f'{prefix}_fps.png')

    fig = plt.figure(figsize=(10, 5))
    plt.plot(t, col(rows, 'loop_ms'), label='Loop time')
    plt.xlabel('Time [s]')
    plt.ylabel('Time [ms]')
    plt.title('Software loop time')
    plt.grid(True, alpha=0.3)
    plt.legend()
    save(fig, f'{prefix}_loop_time.png')

    fig = plt.figure(figsize=(7, 7))
    plt.plot(col(rows, 'my_x'), col(rows, 'my_y'), label='My robot')
    plt.plot(col(rows, 'enemy_x'), col(rows, 'enemy_y'), label='Enemy')
    plt.xlabel('x [px]')
    plt.ylabel('y [px]')
    plt.title('XY trajectories')
    plt.gca().invert_yaxis()
    plt.grid(True, alpha=0.3)
    plt.legend()
    save(fig, f'{prefix}_xy.png')


if __name__ == '__main__':
    main()
