#!/usr/bin/python3
"""Generate dark-themed SVG benchmark charts for the project website."""

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

ASSETS = os.path.join(os.path.dirname(__file__), '..', 'docs', 'assets')
os.makedirs(ASSETS, exist_ok=True)

DARK_BG = '#0f172a'
CARD_BG = '#1e293b'
TEXT = '#e2e8f0'
GRID = '#334155'
CYAN = '#22d3ee'
RED = '#f87171'


def style_ax(ax, title, ylabel):
    ax.set_facecolor(CARD_BG)
    ax.set_title(title, color=TEXT, fontsize=16, fontweight='bold', pad=14)
    ax.set_ylabel(ylabel, color=TEXT, fontsize=12)
    ax.tick_params(colors=TEXT, labelsize=11)
    ax.spines['bottom'].set_color(GRID)
    ax.spines['left'].set_color(GRID)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.yaxis.set_major_locator(mticker.MaxNLocator(integer=True))


def cpu_chart():
    fig, ax = plt.subplots(figsize=(6, 4.5), facecolor=DARK_BG)
    labels = ['Legacy\n(image_proc)', 'Accelerated\n(GstAdaptNode)']
    values = [180, 16]
    colors = [RED, CYAN]

    bars = ax.bar(labels, values, color=colors, width=0.55, edgecolor='none',
                  zorder=3)
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 5,
                f'{val}%', ha='center', va='bottom', color=TEXT,
                fontsize=14, fontweight='bold')

    style_ax(ax, 'CPU Utilization — 4K Resize @ 30 Hz', 'CPU %')
    ax.set_ylim(0, 220)
    ax.axhline(100, color=GRID, linestyle='--', linewidth=0.8, zorder=1)
    ax.text(1.42, 102, '1 core', color=GRID, fontsize=9, va='bottom')
    ax.grid(axis='y', color=GRID, alpha=0.3, zorder=0)

    fig.tight_layout()
    path = os.path.join(ASSETS, 'cpu_chart.svg')
    fig.savefig(path, format='svg', facecolor=DARK_BG)
    plt.close(fig)
    print(f'Saved {path}')


def latency_chart():
    fig, ax = plt.subplots(figsize=(6, 4.5), facecolor=DARK_BG)
    labels = ['Legacy\n(image_proc)', 'Accelerated\n(GstAdaptNode)']
    values = [45, 30]
    colors = [RED, CYAN]

    bars = ax.bar(labels, values, color=colors, width=0.55, edgecolor='none',
                  zorder=3)
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1.5,
                f'{val} ms', ha='center', va='bottom', color=TEXT,
                fontsize=14, fontweight='bold')

    style_ax(ax, 'Glass-to-Glass Latency — 4K → 640x480', 'Latency (ms)')
    ax.set_ylim(0, 65)
    ax.grid(axis='y', color=GRID, alpha=0.3, zorder=0)

    fig.tight_layout()
    path = os.path.join(ASSETS, 'latency_chart.svg')
    fig.savefig(path, format='svg', facecolor=DARK_BG)
    plt.close(fig)
    print(f'Saved {path}')


if __name__ == '__main__':
    cpu_chart()
    latency_chart()
    print('Done.')
