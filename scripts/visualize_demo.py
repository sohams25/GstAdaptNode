#!/usr/bin/python3
"""
GstAdaptNode A/B Web Dashboard.
Two-container architecture: tracks legacy_container and accel_container CPU independently.
"""

import json, logging, os, signal, subprocess, threading, time, warnings
import cv2, numpy as np, psutil, rclpy
from flask import Flask, Response
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image

PORT = 8080
JQ = [cv2.IMWRITE_JPEG_QUALITY, 60]
SW, SH = 320, 240


class DashNode(Node):
    def __init__(self):
        super().__init__('visualize_demo')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/legacy/image_processed', self._on_l, qos)
        self.create_subscription(Image, '/accelerated/image_processed', self._on_a, qos)

        self.lock = threading.Lock()
        blk = self._black()
        self.l_jpg = blk; self.a_jpg = blk
        self.l_evt = threading.Event(); self.a_evt = threading.Event()
        self.l_lat = 0.0; self.a_lat = 0.0
        self.l_fps = 0.0; self.a_fps = 0.0
        self.l_cpu = 0.0; self.a_cpu = 0.0
        self.l_ram = 0.0; self.a_ram = 0.0
        self._lc = 0; self._ac = 0; self._t = time.monotonic()
        self._lp = None; self._ap = None
        self.create_timer(1.0, self._stats)

    @staticmethod
    def _black():
        _, b = cv2.imencode('.jpg', np.zeros((SH, SW, 3), np.uint8), JQ)
        return b.tobytes()

    def _enc(self, msg):
        if msg.width == 0 or msg.height == 0 or len(msg.data) < msg.height * msg.step:
            return None
        if msg.encoding == 'bgr8':
            f = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == 'rgb8':
            f = cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3),
                             cv2.COLOR_RGB2BGR)
        else:
            return None
        _, b = cv2.imencode('.jpg', cv2.resize(f, (SW, SH)), JQ)
        return b.tobytes()

    def _lat(self, m):
        return (self.get_clock().now() - Time.from_msg(m.header.stamp)).nanoseconds / 1e6

    def _on_l(self, m):
        try:
            j = self._enc(m)
            if j:
                with self.lock:
                    self.l_jpg = j; self.l_lat = self._lat(m); self._lc += 1
                self.l_evt.set()
        except Exception: pass

    def _on_a(self, m):
        try:
            j = self._enc(m)
            if j:
                with self.lock:
                    self.a_jpg = j; self.a_lat = self._lat(m); self._ac += 1
                self.a_evt.set()
        except Exception: pass

    def _find(self, name):
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                if 'component_container' in ' '.join(p.info['cmdline'] or []) and name in ' '.join(p.info['cmdline'] or []):
                    p.cpu_percent(); return p
            except Exception: pass
        return None

    def _stats(self):
        try:
            if not self._lp: self._lp = self._find('legacy_container')
            if not self._ap: self._ap = self._find('accel_container')
            lc = self._lp.cpu_percent() if self._lp else 0
            lr = self._lp.memory_info().rss / 1048576 if self._lp else 0
            ac = self._ap.cpu_percent() if self._ap else 0
            ar = self._ap.memory_info().rss / 1048576 if self._ap else 0
        except Exception:
            lc, lr, ac, ar = 0, 0, 0, 0
            self._lp = None; self._ap = None
        dt = time.monotonic() - self._t
        with self.lock:
            self.l_cpu = lc; self.l_ram = lr; self.a_cpu = ac; self.a_ram = ar
            self.l_fps = self._lc / dt if dt > 0 else 0
            self.a_fps = self._ac / dt if dt > 0 else 0
            self._lc = 0; self._ac = 0
        self._t = time.monotonic()

    def json(self):
        with self.lock:
            return {
                'll': round(self.l_lat, 1), 'al': round(self.a_lat, 1),
                'lf': round(self.l_fps, 1), 'af': round(self.a_fps, 1),
                'lc': round(self.l_cpu, 0), 'ac': round(self.a_cpu, 0),
                'lr': round(self.l_ram, 0), 'ar': round(self.a_ram, 0),
                'd': round(abs(self.l_lat - self.a_lat), 1),
            }


app = Flask(__name__)
nd = None

def _mj(side):
    ev = nd.l_evt if side == 'legacy' else nd.a_evt
    while True:
        ev.wait(timeout=0.15); ev.clear()
        with nd.lock:
            j = nd.l_jpg if side == 'legacy' else nd.a_jpg
        yield b'--f\r\nContent-Type:image/jpeg\r\n\r\n' + j + b'\r\n'

@app.route('/stream/<side>')
def stream(side):
    if side not in ('legacy', 'accelerated'): return '', 404
    return Response(_mj(side), mimetype='multipart/x-mixed-replace;boundary=f')

@app.route('/api/stats')
def stats():
    return Response(json.dumps(nd.json()), mimetype='application/json')

@app.route('/')
def index(): return HTML

HTML = r'''<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GstAdaptNode Dashboard</title>
<script src="https://cdn.tailwindcss.com"></script>
<script>tailwind.config={theme:{extend:{colors:{bg:'#0a0e1a',card:'#111827',bdr:'#1e293b'}}}}</script>
<style>
body{background:#0a0e1a;margin:0}
.live{animation:b 1.5s infinite}@keyframes b{0%,100%{opacity:1}50%{opacity:.3}}
.tn{font-variant-numeric:tabular-nums}
</style>
</head>
<body class="text-gray-200 font-[system-ui] antialiased">

<header class="flex items-center justify-between px-5 py-2.5 border-b border-bdr bg-card/60 backdrop-blur sticky top-0 z-10">
  <div class="flex items-center gap-2">
    <div class="w-1.5 h-1.5 rounded-full bg-emerald-400 live"></div>
    <span class="font-semibold text-white text-sm">GstAdaptNode</span>
    <span class="text-gray-600 text-xs">A/B Benchmark</span>
  </div>
</header>

<!-- KPI Strip -->
<div class="max-w-[1100px] mx-auto px-3 pt-3 pb-1.5">
  <div class="grid grid-cols-6 gap-2">
    <div class="bg-card rounded-lg px-3 py-2.5 border border-bdr">
      <div class="text-[9px] font-bold text-amber-400 uppercase tracking-[0.15em]">Legacy Latency</div>
      <div class="text-lg font-bold tn" id="ll">—</div>
    </div>
    <div class="bg-card rounded-lg px-3 py-2.5 border border-bdr">
      <div class="text-[9px] font-bold text-emerald-400 uppercase tracking-[0.15em]">Accel Latency</div>
      <div class="text-lg font-bold tn" id="al">—</div>
    </div>
    <div class="bg-card rounded-lg px-3 py-2.5 border border-cyan-900/30">
      <div class="text-[9px] font-bold text-cyan-400 uppercase tracking-[0.15em]">Delta</div>
      <div class="text-lg font-bold text-cyan-400 tn" id="dd">—</div>
    </div>
    <div class="bg-card rounded-lg px-3 py-2.5 border border-bdr">
      <div class="text-[9px] font-bold text-amber-400 uppercase tracking-[0.15em]">Legacy CPU</div>
      <div class="text-lg font-bold tn" id="lc">—</div>
      <div class="text-[9px] text-gray-600 tn" id="lr"></div>
    </div>
    <div class="bg-card rounded-lg px-3 py-2.5 border border-bdr">
      <div class="text-[9px] font-bold text-emerald-400 uppercase tracking-[0.15em]">Accel CPU</div>
      <div class="text-lg font-bold tn" id="ac">—</div>
      <div class="text-[9px] text-gray-600 tn" id="ar"></div>
    </div>
    <div class="bg-card rounded-lg px-3 py-2.5 border border-bdr">
      <div class="text-[9px] font-bold text-gray-500 uppercase tracking-[0.15em]">FPS</div>
      <div class="flex gap-2">
        <span class="text-sm font-bold tn" id="lf">—</span>
        <span class="text-sm font-bold tn" id="af">—</span>
      </div>
      <div class="text-[9px] text-gray-600">L / A</div>
    </div>
  </div>
</div>

<!-- Video -->
<div class="max-w-[1100px] mx-auto px-3 pb-4">
  <div class="grid md:grid-cols-2 gap-2.5">
    <div class="bg-card rounded-xl overflow-hidden border border-bdr">
      <div class="flex items-center gap-1.5 px-3 py-1 border-b border-bdr">
        <div class="w-1.5 h-1.5 rounded-full bg-amber-400"></div>
        <span class="text-[9px] font-bold text-amber-400 uppercase tracking-[0.15em]">Legacy</span>
        <span class="text-[9px] text-gray-600 ml-auto">image_proc::ResizeNode</span>
      </div>
      <img src="/stream/legacy" class="w-full aspect-video bg-black" alt="">
      <div class="px-3 py-1 text-[9px] text-gray-600 flex justify-between border-t border-bdr">
        <span>OpenCV cv::resize (CPU)</span><span>image_transport + CameraSubscriber</span>
      </div>
    </div>
    <div class="bg-card rounded-xl overflow-hidden border border-emerald-900/20">
      <div class="flex items-center gap-1.5 px-3 py-1 border-b border-emerald-900/20">
        <div class="w-1.5 h-1.5 rounded-full bg-emerald-400 live"></div>
        <span class="text-[9px] font-bold text-emerald-400 uppercase tracking-[0.15em]">Accelerated</span>
        <span class="text-[9px] text-gray-600 ml-auto">gst_adapt_node::ResizeNode</span>
      </div>
      <img src="/stream/accelerated" class="w-full aspect-video bg-black" alt="">
      <div class="px-3 py-1 text-[9px] text-gray-600 flex justify-between border-t border-emerald-900/20">
        <span>Zero-copy intra-process cv::resize</span><span>Direct callback (no image_transport)</span>
      </div>
    </div>
  </div>
</div>

<footer class="text-center text-[9px] text-gray-700 py-2 border-t border-bdr">
  GstAdaptNode &middot; Apache-2.0 &middot; <a href="https://github.com/sohams25/GstAdaptNode" class="hover:text-cyan-500">GitHub</a>
</footer>

<script>
const c=ms=>ms<30?'#22c55e':ms<80?'#67e8f9':'#f87171';
const cc=p=>p<80?'#22c55e':p<150?'#fbbf24':'#f87171';
async function u(){try{
const d=await(await fetch('/api/stats')).json();
const $=id=>document.getElementById(id);
$('ll').textContent=d.ll+' ms';$('ll').style.color=c(d.ll);
$('al').textContent=d.al+' ms';$('al').style.color=c(d.al);
$('dd').textContent=d.d+' ms';
$('lc').textContent=d.lc+'%';$('lc').style.color=cc(d.lc);
$('ac').textContent=d.ac+'%';$('ac').style.color=cc(d.ac);
$('lr').textContent=d.lr+' MB';
$('ar').textContent=d.ar+' MB';
$('lf').textContent=d.lf;$('af').textContent=d.af;
}catch(e){}}
setInterval(u,500);u();
</script>
</body></html>'''


def main(args=None):
    global nd
    try:
        out = subprocess.check_output(['fuser', f'{PORT}/tcp'], stderr=subprocess.DEVNULL).decode()
        for p in out.split():
            if p.strip().isdigit(): os.kill(int(p.strip()), signal.SIGKILL)
        time.sleep(0.3)
    except Exception: pass

    rclpy.init(args=args)
    nd = DashNode()
    ex = SingleThreadedExecutor(); ex.add_node(nd)
    stop = threading.Event()
    def spin():
        while not stop.is_set():
            try: ex.spin_once(timeout_sec=0.05)
            except Exception: break
    threading.Thread(target=spin, daemon=True).start()
    nd.get_logger().info(f'Dashboard at http://localhost:{PORT}')

    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    warnings.filterwarnings('ignore', message='.*development server.*')

    try: app.run(host='0.0.0.0', port=PORT, threaded=True, use_reloader=False)
    except KeyboardInterrupt: pass
    stop.set(); ex.shutdown(); nd.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__': main()
