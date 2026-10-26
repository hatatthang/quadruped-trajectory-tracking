"""Quay video cung luot ghi cua trang /final. Chi DOC, khong dieu khien robot.

Khong sua line_tracking.py / final_control.py. Chay o luong rieng, moi ~0,2 s:
  * lay anh xem truoc /stream cua phan mem (dung anh GHI_HINH van ghi, co khung marker xanh);
  * tuy chon ve them: vet tam than da do (cam) va duong tham chieu Y = 0, tu moc 0 den 100 cm (xanh duong);
  * ghi f00001.jpg..., index.csv, moc.csv, meta.json vao line_runs/<luot>/video/ (cung luot voi samples.csv);
  * khi luot ghi ket thuc (bat ky cach nao) thi tu dung, goi docs/dung_video_gif.py dung video.mp4, nhanh.gif.
Toa do vet va duong tham chieu la toa do line (X doc line, Y lech ngang) o do cao mat marker,
chieu nguoc len anh bang chinh mat phang + noi tai ma trang /line dang dung.
"""
import copy
import json
import math
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from measurement_geometry import cv2, np
from measurement_session import save_json

BUILDER = Path(__file__).resolve().parent / 'docs' / 'dung_video_gif.py'
PERIOD_S, JPEG_Q = 0.2, 88
REF_BGR, TRAIL_BGR = (255, 120, 20), (26, 122, 255)   # tham chieu xanh duong, vet cam


class FinalVideoRecorder:
    def __init__(self, app, observer):
        self.app, self.obs = app, observer
        self.lock = threading.RLock()
        self.thread, self.stop_event = None, threading.Event()
        self.video = None            # thong tin luot dang quay
        self.last = None             # luot quay gan nhat (trang thai dung video)

    # --------------------------------------------------------- tien ich
    def _projector(self):
        """Ham doi toa do line (mm) -> diem anh 1280 px, o do cao mat marker. None neu chua co he truc."""
        with self.obs.lock:
            setup, ctx = copy.deepcopy(self.obs.setup), copy.deepcopy(self.obs.context)
        if not setup or not ctx:
            return None
        plane, cal = ctx['plane'], ctx['calibration']
        R, t = np.asarray(plane['R_plane_to_camera'], float), np.asarray(plane['t_plane_to_camera_mm'], float)
        rv = cv2.Rodrigues(R)[0]
        K, dist = np.asarray(cal['camera_matrix'], float), np.asarray(cal['distortion_coefficients'], float)
        z = float(ctx['marker_height_mm']) - float(plane['board_height_mm'])
        a, o = setup['line_angle_rad'], np.asarray(setup['origin_plane_mm'], float)
        c, s = math.cos(a), math.sin(a)

        def to_px(pts, scale):
            pts = np.asarray(pts, float).reshape(-1, 2)
            xy = o + np.c_[c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]]   # nguoc cua pose_in_line
            px = cv2.projectPoints(np.c_[xy, np.full(len(xy), z)], rv, t, K, dist)[0].reshape(-1, 2)
            return np.int32(px * scale)
        return to_px

    def _draw(self, img, trail, to_px, opts):
        k = img.shape[1] / 1920.
        if to_px is not None and opts['reference']:           # duong tham chieu Y = 0 tu 0 den 100 cm
            line = to_px(np.c_[np.linspace(0, 1000, 41), np.zeros(41)], k)
            cv2.polylines(img, [line], False, REF_BGR, 2, cv2.LINE_AA)
            for x, label in ((0, '0'), (1000, '100 cm')):
                p = to_px([[x, 0]], k)[0]
                cv2.circle(img, tuple(int(v) for v in p), 5, REF_BGR, -1, cv2.LINE_AA)
                cv2.putText(img, label, (int(p[0]) + 6, int(p[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, .55, REF_BGR, 2, cv2.LINE_AA)
        if to_px is not None and opts['trail']:                # vet tam than da do, ngat tai cho mat marker
            seg, segs = [], []
            for q in trail:
                if q is None:
                    if len(seg) > 1:
                        segs.append(seg)
                    seg = []
                else:
                    seg.append(q)
            if len(seg) > 1:
                segs.append(seg)
            for g in segs:
                cv2.polylines(img, [to_px(g, k)], False, TRAIL_BGR, 3, cv2.LINE_AA)
        return img

    # ------------------------------------------------------------- quay
    def start(self, payload):
        with self.lock:
            if self.thread and self.thread.is_alive():
                raise ValueError('Đang quay video của lượt này.')
            with self.obs.lock:
                run = copy.deepcopy(self.obs.run)
            if not run:
                raise ValueError('Bắt đầu ghi lượt trước; video đi cùng lượt ghi.')
            folder = Path(run['directory']) / 'video'
            folder.mkdir(exist_ok=False)
            index = (folder / 'index.csv').open('x', encoding='ascii')
            index.write('frame,unix,mono,file,frame_id\n')      # 4 cot dau giong GHI_HINH
            self.video = {'dir': folder, 'run_dir': run['directory'], 'index': index, 'count': 0,
                          'opts': {'trail': payload.get('trail') is True, 'reference': payload.get('reference') is True},
                          'started_unix': time.time(), 'started_mono': time.monotonic(), 'error': ''}
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()
            return 'Đang quay video cùng lượt ghi.'

    def _loop(self):
        v, last_no = self.video, None
        to_px = self._projector()
        try:
            while not self.stop_event.is_set() and not self.app.quit.is_set():
                with self.obs.lock:
                    run = self.obs.run
                    alive = bool(run) and run.get('directory') == v['run_dir']
                    trail = list(self.obs.trail) if v['opts']['trail'] else []
                if not alive:                                   # luot ghi da ket thuc -> dung video
                    break
                with self.app.lock:
                    data, no, epoch = self.app.jpeg, self.app.jpeg_no, self.app.camera_epoch
                if data and no != last_no:
                    last_no = no
                    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        if to_px is None:
                            to_px = self._projector()
                        self._draw(img, trail, to_px, v['opts'])
                        ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                        if ok:
                            v['count'] += 1
                            name = 'f%05d.jpg' % v['count']
                            (v['dir'] / name).write_bytes(enc.tobytes())
                            v['index'].write('%d,%.3f,%.3f,%s,%s\n' % (v['count'], time.time(), time.monotonic(), name, f'{epoch}:{no}'))
                            v['index'].flush()
                self.stop_event.wait(PERIOD_S)
        except Exception as exc:                                # loi video khong anh huong ghi du lieu
            v['error'] = str(exc)
        finally:
            self._close(v)

    def _close(self, v):
        v['index'].close()
        try:
            with self.obs.lock:
                events = copy.deepcopy((self.obs.last_run or {}).get('events') or (self.obs.run or {}).get('events') or [])
            with (v['dir'] / 'moc.csv').open('w', encoding='utf-8') as moc:   # moc danh dau thu cong cua luot
                moc.write('unix,mono,ghi_chu\n')
                for e in events:
                    moc.write('%.3f,%.3f,%s\n' % (v['started_unix'] + e['elapsed_s'], v['started_mono'] + e['elapsed_s'], e['name']))
        except (OSError, KeyError, TypeError):
            pass
        span = time.monotonic() - v['started_mono']
        save_json(v['dir'] / 'meta.json', {
            'ten': Path(v['run_dir']).name, 'bat_dau_unix': v['started_unix'], 'so_giay': round(span, 1),
            'so_anh': v['count'], 'hinh_moi_giay': round(v['count'] / max(span, .001), 2), 'giu_1_tren': 1,
            'nguon': 'final_video: anh /stream cua phan mem, cung luot voi samples.csv (cot frame_id)',
            'thu_muc_luot': v['run_dir'], 've_vet': v['opts']['trail'], 'duong_tham_chieu': v['opts']['reference'],
            'loi': v['error'] or None})
        with self.lock:
            self.last = {'folder': str(v['dir']), 'frames': v['count'], 'build': 'dang_dung' if v['count'] else 'khong_co_anh',
                         'files': [], 'error': v['error']}
        if v['count']:
            threading.Thread(target=self._build, args=(v['dir'],), daemon=True).start()

    def _build(self, folder):
        status, files = 'loi', []
        try:
            with open(folder / 'dung_video.log', 'w', encoding='utf-8') as log:
                done = subprocess.run([sys.executable, '-X', 'utf8', '-B', str(BUILDER), str(folder)], stdout=log,
                                      stderr=subprocess.STDOUT, timeout=3600, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            files = [n for n in ('video.mp4', 'video.avi', 'nhanh.gif') if (folder / n).exists()]
            status = 'xong' if done.returncode == 0 and files else 'loi'
        except (OSError, subprocess.SubprocessError) as exc:
            with open(folder / 'dung_video.log', 'a', encoding='utf-8') as log:
                log.write('\n' + str(exc))
        with self.lock:
            if self.last and self.last['folder'] == str(folder):
                self.last.update(build=status, files=files)

    def stop(self, payload=None):
        self.stop_event.set()
        return 'Đã dừng quay video.'

    def state(self):
        with self.lock:
            v = self.video if self.thread and self.thread.is_alive() else None
            return {'recording': bool(v), 'frames': v['count'] if v else None,
                    'options': copy.deepcopy(v['opts']) if v else None, 'last': copy.deepcopy(self.last)}

    def action(self, route, payload):
        if route == 'start':
            return self.start(payload)
        if route == 'stop':
            return self.stop(payload)
        raise ValueError('Thao tác video không hợp lệ.')
