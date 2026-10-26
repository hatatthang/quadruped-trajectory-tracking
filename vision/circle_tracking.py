"""Trang /circle: ghi quy dao robot di vong / xoay bang marker. KHONG UART, KHONG gui lenh.

Khac /line (line_tracking.py, KHONG sua):
  * Mat san gop tu NHIEU lan dat ban co (circle_geometry.combine_planes), giu
    RIENG trong dich vu nay. Khong ghi de mat phang cua /plane, nen /line va
    /final van dung mat phang cua chung.
  * Khong can bang keo: x sang phai anh, y len tren anh, goc o trong tam vung ban co.
  * Tu kiem tra camera co bi cham dich (so nen voi anh tham chieu).
  * Ve quy dao len video va len do thi ti le that; khi dung ghi tinh ban kinh,
    tam, goc quet, goc xoay than va luu hinh trajectory.svg.
Dung chung app.action_lock voi /line, /plane, /final: tai mot thoi diem chi mot
trang duoc do (loai tru nhau, khong chen nhau). Goc lap marker va vecto bu tam
than doc tu data/line_setup_latest.json (chi doc, khong ghi).

Thu muc circle_runs/:
  mat_san_<thoi_gian>/         moi lan dat camera (hieu chuan mat san):
    reference.png              anh tham chieu de kiem tra camera dich
    board_01/, board_02/ ...   moi lan dat ban co: records.jsonl, summary.json, raw.png
    floor_plane.json           mat san gop + chan doan tung ban co
    ab_<thoi_gian>.json        kiem tra khoang cach A-B tren mat san gop (sai so doc lap)
  run_<thoi_gian>/             moi luot ghi (context.json ghi ro dung mat san nao): records.jsonl, samples.csv,
                               events.json, summary.json, quy_dao.svg/.png (hinh phan tich)
        video/                 CUNG luot: f00001.jpg... (chinh cac khung anh da dung de do,
                               frame_id khop samples.csv), index.csv, moc.csv, meta.json
                               -> tu dung video.mp4, video.avi, nhanh.gif bang docs/dung_video_gif.py
                               (dung dinh dang thu muc cua GHI_HINH.cmd)
"""
import copy
import csv
import json
import math
import shutil
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from datetime import datetime
from pathlib import Path

from measurement_geometry import cv2, np, validate_intrinsics
from measurement_session import number, save_json
from plane_geometry import (detect_board_corners, fit_plane, map_marker, _validate_plane,
                            summarize_plane_samples, plane_distance_validation)
from circle_geometry import (combine_planes, background_features, camera_shift, trajectory_metrics,
                             trajectory_svg, SHIFT_BAD_PX, KINDS, anchor_pose, reference_center, reference_errors)

BOARD_DEADLINE_S, BOARD_FRAMES, BOARD_MIN_S = 35, 15, 3   # giong trang /plane
CHECK_EVERY_S = 60                   # kiem tra camera dich trong luc ghi
TRAIL_MAX, DISPLAY_MAX = 36000, 2500     # vet ~2 gio o 5 Hz; so diem gui len trang moi lan
RUN_LIMIT_S = 3600
VIDEO_SIZE, VIDEO_JPEG_Q = (1280, 720), 88     # nhu anh /stream ma GHI_HINH ghi
MIN_FREE_BYTES = 1.5e9                        # 15 phut ~ 0,7-0,9 GB anh + video dung lai
AB_WINDOW_S = 4.0                             # A/B: lay cac anh trong 4 s gan nhat
VIDEO_BUILDER = Path(__file__).resolve().parent / 'docs' / 'dung_video_gif.py'
FIELDS = ['elapsed_s', 'time_unix', 'frame_id', 'detected', 'reason_code', 'x_mm', 'y_mm', 'heading_deg',
          'marker_x_mm', 'marker_y_mm', 'body_offset_applied', 'extrapolation_warning',
          'outside_board_mm', 'mean_edge_error_percent', 'processing_age_s']
LATEST_KEYS = ('detected', 'reason', 'x_mm', 'y_mm', 'heading_deg', 'marker_x_mm', 'marker_y_mm',
               'corners_px', 'body_px', 'extrapolation_warning', 'outside_board_mm', 'mean_edge_error_percent', 'mono')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _decimate(trail, limit=DISPLAY_MAX):
    """Rut gon vet gui len trang (du lieu day du nam trong samples.csv), giu cho dut."""
    step = max(1, math.ceil(len(trail) / limit))
    if step == 1:
        return trail
    out = []
    for i in range(0, len(trail), step):
        chunk = trail[i:i + step]
        p = next((q for q in chunk if q is not None), None)
        out.append(p)
        if p is not None and any(q is None for q in chunk):
            out.append(None)
    if trail[-1] is not None:
        out.append(trail[-1])
    return out


def _json_safe(value, fallback):
    try:
        json.dumps(value, allow_nan=False)
        return value
    except (TypeError, ValueError):
        return fallback


class CircleTrackingService:
    def __init__(self, app, root):
        self.app, self.root = app, Path(root)
        self.out_dir = self.root / 'circle_runs'
        self.setup_file = self.root / 'data' / 'line_setup_latest.json'
        self.lock = threading.RLock()
        self.worker = None
        self.stop_event = threading.Event()
        self.setup_dir = None           # thu muc cua lan dat camera hien tai
        self.boards = []                # ket qua fit_plane cua tung lan dat ban co
        self.plane = self.diagnostics = None
        self.reference = None           # dac trung nen cua anh tham chieu
        self.job = {'busy': False, 'kind': None, 'accepted': 0, 'attempted': 0, 'message': ''}
        self.context = None
        self.trail = deque(maxlen=TRAIL_MAX)
        self.latest = None
        self.last_error = ''
        self.camera_check = None
        self.run = self.last_run = None
        self.run_points, self.run_edges, self.run_outside = [], [], []
        self.live_metrics, self.live_at = None, 0.
        self.writer = self.csvfile = self.logfile = None
        self.recent = deque(maxlen=80)  # anh vua xu ly, dung cho A/B
        self.ab = None                  # ket qua kiem tra A-B tren mat san hien tai
        self.video = None               # dang ghi anh video cua luot hien tai
        self.build = None               # trang thai dung video sau khi ket thuc luot

    # ------------------------------------------------------------ tien ich
    def _raw(self):
        # Chi doc ban sao anh moi nhat, dung cach /line dang lam.
        return self.app.plane_measurement._raw()

    def _intrinsics(self, raw):
        with self.app.lock:
            candidate = copy.deepcopy(self.app.calibration)
        return candidate, validate_intrinsics(candidate, [raw[0].shape[1], raw[0].shape[0]])

    def _polygons(self, boards=None):
        return [b['corners_px'] for b in (self.boards if boards is None else boards) if b.get('corners_px')]

    def _take_lock(self):
        if not self.app.action_lock.acquire(False):
            raise ValueError('Đang có lượt đo khác (/line, /final, /plane hoặc đang xem vị trí ở trang này). '
                             'Dừng lượt đó trước.')

    def _active(self):
        return bool(self.worker and self.worker.is_alive())

    def _save_floor(self):
        save_json(self.setup_dir / 'floor_plane.json',
                  {'schema_version': 1, 'saved_at': datetime.now().isoformat(), 'plane': self.plane,
                   'diagnostics': self.diagnostics, 'boards': self.boards})

    def _body(self, use_offset):
        """Goc lap marker + vecto bu tam than da chot o /line (chi doc)."""
        try:
            s = json.loads(self.setup_file.read_text(encoding='utf-8-sig'))
            yaw, off = float(s['mount_yaw_rad']), [float(v) for v in s['offset_body_mm']]
            if not all(map(math.isfinite, [yaw, *off])):
                raise ValueError
            source = {'file': 'data/line_setup_latest.json', 'created_at': s.get('created_at'),
                      'mount_yaw_deg': round(math.degrees(yaw), 2), 'offset_body_mm': off}
        except (OSError, ValueError, KeyError, TypeError, IndexError):
            yaw, off, source = 0., [0., 0.], None
        applied = bool(use_offset and source)
        return {'mount_yaw_rad': yaw, 'offset_body_mm': off if applied else [0., 0.],
                'offset_applied': applied, 'heading_is_body': source is not None, 'source': source,
                'tracked_point': 'body_geometric_center' if applied else 'marker_center'}

    # -------------------------------------------------------- thu ban co
    def add_board(self, payload):
        height = number(payload.get('board_height_mm'), 0, 100, 'Độ cao mặt phẳng so với sàn (mm)')
        self._take_lock()
        try:
            raw = self._raw()
            if raw is None:
                raise ValueError('Chưa có ảnh camera mới. Bấm «Mở camera» trước.')
            candidate, calibration = self._intrinsics(raw)
            with self.lock:
                if self.boards and (self.boards[0]['camera_epoch'] != raw[4] or
                                    self.boards[0]['calibration_fingerprint'] != calibration['calibration_fingerprint']):
                    raise ValueError('Camera đã mở lại hoặc đổi nội tại sau lần đặt trước. '
                                     'Bấm «Khôi phục» (nếu camera không dịch) hoặc «Làm lại từ đầu».')
                self.stop_event.clear()
                self.last_error = ''
                self.job = {'busy': True, 'kind': 'board', 'accepted': 0, 'attempted': 0,
                            'message': f'Đang thu lần đặt mặt phẳng thứ {len(self.boards) + 1}; giữ mặt phẳng đứng yên, tay ra khỏi khung hình.'}
            with self.app.lock:
                self.app.busy = True
                self.app.mode = 'calibration'
            self.worker = threading.Thread(target=self._board_loop, args=(candidate, raw[4], height), daemon=True)
            self.worker.start()
            return self.job['message']
        except Exception:
            with self.lock:
                self.job['busy'] = False
            with self.app.lock:
                self.app.busy = False
            self.app.action_lock.release()
            raise

    def _board_loop(self, candidate, epoch, height):
        records, first, previous, started = [], None, None, time.monotonic()
        try:
            while not self.stop_event.is_set() and not self.app.quit.is_set():
                elapsed = time.monotonic() - started
                if elapsed >= BOARD_DEADLINE_S:
                    break
                raw = self._raw()
                if raw is None:
                    self.stop_event.wait(.05)
                    continue
                frame, frame_no, stamp, wall, ep = raw
                if ep != epoch:
                    raise ValueError('Camera đổi phiên trong lúc thu mặt phẳng.')
                if (ep, frame_no) == previous or stamp < started:
                    self.stop_event.wait(.04)
                    continue
                previous = (ep, frame_no)
                row = detect_board_corners(frame)
                row.update(frame_id=f'{ep}:{frame_no}', camera_epoch=ep, time_unix=wall, elapsed_s=stamp - started)
                records.append(row)
                accepted = sum(bool(r.get('detected')) for r in records)
                if row.get('detected') and first is None:
                    first = frame
                with self.lock:
                    self.job.update(accepted=accepted, attempted=len(records))
                if accepted >= BOARD_FRAMES and elapsed >= BOARD_MIN_S:
                    break
                self.stop_event.wait(.05)
            if self.stop_event.is_set() or self.app.quit.is_set():
                raise ValueError('Đã huỷ lượt thu mặt phẳng.')
            board = fit_plane(records, candidate, height, camera_epoch=epoch)
            if not board.get('valid') or first is None:
                raise ValueError('Mặt phẳng chưa dùng được: ' + (board.get('reason') or 'không thấy mặt phẳng.'))
            with self.lock:
                boards, reference, setup_dir = list(self.boards), self.reference, self.setup_dir
            polys = self._polygons(boards + [board])
            if reference is None:       # lan dat dau tien: chup anh tham chieu cho ca lan dat camera nay
                reference = background_features(first, polys)
                setup_dir = self.out_dir / ('mat_san_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
                setup_dir.mkdir(parents=True, exist_ok=False)
                if not cv2.imwrite(str(setup_dir / 'reference.png'), first):
                    raise RuntimeError('Không lưu được ảnh tham chiếu.')
                shift = {'status': 'tham_chieu', 'median_px': 0.0, 'ok': True}
            else:
                shift = camera_shift(reference, first, polys)
                if not shift['ok']:
                    raise ValueError(f'Camera đã dịch khoảng {shift["median_px"]} px so với lần đặt đầu '
                                     f'({shift["status"]}); không gộp được. Bấm «Làm lại từ đầu».')
            plane, diag = combine_planes(boards + [board])
            n = len(boards) + 1
            while (setup_dir / f'board_{n:02d}').exists():   # khong ghi de thu muc da co
                n += 1
            bdir = setup_dir / f'board_{n:02d}'
            bdir.mkdir(exist_ok=False)
            board = {**board, 'folder': bdir.name}
            with (bdir / 'records.jsonl').open('x', encoding='utf-8') as log:
                for r in records:
                    log.write(json.dumps(r, ensure_ascii=False, allow_nan=False) + '\n')
            save_json(bdir / 'summary.json', {**board, 'camera_shift_vs_reference': shift})
            cv2.imwrite(str(bdir / 'raw.png'), first)
            with self.lock:
                self.boards = boards + [board]
                self.ab = None
                self.plane, self.diagnostics = plane, diag
                self.reference, self.setup_dir = reference, setup_dir
                self.camera_check = {**shift, 'time': datetime.now().isoformat(), 'where': 'thu_ban_co'}
                self._save_floor()
                d = diag['boards'][-1]
                self.job['message'] = (f'Đã thu lần đặt {len(self.boards)} (nhấc mặt phẳng sang chỗ khác để thu tiếp): nghiêng {d["tilt_vs_floor_deg"]}°, '
                                       f'lệch RMS {d["rms_offset_mm"]} mm so với mặt sàn chung. Vùng đã hiệu chuẩn '
                                       f'{diag["calibrated_span_mm"][0]:.0f} × {diag["calibrated_span_mm"][1]:.0f} mm.')
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
                self.job['message'] = str(exc)
        finally:
            with self.lock:
                self.job['busy'] = False
            with self.app.lock:
                self.app.busy = False
                self.app.mode = 'marker'
            self.app.action_lock.release()

    def _board_folder(self, board):
        """Thu muc luu cua mot lan thu trong setup_dir hien tai: theo khoa 'folder', thieu thi doi chieu frame_ids."""
        root = self.setup_dir
        if root is None:
            return None
        name = board.get('folder')
        if isinstance(name, str) and name.startswith('board_') and (root / name).is_dir():
            return root / name
        ids = board.get('frame_ids')
        if not ids:
            return None
        for d in sorted(root.glob('board_*')):
            try:
                if json.loads((d / 'summary.json').read_text(encoding='utf-8')).get('frame_ids') == ids:
                    return d
            except (OSError, ValueError):
                continue
        return None

    def remove_board(self, payload):
        self._take_lock()
        try:
            with self.lock:
                if not self.boards:
                    raise ValueError('Chưa có mặt phẳng nào.')
                i = int(number(payload.get('index'), 0, len(self.boards) - 1, 'Số thứ tự mặt phẳng'))
                folder = self._board_folder(self.boards[i])
                self.boards = self.boards[:i] + self.boards[i + 1:]
                self.ab = None
                if self.boards:
                    self.plane, self.diagnostics = combine_planes(self.boards)
                else:
                    self.plane = self.diagnostics = None
                self._save_floor()
                if folder is not None:
                    shutil.rmtree(folder, ignore_errors=True)
                    return f'Đã bỏ lần đặt {i + 1} và xoá thư mục {folder.name}; thu tiếp bình thường.'
                return f'Đã bỏ lần đặt {i + 1} khỏi mặt sàn gộp.'
        finally:
            self.app.action_lock.release()

    def reset(self, payload):
        self._take_lock()
        try:
            with self.lock:
                self.boards, self.plane, self.diagnostics = [], None, None
                self.ab = None
                self.reference = self.setup_dir = self.camera_check = None
                self.trail.clear()
                self.last_error = ''
            return 'Đã làm lại từ đầu. Các tệp đã lưu vẫn còn trong circle_runs.'
        finally:
            self.app.action_lock.release()

    def restore(self, payload):
        """Dung lai mat san da luu, CHI khi so nen xac nhan camera khong dich."""
        self._take_lock()
        try:
            raw = self._raw()
            if raw is None:
                raise ValueError('Mở camera và chờ có ảnh mới trước khi khôi phục.')
            candidate, calibration = self._intrinsics(raw)
            files = sorted(self.out_dir.glob('*/floor_plane.json'))
            saved = next((json.loads(f.read_text(encoding='utf-8')) for f in files[::-1]), None)
            if not saved or not saved.get('boards'):
                raise ValueError('Chưa có mặt sàn nào đã lưu để khôi phục.')
            setup_dir, boards = files[-1].parent, saved['boards']
            if any(b['calibration_fingerprint'] != calibration['calibration_fingerprint'] or
                   b['image_size'] != calibration['image_size'] for b in boards):
                raise ValueError('Mặt sàn đã lưu thuộc bản nội tại hoặc độ phân giải khác.')
            ref_img = cv2.imread(str(setup_dir / 'reference.png'))
            if ref_img is None:
                raise ValueError('Thiếu ảnh tham chiếu reference.png; không kiểm tra được camera.')
            polys = self._polygons(boards)
            reference = background_features(ref_img, polys)
            shift = camera_shift(reference, raw[0], polys)
            if not shift['ok']:
                raise ValueError(f'Camera lệch {shift["median_px"]} px so với lúc thu mặt phẳng ({shift["status"]}). '
                                 'Không khôi phục được; thu lại mặt phẳng.')
            for b in boards:
                b['camera_epoch'] = raw[4]
            plane, diag = combine_planes(boards, plane_id=(saved.get('plane') or {}).get('plane_id'))
            _validate_plane(plane, calibration, raw[4])
            plane['restoration'] = {'time': datetime.now().isoformat(), 'camera_shift': shift,
                                    'verified_by': 'background_feature_match_not_user_claim'}
            with self.lock:
                self.boards, self.plane, self.diagnostics = boards, plane, diag
                self.ab = None
                self.reference, self.setup_dir = reference, setup_dir
                self.camera_check = {**shift, 'time': datetime.now().isoformat(), 'where': 'khoi_phuc'}
                self.last_error = ''
            return (f'Đã khôi phục mặt sàn {setup_dir.name} ({len(boards)} lần đặt mặt phẳng). '
                    f'Camera lệch {shift["median_px"]} px so với ảnh tham chiếu.')
        finally:
            self.app.action_lock.release()

    def check_camera(self, payload):
        with self.lock:
            reference, polys = self.reference, self._polygons()
        if reference is None:
            raise ValueError('Chưa có ảnh tham chiếu (thu mặt phẳng trước).')
        raw = self._raw()
        if raw is None:
            raise ValueError('Chưa có ảnh camera mới.')
        shift = camera_shift(reference, raw[0], polys)
        with self.lock:
            self.camera_check = {**shift, 'time': datetime.now().isoformat(), 'where': 'bam_kiem_tra'}
        if shift['median_px'] is None:
            return 'Không đủ chi tiết nền để so; kiểm tra bằng mắt.'
        return f'Camera lệch {shift["median_px"]} px so với ảnh tham chiếu ({shift["status"]}).'

    # --------------------------------------------------------- xem vi tri
    def start(self, payload):
        height = number(payload.get('marker_height_mm'), 1, 500, 'Độ cao mặt marker')
        self._take_lock()
        try:
            raw = self._raw()
            if raw is None:
                raise ValueError('Mở camera và chờ có ảnh mới trước.')
            candidate, calibration = self._intrinsics(raw)
            with self.lock:
                plane, diag = copy.deepcopy(self.plane), copy.deepcopy(self.diagnostics)
                reference, polys = self.reference, self._polygons()
            if not plane:
                raise ValueError('Chưa có mặt sàn. Thu mặt phẳng ít nhất một vị trí (hoặc khôi phục) trước.')
            if not diag['usable']:
                raise ValueError('Có lần đặt mặt phẳng lệch quá ngưỡng so với mặt sàn chung (đánh dấu đỏ). Bỏ lần đó hoặc đặt lại cho phẳng rồi thu.')
            _validate_plane(plane, calibration, raw[4])
            shift = camera_shift(reference, raw[0], polys)
            if shift['median_px'] is not None and shift['median_px'] > SHIFT_BAD_PX:
                raise ValueError(f'Camera đã dịch {shift["median_px"]} px so với lúc thu mặt phẳng. Thu lại mặt phẳng.')
            body = self._body(payload.get('apply_offset') is True)
            self.context = {'calibration': candidate, 'plane': plane, 'diagnostics': diag, 'camera_epoch': raw[4],
                            'marker_height_mm': height, 'black_side_mm': 50., 'body': body,
                            'session_id': self.app.session_id, 'image_size': calibration['image_size'],
                            'setup_dir': str(self.setup_dir)}
            with self.lock:
                self.trail.clear()
                self.latest = None
                self.last_error = ''
                self.camera_check = {**shift, 'time': datetime.now().isoformat(), 'where': 'bat_xem'}
                self.stop_event.clear()
            with self.app.lock:
                self.app.busy = True
                self.app.mode = 'marker'
            self.worker = threading.Thread(target=self._loop, daemon=True)
            self.worker.start()
            note = '' if shift['ok'] else f' Lưu ý: kiểm tra camera cho kết quả «{shift["status"]}».'
            return ('Đang xem vị trí ' + ('tâm thân.' if body['offset_applied'] else 'tâm marker.') + note)
        except Exception:
            self.app.action_lock.release()
            raise

    def _pose(self, row, body, height, rvec, t, K, dist):
        theta = wrap(row['yaw_rad'] + body['mount_yaw_rad'])
        dx, dy = body['offset_body_mm']
        mx, my = row['position_plane_mm']
        cx = mx + math.cos(theta) * dx - math.sin(theta) * dy
        cy = my + math.sin(theta) * dx + math.cos(theta) * dy
        # Chieu tam than (o do cao marker) nguoc ve anh de ve vet len video.
        px, _ = cv2.projectPoints(np.array([[cx, cy, height]], float), rvec, t, K, dist)
        return {'x_mm': cx, 'y_mm': cy, 'heading_deg': math.degrees(theta), 'heading_rad': theta,
                'marker_x_mm': mx, 'marker_y_mm': my, 'body_px': px.reshape(2).tolist(),
                'body_offset_applied': body['offset_applied'], 'tracked_point': body['tracked_point']}

    def _loop(self):
        ctx = self.context
        K = np.asarray(ctx['calibration']['camera_matrix'], float)
        dist = np.asarray(ctx['calibration']['distortion_coefficients'], float)
        rvec, _ = cv2.Rodrigues(np.asarray(ctx['plane']['R_plane_to_camera'], float))
        t = np.asarray(ctx['plane']['t_plane_to_camera_mm'], float)
        # Vung ban co da phu (he mat san). Khoang cach ra ngoai vung nay cho biet muc do ngoai suy.
        hull = np.asarray(ctx['plane']['board_hull_plane_mm'], np.float32).reshape(-1, 1, 2)
        previous, last_check = None, time.monotonic()
        try:
            while not self.stop_event.is_set() and not self.app.quit.is_set():
                raw = self._raw()
                frame = None
                row = {'detected': False, 'reason_code': 'camera_missing_or_stale',
                       'reason': 'Không có ảnh camera mới.', 'mono': time.monotonic(), 'time_unix': time.time()}
                if raw is not None:
                    frame, frame_no, stamp, wall, epoch = raw
                    if epoch != ctx['camera_epoch']:
                        raise ValueError('Camera đã mở lại. Dừng đo; bấm «Khôi phục» để kiểm tra và dùng lại mặt sàn.')
                    if previous == (epoch, frame_no):
                        self.stop_event.wait(.03)
                        continue
                    previous = (epoch, frame_no)
                    row = map_marker(frame, ctx['calibration'], ctx['plane'], ctx['marker_height_mm'], camera_epoch=epoch)
                    row.update(mono=stamp, time_unix=wall, frame_id=f'{epoch}:{frame_no}',
                               processing_age_s=time.monotonic() - stamp)
                    if row['processing_age_s'] > .6:
                        row.update(detected=False, reason_code='processing_stale', reason='Ảnh đo đã quá cũ.')
                    if row.get('detected'):
                        mx, my = map(float, row['position_plane_mm'])
                        row['outside_board_mm'] = round(max(0., -cv2.pointPolygonTest(hull, (mx, my), True)), 1)
                        row.update(self._pose(row, ctx['body'], ctx['marker_height_mm'], rvec, t, K, dist))
                # Trong luc ghi: dinh ky so nen de biet camera co bi cham dich (ngoai khoa, ~0,15 s).
                check = None
                if self.run is not None and frame is not None and time.monotonic() - last_check >= CHECK_EVERY_S:
                    last_check = time.monotonic()
                    check = camera_shift(self.reference, frame, self._polygons())
                with self.lock:
                    self.latest = row
                    self.recent.append(row)
                    ok = bool(row.get('detected'))
                    self.trail.append([round(row['x_mm'], 1), round(row['y_mm'], 1), round(row['body_px'][0], 1),
                                       round(row['body_px'][1], 1), round(row['heading_deg'], 2)] if ok else None)
                    if check is not None:
                        self.camera_check = {**check, 'time': datetime.now().isoformat(), 'where': 'trong_luot_ghi'}
                        if self.run:
                            self.run['camera_checks'].append({**check, 'elapsed_s': time.monotonic() - self.run['started_mono']})
                            if check['median_px'] is not None and check['median_px'] > SHIFT_BAD_PX:
                                self.run['camera_moved_warning'] = True
                    if self.run:
                        entry = {**row, 'elapsed_s': row['mono'] - self.run['started_mono']}
                        if entry['elapsed_s'] >= 0:       # anh chup truoc luc bam ghi khong thuoc luot
                            self.logfile.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
                            self.logfile.flush()
                            self.writer.writerow({k: entry.get(k) for k in FIELDS})
                            self.csvfile.flush()
                            self.run['samples'] += 1
                            if self.video is not None and frame is not None:
                                self._video_write(frame, row, entry)
                            if ok:
                                self.run['detected'] += 1
                                self.run['extrapolated'] += int(bool(row.get('extrapolation_warning')))
                                self.run_points.append((entry['elapsed_s'], row['x_mm'], row['y_mm'], row['heading_rad'],
                                                        float(bool(row.get('extrapolation_warning')))))
                                self.run_edges.append(abs(row['mean_edge_error_percent']))
                                self.run_outside.append(row['outside_board_mm'])
                                self.run['outside_max_mm'] = max(self.run['outside_max_mm'], row['outside_board_mm'])
                        if time.monotonic() - self.run['started_mono'] > RUN_LIMIT_S:
                            self._finish('one_hour_limit')
                self.stop_event.wait(.12)
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
                self.latest = {'detected': False, 'reason': str(exc), 'mono': time.monotonic()}
        finally:
            try:
                with self.lock:
                    self._finish(self.last_error or 'preview_stopped')
            finally:
                with self.app.lock:
                    self.app.busy = False
                self.app.action_lock.release()

    # ----------------------------------------------------------- ghi luot
    def record(self, payload):
        with self.lock:
            if not self._active() or not self.context or self.job.get('busy'):
                raise ValueError('Bật xem vị trí trước.')
            if self.run:
                raise ValueError('Đang ghi một lượt.')
            if not self.latest or not self.latest.get('detected') or time.monotonic() - self.latest['mono'] > .6:
                raise ValueError('Chưa có vị trí marker mới hợp lệ.')
            command = {}       # lenh da nap cho robot -> dung duong dat de so sai so (khong bat buoc)
            kind = payload.get('cmd_kind') or None
            if kind is not None and kind not in KINDS:
                raise ValueError('Kiểu lệnh không hợp lệ.')
            for key, lo, hi, label in (('cmd_radius_mm', 50, 5000, 'R đặt'), ('cmd_angle_deg', 1, 3600, 'Góc đặt')):
                if payload.get(key) not in (None, ''):
                    command[key[4:]] = number(payload.get(key), lo, hi, label)
            if kind:
                command['kind'] = kind
                if kind in ('cw', 'ccw') and 'radius_mm' not in command:
                    raise ValueError('Đi vòng: nhập R đặt để dựng đường đặt.')
            ident = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]
            directory = self.out_dir / ('run_' + ident)      # moi luot nam thang trong circle_runs/
            directory.mkdir(parents=True, exist_ok=False)
            save_json(directory / 'context.json', {
                'schema_version': 1, 'context': self.context, 'note': str(payload.get('note', ''))[:500],
                'camera_check_at_start': self.camera_check, 'ab_check': self._ab_brief(), 'command': command,
                'limitations': ['Mặt marker giả thiết song song sàn ở độ cao cố định.',
                                'Vị trí ngoài vùng mặt phẳng là ngoại suy; cột outside_board_mm cho biết cách vùng mặt phẳng bao xa.',
                                'Chỉ ghi các frame đã xử lý (~5 Hz), không phải mọi frame camera.',
                                'Chỉ quan sát. Không UART, không gửi lệnh cho robot.'],
                'record_start_is_not_gait_start': True})
            self.logfile = (directory / 'records.jsonl').open('x', encoding='utf-8')
            self.csvfile = (directory / 'samples.csv').open('x', encoding='utf-8-sig', newline='')
            self.writer = csv.DictWriter(self.csvfile, fieldnames=FIELDS)
            self.writer.writeheader()
            self.run = {'directory': str(directory), 'started_mono': time.monotonic(),
                        'started_at': datetime.now().isoformat(), 'samples': 0, 'detected': 0,
                        'extrapolated': 0, 'outside_max_mm': 0., 'events': [], 'camera_checks': [], 'camera_moved_warning': False,
                        'note': str(payload.get('note', ''))[:500], 'accuracy_validated': False,
                        'command': command, 'anchor': None}
            self.run_points, self.run_edges, self.run_outside = [], [], []
            self.live_metrics, self.live_at = None, 0.
            self.trail.clear()
            note = ''
            if payload.get('video', True):
                free = shutil.disk_usage(directory).free
                if free < MIN_FREE_BYTES:
                    note = f' KHÔNG quay video: ổ đĩa chỉ còn {free / 1e9:.1f} GB.'
                else:
                    self._video_open(directory / 'video', payload.get('video_trail') is True)
            self.run['video'] = None if self.video is None else {
                'folder': str(self.video['dir']), 'frames': 0, 'draw_trail': self.video['draw_trail'],
                'same_frames_as_samples_csv': True, 'build': 'chua_dung'}
            return ('Đang ghi dữ liệu' + (' và quay video' if self.video else '') +
                    '. Nút ghi không khởi động robot; bật robot bằng firmware như thường lệ.' + note)

    def _finish(self, reason):
        """Dong luot ghi: tinh chi so, luu summary.json va trajectory.svg. Goi khi dang giu self.lock."""
        if self.run is None:
            return
        run = self.run
        run['stop_reason'] = reason
        run['duration_s'] = time.monotonic() - run['started_mono']
        run['finished_at'] = datetime.now().isoformat()
        run['processed_detection_fraction'] = run['detected'] / run['samples'] if run['samples'] else None
        run['extrapolated_fraction'] = run['extrapolated'] / run['detected'] if run['detected'] else None
        run['mean_abs_edge_error_percent'] = float(np.mean(self.run_edges)) if self.run_edges else None
        run['outside_board_p95_mm'] = float(np.percentile(self.run_outside, 95)) if self.run_outside else None
        try:
            metrics = trajectory_metrics(self.run_points)
            run['metrics'] = _json_safe(metrics, {'valid': False, 'reason': 'Chỉ số không hữu hạn.'})
            cmd = run.get('command') or {}
            if cmd.get('kind'):     # duong dat: neo theo nut bat dau buoc, neu khong co thi dau luot ghi
                anchor = run.get('anchor') or anchor_pose(self.run_points)
                run['reference'] = _json_safe(reference_errors(self.run_points, anchor, cmd['kind'], cmd.get('radius_mm'),
                                                               cmd.get('angle_deg')), None)
            hull = (self.context or {}).get('plane', {}).get('board_hull_plane_mm')
            title = 'Quỹ đạo ' + ('tâm thân' if self.context['body']['offset_applied'] else 'tâm marker') + \
                    ' · ' + run['started_at'][:19].replace('T', ' ') + (' · ' + run['note'] if run['note'] else '')
            title = title.replace('&', 'và').replace('<', '')
            try:        # hinh kieu bao cao: quy dao + sai lech ban kinh + goc than (SVG + PNG)
                import circle_figure
                files = circle_figure.make(Path(run['directory']) / 'quy_dao', self.run_points, hull, run['metrics'], title,
                                           ref=run.get('reference'))
                if files:
                    run['figure'], run['figure_png'] = map(str, files)
            except Exception as exc:
                run['figure_error'] = str(exc)
            if not run.get('figure'):   # du phong: hinh SVG don gian
                svg = trajectory_svg(list(self.trail), hull, run['metrics'], title=title)
                if svg:
                    (Path(run['directory']) / 'trajectory.svg').write_text(svg, encoding='utf-8')
                    run['figure'] = str(Path(run['directory']) / 'trajectory.svg')
        except Exception as exc:
            run['metrics_error'] = str(exc)
        folder = self._video_close(run)
        try:
            self.logfile.close()
            self.csvfile.close()
            try:        # hinh danh gia sai so, doc lai tu samples.csv + context.json cua chinh luot nay
                import circle_figure
                files = circle_figure.make_errors(Path(run['directory']), run.get('metrics'), ref=run.get('reference'))
                if files:
                    run['error_figure'], run['error_figure_png'] = map(str, files)
            except Exception as exc:
                run['error_figure_error'] = str(exc)
            save_json(Path(run['directory']) / 'summary.json', run)
        finally:
            self.last_run = copy.deepcopy(run)
            self.run = self.writer = self.logfile = self.csvfile = None
        if folder is not None:      # dung video o tien trinh rieng, khong chan camera
            self.build = {'folder': str(folder), 'status': 'dang_dung', 'files': [], 'started_at': datetime.now().isoformat()}
            threading.Thread(target=self._build_video, args=(folder, Path(run['directory']) / 'summary.json'),
                             daemon=True).start()

    # ---------------------------------------------------- video cung luot
    def _video_open(self, folder, draw_trail):
        folder.mkdir(exist_ok=False)
        index = (folder / 'index.csv').open('x', encoding='ascii')
        index.write('frame,unix,mono,file,frame_id,elapsed_s,detected\n')   # 4 cot dau giong GHI_HINH
        moc = (folder / 'moc.csv').open('x', encoding='utf-8')
        moc.write('unix,mono,ghi_chu\n')
        self.video = {'dir': folder, 'index': index, 'moc': moc, 'count': 0, 'draw_trail': draw_trail,
                      'started_unix': time.time(), 'started_mono': time.monotonic()}

    def _video_write(self, frame, row, entry):
        """Luu CHINH khung anh vua dung de do (thu nho 1280x720), kem khung marker va chu thich."""
        v = self.video
        try:
            img = cv2.resize(frame, VIDEO_SIZE, interpolation=cv2.INTER_AREA)
            k = VIDEO_SIZE[0] / frame.shape[1]
            if v['draw_trail']:
                seg, segs = [], []
                for q in self.trail:
                    if q is None:
                        if len(seg) > 1:
                            segs.append(seg)
                        seg = []
                    else:
                        seg.append((q[2] * k, q[3] * k))
                if len(seg) > 1:
                    segs.append(seg)
                if segs:
                    cv2.polylines(img, [np.int32(g) for g in segs], False, (26, 122, 255), 3, cv2.LINE_AA)
            if v.get('ref_px') is not None:        # duong dat (vang), ve truoc marker
                cv2.polylines(img, [v['ref_px']], False, (40, 210, 255), 2, cv2.LINE_AA)
            if row.get('detected') and row.get('corners_px'):
                c = np.int32(np.asarray(row['corners_px']) * k)
                cv2.polylines(img, [c], True, (80, 225, 90), 3, cv2.LINE_AA)
                x, y = c.mean(axis=0).astype(int)
                cv2.putText(img, 'ID 900', (max(0, x - 40), max(22, y - 30)), cv2.FONT_HERSHEY_SIMPLEX, .6,
                            (80, 225, 90), 2, cv2.LINE_AA)
            text = f"{datetime.fromtimestamp(entry['time_unix']):%H:%M:%S}  t={entry['elapsed_s']:7.1f} s  frame {entry['frame_id']}"
            cv2.rectangle(img, (0, VIDEO_SIZE[1] - 30), (430, VIDEO_SIZE[1]), (0, 0, 0), -1)
            cv2.putText(img, text, (8, VIDEO_SIZE[1] - 9), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
            ok, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, VIDEO_JPEG_Q])
            if not ok:
                return
            v['count'] += 1
            name = 'f%05d.jpg' % v['count']
            (v['dir'] / name).write_bytes(enc.tobytes())
            v['index'].write('%d,%.3f,%.3f,%s,%s,%.3f,%d\n' % (v['count'], entry['time_unix'], entry['mono'], name,
                                                             entry['frame_id'], entry['elapsed_s'], int(bool(row.get('detected')))))
            v['index'].flush()
            self.run['video']['frames'] = v['count']
        except (OSError, cv2.error, KeyError, TypeError, ValueError) as exc:
            self.run['video']['error'] = str(exc)   # loi video khong duoc lam dung ghi du lieu

    def _video_close(self, run):
        v, self.video = self.video, None
        if v is None:
            return None
        v['index'].close()
        v['moc'].close()
        span = time.monotonic() - v['started_mono']
        save_json(v['dir'] / 'meta.json', {
            'ten': run.get('note') or Path(run['directory']).name, 'bat_dau_unix': v['started_unix'],
            'so_giay': round(span, 1), 'so_anh': v['count'], 'hinh_moi_giay': round(v['count'] / max(span, .001), 2),
            'giu_1_tren': 1, 'nguon': 'circle_tracking: chính các khung ảnh đã dùng để đo, frame_id khớp samples.csv',
            'thu_muc_luot': run['directory'], 've_vet_quy_dao': v['draw_trail']})
        return v['dir'] if v['count'] else None

    def _build_video(self, folder, summary_path):
        """Goi docs/dung_video_gif.py (bo dung cua GHI_HINH) -> video.mp4, video.avi, nhanh.gif."""
        status, files = 'loi', []
        try:
            with open(folder / 'dung_video.log', 'w', encoding='utf-8') as log:
                done = subprocess.run([sys.executable, '-X', 'utf8', '-B', str(VIDEO_BUILDER), str(folder)],
                                      stdout=log, stderr=subprocess.STDOUT, timeout=3600,
                                      creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            files = [n for n in ('video.mp4', 'video.avi', 'nhanh.gif') if (folder / n).exists()]
            status = 'xong' if done.returncode == 0 and files else 'loi'
        except (OSError, subprocess.SubprocessError) as exc:
            with open(folder / 'dung_video.log', 'a', encoding='utf-8') as log:
                log.write('\n' + str(exc))
        with self.lock:
            if self.build and self.build['folder'] == str(folder):
                self.build.update(status=status, files=files, finished_at=datetime.now().isoformat())
        try:
            summary = json.loads(summary_path.read_text(encoding='utf-8'))
            summary['video'].update(build=status, files=files)
            save_json(summary_path, summary)
            with self.lock:
                if self.last_run and self.last_run.get('directory') == summary['directory']:
                    self.last_run['video'] = summary['video']
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            pass

    # ------------------------------------------------ kiem tra A-B doc lap
    def _ab_brief(self):
        ab = self.ab
        if not ab:
            return None
        def brief(x):
            if not x:
                return None
            out = {k: x['summary'][k] for k in ('position_plane_mm', 'scatter_p95_mm', 'count_unique_valid')}
            pts = [np.mean(np.asarray(r['corners_px'], float), axis=0) for r in x.get('rows') or [] if r.get('corners_px')]
            if pts:                                     # tam marker tren anh camera (px), trung vi cac anh da gop
                out['center_px'] = [round(float(v), 1) for v in np.median(np.asarray(pts), axis=0)]
            return out
        r = ab.get('result')
        return {'A': brief(ab.get('A')), 'B': brief(ab.get('B')), 'outside_board_mm': ab.get('outside'),
                'result': None if not r else {k: r[k] for k in ('expected_mm', 'measured_mm', 'signed_error_mm', 'signed_error_percent')},
                'marker_height_mm': ab.get('marker_height_mm'), 'plane_fingerprint': ab.get('plane_fingerprint'), 'time': ab.get('time')}

    def ab_capture(self, payload):
        """Robot dung yen tai A (roi B); lay cac anh 4 s gan nhat, dung dung ham A/B cua trang /plane."""
        label = payload.get('label')
        if label not in ('A', 'B'):
            raise ValueError('Chọn A hoặc B.')
        with self.lock:
            if not self._active() or not self.context or self.job.get('busy'):
                raise ValueError('Bật xem vị trí trước khi thu A/B.')
            if self.run:
                raise ValueError('Đang ghi lượt chạy. Thu A/B trước khi bấm ghi.')
            now = time.monotonic()
            rows = [copy.deepcopy(r) for r in self.recent if r.get('detected') and now - r['mono'] <= AB_WINDOW_S]
        if len(rows) < 10:
            raise ValueError(f'Mới có {len(rows)} ảnh thấy marker trong 4 giây gần nhất; giữ robot đứng yên thêm rồi bấm lại.')
        summary = summarize_plane_samples(rows)
        if not summary['valid']:
            raise ValueError(f'Robot chưa đứng yên (dao động p95 {summary.get("scatter_p95_mm") or 0:.1f} mm, cho phép 5 mm). '
                             'Chờ rồi bấm lại.')
        outside = float(np.median([r['outside_board_mm'] for r in rows]))
        if label == 'A':
            with self.lock:
                self.ab = {'A': {'rows': rows, 'summary': summary}, 'B': None, 'result': None, 'outside': [outside, None],
                           'marker_height_mm': self.context['marker_height_mm'],
                           'plane_fingerprint': self.context['plane']['plane_fingerprint'], 'time': datetime.now().isoformat()}
            return (f'Đã thu A ({len(rows)} ảnh, dao động {summary["scatter_p95_mm"]:.1f} mm). '
                    'Đẩy robot thẳng theo thước đúng khoảng cách đã nhập, không xoay thân, rồi thu B.')
        expected = number(payload.get('expected_mm'), 50, 3000, 'Khoảng cách chuẩn A–B (mm)')
        with self.lock:
            ab = self.ab
        if not ab or not ab.get('A'):
            raise ValueError('Thu A trước.')
        result = plane_distance_validation(ab['A']['rows'], rows, expected)
        if not result['valid']:
            raise ValueError('Chưa tính được A–B: ' + result['reason'])
        with self.lock:
            self.ab.update(B={'rows': rows, 'summary': summary}, result=result, time=datetime.now().isoformat())
            self.ab['outside'][1] = outside
            brief, setup_dir = self._ab_brief(), self.setup_dir
        save_json(setup_dir / ('ab_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.json'), {**brief, 'full_result': result})
        return (f'A–B đo được {result["measured_mm"]:.1f} mm, chuẩn {expected:.0f} mm → sai {result["signed_error_mm"]:+.1f} mm '
                f'({result["signed_error_percent"]:+.2f} %). A, B cách vùng mặt phẳng {brief["outside_board_mm"][0]:.0f} / {outside:.0f} mm.')

    def ab_reset(self, payload):
        """Xoa A va B de do lai tu dau; tep ab_*.json da luu van giu."""
        with self.lock:
            if self.run:
                raise ValueError('Đang ghi lượt chạy. Kết thúc lượt rồi mới đo lại A–B.')
            self.ab = None
        return 'Đã xoá A và B. Đặt robot đứng yên ở A, chờ 4 giây rồi bấm «Thu A».'

    def event(self, payload):
        with self.lock:
            if not self.run:
                raise ValueError('Bắt đầu ghi trước khi đánh dấu.')
            self.run['events'].append({'elapsed_s': time.monotonic() - self.run['started_mono'],
                                       'name': str(payload.get('name', 'manual_event'))[:100],
                                       'source': 'manual_button_not_firmware_timestamp'})
            save_json(Path(self.run['directory']) / 'events.json', self.run['events'])
            note = ''
            if self.run['events'][-1]['name'] == 'gait_start':
                anchor = anchor_pose(self.run_points, t_event=self.run['events'][-1]['elapsed_s'])
                if anchor is None:
                    note = ' Chưa đủ ảnh robot đứng yên trong 2,5 s trước đó; đường đặt sẽ neo theo đầu lượt ghi.'
                else:
                    self.run['anchor'] = anchor
                    self._reference_px()
                    note = (f' Đã neo đường đặt tại ({anchor["p0_mm"][0]:.0f}; {anchor["p0_mm"][1]:.0f}) mm, '
                            f'hướng {anchor["heading0_deg"]:.1f}°' + (' — robot CHƯA đứng yên!' if anchor['still_p95_mm'] > 5 else '.'))
            if self.video is not None:
                self.video['moc'].write('%.3f,%.3f,%s\n' % (time.time(), time.monotonic(), self.run['events'][-1]['name']))
                self.video['moc'].flush()
        return 'Đã đánh dấu thời điểm thủ công.' + note

    def _reference_live(self):
        """Duong dat de ve truc tiep (tren do thi trang va video)."""
        run = self.run or {}
        cmd, anchor = run.get('command') or {}, run.get('anchor')
        if not anchor or not cmd.get('kind'):
            return None
        out = {'kind': cmd['kind'], 'p0_mm': anchor['p0_mm'], 'heading0_deg': anchor['heading0_deg']}
        if cmd['kind'] in ('cw', 'ccw'):
            out.update(center_mm=reference_center(anchor, cmd['kind'], cmd['radius_mm']).tolist(), radius_mm=cmd['radius_mm'])
        return out

    def _reference_px(self):
        """Chieu duong dat (o do cao marker) len anh video, tinh mot lan luc neo."""
        ref, v = self._reference_live(), self.video
        if not ref or v is None or not v['draw_trail']:
            return
        ctx = self.context
        if 'center_mm' in ref:
            a = np.linspace(0, 2 * math.pi, 241)
            pts = np.c_[ref['center_mm'][0] + ref['radius_mm'] * np.cos(a), ref['center_mm'][1] + ref['radius_mm'] * np.sin(a)]
        else:
            q = np.asarray(ref['p0_mm'])
            pts = np.array([q + [-40, 0], q + [40, 0], q, q + [0, -40], q + [0, 40]])
        P = np.c_[pts, np.full(len(pts), ctx['marker_height_mm'])]
        rv = cv2.Rodrigues(np.asarray(ctx['plane']['R_plane_to_camera'], float))[0]
        px = cv2.projectPoints(P, rv, np.asarray(ctx['plane']['t_plane_to_camera_mm'], float),
                               np.asarray(ctx['calibration']['camera_matrix'], float),
                               np.asarray(ctx['calibration']['distortion_coefficients'], float))[0].reshape(-1, 2)
        v['ref_px'] = np.int32(px * VIDEO_SIZE[0] / ctx['image_size'][0])

    def stop_recording(self, payload):
        with self.lock:
            if not self.run:
                raise ValueError('Không có lượt nào đang ghi.')
            self._finish('user_stopped_recording')
            m = (self.last_run or {}).get('metrics') or {}
        c = m.get('circle')
        return ('Đã lưu. Thân xoay ' + str(m.get('heading_change_deg', '—')) + '°' +
                (f' · R khớp {c["radius_mm"]} mm · quét {c["swept_deg"]}°' if c else '') + '. Camera vẫn xem vị trí.')

    def cancel(self):
        self.stop_event.set()
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=5)

    # --------------------------------------------------------- trang thai
    def state(self):
        try:
            camera = self.app.state()['camera']
            with self.lock:
                latest = self.latest
                if latest and time.monotonic() - latest.get('mono', 0) > 1.5:
                    latest = {'detected': False, 'reason': 'Ảnh đo đã cũ.'}
                latest = {k: latest.get(k) for k in LATEST_KEYS if k in latest} if latest else None
                if self.run and time.monotonic() - self.live_at > 2:
                    self.live_metrics = _json_safe(trajectory_metrics(self.run_points), None)
                    self.live_at = time.monotonic()
                run = None
                if self.run:
                    run = {k: self.run[k] for k in ('directory', 'samples', 'detected', 'extrapolated',
                                                    'outside_max_mm', 'camera_moved_warning', 'note', 'video')}
                    run.update(elapsed_s=time.monotonic() - self.run['started_mono'], events=len(self.run['events']),
                               metrics=self.live_metrics, reference=self._reference_live(), command=self.run.get('command'))
                last = None
                if self.last_run:
                    last = {k: self.last_run.get(k) for k in ('directory', 'samples', 'detected', 'duration_s',
                            'processed_detection_fraction', 'extrapolated_fraction', 'mean_abs_edge_error_percent',
                            'outside_max_mm', 'outside_board_p95_mm', 'figure_png', 'error_figure_png', 'video', 'reference', 'command',
                            'metrics', 'figure', 'stop_reason', 'camera_moved_warning', 'metrics_error')}
                plane = self.plane
                return {'active': self._active() and self.job.get('busy') is False, 'job': copy.deepcopy(self.job),
                        'camera': camera, 'boards': copy.deepcopy((self.diagnostics or {}).get('boards', [])),
                        'board_polygons_px': [[b['corners_px'][i] for i in (0, 8, 53, 45)] for b in self.boards],
                        'floor': None if not plane else {
                            'plane_id': plane['plane_id'], 'fingerprint': plane['plane_fingerprint'][:12],
                            'hull_mm': plane['board_hull_plane_mm'],
                            **{k: self.diagnostics[k] for k in ('board_count', 'fit_rms_mm', 'worst_tilt_deg',
                               'worst_rms_mm', 'calibrated_span_mm', 'camera_distance_to_floor_mm',
                               'camera_tilt_from_vertical_deg', 'usable')}},
                        'setup_dir': str(self.setup_dir) if self.setup_dir else None,
                        'restore_available': any(self.out_dir.glob('*/floor_plane.json')),
                        'camera_check': copy.deepcopy(self.camera_check), 'body': self._body(True)['source'],
                        'tracking': None if not self.context else {
                            'marker_height_mm': self.context['marker_height_mm'],
                            'offset_applied': self.context['body']['offset_applied']},
                        'latest': copy.deepcopy(latest), 'trail': _decimate(list(self.trail)),
                        'recording': copy.deepcopy(run), 'last_run': copy.deepcopy(last), 'error': self.last_error,
                        'ab': self._ab_brief(), 'build': copy.deepcopy(self.build)}
        except Exception as exc:          # trang thai khong duoc lam hong may chu
            return {'error': 'Lỗi đọc trạng thái /circle: ' + str(exc)}

    def action(self, route, payload):
        handlers = {'ab': self.ab_capture, 'ab_reset': self.ab_reset, 'board': self.add_board, 'remove_board': self.remove_board, 'reset': self.reset,
                    'restore': self.restore, 'check_camera': self.check_camera, 'preview': self.start,
                    'record': self.record, 'event': self.event, 'stop': self.stop_recording}
        if route == 'cancel':
            self.cancel()
            return 'Đã dừng (lượt đang ghi, nếu có, đã được lưu).'
        if route not in handlers:
            raise ValueError('Thao tác /circle không hợp lệ.')
        try:
            return handlers[route](payload)
        except (ValueError, RuntimeError):
            raise
        except Exception as exc:           # moi loi khac cung tra ve thong bao, khong lam roi may chu
            raise ValueError(f'Lỗi /circle: {exc}') from exc
