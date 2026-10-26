"""Fixed-plane line observer. No serial connection or robot commands.

Floor clicks define the physical line; a stationary, manually aligned robot
defines the mounting yaw. Body offset is optional and explicitly confirmed.
"""
import copy
import csv
import json
import math
import threading
import time
import uuid
from collections import deque
from datetime import datetime
from pathlib import Path

from measurement_geometry import np, validate_intrinsics
from measurement_session import number, save_json
from plane_geometry import map_marker, _intersect, _validate_plane, _IDENTITY_KEYS, _hash


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_in_line(xy, marker_yaw, setup):
    angle = setup['line_angle_rad']
    theta = wrap(marker_yaw + setup['mount_yaw_rad'])
    dx, dy = setup['offset_body_mm'] if setup['offset_confirmed'] else (0., 0.)
    center = [xy[0] + math.cos(theta)*dx - math.sin(theta)*dy,
              xy[1] + math.sin(theta)*dx + math.cos(theta)*dy]
    def transform(p):
        x, y = p[0]-setup['origin_plane_mm'][0], p[1]-setup['origin_plane_mm'][1]
        return [math.cos(angle)*x + math.sin(angle)*y,
                -math.sin(angle)*x + math.cos(angle)*y]
    marker = transform(xy)
    point = transform(center)
    return {'x_mm': point[0], 'y_mm': point[1], 'marker_x_mm': marker[0],
            'marker_y_mm': marker[1], 'heading_deg': math.degrees(wrap(theta-angle)),
            'body_offset_applied': setup['offset_confirmed'],
            'tracked_point': 'body_geometric_center' if setup['offset_confirmed'] else 'marker_center'}


class LineTrackingService:
    def __init__(self, app, root):
        self.app, self.root = app, Path(root)
        self.lock = threading.RLock()
        self.worker = None
        self.stop_event = threading.Event()
        self.setup = None
        self.context = None
        self.recent = deque(maxlen=40)
        self.trail = deque(maxlen=3000)
        self.latest = None
        self.last_error = ''
        self.writer = self.csvfile = self.logfile = None
        self.run = None
        self.last_run = None
        self.resume_path = self.root/'data'/'line_resume_candidate.json'

    def state(self):
        with self.lock:
            latest = copy.deepcopy(self.latest)
            if latest and time.monotonic()-latest['mono'] > 1.5:
                latest = {'detected': False, 'reason': 'Ảnh đo đã cũ.'}
            return {'active': bool(self.worker and self.worker.is_alive()),
                    'setup': copy.deepcopy(self.setup), 'latest': latest,
                    'recording': copy.deepcopy(self.run), 'last_run': copy.deepcopy(self.last_run),
                    'trail': list(self.trail), 'error': self.last_error,
                    'resume_available': self.resume_path.exists()}

    def _camera_context(self):
        raw = self.app.plane_measurement._raw()
        if raw is None:
            raise ValueError('Mở camera và chờ có ảnh mới trước.')
        with self.app.lock:
            candidate = copy.deepcopy(self.app.calibration)
        size = [raw[0].shape[1], raw[0].shape[0]]
        calibration = validate_intrinsics(candidate, size)
        plane = self.app.plane_measurement._current_plane(candidate, size, raw[4])
        if not plane or not plane.get('valid'):
            raise ValueError('Cần mặt phẳng hợp lệ hoặc xác nhận khôi phục khi camera giữ nguyên.')
        _validate_plane(plane, calibration, raw[4])
        return raw, candidate, calibration, plane

    def restore(self, payload):
        if payload.get('camera_unchanged') is not True:
            raise ValueError('Chỉ khôi phục nếu camera, tripod, lấy nét và độ phân giải giữ nguyên.')
        if not self.app.action_lock.acquire(False):
            raise ValueError('Dừng lượt đang thu trước.')
        try:
            raw = self.app.plane_measurement._raw()
            if raw is None:
                raise ValueError('Mở camera trước khi khôi phục.')
            saved = json.loads(self.resume_path.read_text(encoding='utf-8-sig'))
            if saved['session_id'] != self.app.session_id:
                raise ValueError('Mặt phẳng thuộc phiên hiệu chuẩn khác.')
            calibration = validate_intrinsics(self.app.calibration, [raw[0].shape[1], raw[0].shape[0]])
            plane = copy.deepcopy(saved['plane'])
            _validate_plane(plane, calibration, None)
            plane['restored_from_fingerprint'] = plane['plane_fingerprint']
            plane['camera_epoch'] = raw[4]
            plane['plane_fingerprint'] = _hash({k: plane[k] for k in _IDENTITY_KEYS})
            plane['restoration'] = {'time': datetime.now().isoformat(), 'user_confirmed_camera_unchanged': True,
                                    'not_an_independent_revalidation': True}
            with self.app.plane_measurement.lock:
                self.app.plane_measurement.plane = plane
                self.app.plane_measurement.config = saved['config']
                self.app.plane_measurement.sets.clear()
                self.app.plane_measurement.result = None
            self.setup = None
            save_json(self.root/'data'/'line_restored_plane.json', plane)
            return 'Đã khôi phục theo xác nhận giữ nguyên camera; dữ liệu A/B cũ vẫn còn.'
        finally:
            self.app.action_lock.release()

    def start(self, payload):
        if not self.app.action_lock.acquire(False):
            raise ValueError('Đang có lượt đo khác; dừng lượt đó trước.')
        try:
            raw, candidate, calibration, plane = self._camera_context()
            height = number(payload.get('marker_height_mm'), 1, 500, 'Độ cao mặt marker')
            self.context = {'calibration': candidate, 'plane': plane, 'camera_epoch': raw[4],
                            'marker_height_mm': height, 'black_side_mm': 50.,
                            'session_id': self.app.session_id, 'image_size': calibration['image_size']}
            with self.lock:
                self.setup = None
                self.recent.clear()
                self.trail.clear()
                self.latest = None
                self.last_error = ''
                self.stop_event.clear()
            with self.app.lock:
                self.app.busy = True
                self.app.mode = 'marker'
            self.worker = threading.Thread(target=self._loop, daemon=True)
            self.worker.start()
            return 'Đang xem vị trí marker. Chọn hai mốc trên băng keo rồi xác nhận hướng thân.'
        except Exception:
            self.app.action_lock.release()
            raise

    def measure(self, payload):
        # Do thu khoang cach hai moc. Khong ghi gi, khong doi he truc.
        with self.lock:
            if not self.worker or not self.worker.is_alive():
                raise ValueError('Bật xem vị trí trước khi đo thử khoảng cách.')
            _, _, calibration, plane = self._camera_context()
            pts = np.asarray(payload.get('points_px'), dtype=float)
            w, h = self.context['image_size']
            if pts.shape != (2, 2) or not np.isfinite(pts).all() or (pts < 0).any() or (pts[:, 0] >= w).any() or (pts[:, 1] >= h).any():
                raise ValueError('Chọn đủ mốc 0 và 100 cm trong ảnh.')
            floor, _ = _intersect(pts, calibration, plane, 0.)
            delta = floor[1]-floor[0]
            length = float(np.linalg.norm(delta))
            angle = math.degrees(math.atan2(float(delta[1]), float(delta[0])))
            return (f'Hai mốc cách {length:.1f} mm · lệch {length-1000:+.1f} mm so với 1000 mm · '
                    f'góc {angle:.2f}°. Chưa lưu gì.')

    def configure(self, payload):
        with self.lock:
            if not self.worker or not self.worker.is_alive() or self.run:
                raise ValueError('Bật xem vị trí; dừng ghi trước khi thay đổi hệ trục.')
            if payload.get('heading_aligned') is not True:
                raise ValueError('Căn đầu robot song song băng keo, hướng 0 → 100 cm rồi xác nhận.')
            _, _, calibration, plane = self._camera_context()
            if plane['plane_fingerprint'] != self.context['plane']['plane_fingerprint']:
                raise ValueError('Mặt phẳng đã thay đổi. Bật xem lại.')
            pts = np.asarray(payload.get('points_px'), dtype=float)
            w, h = self.context['image_size']
            if pts.shape != (2, 2) or not np.isfinite(pts).all() or (pts < 0).any() or (pts[:, 0] >= w).any() or (pts[:, 1] >= h).any():
                raise ValueError('Chọn đủ mốc 0 và 100 cm trong ảnh.')
            floor, _ = _intersect(pts, calibration, plane, 0.)
            delta = floor[1]-floor[0]
            length = float(np.linalg.norm(delta))
            if not 700 <= length <= 1300:
                raise ValueError(f'Hai mốc cách khoảng {length:.0f} mm. Hãy chọn đúng mốc 0 và 100 cm trên sàn.')
            now = time.monotonic()
            rows = [r for r in self.recent if now-r['mono'] < 4 and r.get('detected')]
            if len(rows) < 10 or now-rows[-1]['mono'] > .6:
                raise ValueError('Cần marker đứng yên, nhìn rõ khoảng 3–4 giây.')
            xy = np.array([r['position_plane_mm'] for r in rows])
            p95 = float(np.percentile(np.linalg.norm(xy-np.median(xy, axis=0), axis=1), 95))
            yaws = [r['yaw_rad'] for r in rows]
            yaw = math.atan2(sum(map(math.sin, yaws)), sum(map(math.cos, yaws)))
            spread = max(abs(wrap(y-yaw)) for y in yaws)
            if p95 > 2 or spread > math.radians(2):
                raise ValueError('Robot hoặc góc đo chưa ổn định. Giữ đứng yên rồi xác nhận lại.')
            angle = math.atan2(float(delta[1]), float(delta[0]))
            offset = [number(payload.get('offset_x_mm', 82), -300, 300, 'Offset X'),
                      number(payload.get('offset_y_mm', 1), -300, 300, 'Offset Y')]
            self.setup = {'line_angle_rad': angle, 'origin_plane_mm': floor[0].tolist(),
                          'end_plane_mm': floor[1].tolist(), 'points_px': pts.tolist(),
                          'clicked_line_length_mm': length, 'mount_yaw_rad': wrap(angle-yaw),
                          'offset_body_mm': offset, 'offset_confirmed': payload.get('offset_confirmed') is True,
                          'mount_yaw_method': 'manual_body_parallel_to_floor_line',
                          'alignment_position_p95_mm': p95, 'alignment_samples': len(rows),
                          'marker_height_mm': self.context['marker_height_mm'],
                          'plane_fingerprint': plane['plane_fingerprint'],
                          'created_at': datetime.now().isoformat(), 'accuracy_validated': False}
            self.trail.clear()
            save_json(self.root/'data'/'line_setup_latest.json', self.setup)
            return 'Đã đặt hệ trục. Y dương là bên trái khi nhìn từ 0 về 100 cm. Có thể ghi lượt đứng yên.'

    def _finish(self, reason):
        if self.run is None:
            return
        run = self.run
        run['stop_reason'] = reason
        run['duration_s'] = time.monotonic()-run['started_mono']
        run['finished_at'] = datetime.now().isoformat()
        run['processed_detection_fraction'] = run['detected']/run['samples'] if run['samples'] else None
        try:
            self.logfile.close()
            self.csvfile.close()
            save_json(Path(run['directory'])/'summary.json', run)
        finally:
            self.last_run = copy.deepcopy(run)
            self.run = self.writer = self.logfile = self.csvfile = None

    def record(self, payload):
        with self.lock:
            if not self.setup or not self.worker or not self.worker.is_alive():
                raise ValueError('Xác lập hệ trục và hướng thân trước.')
            if self.run:
                raise ValueError('Đang ghi một lượt.')
            if not self.latest or not self.latest.get('detected') or time.monotonic()-self.latest['mono'] > .6:
                raise ValueError('Chưa có vị trí marker mới hợp lệ.')
            ident = datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
            directory = self.root/'line_runs'/ident
            directory.mkdir(parents=True, exist_ok=False)
            metadata = {'schema_version': 1, 'context': self.context, 'setup': self.setup,
                        'note': str(payload.get('note', ''))[:500],
                        'limitations': ['Fixed marker height and parallel plane assumed.',
                                        'Manual heading alignment; no independent yaw validation.',
                                        'Only processed frames recorded, not every camera frame.',
                                        'Observation only. No feedback commands.'],
                        'record_start_is_not_gait_start': True}
            save_json(directory/'context.json', metadata)
            self.logfile = (directory/'records.jsonl').open('x', encoding='utf-8')
            self.csvfile = (directory/'samples.csv').open('x', encoding='utf-8-sig', newline='')
            self.writer = csv.DictWriter(self.csvfile, fieldnames=['elapsed_s', 'time_unix', 'frame_id',
                'detected', 'reason_code', 'x_mm', 'y_mm', 'heading_deg', 'marker_x_mm', 'marker_y_mm',
                'body_offset_applied', 'tracked_point', 'processing_age_s'])
            self.writer.writeheader()
            self.run = {'directory': str(directory), 'started_mono': time.monotonic(),
                        'started_at': datetime.now().isoformat(), 'samples': 0, 'detected': 0,
                        'events': [], 'accuracy_validated': False}
            self.trail.clear()
            return 'Đang ghi dữ liệu. Nút ghi không khởi động robot.'

    def _loop(self):
        previous = None
        try:
            while not self.stop_event.is_set() and not self.app.quit.is_set():
                raw = self.app.plane_measurement._raw()
                now = time.monotonic()
                row = {'detected': False, 'reason_code': 'camera_missing_or_stale',
                       'reason': 'Không có ảnh camera mới.', 'mono': now, 'time_unix': time.time()}
                if raw is not None:
                    frame, frame_no, stamp, wall, epoch = raw
                    if epoch != self.context['camera_epoch']:
                        raise ValueError('Camera đã mở lại. Dừng đo và xác lập lại hệ trục.')
                    if previous == (epoch, frame_no):
                        self.stop_event.wait(.03)
                        continue
                    previous = (epoch, frame_no)
                    row = map_marker(frame, self.context['calibration'], self.context['plane'],
                                     self.context['marker_height_mm'], camera_epoch=epoch)
                    row.update(mono=stamp, time_unix=wall, frame_id=f'{epoch}:{frame_no}',
                               processing_age_s=time.monotonic()-stamp)
                    if row['processing_age_s'] > .6:
                        row.update(detected=False, reason_code='processing_stale', reason='Ảnh đo đã quá cũ.')
                with self.lock:
                    self.recent.append(row)
                    if row.get('detected') and self.setup:
                        row.update(pose_in_line(row['position_plane_mm'], row['yaw_rad'], self.setup))
                        self.trail.append([row['x_mm'], row['y_mm']])
                    else:
                        self.trail.append(None)
                    self.latest = row
                    if self.run:
                        entry = {**row, 'elapsed_s': row['mono']-self.run['started_mono']}
                        # Frames acquired before the button click are not part of the run.
                        if entry['elapsed_s'] >= 0:
                            self.logfile.write(json.dumps(entry, ensure_ascii=False, allow_nan=False)+'\n')
                            self.logfile.flush()
                            self.writer.writerow({k: entry.get(k) for k in self.writer.fieldnames})
                            self.csvfile.flush()
                            self.run['samples'] += 1
                            self.run['detected'] += int(bool(row.get('detected')))
                        if time.monotonic()-self.run['started_mono'] > 3600:
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

    def cancel(self):
        self.stop_event.set()
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=5)

    def action(self, route, payload):
        if route == 'restore':
            return self.restore(payload)
        if route == 'preview':
            return self.start(payload)
        if route == 'measure':
            return self.measure(payload)
        if route == 'configure':
            return self.configure(payload)
        if route == 'record':
            return self.record(payload)
        if route == 'stop':
            with self.lock:
                self._finish('user_stopped_recording')
            return 'Đã lưu lượt ghi. Camera vẫn xem vị trí.'
        if route == 'event':
            with self.lock:
                if not self.run:
                    raise ValueError('Bắt đầu ghi trước khi đánh dấu.')
                event = {'elapsed_s': time.monotonic()-self.run['started_mono'],
                         'name': str(payload.get('name', 'manual_event'))[:100],
                         'source': 'manual_button_not_firmware_timestamp'}
                self.run['events'].append(event)
                save_json(Path(self.run['directory'])/'events.json', self.run['events'])
            return 'Đã đánh dấu thời điểm thủ công.'
        if route == 'cancel':
            self.cancel()
            return 'Đã dừng xem vị trí và lưu lượt đang ghi.'
        raise ValueError('Thao tác line không hợp lệ.')
