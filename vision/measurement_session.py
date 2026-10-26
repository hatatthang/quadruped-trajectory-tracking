"""Local measurement capture and audit logging; contains no robot transport."""
from datetime import datetime
from pathlib import Path
import copy
import csv
import json
import math
import os
import threading
import time
import uuid

import cv2
from measurement_geometry import (estimate_marker_pose, validate_intrinsics,
                                  summarize_position_samples, distance_validation)


def save_json(path, value):
    path = Path(path)
    pending = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.pending')
    try:
        pending.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        os.replace(pending, path)
    finally:
        pending.unlink(missing_ok=True)


def number(value, low, high, label):
    if isinstance(value, bool):
        raise ValueError(label + ' không hợp lệ.')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(label + ' phải là số.') from None
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{label} phải trong khoảng {low}–{high}.')
    return value


class MeasurementService:
    def __init__(self, app, output_dir):
        self.app, self.output_dir = app, Path(output_dir)
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.worker = None
        self.capture = {'busy': False, 'label': None, 'accepted': 0, 'target': 15, 'message': ''}
        self.recording = {'active': False, 'sample_count': 0, 'valid_count': 0,
                          'elapsed_seconds': 0, 'directory': None, 'message': ''}
        self.sets = {}
        self.config = {'black_side_mm': 50.0, 'expected_mm': 200.0}
        self.result = None
        self.last_error = ''
        self.active_directory = None
        self.job_started = None

    def state(self):
        current = self.app.state()
        cam = current['camera']
        with self.lock:
            return {'camera': {'status': cam['status'], 'fresh': cam['status'] == 'live',
                               'actual_size': [cam['width'], cam['height']], 'error': cam['error']},
                    'session_id': self.app.session_id, 'mode': current['mode'], 'calibration': current['calibration'],
                    'capture': copy.deepcopy(self.capture), 'recording': copy.deepcopy(self.recording),
                    'config': copy.deepcopy(self.config),
                    'A': copy.deepcopy(self.sets.get('A', {}).get('summary')),
                    'B': copy.deepcopy(self.sets.get('B', {}).get('summary')),
                    'result': copy.deepcopy(self.result), 'last_error': self.last_error}

    def _raw(self):
        with self.app.lock:
            age = time.monotonic() - self.app.raw_mono
            if self.app.raw is None or self.app.camera_status != 'live' or not 0 <= age <= 1.5:
                return None
            return (self.app.raw.copy(), self.app.frame_no, self.app.raw_mono,
                    self.app.raw_wall, self.app.camera_epoch)

    def _begin(self, kind, payload):
        side = number(payload.get('black_side_mm', 50), 10, 200, 'Cạnh ngoài khung đen (mm)')
        expected = number(payload.get('expected_mm', 200), 1, 10000, 'Khoảng cách chuẩn (mm)')
        label = payload.get('label') if kind == 'capture' else None
        if kind == 'capture' and label not in ('A', 'B'):
            raise ValueError('Chọn vị trí A hoặc B.')
        mode, note = payload.get('trial_mode', 'stationary'), payload.get('note', '')
        if mode not in ('stationary', 'line', 'circle') or not isinstance(note, str) or len(note) > 1000:
            raise ValueError('Thông tin lượt ghi không hợp lệ.')
        if not self.app.action_lock.acquire(blocking=False):
            raise ValueError('Đang xử lý thao tác khác; hãy chờ hoặc dừng lượt ghi.')
        try:
            raw = self._raw()
            if raw is None:
                raise ValueError('Chưa có ảnh camera mới. Cắm Rapoo và bấm Mở camera trước.')
            with self.app.lock:
                candidate = copy.deepcopy(self.app.calibration)
            validate_intrinsics(candidate, (raw[0].shape[1], raw[0].shape[0]))
            with self.lock:
                if label == 'B':
                    if 'A' not in self.sets:
                        raise ValueError('Thu vị trí A trước khi thu B.')
                    if self.sets['A']['epoch'] != raw[4]:
                        raise ValueError('Camera đã mở lại sau A. Cố định camera rồi thu lại cả A và B.')
                    if self.sets['A']['side_mm'] != side:
                        raise ValueError('Cạnh marker khác lượt A; cần thu lại A với số đo đúng.')
                self.cancel_event.clear()
                self.last_error = ''
                if kind == 'capture':
                    self.config = {'black_side_mm': side, 'expected_mm': expected}
                self.job_started = time.monotonic()
                self.capture.update(busy=kind == 'capture', label=label, accepted=0, message='Đang thu; giữ marker đứng yên.' if kind == 'capture' else '')
                self.recording.update(active=kind == 'record', sample_count=0, valid_count=0,
                                      elapsed_seconds=0, directory=None, message='Đang ghi.' if kind == 'record' else '')
                if label == 'A':
                    self.sets.clear()
                    self.result = None
                elif label == 'B':
                    self.sets.pop('B', None)
                    self.result = None
            with self.app.lock:
                self.app.busy = True
                self.app.mode = 'marker'
            context = {'kind': kind, 'label': label, 'black_side_mm': side, 'expected_mm': expected,
                       'trial_mode': mode, 'note': note, 'camera_epoch': raw[4],
                       'camera_name': self.app.name, 'session_id': self.app.session_id,
                       'calibration': candidate, 'coordinate_frame': 'camera', 'units': 'mm',
                       'world_ready': False, 'body_offset_applied': False,
                       'time_origin': 'PC recording start; gait_start must be marked separately',
                       'created_at': datetime.now().isoformat()}
            self.worker = threading.Thread(target=self._run, args=(context,), daemon=True)
            self.worker.start()
        except Exception:
            with self.lock:
                self.capture['busy'] = False
                self.recording['active'] = False
            with self.app.lock:
                self.app.busy = False
            self.app.action_lock.release()
            raise

    def _run(self, context):
        rows, directory, failure = [], None, None
        recording = context['kind'] == 'record'
        target, timeout = 15, 2100 if recording else 15
        previous, last_gap = None, -1.0
        try:
            ident = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
            directory = self.output_dir / (context['kind'] + '_' + ident)
            directory.mkdir(parents=True, exist_ok=False)
            save_json(directory / 'context.json', context)
            with self.lock:
                self.active_directory = directory
                self.recording['directory'] = str(directory) if recording else None
            fields = ['frame_id', 'time_unix', 'elapsed_s', 'detected', 'reason', 'x_camera_mm',
                      'y_camera_mm', 'z_camera_mm', 'reprojection_rms_px', 'ambiguity_warning']
            with (directory / 'records.jsonl').open('x', encoding='utf-8') as data, (directory / 'samples.csv').open('x', encoding='utf-8-sig', newline='') as table:
                writer = csv.DictWriter(table, fieldnames=fields)
                writer.writeheader()
                while not self.cancel_event.is_set() and not self.app.quit.is_set():
                    elapsed = time.monotonic() - self.job_started
                    if elapsed >= timeout:
                        break
                    raw = self._raw()
                    if raw is None:
                        if elapsed - last_gap < 1:
                            self.cancel_event.wait(.05)
                            continue
                        last_gap = elapsed
                        row = {'detected': False, 'reason': 'camera_missing_or_stale', 'frame_id': None,
                               'time_unix': time.time(), 'elapsed_s': elapsed}
                    else:
                        frame, frame_no, stamp, wall, epoch = raw
                        if epoch != context['camera_epoch']:
                            raise ValueError('Camera đã đổi phiên trong lượt ghi; cần bắt đầu lượt mới.')
                        if (epoch, frame_no) == previous or stamp < self.job_started:
                            self.cancel_event.wait(.04)
                            continue
                        previous = (epoch, frame_no)
                        row = estimate_marker_pose(frame, context['calibration'], context['black_side_mm'])
                        row.update(frame_id=f'{epoch}:{frame_no}', frame_no=frame_no,
                                   time_unix=wall, elapsed_s=stamp - self.job_started,
                                   processing_age_s=time.monotonic() - stamp)
                        if row['processing_age_s'] > 1.5:
                            row.update(detected=False, reason='processing_result_stale')
                        if row.get('detected'):
                            rows.append(row)
                            if not recording and len(rows) in (1, 8, target):
                                image_path = directory / f'raw_{len(rows):02d}.png'
                                if not cv2.imwrite(str(image_path), frame):
                                    raise RuntimeError('Không lưu được ảnh gốc kiểm chứng.')
                    data.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
                    data.flush()
                    position = row.get('tvec_mm') if row.get('detected') else None
                    csv_row = {key: row.get(key) for key in fields}
                    for axis, value in zip(('x_camera_mm', 'y_camera_mm', 'z_camera_mm'), position or (None, None, None)):
                        csv_row[axis] = value
                    writer.writerow(csv_row)
                    table.flush()
                    with self.lock:
                        self.capture['accepted'] = len(rows)
                        self.recording['sample_count'] += 1
                        self.recording['valid_count'] = len(rows)
                        self.recording['elapsed_seconds'] = round(elapsed, 2)
                    if not recording and len(rows) >= target and elapsed >= 3:
                        break
                    self.cancel_event.wait(.08)
            if not recording:
                if self.cancel_event.is_set() or self.app.quit.is_set():
                    raise ValueError('Đã hủy thu. Dữ liệu đã ghi vẫn được giữ để kiểm tra.')
                summary = summarize_position_samples(rows)
                summary.update(directory=str(directory), label=context['label'])
                save_json(directory / 'summary.json', summary)
                with self.lock:
                    self.sets[context['label']] = {'rows': rows, 'summary': summary,
                                                  'epoch': context['camera_epoch'], 'side_mm': context['black_side_mm']}
                    if context['label'] == 'B':
                        self.result = distance_validation(self.sets['A']['rows'], rows, context['expected_mm'])
                        save_json(directory / 'distance_result.json', self.result)
                    self.capture['message'] = f'Đã thu {len(rows)} ảnh mới tại {context["label"]}; xem độ dao động và cảnh báo.'
            else:
                save_json(directory / 'summary.json', {'valid_count': len(rows), 'world_ready': False,
                    'coordinate_frame': 'camera', 'body_offset_applied': False,
                    'stop_reason': 'requested' if self.cancel_event.is_set() else 'shutdown' if self.app.quit.is_set() else '35_minute_limit'})
                with self.lock:
                    self.recording['message'] = 'Đã lưu CSV và JSONL. Đây là tọa độ camera, chưa phải quỹ đạo tâm thân.'
        except Exception as exc:
            failure = str(exc)
            with self.lock:
                self.last_error = failure
                self.capture['message'] = failure
                self.recording['message'] = failure
            if directory is not None:
                try:
                    save_json(directory / 'error.json', {'error': failure, 'world_ready': False})
                except OSError:
                    pass
        finally:
            with self.lock:
                self.capture['busy'] = False
                self.recording['active'] = False
                self.active_directory = None
            with self.app.lock:
                self.app.busy = False
            self.app.action_lock.release()

    def cancel(self, wait=False):
        self.cancel_event.set()
        worker = self.worker
        if wait and worker is not None and worker is not threading.current_thread():
            worker.join(timeout=3)

    def action(self, route, payload):
        if route == 'capture':
            self._begin('capture', payload)
            return 'Đang thu vị trí ' + payload['label'] + '.'
        if route == 'record/start':
            self._begin('record', payload)
            return 'Đang ghi dữ liệu camera; chưa đồng bộ thời điểm gait.'
        if route in ('cancel', 'record/stop'):
            self.cancel()
            return 'Đang kết thúc và lưu dữ liệu.'
        if route == 'reset':
            with self.lock:
                if self.capture['busy'] or self.recording['active']:
                    raise ValueError('Dừng lượt đang thu trước khi đặt lại A/B.')
                self.sets.clear()
                self.result, self.last_error = None, ''
                self.capture['message'] = 'Đã đặt lại A/B trên màn hình; các file đã lưu vẫn còn.'
            return self.capture['message']
        if route == 'record/event':
            event, note = payload.get('event'), payload.get('note', '')
            if event not in ('gait_start', 'intervention', 'note') or not isinstance(note, str) or len(note) > 1000:
                raise ValueError('Sự kiện không hợp lệ.')
            with self.lock:
                if not self.recording['active'] or self.active_directory is None:
                    raise ValueError('Chưa có lượt ghi đang chạy.')
                row = {'event': event, 'note': note, 'time_unix': time.time(),
                       'elapsed_s': time.monotonic() - self.job_started, 'source': 'manual_PC_button',
                       'synchronization_uncertainty_s': None}
                with (self.active_directory / 'events.jsonl').open('a', encoding='utf-8') as f:
                    f.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            return 'Đã đánh dấu sự kiện bằng thời gian máy tính.'
        raise ValueError('Thao tác đo không hợp lệ.')
