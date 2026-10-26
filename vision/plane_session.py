"""Stationary fixed-plane validation. No robot I/O or moving-run rejection gate."""
from collections import Counter
from datetime import datetime
from pathlib import Path
import copy
import csv
import json
import threading
import time
import uuid

import cv2
from measurement_session import number, save_json
from measurement_geometry import validate_intrinsics
from plane_geometry import (detect_board_corners, fit_plane, map_marker,
                            summarize_plane_samples, plane_distance_validation)


class PlaneMeasurementService:
    def __init__(self, app, output_dir):
        self.app, self.output_dir = app, Path(output_dir)
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.worker = None
        self.plane = None
        self.sets = {}
        self.result = None
        self.config = {'board_height_mm': 1., 'marker_height_mm': 1.,
                       'black_side_mm': 50., 'expected_mm': 200.}
        self.job = {'busy': False, 'kind': None, 'label': None, 'accepted': 0,
                    'attempted': 0, 'failed': 0, 'message': ''}
        self.last_error = ''

    def _raw(self):
        with self.app.lock:
            age = time.monotonic() - self.app.raw_mono
            if self.app.raw is None or self.app.camera_status != 'live' or not 0 <= age <= 1.5:
                return None
            return (self.app.raw.copy(), self.app.frame_no, self.app.raw_mono,
                    self.app.raw_wall, self.app.camera_epoch)

    def _current_plane(self, candidate, size, epoch):
        with self.lock:
            plane = copy.deepcopy(self.plane)
        if plane is None or not plane.get('valid'):
            return plane
        try:
            calibration = validate_intrinsics(candidate, size)
            if (plane['camera_epoch'] != epoch or
                    plane['calibration_fingerprint'] != calibration['calibration_fingerprint']):
                raise ValueError('Camera hoặc nội tại đã đổi. Cần lấy lại mặt phẳng và ghi lại A/B.')
        except (ValueError, TypeError, KeyError) as exc:
            plane.update(valid=False, reason=str(exc), reason_code='stale_plane')
        return plane

    def state(self):
        current = self.app.state()
        cam = current['camera']
        size = [cam['width'], cam['height']]
        with self.app.lock:
            epoch = self.app.camera_epoch
        plane = self._current_plane(current['calibration'], size, epoch)
        with self.lock:
            return {'camera': {'status': cam['status'], 'fresh': cam['status'] == 'live',
                               'actual_size': size, 'error': cam['error']},
                    'session_id': self.app.session_id, 'mode': current['mode'],
                    'calibration': current['calibration'], 'busy': current['busy'],
                    'job': copy.deepcopy(self.job), 'plane': plane,
                    'config': copy.deepcopy(self.config),
                    'A': copy.deepcopy(self.sets.get('A', {}).get('summary')),
                    'B': copy.deepcopy(self.sets.get('B', {}).get('summary')),
                    'result': copy.deepcopy(self.result), 'last_error': self.last_error}

    def _begin(self, kind, payload):
        if kind == 'plane':
            # No default: paper on backing cannot silently be treated as floor.
            settings = {'board_height_mm': number(payload.get('board_height_mm'), 0, 100,
                                                  'Độ cao mặt in bàn cờ so với sàn (mm)')}
            label = None
        else:
            label = payload.get('label')
            if label not in ('A', 'B'):
                raise ValueError('Chọn vị trí A hoặc B.')
            settings = {'marker_height_mm': number(payload.get('marker_height_mm'), 0, 1000,
                                                   'Độ cao mặt marker so với sàn (mm)'),
                        'black_side_mm': number(payload.get('black_side_mm', 50), 10, 200,
                                                'Cạnh ngoài khung đen (mm)'),
                        'expected_mm': number(payload.get('expected_mm', 200), 1, 10000,
                                              'Khoảng cách chuẩn (mm)')}
        if not self.app.action_lock.acquire(blocking=False):
            raise ValueError('Đang xử lý thao tác khác. Chờ hoặc dừng lượt đang ghi.')
        try:
            raw = self._raw()
            if raw is None:
                raise ValueError('Chưa có ảnh camera mới. Bấm Mở camera trước.')
            with self.app.lock:
                candidate = copy.deepcopy(self.app.calibration)
            validate_intrinsics(candidate, [raw[0].shape[1], raw[0].shape[0]])
            plane = self._current_plane(candidate, [raw[0].shape[1], raw[0].shape[0]], raw[4])
            if kind != 'plane' and (plane is None or not plane.get('valid')):
                raise ValueError((plane or {}).get('reason', 'Lấy mặt phẳng bàn cờ trước khi ghi A/B.'))
            with self.lock:
                if kind == 'capture' and self.sets:
                    # A recapture can replace a pair, but cannot silently change its physical settings.
                    if any(settings[k] != self.config[k] for k in settings):
                        raise ValueError('Đặt lại A/B trước khi thay đổi kích thước, độ cao hoặc khoảng cách.')
                if label == 'B':
                    a = self.sets.get('A', {}).get('summary', {})
                    if not a.get('valid'):
                        raise ValueError('Cần nhóm A đứng yên đủ ổn định trước khi thu B.')
                    if a.get('plane_fingerprint') != plane['plane_fingerprint']:
                        raise ValueError('Mặt phẳng đã đổi sau A. Cần ghi lại A.')
                self.cancel_event.clear()
                self.last_error = ''
                self.config.update(settings)
                if kind == 'plane':
                    self.plane = None
                    self.sets.clear()
                    self.result = None
                elif label == 'A':
                    self.sets.clear()
                    self.result = None
                else:
                    self.sets.pop('B', None)
                    self.result = None
                self.job = {'busy': True, 'kind': kind, 'label': label, 'accepted': 0,
                            'attempted': 0, 'failed': 0,
                            'message': 'Đang lấy bàn cờ đứng yên.' if kind == 'plane' else 'Đang ghi marker đứng yên tại ' + label + '.'}
            with self.app.lock:
                self.app.busy = True
                self.app.mode = 'calibration' if kind == 'plane' else 'marker'
            context = {'schema_version': 1, 'kind': kind, 'label': label, **settings,
                       'camera_epoch': raw[4], 'camera_name': self.app.name,
                       'session_id': self.app.session_id, 'calibration': candidate, 'plane': plane if kind != 'plane' else None,
                       'stationary_trial_only': True, 'world_ready': False,
                       'body_offset_applied': False, 'created_at': datetime.now().isoformat(),
                       'image_save_policy': 'accepted samples 1/8/15; first 3 failures then at most 1 per second, maximum 15 failure PNGs',
                       'processed_samples_are_not_all_camera_frames': True}
            started = time.monotonic()
            self.worker = threading.Thread(target=self._run, args=(context, started), daemon=True)
            self.worker.start()
        except Exception:
            with self.lock:
                self.job['busy'] = False
            with self.app.lock:
                self.app.busy = False
            self.app.action_lock.release()
            raise

    @staticmethod
    def _stats(records):
        frames = [r for r in records if r.get('frame_id') is not None]
        valid = [r for r in frames if r.get('detected')]
        failures = [r for r in frames if not r.get('detected')]
        stamps = [r['elapsed_s'] for r in valid]
        return {'attempted': len(frames), 'detected': len(valid), 'missed': len(failures),
                'stale_gap_records': sum(r.get('frame_id') is None for r in records),
                'reason_counts': dict(Counter(r.get('reason_code', 'unknown') for r in failures)),
                'time_to_first_valid_s': stamps[0] if stamps else None,
                'max_valid_gap_s': max((b-a for a, b in zip(stamps, stamps[1:])), default=None),
                'processed_sample_detection_fraction': len(valid)/len(frames) if frames else None,
                'live_frame_uptime_not_measured': True}

    def _run(self, context, started):
        records, accepted, directory = [], [], None
        previous, gap_at, failure_at, failure_images = None, -1., -1., 0
        is_plane = context['kind'] == 'plane'
        deadline = 35 if is_plane else 15
        try:
            ident = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
            directory = self.output_dir / (('plane_' if is_plane else 'capture_') + ident)
            directory.mkdir(parents=True, exist_ok=False)
            save_json(directory/'context.json', context)
            fields = ['frame_id', 'time_unix', 'elapsed_s', 'detected', 'reason_code', 'reason',
                      'x_plane_mm', 'y_plane_mm', 'yaw_rad', 'projected_mean_edge_px',
                      'mean_measured_edge_mm', 'extrapolation_warning', 'processing_age_s', 'raw_image']
            with (directory/'records.jsonl').open('x', encoding='utf-8') as log, (directory/'samples.csv').open('x', encoding='utf-8-sig', newline='') as table:
                writer = csv.DictWriter(table, fieldnames=fields)
                writer.writeheader()
                while not self.cancel_event.is_set() and not self.app.quit.is_set():
                    elapsed = time.monotonic()-started
                    if elapsed >= deadline:
                        break
                    raw = self._raw()
                    frame = None
                    if raw is None:
                        if elapsed-gap_at < 1:
                            self.cancel_event.wait(.04)
                            continue
                        gap_at = elapsed
                        row = {'detected': False, 'reason_code': 'camera_missing_or_stale',
                               'reason': 'Không có ảnh camera mới.', 'frame_id': None,
                               'time_unix': time.time(), 'elapsed_s': elapsed}
                    else:
                        frame, frame_no, stamp, wall, epoch = raw
                        if epoch != context['camera_epoch']:
                            raise ValueError('Camera đổi phiên trong lúc thu. Lấy lại mặt phẳng trước.')
                        if (epoch, frame_no) == previous or stamp < started:
                            self.cancel_event.wait(.04)
                            continue
                        previous = (epoch, frame_no)
                        row = detect_board_corners(frame) if is_plane else map_marker(
                            frame, context['calibration'], context['plane'], context['marker_height_mm'],
                            camera_epoch=epoch, black_side_mm=context['black_side_mm'])
                        row.update(frame_id=f'{epoch}:{frame_no}', frame_no=frame_no, camera_epoch=epoch,
                                   time_unix=wall, elapsed_s=stamp-started, processing_age_s=time.monotonic()-stamp)
                        row.setdefault('reason_code', 'board_found' if row.get('detected') else 'board_not_found')
                        if row['processing_age_s'] > 1.5:
                            row.update(detected=False, reason_code='processing_result_stale',
                                       reason='Ảnh đã quá cũ sau xử lý; không dùng vị trí này.')
                        if row.get('detected'):
                            accepted.append(row)
                            if len(accepted) in (1, 8, 15):
                                row['raw_image'] = f'raw_{len(accepted):02d}.png'
                        elif failure_images < 15 and (failure_images < 3 or elapsed-failure_at >= 1):
                            failure_images += 1
                            failure_at = elapsed
                            row['raw_image'] = f'failed_{failure_images:02d}.png'
                        if row.get('raw_image') and not cv2.imwrite(str(directory/row['raw_image']), frame):
                            raise RuntimeError('Không lưu được ảnh kiểm chứng.')
                    records.append(row)
                    log.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
                    log.flush()
                    csv_row = {k: row.get(k) for k in fields}
                    xy = row.get('position_plane_mm') if row.get('detected') else None
                    csv_row.update(x_plane_mm=xy[0] if xy else None, y_plane_mm=xy[1] if xy else None)
                    writer.writerow(csv_row)
                    table.flush()
                    with self.lock:
                        self.job.update(accepted=len(accepted), attempted=len(records), failed=len(records)-len(accepted))
                    if len(accepted) >= 15 and elapsed >= 3:
                        break
                    self.cancel_event.wait(.05)
            if self.cancel_event.is_set() or self.app.quit.is_set():
                raise ValueError('Đã hủy lượt thu. Các tệp đã ghi vẫn được giữ.')
            with self.app.lock:
                if self.app.camera_epoch != context['camera_epoch']:
                    raise ValueError('Camera đã đổi phiên; không dùng nhóm mẫu này.')
                current = validate_intrinsics(self.app.calibration, context['calibration']['image_size'])
            original = validate_intrinsics(context['calibration'], context['calibration']['image_size'])
            if current['calibration_fingerprint'] != original['calibration_fingerprint']:
                raise ValueError('Bản nội tại đã đổi; không dùng nhóm mẫu này.')
            summary = (fit_plane(records, context['calibration'], context['board_height_mm'],
                                 camera_epoch=context['camera_epoch']) if is_plane else summarize_plane_samples(records))
            summary.update(directory=str(directory), label=context['label'], detection_stats=self._stats(records),
                           static_scatter_gate_only=True, failure_images_saved=failure_images)
            with self.lock:
                if self.cancel_event.is_set() or self.app.quit.is_set():
                    raise ValueError('Đã hủy lượt thu trước khi chốt kết quả. Các tệp đã ghi vẫn được giữ.')
                save_json(directory/'summary.json', summary)
                if is_plane:
                    self.plane = summary
                else:
                    self.sets[context['label']] = {'rows': records, 'summary': summary}
                    if context['label'] == 'B':
                        self.result = plane_distance_validation(self.sets['A']['rows'], records, context['expected_mm'])
                        save_json(directory/'distance_result.json', self.result)
                self.job['message'] = summary['reason']
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
                self.job['message'] = str(exc)
            if directory is not None:
                save_json(directory/'error.json', {'error': str(exc), 'world_ready': False})
        finally:
            with self.lock:
                self.job['busy'] = False
            with self.app.lock:
                self.app.busy = False
                if is_plane:
                    self.app.mode = 'marker'
            self.app.action_lock.release()

    def cancel(self, wait=False):
        with self.lock:
            self.cancel_event.set()
            worker = self.worker
        if wait and worker is not None and worker is not threading.current_thread():
            worker.join(timeout=3)

    def action(self, route, payload):
        if route in ('fit', 'capture'):
            self._begin('plane' if route == 'fit' else 'capture', payload)
            return 'Đang lấy mặt phẳng từ bảng đứng yên.' if route == 'fit' else 'Đang thu marker đứng yên tại ' + payload['label'] + '.'
        if route == 'cancel':
            self.cancel()
            return 'Đang hủy và lưu phần dữ liệu đã thu.'
        if route in ('reset_ab', 'reset_plane'):
            if not self.app.action_lock.acquire(blocking=False):
                raise ValueError('Dừng lượt đang thu trước khi đặt lại.')
            try:
                with self.lock:
                    self.sets.clear()
                    self.result = None
                    self.last_error = ''
                    if route == 'reset_plane':
                        self.plane = None
                    self.job['message'] = 'Đã đặt lại trên màn hình; các tệp đã lưu vẫn còn.'
            finally:
                self.app.action_lock.release()
            return self.job['message']
        raise ValueError('Thao tác đo mặt phẳng không hợp lệ.')
