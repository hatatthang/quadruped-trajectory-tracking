"""Camera measurement and explicit UART controls on /final."""
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import copy
import ctypes as ct
from ctypes import wintypes as wt
import json
import math
import os
import re
import secrets
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent
sys.path.insert(0, str(WORKSPACE / 'tmp' / 'pdfs' / 'aruco_deps'))
import cv2
import numpy as np
from measurement_session import MeasurementService
from plane_session import PlaneMeasurementService
from line_tracking import LineTrackingService
from final_control import FinalControl
try:  # Trang /circle (bo sung): loi o day chi tat /circle, /line va /final chay binh thuong
    from circle_tracking import CircleTrackingService
    CIRCLE_ERROR = ''
except Exception as exc:
    CircleTrackingService, CIRCLE_ERROR = None, str(exc)
try:  # Quay video cung luot ghi cua /final (bo sung): loi o day chi tat quay video
    from final_video import FinalVideoRecorder
    FINAL_VIDEO_ERROR = ''
except Exception as exc:
    FinalVideoRecorder, FINAL_VIDEO_ERROR = None, str(exc)

PATTERN = (9, 6)
MIN_VIEWS = 15
MAX_AGE = 1.5


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.pending')
    try:
        pending.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        os.replace(pending, path)
    finally:
        pending.unlink(missing_ok=True)


def enumerate_cameras():
    """DirectShow device order, matching CAP_DSHOW; select by friendly name."""
    class GUID(ct.Structure):
        _fields_ = [('d1', wt.DWORD), ('d2', wt.WORD), ('d3', wt.WORD), ('d4', ct.c_ubyte * 8)]
    def guid(value):
        return GUID.from_buffer_copy(uuid.UUID(value).bytes_le)
    def method(ptr, slot, result, *args):
        fn = ct.cast(ptr, ct.POINTER(ct.POINTER(ct.c_void_p))).contents[slot]
        return ct.WINFUNCTYPE(result, ct.c_void_p, *args)(fn)
    ole = ct.OleDLL('ole32')
    ole.CoInitializeEx(None, 2)
    dev, enum = ct.c_void_p(), ct.c_void_p()
    names = []
    try:
        cls = guid('62BE5D10-60EB-11d0-BD3B-00A0C911CE86')
        iid = guid('29840822-5B84-11D0-BD3B-00A0C911CE86')
        category = guid('860BB310-5D01-11d0-BD3B-00A0C911CE86')
        bagid = guid('55272A00-42CB-11CE-8135-00AA004BB851')
        ole.CoCreateInstance(ct.byref(cls), None, 1, ct.byref(iid), ct.byref(dev))
        if not dev.value:
            return []
        hr = method(dev, 3, ct.c_long, ct.POINTER(GUID), ct.POINTER(ct.c_void_p), wt.DWORD)(dev, ct.byref(category), ct.byref(enum), 0)
        if hr != 0:
            return []
        while True:
            mon, fetched = ct.c_void_p(), wt.ULONG()
            hr = method(enum, 3, ct.c_long, wt.ULONG, ct.POINTER(ct.c_void_p), ct.POINTER(wt.ULONG))(enum, 1, ct.byref(mon), ct.byref(fetched))
            if hr != 0:
                break
            bag = ct.c_void_p()
            try:
                hr = method(mon, 9, ct.c_long, ct.c_void_p, ct.c_void_p, ct.POINTER(GUID), ct.POINTER(ct.c_void_p))(mon, None, None, ct.byref(bagid), ct.byref(bag))
                name = 'Unknown camera'
                if hr == 0:
                    value = ct.create_string_buffer(24)
                    hr = method(bag, 3, ct.c_long, wt.LPCWSTR, ct.c_void_p, ct.c_void_p)(bag, 'FriendlyName', value, None)
                    if hr == 0 and ct.cast(value, ct.POINTER(ct.c_ushort))[0] == 8:
                        name = ct.wstring_at(ct.c_void_p.from_buffer(value, 8).value)
                    ct.OleDLL('oleaut32').VariantClear(value)
                names.append(name)
            finally:
                if bag.value:
                    method(bag, 2, wt.ULONG)(bag)
                method(mon, 2, wt.ULONG)(mon)
        return names
    finally:
        if enum.value:
            method(enum, 2, wt.ULONG)(enum)
        if dev.value:
            method(dev, 2, wt.ULONG)(dev)
        ole.CoUninitialize()


def find_board(raw, accurate=False):
    gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY) if raw.ndim == 3 else raw
    if accurate:
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY
        found, corners = cv2.findChessboardCornersSB(gray, PATTERN, flags=flags)
    else:
        scale = min(1.0, 1000.0 / gray.shape[1])
        small = cv2.resize(gray, None, fx=scale, fy=scale) if scale < 1 else gray
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(small, PATTERN, flags=flags)
        if found and corners is not None and corners.shape == (54, 1, 2) and np.isfinite(corners).all():
            corners = (corners / scale).astype(np.float32)
            cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    if not found or corners is None or corners.shape != (54, 1, 2) or not np.isfinite(corners).all():
        return None
    return corners.astype(np.float32)


def find_capture_board(raw):
    """Prefer SB; fall back to the complete, subpixel-refined preview detector."""
    corners = find_board(raw, accurate=True)
    if corners is not None:
        return corners, 'findChessboardCornersSB'
    corners = find_board(raw, accurate=False)
    return corners, 'findChessboardCorners+cornerSubPix' if corners is not None else None


def require_fresh(status, age, shape, expected_size=None):
    if status != 'live' or age > MAX_AGE or age < 0:
        raise ValueError('Ảnh camera không còn mới. Chờ hình trực tiếp ổn định rồi chụp lại.')
    size = (int(shape[1]), int(shape[0]))
    if expected_size is not None and size != tuple(expected_size):
        raise ValueError('Độ phân giải đã thay đổi. Không trộn ảnh khác kích thước trong một bộ hiệu chuẩn.')
    return size


def coverage_bins(samples, size):
    bins = set()
    w, h = size
    for sample in samples:
        for x, y in np.asarray(sample['corners']).reshape(-1, 2):
            bins.add((min(2, max(0, int(3*x/w))), min(2, max(0, int(3*y/h)))))
    return len(bins)


def fit_calibration(samples, size, square_mm):
    if len(samples) < MIN_VIEWS:
        raise ValueError(f'Cần ít nhất {MIN_VIEWS} ảnh khác tư thế; nên thu khoảng 20-25 ảnh.')
    if not math.isfinite(square_mm) or not 5 <= square_mm <= 100:
        raise ValueError('Cạnh ô phải là số đo thực, từ 5 đến 100 mm.')
    if any(tuple(s['image_size']) != tuple(size) for s in samples):
        raise ValueError('Các ảnh hiệu chuẩn phải có cùng độ phân giải.')
    obj = np.zeros((54, 3), np.float32)
    obj[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * square_mm
    images = [np.asarray(s['corners'], np.float32).reshape(54, 1, 2) for s in samples]
    rms, K, dist, rvecs, tvecs, std_intrinsics, _, _ = cv2.calibrateCameraExtended(
        [obj.copy() for _ in images], images, tuple(size), None, None,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    if not np.isfinite(K).all() or not np.isfinite(dist).all() or not np.isfinite(rms) or K[0,0] <= 0 or K[1,1] <= 0:
        raise ValueError('Nghiệm hiệu chuẩn không hợp lệ. Cần thu lại ảnh đa dạng hơn.')
    per_view, tilts, areas = [], [], []
    for sample, corners, rvec, tvec in zip(samples, images, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=2))))
        per_view.append({'id': sample['id'], 'rms_px': err})
        rotation, _ = cv2.Rodrigues(rvec)
        tilts.append(float(np.degrees(np.arccos(np.clip(abs(rotation[2,2]), 0, 1)))))
        areas.append(float(cv2.contourArea(cv2.convexHull(corners))))
    warnings = []
    if rms > 1.0:
        warnings.append('Sai số chiếu lại còn lớn; xem các ảnh có lỗi cao và thu thêm ảnh rõ hơn.')
    if coverage_bins(samples, size) < 6:
        warnings.append('Bảng chưa phủ đủ các vùng ảnh; cần thêm ảnh gần mép và góc khung hình.')
    if max(tilts) < 15:
        warnings.append('Các tư thế còn ít nghiêng; thêm ảnh nghiêng bảng theo hai hướng.')
    if max(areas) / max(min(areas), 1) < 1.5:
        warnings.append('Kích thước bảng trong ảnh quá giống nhau; thêm ảnh ở khoảng cách khác.')
    w, h = size
    if not 0.8 < K[0,0]/K[1,1] < 1.25 or not 0 <= K[0,2] <= w or not 0 <= K[1,2] <= h:
        warnings.append('Thông số camera bất thường; cần kiểm tra lại bố trí bảng và dữ liệu.')
    relative_focal_uncertainty = max(float(std_intrinsics[0,0]/K[0,0]), float(std_intrinsics[1,0]/K[1,1]))
    if not math.isfinite(relative_focal_uncertainty) or relative_focal_uncertainty > 0.05:
        warnings.append('Độ bất định tiêu cự còn cao; cần thêm tư thế khác góc nghiêng.')
    return {'status': 'review_required' if warnings else 'candidate', 'world_ready': False,
            'created_at': datetime.now().isoformat(), 'opencv_version': cv2.__version__,
            'image_size': list(size), 'pattern_inner_corners': list(PATTERN), 'square_mm': square_mm,
            'camera_matrix': K.tolist(), 'distortion_coefficients': dist.flatten().tolist(),
            'std_intrinsics': std_intrinsics.flatten().tolist(), 'rms_px': float(rms),
            'per_view': per_view, 'warnings': warnings,
            'included_ids': [s['id'] for s in samples],
            'measurement_validation': 'not_performed', 'maximum_board_tilt_deg': max(tilts)}


def load_session(data_dir, session_id, camera_name):
    """Validate a saved session completely before adopting it; never write here."""
    if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}', session_id):
        raise ValueError('Mã phiên không hợp lệ; chỉ dùng tên thư mục con trực tiếp trong data.')
    data_dir = Path(data_dir).resolve()
    try:
        session_dir = (data_dir / session_id).resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise ValueError('Không tìm thấy thư mục phiên cần mở lại.') from exc
    if session_dir.parent != data_dir or not session_dir.is_dir():
        raise ValueError('Phiên phải nằm trực tiếp trong thư mục data.')
    try:
        metadata_path = (session_dir / 'session.json').resolve(strict=True)
        if metadata_path.parent != session_dir or not metadata_path.is_file():
            raise ValueError('Tệp session.json phải nằm trong thư mục phiên.')
        def reject_constant(value):
            raise ValueError(f'Giá trị số không hữu hạn trong dữ liệu: {value}.')
        def finite_float(value):
            number = float(value)
            if not math.isfinite(number):
                reject_constant(value)
            return number
        document = json.loads(metadata_path.read_text(encoding='utf-8-sig'), parse_constant=reject_constant,
                              parse_float=finite_float)
    except (OSError, RuntimeError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError('Không đọc được session.json hợp lệ của phiên đã chọn.') from exc
    if not isinstance(document, dict) or document.get('session_id') != session_id:
        raise ValueError('Mã phiên trong session.json không khớp thư mục.')
    saved_camera = document.get('camera')
    if not isinstance(saved_camera, str) or saved_camera.casefold() != camera_name.casefold():
        raise ValueError('Phiên đã lưu thuộc camera khác; không trộn dữ liệu camera.')
    if (document.get('pattern') != list(PATTERN)
            or any(type(n) is not int for n in document['pattern'])):
        raise ValueError('Phiên phải dùng bảng 9 × 6 góc trong.')
    samples = document.get('samples')
    if not isinstance(samples, list):
        raise ValueError('Danh sách ảnh của phiên không hợp lệ.')
    size = document.get('image_size')
    valid_size = (isinstance(size, list) and len(size) == 2
                  and all(type(n) is int and n > 0 for n in size))
    if not valid_size and not (size is None and not samples):
        raise ValueError('Độ phân giải lưu trong phiên không hợp lệ.')
    raw_paths = []
    for index, sample in enumerate(samples, 1):
        if not isinstance(sample, dict) or type(sample.get('id')) is not int or sample['id'] != index:
            raise ValueError('Các mã ảnh phải liên tiếp, duy nhất và đúng thứ tự trong phiên.')
        name = f'view_{index:03d}.png'
        if sample.get('name') != name or sample.get('image_size') != size:
            raise ValueError(f'Tên hoặc độ phân giải của ảnh {index} không hợp lệ.')
        if 'included' in sample and type(sample['included']) is not bool:
            raise ValueError(f'Trạng thái sử dụng ảnh {index} không hợp lệ.')
        frame_no, stamp = sample.get('frame_no'), sample.get('time_unix')
        if type(frame_no) is not int or frame_no < 0 or type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
            raise ValueError(f'Mốc khung hình hoặc thời gian của ảnh {index} không hợp lệ.')
        try:
            corners = np.asarray(sample.get('corners'), dtype=np.float64)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f'Tọa độ góc ảnh {index} không hợp lệ.') from exc
        if corners.shape != (54, 1, 2) or not np.isfinite(corners).all():
            raise ValueError(f'Ảnh {index} phải có đủ 54 tọa độ góc hữu hạn.')
        points = corners[:, 0, :]
        if np.any(points < 0) or np.any(points[:, 0] >= size[0]) or np.any(points[:, 1] >= size[1]):
            raise ValueError(f'Tọa độ góc ảnh {index} nằm ngoài độ phân giải đã lưu.')
        try:
            raw_path = (session_dir / name).resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise ValueError(f'Thiếu ảnh gốc {name}; chưa mở lại phiên.') from exc
        if raw_path.parent != session_dir or not raw_path.is_file():
            raise ValueError(f'Ảnh gốc {name} phải nằm trong thư mục phiên.')
        raw_paths.append(raw_path)
    calibration = document.get('calibration')
    if not isinstance(calibration, dict) or calibration.get('status') not in ('not_calibrated', 'computing', 'candidate', 'review_required'):
        raise ValueError('Trạng thái hiệu chuẩn đã lưu không hợp lệ.')
    if not isinstance(calibration.get('warnings'), list) or any(not isinstance(w, str) for w in calibration['warnings']):
        raise ValueError('Nhận xét hiệu chuẩn đã lưu không hợp lệ.')
    if calibration['status'] in ('candidate', 'review_required'):
        active_ids = [s['id'] for s in samples if s.get('included', True)]
        if len(active_ids) < MIN_VIEWS or calibration.get('image_size') != size or calibration.get('pattern_inner_corners') != list(PATTERN):
            raise ValueError('Kết quả hiệu chuẩn không khớp bộ ảnh đã lưu.')
        try:
            matrix = np.asarray(calibration['camera_matrix'], dtype=np.float64)
            distortion = np.asarray(calibration['distortion_coefficients'], dtype=np.float64)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError('Thiếu hoặc sai thông số nội tại camera đã lưu.') from exc
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0, 0, 1]) or distortion.ndim != 1
                or distortion.size not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all()):
            raise ValueError('Thông số nội tại camera đã lưu không hợp lệ.')
        rms, square = calibration.get('rms_px'), calibration.get('square_mm')
        if (type(rms) not in (int, float) or not math.isfinite(rms) or rms < 0
                or type(square) not in (int, float) or not math.isfinite(square) or not 5 <= square <= 100):
            raise ValueError('Sai số hoặc kích thước ô hiệu chuẩn đã lưu không hợp lệ.')
        if calibration.get('included_ids') != active_ids:
            raise ValueError('Kết quả hiệu chuẩn không dùng đúng các ảnh đang được chọn.')
        per_view = calibration.get('per_view')
        if not isinstance(per_view, list) or len(per_view) != len(active_ids):
            raise ValueError('Thiếu sai số theo ảnh của kết quả hiệu chuẩn.')
        for view, ident in zip(per_view, active_ids):
            error = view.get('rms_px') if isinstance(view, dict) else None
            if (not isinstance(view, dict) or type(view.get('id')) is not int or view['id'] != ident
                    or type(error) not in (int, float) or not math.isfinite(error) or error < 0):
                raise ValueError('Sai số theo ảnh của kết quả hiệu chuẩn không hợp lệ.')
        result_camera = calibration.get('camera_name', saved_camera)
        if (not isinstance(result_camera, str) or result_camera.casefold() != camera_name.casefold()
                or calibration.get('session_id', session_id) != session_id):
            raise ValueError('Kết quả hiệu chuẩn thuộc camera hoặc phiên khác.')
    for raw_path in raw_paths:
        try:
            image_bytes = raw_path.read_bytes()
            if not image_bytes.startswith(b'\x89PNG\r\n\x1a\n'):
                raise ValueError(f'Ảnh gốc {raw_path.name} không phải PNG hợp lệ.')
            raw = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        except (OSError, cv2.error) as exc:
            raise ValueError(f'Không đọc được ảnh gốc {raw_path.name}.') from exc
        if raw is None or raw.dtype != np.uint8 or raw.shape != (size[1], size[0], 3):
            raise ValueError(f'Ảnh gốc {raw_path.name} sai định dạng hoặc độ phân giải.')
    # A restored candidate remains unverified; restoration never establishes a floor frame.
    document = copy.deepcopy(document)
    if calibration['status'] == 'computing':
        document['calibration'] = {'status': 'not_calibrated', 'world_ready': False,
                                   'warnings': ['Lần tính trước bị gián đoạn. Bộ ảnh còn nguyên; có thể tính lại.']}
    else:
        document['calibration']['world_ready'] = False
        if calibration['status'] in ('candidate', 'review_required'):
            document['calibration']['measurement_validation'] = 'not_performed'
    return session_dir, document


class CameraApp:
    def __init__(self, data_dir=ROOT/'data', camera_name='Rapoo Camera', resume_session=None):
        self.data_dir = Path(data_dir)
        restored = None
        if resume_session is None:
            self.data_dir.mkdir(parents=True, exist_ok=True)
            self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + secrets.token_hex(2)
            self.session_dir = self.data_dir / self.session_id
            self.session_dir.mkdir()
        else:
            self.session_dir, restored = load_session(self.data_dir, resume_session, camera_name)
            self.session_id = restored['session_id']
        self.lock = threading.RLock()
        self.action_lock = threading.Lock()
        self.camera_stop = threading.Event()
        self.quit = threading.Event()
        self.camera_thread = None
        self.detector_thread = None
        self.name = camera_name
        self.mode = 'calibration'
        self.camera_status = 'stopped'
        self.camera_epoch = 0
        self.error = ''
        self.message = 'Chuẩn bị bảng ô vuông 25 mm, giữ phẳng và chụp các tư thế khác nhau.'
        self.raw = None
        self.raw_mono = 0.0
        self.raw_wall = 0.0
        self.frame_no = 0
        self.jpeg = b''
        self.jpeg_mono = 0.0
        self.jpeg_no = 0
        self.detection_mono = 0.0
        self.detection_no = 0
        self.detection_mode = None
        self.detection_error = ''
        self.fps = 0.0
        self.board = {'found': False, 'corner_count': 0, 'stability_px': None, 'span_px': None}
        self.markers = {'ids': [], 'target_count': 0, 'unique_target': False}
        self.samples = restored['samples'] if restored is not None else []
        self.size = tuple(restored['image_size']) if restored is not None and restored['image_size'] is not None else None
        # Old frame numbers identify a previous process, not frames from this capture loop.
        self.last_saved_frame = -1
        self.calibration = restored['calibration'] if restored is not None else {'status': 'not_calibrated', 'world_ready': False, 'warnings': []}
        self.busy = False
        self.measurement = MeasurementService(self, ROOT/'measurements')
        self.plane_measurement = PlaneMeasurementService(self, ROOT/'plane_measurements')
        self.line_tracking = LineTrackingService(self, ROOT)
        self.final_control = FinalControl(self.line_tracking, ROOT)
        try:  # /circle co mat phang rieng, khong dung mat phang cua /plane
            self.circle_tracking = CircleTrackingService(self, ROOT) if CircleTrackingService else None
        except Exception as exc:
            self.circle_tracking = None
            globals()['CIRCLE_ERROR'] = str(exc)
        try:  # chi doc anh xem truoc + vet cua line_tracking; khong dieu khien robot
            self.final_video = FinalVideoRecorder(self, self.line_tracking) if FinalVideoRecorder else None
        except Exception as exc:
            self.final_video = None
            globals()['FINAL_VIDEO_ERROR'] = str(exc)
        if restored is not None:
            self.message = f'Đã mở lại phiên với {len(self.active_samples())} ảnh; dữ liệu gốc còn nguyên. Kết quả đo vẫn cần kiểm chứng.'

    def active_samples(self):
        return [s for s in self.samples if s.get('included', True)]

    def persist_session(self):
        write_json(self.session_dir/'session.json', {
            'session_id': self.session_id, 'camera': self.name, 'image_size': self.size,
            'pattern': list(PATTERN), 'samples': self.samples,
            'robot_marker': {'dictionary': 'DICT_4X4_1000', 'id': 900, 'nominal_black_side_mm': 50,
                             'marker_to_body_xy_mm_approx': [82, 1], 'frame': 'body_X_forward_Y_left',
                             'z_offset_mm': None, 'mount_rotation_verified': False},
            'calibration': self.calibration})

    def state(self):
        with self.lock:
            now = time.monotonic()
            age = now-self.raw_mono if self.raw_mono else None
            live = self.camera_status == 'live' and age is not None and 0 <= age <= MAX_AGE
            detection_age = now-self.detection_mono if self.detection_mono else None
            detection_fresh = (live and not self.detection_error and self.detection_mode == self.mode
                               and detection_age is not None and 0 <= detection_age <= MAX_AGE)
            detection_status = ('error' if self.detection_error else 'live' if detection_fresh
                                else 'stale' if live and self.detection_mono else 'pending' if live else 'stopped')
            shape = self.raw.shape if self.raw is not None else None
            active = self.active_samples()
            return {'camera': {'name': self.name, 'status': self.camera_status if live or self.camera_status != 'live' else 'stale',
                               'width': int(shape[1]) if shape is not None else 0, 'height': int(shape[0]) if shape is not None else 0,
                               'fps': self.fps if live else 0, 'age_seconds': age, 'error': self.error},
                    'mode': self.mode, 'board': copy.deepcopy(self.board) if detection_fresh else {'found': False, 'corner_count': 0},
                    'markers': copy.deepcopy(self.markers) if detection_fresh else {'ids': [], 'target_count': 0, 'unique_target': False},
                    'detection': {'status': detection_status, 'age_seconds': detection_age,
                                  'frame_no': self.detection_no, 'error': self.detection_error},
                    'session': {'id': self.session_id, 'count': len(active), 'min_views': MIN_VIEWS,
                                'images': [{'id': s['id'], 'name': s['name'], 'url': '/api/sample/'+str(s['id'])} for s in active],
                                'coverage_bins': coverage_bins(active, self.size) if active else 0},
                    'calibration': copy.deepcopy(self.calibration), 'busy': self.busy, 'message': self.message}

    def start_camera(self):
        with self.lock:
            if any(worker and worker.is_alive() for worker in (self.camera_thread, self.detector_thread)):
                if self.camera_stop.is_set():
                    raise ValueError('Camera đang đóng thao tác trước. Chờ vài giây rồi mở lại.')
                return
            self.camera_stop.clear()
            self.camera_epoch += 1
            self.camera_status = 'starting'
            self.error = ''
            self.raw = None
            self.raw_mono = 0
            self.jpeg = b''
            self.detection_mono = 0
            self.detection_error = ''
            self.detection_mode = None
            self.camera_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.detector_thread = threading.Thread(target=self._detect_loop, daemon=True)
            self.camera_thread.start()
            self.detector_thread.start()

    def stop_camera(self):
        self.line_tracking.cancel()
        self.measurement.cancel(wait=True)
        self.plane_measurement.cancel(wait=True)
        self.camera_stop.set()
        with self.lock:
            self.camera_status = 'stopped'
            self.board = {'found': False, 'corner_count': 0}
            self.markers = {'ids': [], 'target_count': 0, 'unique_target': False}
            self.jpeg = b''
            self.detection_mono = 0
            self.detection_error = ''
        for worker in (self.camera_thread, self.detector_thread):
            if worker and worker is not threading.current_thread():
                worker.join(timeout=6)
        write_json(self.data_dir/'runtime_status.json', self.state())

    def _capture_loop(self):
        cap = None
        stamps = deque(maxlen=40)
        try:
            names = enumerate_cameras()
            matches = [i for i, n in enumerate(names) if n.casefold() == self.name.casefold()]
            if len(matches) != 1:
                raise RuntimeError(f'Cần đúng một {self.name}; camera hiện có: '+', '.join(names))
            cap = cv2.VideoCapture(matches[0], cv2.CAP_DSHOW)
            if not cap.isOpened():
                raise RuntimeError('Không mở được Rapoo. Kiểm tra USB và ứng dụng khác đang dùng camera.')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            failures = 0
            while not self.camera_stop.is_set() and not self.quit.is_set():
                ok, raw = cap.read()
                if not ok:
                    failures += 1
                    if failures > 15:
                        raise RuntimeError('Camera ngừng trả hình. Kiểm tra dây USB.')
                    self.camera_stop.wait(0.05)
                    continue
                failures = 0
                now = time.monotonic()
                stamps.append(now)
                with self.lock:
                    if self.camera_stop.is_set() or self.quit.is_set():
                        break
                    self.raw = raw
                    self.raw_mono, self.raw_wall = now, time.time()
                    self.frame_no += 1
                    self.camera_status = 'live'
                    self.fps = (len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0
        except Exception as exc:
            with self.lock:
                if not self.camera_stop.is_set() and not self.quit.is_set():
                    self.camera_status = 'error'
                    self.error = str(exc)
                self.jpeg = b''
                self.detection_mono = 0
                self.board = {'found': False, 'corner_count': 0}
                self.markers = {'ids': [], 'target_count': 0, 'unique_target': False}
            self.camera_stop.set()
        finally:
            if cap is not None:
                cap.release()
            write_json(self.data_dir/'runtime_status.json', self.state())

    def _detect_loop(self):
        detector = None
        previous_board = None
        previous_frame = -1
        last_written = 0
        while not self.camera_stop.is_set() and not self.quit.is_set():
            try:
                with self.lock:
                    raw = None if self.raw is None else self.raw.copy()
                    number, mode, stamp = self.frame_no, self.mode, self.raw_mono
                    camera_live = self.camera_status == 'live'
                if not camera_live or raw is None or number == previous_frame or not 0 <= time.monotonic()-stamp <= MAX_AGE:
                    self.camera_stop.wait(0.05)
                    continue
                previous_frame = number
                shown = raw.copy()
                board = {'found': False, 'corner_count': 0, 'stability_px': None, 'span_px': None}
                markers = {'ids': [], 'target_count': 0, 'unique_target': False}
                if mode == 'calibration':
                    corners = find_board(raw)
                    if corners is not None:
                        stable = float(np.sqrt(np.mean((corners-previous_board)**2))) if previous_board is not None else None
                        board = {'found': True, 'corner_count': 54, 'stability_px': stable,
                                 'span_px': float(np.linalg.norm(corners.max(axis=0)-corners.min(axis=0)))}
                        cv2.drawChessboardCorners(shown, PATTERN, corners, True)
                    previous_board = corners
                else:
                    if detector is None:
                        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000))
                    corners, ids, _ = detector.detectMarkers(raw)
                    values = [] if ids is None else ids.flatten().tolist()
                    targets = values.count(900)
                    markers = {'ids': values, 'target_count': targets, 'unique_target': targets == 1}
                    for c, i in zip(corners, values):
                        if i == 900:
                            color = (80, 225, 90) if targets == 1 else (0, 185, 255)
                            cv2.polylines(shown, [c[0].astype('int32')], True, color, 3)
                            x, y = c[0].mean(axis=0).astype(int)
                            cv2.putText(shown, 'ID 900' if targets == 1 else 'ID 900 - MULTIPLE', (max(0,x-55), max(25,y-15)), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2, cv2.LINE_AA)
                small = cv2.resize(shown, (1280, round(shown.shape[0]*1280/shown.shape[1]))) if shown.shape[1] > 1280 else shown
                ok, encoded = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 88])
                if not ok:
                    raise RuntimeError('Không dựng được ảnh xem trực tiếp.')
                with self.lock:
                    if self.camera_stop.is_set() or self.quit.is_set():
                        break
                    if self.mode != mode or self.camera_status != 'live' or time.monotonic()-stamp > MAX_AGE:
                        continue
                    recovered = bool(self.detection_error)
                    self.board, self.markers = board, markers
                    self.detection_mono = stamp
                    self.detection_no = number
                    self.detection_mode = mode
                    self.detection_error = ''
                    if recovered:
                        self.message = 'Nhận ảnh đã hoạt động lại. Có thể tiếp tục đổi tư thế bảng và thu ảnh.'
                    self.jpeg = encoded.tobytes()
                    self.jpeg_no = number
                    self.jpeg_mono = stamp
                now = time.monotonic()
                if now-last_written > 1 and ok:
                    temp = self.data_dir/'latest.pending.jpg'
                    temp.write_bytes(encoded.tobytes())
                    os.replace(temp, self.data_dir/'latest.jpg')
                    write_json(self.data_dir/'runtime_status.json', self.state())
                    last_written = now
                self.camera_stop.wait(0.12)
            except Exception as exc:
                with self.lock:
                    if self.camera_stop.is_set() or self.quit.is_set():
                        break
                    self.detection_error = str(exc)
                    self.detection_mono = 0
                    self.board = {'found': False, 'corner_count': 0}
                    self.markers = {'ids': [], 'target_count': 0, 'unique_target': False}
                    self.jpeg = b''
                    self.message = 'Lỗi nhận ảnh: '+str(exc)+'. Đang thử lại khi có khung hình mới.'
                previous_board = None
                self.camera_stop.wait(0.35)

    def capture_sample(self):
        with self.lock:
            age = time.monotonic()-self.raw_mono if self.raw_mono else float('inf')
            if self.raw is None:
                raise ValueError('Chưa có ảnh camera.')
            size = require_fresh(self.camera_status, age, self.raw.shape, self.size)
            if self.frame_no == self.last_saved_frame:
                raise ValueError('Khung hình này đã được lưu. Di chuyển bảng sang tư thế mới.')
            raw, number, wall = self.raw.copy(), self.frame_no, self.raw_wall
        corners, corner_detector = find_capture_board(raw)
        if corners is None:
            raise ValueError('Chưa nhận đủ 54 góc. Đưa toàn bộ bảng vào hình, giữ phẳng và để yên.')
        pts = corners.reshape(-1, 2)
        if np.min(pts) < 8 or np.any(pts[:,0] > size[0]-8) or np.any(pts[:,1] > size[1]-8):
            raise ValueError('Bảng quá sát mép ảnh. Đưa bảng vào trong một chút.')
        grid = corners.reshape(PATTERN[1], PATTERN[0], 2)
        cell_sizes = np.r_[np.linalg.norm(np.diff(grid, axis=0), axis=2).flatten(), np.linalg.norm(np.diff(grid, axis=1), axis=2).flatten()]
        if np.percentile(cell_sizes, 10) < 10:
            raise ValueError('Bảng còn nhỏ trong ảnh. Đưa bảng gần camera hơn.')
        with self.lock:
            for old in self.active_samples():
                q = np.asarray(old['corners'], np.float32).reshape(-1, 2)
                distance = min(float(np.sqrt(np.mean(np.sum((pts-q)**2, axis=1)))), float(np.sqrt(np.mean(np.sum((pts[::-1]-q)**2, axis=1)))))
                if distance < 0.012*size[0]:
                    raise ValueError('Tư thế quá giống ảnh đã lưu. Dịch bảng, đổi khoảng cách hoặc nghiêng theo hướng khác.')
            index = len(self.samples)+1
            name = f'view_{index:03d}.png'
            if (self.session_dir/name).exists():
                raise ValueError(f'Ảnh {name} đã tồn tại ngoài danh sách phiên. Giữ nguyên ảnh và kiểm tra trước khi chụp tiếp.')
            if not cv2.imwrite(str(self.session_dir/name), raw):
                raise RuntimeError('Không lưu được ảnh gốc.')
            self.samples.append({'id': index, 'name': name, 'included': True, 'frame_no': number,
                                 'time_unix': wall, 'image_size': list(size), 'corners': corners.tolist(),
                                 'corner_detector': corner_detector})
            self.size = size
            self.last_saved_frame = number
            self.calibration = {'status': 'not_calibrated', 'world_ready': False, 'warnings': []}
            self.persist_session()
            self.message = f'Đã lưu ảnh {index}. Đổi vị trí, kích thước hoặc góc nghiêng cho ảnh tiếp theo.'

    def run_calibration(self, square_mm):
        with self.lock:
            samples, size = copy.deepcopy(self.active_samples()), self.size
            self.calibration = {'status': 'computing', 'world_ready': False, 'warnings': []}
        try:
            result = fit_calibration(samples, size, square_mm)
            result['camera_name'] = self.name
            result['session_id'] = self.session_id
            result['excluded_ids'] = [s['id'] for s in self.samples if not s['included']]
            write_json(self.session_dir/'intrinsics_candidate.json', result)
            with self.lock:
                self.calibration = result
                self.message = 'Đã tính nội tại camera. Cần kiểm chứng kích thước và đặt hệ tọa độ vùng chạy trước khi đo robot.'
                self.persist_session()
        except Exception:
            with self.lock:
                self.calibration = {'status': 'not_calibrated', 'world_ready': False, 'warnings': []}
            raise

    def exclude_last(self):
        with self.lock:
            active = self.active_samples()
            if not active:
                raise ValueError('Chưa có ảnh để bỏ khỏi bộ hiệu chuẩn.')
            active[-1]['included'] = False
            self.calibration = {'status': 'not_calibrated', 'world_ready': False, 'warnings': []}
            self.message = 'Đã bỏ ảnh cuối khỏi phép tính; ảnh gốc vẫn được giữ lại.'
            self.persist_session()

    def perform(self, action, payload):
        if action == 'stop':
            self.line_tracking.cancel()
            self.measurement.cancel(wait=True)
            self.plane_measurement.cancel(wait=True)
        if not self.action_lock.acquire(blocking=False):
            raise ValueError('Đang xử lý thao tác trước; vui lòng chờ.')
        with self.lock:
            self.busy = True
        try:
            if action == 'capture':
                self.capture_sample()
            elif action == 'calibrate':
                self.run_calibration(float(payload.get('square_mm', 25)))
            elif action == 'exclude_last':
                self.exclude_last()
            elif action == 'mode':
                if payload.get('mode') not in ('calibration', 'marker'):
                    raise ValueError('Chế độ không hợp lệ.')
                with self.lock:
                    self.mode = payload['mode']
                    self.board = {'found': False, 'corner_count': 0}
                    self.markers = {'ids': [], 'target_count': 0, 'unique_target': False}
                    self.detection_mono = 0
                    self.detection_error = ''
                    self.detection_mode = None
                    self.jpeg = b''
            elif action == 'stop':
                self.stop_camera()
            elif action == 'start':
                self.start_camera()
            else:
                raise ValueError('Thao tác không hợp lệ.')
        finally:
            with self.lock:
                self.busy = False
            self.action_lock.release()


def make_handler(app, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def reply(self, data, mime='application/json; charset=utf-8', code=200):
            if not isinstance(data, bytes):
                data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
        def do_GET(self):
            route = self.path.split('?',1)[0]
            if route == '/final':
                html = (ROOT/'final.html').read_text(encoding='utf-8').replace('__TOKEN__', token)
                return self.reply(html.encode('utf-8'), 'text/html; charset=utf-8')
            if route == '/api/final/state':
                return self.reply(app.final_control.state())
            if route == '/line':
                html = (ROOT/'line.html').read_text(encoding='utf-8').replace('__TOKEN__', token)
                return self.reply(html.encode('utf-8'), 'text/html; charset=utf-8')
            if route == '/api/line/state':
                return self.reply(app.line_tracking.state())
            if route in ('/circle', '/api/circle/state') and app.circle_tracking is None:
                return self.reply({'ok': False, 'error': 'Trang /circle chưa nạp được: ' + CIRCLE_ERROR}, code=503)
            if route == '/circle':
                html = (ROOT/'circle.html').read_text(encoding='utf-8').replace('__TOKEN__', token)
                return self.reply(html.encode('utf-8'), 'text/html; charset=utf-8')
            if route == '/api/circle/state':
                return self.reply(app.circle_tracking.state())
            if route == '/api/finalvideo/state':
                if app.final_video is None:
                    return self.reply({'ok': False, 'error': 'Quay video chưa nạp được: ' + FINAL_VIDEO_ERROR}, code=503)
                return self.reply(app.final_video.state())
            if route == '/':
                html = (ROOT/'viewer.html').read_text(encoding='utf-8').replace('__TOKEN__', token)
                return self.reply(html.encode('utf-8'), 'text/html; charset=utf-8')
            if route == '/measurement':
                html = (ROOT/'measurement.html').read_text(encoding='utf-8').replace('__TOKEN__', token)
                return self.reply(html.encode('utf-8'), 'text/html; charset=utf-8')
            if route == '/api/measurement/state':
                return self.reply(app.measurement.state())
            if route == '/plane':
                html = (ROOT/'plane.html').read_text(encoding='utf-8').replace('__TOKEN__', token)
                return self.reply(html.encode('utf-8'), 'text/html; charset=utf-8')
            if route == '/api/plane/state':
                return self.reply(app.plane_measurement.state())
            if route == '/board.pdf':
                return self.reply((WORKSPACE/'output/pdf/Checkerboard_A4_25mm_9x6.pdf').read_bytes(), 'application/pdf')
            if route == '/api/state':
                return self.reply(app.state())
            if route.startswith('/api/sample/'):
                try:
                    index = int(route.rsplit('/',1)[1])
                    with app.lock:
                        sample = next(s for s in app.samples if s['id'] == index)
                    return self.reply((app.session_dir/sample['name']).read_bytes(), 'image/png')
                except (ValueError, StopIteration, FileNotFoundError):
                    return self.reply({'ok': False}, code=404)
            if route == '/snapshot.jpg':
                with app.lock:
                    data = app.jpeg if app.camera_status == 'live' and time.monotonic()-app.jpeg_mono < MAX_AGE else b''
                return self.reply(data, 'image/jpeg', 200 if data else 503)
            if route != '/stream':
                return self.reply({'ok': False}, code=404)
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            previous = -1
            try:
                while not app.quit.is_set():
                    with app.lock:
                        data, number, fresh = app.jpeg, app.jpeg_no, app.camera_status == 'live' and time.monotonic()-app.jpeg_mono < MAX_AGE
                    if data and fresh and number != previous:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n')
                        self.wfile.flush()
                        previous = number
                    app.quit.wait(.1)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
        def do_POST(self):
            if self.headers.get('X-Session-Token') != token:
                return self.reply({'ok': False, 'message': 'Phiên không hợp lệ. Tải lại trang.'}, code=403)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 4096:
                    raise ValueError('Yêu cầu quá lớn.')
                payload = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(payload, dict):
                    raise ValueError('Yêu cầu không hợp lệ.')
                if self.path.startswith('/api/final/'):
                    message = app.final_control.action(self.path.removeprefix('/api/final/'), payload)
                    return self.reply({'ok': True, 'message': message})
                if self.path.startswith('/api/line/'):
                    message = app.line_tracking.action(self.path.removeprefix('/api/line/'), payload)
                    return self.reply({'ok': True, 'message': message})
                if self.path.startswith('/api/circle/'):
                    if app.circle_tracking is None:
                        raise ValueError('Trang /circle chưa nạp được: ' + CIRCLE_ERROR)
                    message = app.circle_tracking.action(self.path.removeprefix('/api/circle/'), payload)
                    return self.reply({'ok': True, 'message': message})
                if self.path.startswith('/api/finalvideo/'):
                    if app.final_video is None:
                        raise ValueError('Quay video chưa nạp được: ' + FINAL_VIDEO_ERROR)
                    message = app.final_video.action(self.path.removeprefix('/api/finalvideo/'), payload)
                    return self.reply({'ok': True, 'message': message})
                if self.path.startswith('/api/measurement/'):
                    message = app.measurement.action(self.path.removeprefix('/api/measurement/'), payload)
                    return self.reply({'ok': True, 'message': message})
                if self.path.startswith('/api/plane/'):
                    message = app.plane_measurement.action(self.path.removeprefix('/api/plane/'), payload)
                    return self.reply({'ok': True, 'message': message})
                if self.path == '/api/shutdown':
                    app.final_control.close()
                    app.quit.set()
                    app.stop_camera()
                    self.reply({'ok': True, 'message': 'Đã đóng công cụ camera.'})
                    threading.Thread(target=self.server.shutdown, daemon=True).start()
                    return
                action = self.path.removeprefix('/api/')
                app.perform(action, payload)
                self.reply({'ok': True, 'message': app.message})
            except (ValueError, RuntimeError, cv2.error) as exc:
                with app.lock:
                    app.message = str(exc)
                self.reply({'ok': False, 'message': str(exc)}, code=400)
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--resume-session', metavar='SESSION_ID', help='Mở lại thư mục phiên có sẵn trong Camera_Ver0/data, giữ nguyên ảnh.')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    try:
        app = CameraApp(resume_session=args.resume_session)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    token = secrets.token_urlsafe(24)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(app, token))
    write_json(ROOT/'connection.json', {'url': f'http://127.0.0.1:{server.server_port}', 'pid': os.getpid(), 'token': token, 'started_at': datetime.now().isoformat()})
    if not args.no_camera:
        app.start_camera()
    print(f'Camera viewer: http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        app.final_control.close()
        app.quit.set()
        app.stop_camera()
        server.server_close()


if __name__ == '__main__':
    main()
