"""Offline-capable marker geometry. Camera coordinates only; never world-ready.

Input frames are full-resolution, original BGR/gray uint8 images. Callers attach
frame_no (or frame_id) to each returned estimate before accumulating positions.
OpenCV IPPE_SQUARE: object corners TL, TR, BR, BL are (-h,+h), (+h,+h),
(+h,-h), (-h,-h). Camera axes are X right, Y down, Z forward, in mm.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

WORKSPACE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORKSPACE/'tmp'/'pdfs'/'aruco_deps'))
import cv2
import numpy as np

DICTIONARY = 'DICT_4X4_1000'
COORDINATE_FRAME = 'camera_X_right_Y_down_Z_forward'
MIN_POSITION_FRAMES = 10


def _finite_scalar(value, name, positive=False):
    if isinstance(value, (bool, str)):
        raise ValueError(f'{name} phải là số hữu hạn.')
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} phải là số hữu hạn.') from None
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{name} phải là số hữu hạn' + (' lớn hơn 0.' if positive else '.'))
    return value


def validate_intrinsics(candidate, size):
    """Return JSON-safe normalized metadata, or raise ValueError; never rescale K."""
    if not isinstance(candidate, dict):
        raise ValueError('Chưa có dữ liệu nội tại camera.')
    if candidate.get('status') not in ('candidate', 'review_required'):
        raise ValueError('Nội tại camera chưa được tính xong.')
    try:
        actual = list(size)
        expected = list(candidate['image_size'])
        K = np.asarray(candidate['camera_matrix'], dtype=float)
        dist = np.asarray(candidate['distortion_coefficients'], dtype=float)
    except (KeyError, TypeError, ValueError):
        raise ValueError('Cấu trúc nội tại camera không hợp lệ.') from None
    if len(actual) != 2 or len(expected) != 2:
        raise ValueError('Kích thước ảnh phải là [rộng, cao].')
    if any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v <= 0 for v in actual+expected):
        raise ValueError('Kích thước ảnh phải gồm hai số nguyên dương.')
    if actual != expected:
        raise ValueError('Độ phân giải ảnh khác bản hiệu chuẩn; không được tự co giãn nội tại.')
    if K.shape != (3, 3) or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError('Ma trận camera không hợp lệ.')
    if not np.allclose(K[2], [0, 0, 1], atol=1e-9, rtol=0) or abs(K[0,1]) > 1e-9 or abs(K[1,0]) > 1e-9:
        raise ValueError('Ma trận camera phải đúng dạng nội tại pinhole của OpenCV.')
    if dist.ndim not in (1, 2) or (dist.ndim == 2 and min(dist.shape) != 1):
        raise ValueError('Hệ số méo phải là một vector.')
    dist = dist.flatten()
    if len(dist) not in (4, 5, 8, 12, 14) or not np.isfinite(dist).all():
        raise ValueError('Hệ số méo không hợp lệ.')
    rms = _finite_scalar(candidate.get('rms_px'), 'RMS hiệu chuẩn')
    if rms < 0:
        raise ValueError('RMS hiệu chuẩn không được âm.')
    identity = {'camera_matrix': K.tolist(), 'distortion_coefficients': dist.tolist(),
                'image_size': [int(v) for v in expected]}
    fingerprint = hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    return {**identity, 'calibration_fingerprint': fingerprint, 'calibration_rms_px': rms,
            'calibration_status': candidate['status'], 'calibration_session_id': candidate.get('session_id'),
            'camera_name': candidate.get('camera_name'),
            'calibration_warnings': [str(w) for w in candidate.get('warnings', [])],
            'world_ready': False, 'measurement_validation': 'not_performed'}


def estimate_marker_pose(frame, candidate, black_side_mm=50, target_id=900):
    """Detect a unique target and choose the best positive-depth IPPE pose.

    Planar alternatives remain exposed. `detected` means a usable pose was
    computed, not that its metric accuracy or its planar orientation is proven.
    """
    result = {'detected': False, 'reason': '', 'reason_code': '', 'ids': [], 'target_count': 0,
              'center_px': None, 'corners_px': None, 'tvec_mm': None, 'rvec': None,
              'rotation_matrix': None, 'reprojection_rms_px': None, 'ambiguity_warning': False,
              'dictionary': DICTIONARY, 'coordinate_frame': COORDINATE_FRAME,
              'position_units': 'mm', 'world_ready': False, 'measurement_validation': 'not_performed'}
    def reject(code, message):
        result.update(reason_code=code, reason=message)
        return result
    try:
        side = _finite_scalar(black_side_mm, 'Cạnh vuông đen', positive=True)
        if isinstance(target_id, bool) or not isinstance(target_id, (int, np.integer)) or not 0 <= target_id < 1000:
            raise ValueError('ID phải là số nguyên từ 0 đến 999 trong DICT_4X4_1000.')
        result.update(black_side_mm=side, target_id=int(target_id))
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim not in (2, 3):
            raise ValueError('Cần ảnh gốc uint8, dạng xám hoặc BGR/BGRA.')
        if frame.shape[0] == 0 or frame.shape[1] == 0 or (frame.ndim == 3 and frame.shape[2] not in (3, 4)):
            raise ValueError('Kích thước hoặc số kênh ảnh không hợp lệ.')
        calibration = validate_intrinsics(candidate, (frame.shape[1], frame.shape[0]))
        result['calibration'] = calibration
        result['calibration_fingerprint'] = calibration['calibration_fingerprint']
        result['calibration_rms_px'] = calibration['calibration_rms_px']
        result['image_size'] = calibration['image_size']
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY if frame.shape[2] == 4 else cv2.COLOR_BGR2GRAY)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), params)
        corners, ids, _ = detector.detectMarkers(gray)
        values = [] if ids is None else [int(v) for v in ids.flatten()]
        result['ids'] = values
        result['target_count'] = values.count(int(target_id))
        if result['target_count'] == 0:
            return reject('target_not_found', 'Chưa nhận được marker mục tiêu.')
        if result['target_count'] != 1:
            return reject('duplicate_target', 'Có nhiều marker cùng ID; cất các bản dự phòng khỏi vùng camera.')
        pts = np.asarray(corners[values.index(int(target_id))], dtype=np.float64).reshape(-1, 2)
        if pts.shape != (4,2) or not np.isfinite(pts).all():
            return reject('invalid_corners', 'Bốn góc marker không hợp lệ.')
        if np.any(pts < 0) or np.any(pts[:,0] >= frame.shape[1]) or np.any(pts[:,1] >= frame.shape[0]):
            return reject('invalid_corners', 'Góc marker nằm ngoài ảnh gốc.')
        if not cv2.isContourConvex(pts.astype(np.float32)) or cv2.contourArea(pts.astype(np.float32)) < 16:
            return reject('degenerate_corners', 'Marker quá nhỏ hoặc hình học bốn góc không hợp lệ.')
        result['corners_px'] = pts.tolist()
        half = side/2
        obj = np.array([[-half,half,0], [half,half,0], [half,-half,0], [-half,-half,0]], np.float64)
        K = np.asarray(calibration['camera_matrix'], dtype=float)
        dist = np.asarray(calibration['distortion_coefficients'], dtype=float)
        solved = cv2.solvePnPGeneric(obj, pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        poses = []
        for rvec, tvec in zip(solved[1], solved[2]):
            rvec, tvec = np.asarray(rvec, float).reshape(3), np.asarray(tvec, float).reshape(3)
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                continue
            R, _ = cv2.Rodrigues(rvec)
            depths = (obj @ R.T + tvec)[:,2]
            if tvec[2] <= 0 or np.min(depths) <= 0:
                continue
            projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
            rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-pts)**2, axis=1))))
            if not math.isfinite(rms):
                continue
            center, _ = cv2.projectPoints(np.zeros((1,3), float), rvec, tvec, K, dist)
            poses.append({'rvec':rvec.tolist(), 'tvec_mm':tvec.tolist(), 'rotation_matrix':R.tolist(),
                          'reprojection_rms_px':rms, 'center_px':center.reshape(2).tolist()})
        if not poses:
            return reject('no_positive_depth_pose', 'Không có nghiệm tư thế hữu hạn ở phía trước camera.')
        poses.sort(key=lambda p:p['reprojection_rms_px'])
        best = poses[0]
        gap = poses[1]['reprojection_rms_px']-best['reprojection_rms_px'] if len(poses)>1 else None
        separation = float(np.linalg.norm(np.asarray(poses[1]['tvec_mm'])-best['tvec_mm'])) if len(poses)>1 else None
        ambiguity = gap is not None and (gap <= 0.5 or poses[1]['reprojection_rms_px'] <= 1.25*max(best['reprojection_rms_px'],1e-9))
        result.update(best)
        result.update(detected=True, reason='Đã tính tư thế marker trong hệ camera; chưa xác nhận độ chính xác mm.',
                      reason_code='pose_estimated', pose_method='SOLVEPNP_IPPE_SQUARE', corner_refinement='SUBPIX',
                      pose_candidates=poses, ambiguity_warning=bool(ambiguity),
                      ambiguity_rms_gap_px=gap, ambiguity_translation_separation_mm=separation,
                      reprojection_warning=best['reprojection_rms_px']>2.0,
                      planar_ambiguity_possible=True, center_px_definition='projection_of_marker_origin')
        return result
    except (ValueError, TypeError, cv2.error) as exc:
        return reject('invalid_input_or_pose', str(exc))


def summarize_position_samples(records, min_samples=MIN_POSITION_FRAMES, scatter_limit_mm=5.0):
    """Median camera position and scatter, with real-frame deduplication.

    Limits are screening choices, not a universal calibration pass criterion.
    Records lacking frame_id/frame_no are excluded; list indices are not frames.
    """
    if not isinstance(min_samples, int) or isinstance(min_samples, bool) or min_samples < MIN_POSITION_FRAMES:
        raise ValueError('Cần ít nhất 10 khung hình khác nhau cho mỗi vị trí.')
    limit = _finite_scalar(scatter_limit_mm, 'Ngưỡng phân tán', positive=True)
    records = list(records)
    seen, accepted, duplicates, invalid = set(), [], 0, 0
    for record in records:
        if not isinstance(record, dict) or not record.get('detected') or record.get('target_count') != 1:
            invalid += 1
            continue
        frame_id = record.get('frame_id', record.get('frame_no'))
        if isinstance(frame_id, bool) or not isinstance(frame_id, (str,int)) or frame_id == '':
            invalid += 1
            continue
        identity = (type(frame_id).__name__, frame_id)
        if identity in seen:
            duplicates += 1
            continue
        try:
            position = np.asarray(record.get('tvec_mm'), dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all() or position[2] <= 0:
                raise ValueError('invalid translation')
            if record.get('coordinate_frame') != COORDINATE_FRAME or not record.get('calibration_fingerprint'):
                raise ValueError('missing coordinate/calibration identity')
            _finite_scalar(record.get('black_side_mm'), 'Cạnh marker', positive=True)
            _finite_scalar(record.get('calibration_rms_px'), 'RMS hiệu chuẩn')
        except (ValueError, TypeError):
            invalid += 1
            continue
        seen.add(identity)
        accepted.append(record)
    result = {'valid':False, 'quality':'insufficient_frames', 'reason':'Cần ít nhất 10 khung hình hợp lệ khác nhau.',
              'count_received':len(records), 'count_unique_valid':len(accepted), 'duplicate_frames':duplicates,
              'invalid_records':invalid, 'min_samples':min_samples, 'scatter_limit_mm':limit,
              'position_mm':None, 'scatter_rms_mm':None, 'scatter_p95_mm':None, 'scatter_max_mm':None,
              'coordinate_frame':COORDINATE_FRAME, 'world_ready':False, 'measurement_validation':'not_performed'}
    if len(accepted) < min_samples:
        return result
    identities={(r['calibration_fingerprint'],float(r['black_side_mm']),r.get('target_id'),r.get('dictionary')) for r in accepted}
    if len(identities) != 1:
        result.update(quality='mixed_configuration', reason='Không trộn bản nội tại, kích thước hoặc ID marker trong một nhóm đo.')
        return result
    points=np.asarray([r['tvec_mm'] for r in accepted],dtype=float)
    center=np.median(points,axis=0)
    radii=np.linalg.norm(points-center,axis=1)
    ambiguity_count=sum(bool(r.get('ambiguity_warning')) for r in accepted)
    separations=[float(r.get('ambiguity_translation_separation_mm') or 0) for r in accepted if r.get('ambiguity_warning')]
    broad=float(np.percentile(radii,95))>limit
    ambiguous_position=bool(separations and max(separations)>limit)
    poor_reprojection=any(bool(r.get('reprojection_warning')) for r in accepted)
    valid=not (broad or ambiguous_position or poor_reprojection)
    quality='broad_scatter' if broad else 'ambiguous_translation' if ambiguous_position else 'poor_reprojection' if poor_reprojection else 'stable_for_validation'
    result.update(valid=valid, quality=quality,
                  reason='Nhóm vị trí đủ ổn định để tính thử khoảng cách; chưa xác nhận độ chính xác.' if valid else 'Nhóm vị trí chưa ổn định hoặc tư thế chưa đủ rõ để kiểm chứng khoảng cách.',
                  position_mm=center.tolist(), scatter_rms_mm=float(np.sqrt(np.mean(radii*radii))),
                  scatter_p95_mm=float(np.percentile(radii,95)), scatter_max_mm=float(radii.max()),
                  ambiguity_warning=ambiguity_count>0, ambiguity_count=ambiguity_count,
                  max_ambiguous_translation_separation_mm=max(separations,default=0),
                  calibration_fingerprint=accepted[0]['calibration_fingerprint'],
                  calibration_rms_px=float(accepted[0]['calibration_rms_px']),
                  calibration_status=accepted[0].get('calibration',{}).get('calibration_status'),
                  black_side_mm=float(accepted[0]['black_side_mm']), target_id=accepted[0].get('target_id'),
                  dictionary=accepted[0].get('dictionary'), frame_ids=[r.get('frame_id',r.get('frame_no')) for r in accepted])
    return result


def distance_validation(A, B, expected_mm, min_samples=MIN_POSITION_FRAMES, scatter_limit_mm=5.0):
    """Compare two stationary record lists. Camera must remain fixed between A/B."""
    expected = _finite_scalar(expected_mm, 'Khoảng cách chuẩn A–B', positive=True)
    a=summarize_position_samples(A,min_samples,scatter_limit_mm)
    b=summarize_position_samples(B,min_samples,scatter_limit_mm)
    result={'valid':False, 'reason':'Chưa đủ hai nhóm vị trí hợp lệ.', 'A':a, 'B':b,
            'expected_mm':expected, 'measured_mm':None, 'signed_error_mm':None, 'absolute_error_mm':None,
            'signed_error_percent':None, 'absolute_error_percent':None, 'coordinate_frame':COORDINATE_FRAME,
            'world_ready':False, 'measurement_validation':'independent_distance_trial',
            'requires_fixed_camera':True, 'accuracy_pass':None}
    if not a['valid'] or not b['valid']:
        return result
    keys=('calibration_fingerprint','black_side_mm','target_id','dictionary')
    if any(a[k]!=b[k] for k in keys):
        result['reason']='Hai vị trí A/B không dùng cùng bản nội tại và cùng marker.'
        return result
    if set(map(str,a['frame_ids'])) & set(map(str,b['frame_ids'])):
        result['reason']='Nhóm A và B chứa cùng khung hình; cần thu hai vị trí độc lập.'
        return result
    delta=np.asarray(b['position_mm'])-a['position_mm']
    measured=float(np.linalg.norm(delta))
    if measured <= 1e-9:
        result['reason']='Hai nhóm cho cùng vị trí; chưa có phép dịch chuyển A–B hợp lệ.'
        return result
    error=measured-expected
    result.update(valid=True, reason='Đã tính sai lệch của lượt kiểm chứng A–B; cần đánh giá nhiều vị trí trước khi chốt độ chính xác.',
                  measured_mm=measured, delta_camera_mm=delta.tolist(), signed_error_mm=error,
                  absolute_error_mm=abs(error), signed_error_percent=100*error/expected,
                  absolute_error_percent=100*abs(error)/expected,
                  calibration_rms_px=a['calibration_rms_px'], calibration_fingerprint=a['calibration_fingerprint'],
                  ambiguity_warning=bool(a['ambiguity_warning'] or b['ambiguity_warning']))
    return result
