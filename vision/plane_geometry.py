"""Fixed-plane geometry; no camera I/O and no automatic accuracy approval.

Inputs are original full-resolution frames. Heights are measured from the physical
floor, positive toward the camera. Board origin is the first aligned inner corner;
x follows its first row, y opposes row-down, and +z is the camera-facing normal.
Room/body axes and COM offsets are deliberately not inferred.

Reference: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
"""
import hashlib
import json
import math
import uuid

from measurement_geometry import cv2, np, validate_intrinsics, _finite_scalar, DICTIONARY

PATTERN = (9, 6)
FRAME = 'board_plane_x_first_row_y_opposite_rows_z_toward_camera'
METHOD = 'fixed_plane_ray_intersection'
MIN_FRAMES = 10


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _gray(frame):
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim not in (2, 3):
        raise ValueError('Cần ảnh gốc uint8 dạng xám hoặc BGR/BGRA.')
    if min(frame.shape[:2]) <= 0 or (frame.ndim == 3 and frame.shape[2] not in (3, 4)):
        raise ValueError('Kích thước ảnh hoặc số kênh không hợp lệ.')
    return frame if frame.ndim == 2 else cv2.cvtColor(
        frame, cv2.COLOR_BGRA2GRAY if frame.shape[2] == 4 else cv2.COLOR_BGR2GRAY)


def _identity(value, name):
    if isinstance(value, bool) or not isinstance(value, (str, int)) or value == '':
        raise ValueError(f'{name} phải là chuỗi hoặc số nguyên nhận diện duy nhất.')
    return value


def _height(value, name):
    value = _finite_scalar(value, name)
    if value < 0:
        raise ValueError(f'{name} không được âm so với sàn.')
    return value


def _corners(value, count, size):
    pts = np.asarray(value, dtype=float)
    if pts.shape == (count, 1, 2):
        pts = pts.reshape(count, 2)
    if pts.shape != (count, 2) or not np.isfinite(pts).all():
        raise ValueError(f'Cần đủ {count} góc hữu hạn, không suy đoán góc thiếu.')
    if np.any(pts < 0) or np.any(pts[:, 0] >= size[0]) or np.any(pts[:, 1] >= size[1]):
        raise ValueError('Góc nằm ngoài ảnh gốc.')
    return pts


def detect_board_corners(frame):
    """Return JSON-safe full 54-corner detection. Caller adds frame_id/epoch."""
    result = {'detected': False, 'reason': '', 'corners_px': None, 'detector': None,
              'image_size': None, 'pattern_inner_corners': list(PATTERN)}
    try:
        gray = _gray(frame)
        result['image_size'] = [gray.shape[1], gray.shape[0]]
        found, corners = cv2.findChessboardCornersSB(
            gray, PATTERN, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
        detector = 'findChessboardCornersSB'
        if not found or corners is None:
            found, corners = cv2.findChessboardCorners(
                gray, PATTERN, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            detector = 'findChessboardCorners+cornerSubPix'
            if found and corners is not None:
                cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
        if not found or corners is None:
            result['reason'] = 'Chưa thấy đủ bảng 9×6 góc trong.'
            return result
        pts = _corners(corners, 54, result['image_size'])
        grid = pts.reshape(6, 9, 2)
        cells = np.concatenate((np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
                                np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel()))
        if np.min(cells) < 8:
            raise ValueError('Ô bảng quá nhỏ trên ảnh; cần ít nhất 8 px giữa hai góc kề.')
        result.update(detected=True, reason='Đã thấy đủ 54 góc.', corners_px=pts.tolist(),
                      detector=detector, min_cell_px=float(cells.min()))
    except (ValueError, TypeError, cv2.error) as exc:
        result['reason'] = str(exc)
    return result


def _object_points(square):
    points = np.zeros((54, 3), float)
    points[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * [square, -square]
    return points


def _undistort(pts, calibration):
    K = np.asarray(calibration['camera_matrix'], float)
    dist = np.asarray(calibration['distortion_coefficients'], float)
    # Tight iteration avoids the default five-iteration residual near image edges.
    norm = cv2.undistortPointsIter(np.asarray(pts, float).reshape(-1, 1, 2), K, dist,
                                 None, None, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-12))
    norm = norm.reshape(-1, 2)
    if not np.isfinite(norm).all():
        raise ValueError('Không thể khử méo thành tia nhìn hữu hạn.')
    return norm


def fit_plane(samples, candidate, board_height_mm, square_mm=25, camera_epoch=None,
              min_samples=MIN_FRAMES, motion_limit_px=1.0, reprojection_limit_px=3.0):
    """Fit one stationary board, preserving K/dist. No writes; JSON result.

    Samples: detect_board_corners results + unique frame_id + camera_epoch.
    180-degree checker ordering is aligned to the first usable frame. Neither
    physical board rotation nor camera movement can be inferred from frame IDs.
    """
    result = {'valid': False, 'reason': '', 'reason_code': 'invalid_plane',
              'coordinate_frame': FRAME, 'method': METHOD, 'world_ready': False,
              'measurement_validation': 'not_performed', 'requires_fixed_camera': True}
    try:
        if isinstance(min_samples, bool) or not isinstance(min_samples, int) or min_samples < MIN_FRAMES:
            raise ValueError('Cần ít nhất 10 frame độc lập để xác lập mặt phẳng.')
        height = _height(board_height_mm, 'Độ cao mặt in bảng')
        square = _finite_scalar(square_mm, 'Cạnh ô', positive=True)
        motion_limit = _finite_scalar(motion_limit_px, 'Ngưỡng dịch góc', positive=True)
        reproj_limit = _finite_scalar(reprojection_limit_px, 'Ngưỡng chiếu lại', positive=True)
        calibration = validate_intrinsics(candidate, candidate.get('image_size', []))
        samples = list(samples)
        seen, used, grids, epochs, flipped = set(), [], [], set(), 0
        rejected = duplicates = 0
        for s in samples:
            if not isinstance(s, dict) or not s.get('detected'):
                rejected += 1
                continue
            fid = _identity(s.get('frame_id'), 'frame_id')
            key = (type(fid).__name__, fid)
            if key in seen:
                duplicates += 1
                continue
            if s.get('image_size') != calibration['image_size']:
                raise ValueError('Mẫu bảng không cùng độ phân giải gốc với nội tại.')
            epoch = _identity(s.get('camera_epoch', camera_epoch), 'camera_epoch')
            epochs.add((type(epoch).__name__, epoch))
            if camera_epoch is not None and epoch != camera_epoch:
                raise ValueError('Camera đã đổi phiên trong nhóm mẫu bảng.')
            pts = _corners(s.get('corners_px'), 54, calibration['image_size'])
            if grids and np.mean(np.sum((pts[::-1]-grids[0])**2, axis=1)) < np.mean(np.sum((pts-grids[0])**2, axis=1)):
                pts = pts[::-1].copy()
                flipped += 1
            seen.add(key)
            used.append(s)
            grids.append(pts)
        result.update(count_received=len(samples), count_unique_valid=len(used),
                      duplicate_frames=duplicates, invalid_records=rejected, aligned_180_count=flipped)
        if len(used) < min_samples:
            raise ValueError('Chưa đủ 10 frame bảng hợp lệ khác nhau.')
        if len(epochs) != 1:
            raise ValueError('Không được trộn các phiên camera.')
        epoch = next(iter(epochs))[1]
        allpts = np.asarray(grids)
        pts = np.median(allpts, axis=0)
        deviations = np.linalg.norm(allpts-pts, axis=2)
        motion_p95 = float(np.percentile(deviations, 95))
        frame_p95_max = float(np.max(np.percentile(deviations, 95, axis=1)))
        result.update(corner_motion_p95_px=motion_p95, worst_frame_motion_p95_px=frame_p95_max,
                      corner_motion_max_px=float(deviations.max()), motion_limit_px=motion_limit)
        if frame_p95_max > motion_limit:
            raise ValueError('Bảng hoặc ảnh còn dao động; giữ bảng và camera đứng yên rồi thu lại.')
        grid = pts.reshape(6, 9, 2)
        cells = np.concatenate((np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
                                np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel()))
        if cells.min() < 8:
            raise ValueError('Khoảng cách góc bảng dưới 8 px; bảng quá nhỏ hoặc hình học sai.')
        obj = _object_points(square)
        K, dist = np.asarray(calibration['camera_matrix'], float), np.asarray(calibration['distortion_coefficients'], float)
        rays = _undistort(pts, calibration)
        Hfit, _ = cv2.findHomography(rays, obj[:, :2], method=0)
        if Hfit is None or not np.isfinite(Hfit).all() or np.linalg.cond(Hfit) > 1e10:
            raise ValueError('Homography suy biến; cần bảng nhìn rõ diện tích hai chiều.')
        hompoints = cv2.perspectiveTransform(rays.reshape(-1, 1, 2), Hfit).reshape(-1, 2)
        fit_rms_mm = float(np.sqrt(np.mean(np.sum((hompoints-obj[:, :2])**2, axis=1))))
        solved = cv2.solvePnPGeneric(obj, pts, K, dist, flags=cv2.SOLVEPNP_IPPE)
        poses = []
        for rv, tv in zip(solved[1], solved[2]):
            rv, tv = cv2.solvePnPRefineLM(obj, pts, K, dist, rv.copy(), tv.copy())
            R, _ = cv2.Rodrigues(rv)
            t = tv.reshape(3)
            if not np.isfinite(R).all() or not np.isfinite(t).all():
                continue
            if np.min((obj @ R.T+t)[:, 2]) <= 0 or float(R[:, 2] @ t) >= -1e-6:
                continue
            projected, _ = cv2.projectPoints(obj, rv, tv, K, dist)
            rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-pts)**2, axis=1))))
            poses.append((rms, R, t))
        if not poses:
            raise ValueError('Không có nghiệm mặt phẳng hữu hạn hướng về camera.')
        poses.sort(key=lambda p: p[0])
        rms, R, t = poses[0]
        result.update(reprojection_rms_px=rms, reprojection_limit_px=reproj_limit,
                      unconstrained_homography_rms_mm=fit_rms_mm)
        if not math.isfinite(rms) or rms > reproj_limit:
            raise ValueError('Sai số chiếu bảng quá lớn; kiểm tra độ phẳng, góc nhận và nội tại.')
        H = np.linalg.inv(np.column_stack((R[:, 0], R[:, 1], t)))
        H /= np.max(np.abs(H))
        if not np.isfinite(H).all() or np.linalg.cond(H) > 1e10:
            raise ValueError('Mặt phẳng quá gần song song tia nhìn hoặc suy biến.')
        angle = gap = None
        if len(poses) > 1:
            angle = float(np.degrees(np.arccos(np.clip(R[:, 2] @ poses[1][1][:, 2], -1, 1))))
            gap = float(poses[1][0]-rms)
        identity = {'schema_version': 1, 'plane_id': uuid.uuid4().hex,
                    'calibration_fingerprint': calibration['calibration_fingerprint'],
                    'camera_epoch': epoch, 'image_size': calibration['image_size'],
                    'board_height_mm': height, 'square_mm': square,
                    'R_plane_to_camera': R.tolist(), 't_plane_to_camera_mm': t.tolist(),
                    'H_normalized_to_plane': H.tolist(), 'coordinate_frame': FRAME,
                    'board_hull_plane_mm': [[0., 0.], [8*square, 0.], [8*square, -5*square], [0., -5*square]]}
        result.update(identity, valid=True, reason_code='plane_fitted',
                      reason='Đã xác lập mặt phẳng; cần kiểm chứng khoảng cách độc lập.',
                      plane_fingerprint=_hash(identity), calibration=calibration,
                      calibration_rms_px=calibration['calibration_rms_px'],
                      frame_ids=[s['frame_id'] for s in used], corners_px=pts.tolist(),
                      H_normalized_to_plane_unconstrained_fit=Hfit.tolist(),
                      min_cell_px=float(cells.min()), normal_camera=R[:, 2].tolist(),
                      camera_distance_to_board_plane_mm=float(-R[:, 2] @ t),
                      alternative_normal_angle_deg=angle, alternative_reprojection_gap_px=gap,
                      plane_ambiguity_warning=bool(angle is not None and angle > 3 and gap < 0.25),
                      origin_definition='first_aligned_inner_checker_corner',
                      height_reference='physical_floor_mm_positive_toward_camera',
                      accuracy_pass=None)
    except (ValueError, TypeError, AttributeError, KeyError, np.linalg.LinAlgError, cv2.error) as exc:
        result.update(valid=False, reason=str(exc))
    return result


_IDENTITY_KEYS = ('schema_version', 'plane_id', 'calibration_fingerprint', 'camera_epoch',
                  'image_size', 'board_height_mm', 'square_mm', 'R_plane_to_camera',
                  't_plane_to_camera_mm', 'H_normalized_to_plane', 'coordinate_frame', 'board_hull_plane_mm')


def _validate_plane(plane, calibration, camera_epoch):
    if not isinstance(plane, dict) or not plane.get('valid'):
        raise ValueError('Chưa có mặt phẳng hợp lệ.')
    if plane.get('calibration_fingerprint') != calibration['calibration_fingerprint'] or plane.get('image_size') != calibration['image_size']:
        raise ValueError('Mặt phẳng không thuộc bản nội tại hoặc độ phân giải này.')
    if plane.get('coordinate_frame') != FRAME or _hash({k: plane[k] for k in _IDENTITY_KEYS}) != plane.get('plane_fingerprint'):
        raise ValueError('Cấu hình mặt phẳng không khớp fingerprint.')
    if camera_epoch is not None and plane.get('camera_epoch') != camera_epoch:
        raise ValueError('Camera đã đổi phiên; cần xác lập lại mặt phẳng.')
    R, t = np.asarray(plane['R_plane_to_camera'], float), np.asarray(plane['t_plane_to_camera_mm'], float)
    if R.shape != (3, 3) or t.shape != (3,) or not np.isfinite(R).all() or not np.isfinite(t).all():
        raise ValueError('Ma trận mặt phẳng không hữu hạn hoặc sai kích thước.')
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or abs(np.linalg.det(R)-1) > 1e-6 or R[:, 2] @ t >= 0:
        raise ValueError('Hệ trục mặt phẳng không trực chuẩn hoặc normal quay khỏi camera.')
    return R, t


def _intersect(pts, calibration, plane, marker_height_mm):
    R, t = _validate_plane(plane, calibration, None)
    offset = marker_height_mm-float(plane['board_height_mm'])
    rays = np.column_stack((_undistort(pts, calibration), np.ones(len(pts))))
    normal = R[:, 2]
    numerator = float(normal @ t+offset)
    denominator = rays @ normal
    if abs(numerator) < 1e-6 or np.any(np.abs(denominator)/np.linalg.norm(rays, axis=1) < 1e-3):
        raise ValueError('Tia nhìn gần song song mặt phẳng hoặc mặt phẳng đi qua camera.')
    scales = numerator/denominator
    if not np.isfinite(scales).all() or np.any(scales <= 0):
        raise ValueError('Giao điểm mặt phẳng nằm sau camera; kiểm tra độ cao.')
    camera = rays*scales[:, None]
    coords = (camera-t) @ R
    return coords[:, :2], camera


def _diagonal_center(points):
    matrix = np.column_stack((points[2]-points[0], -(points[3]-points[1])))
    if not np.isfinite(matrix).all() or np.linalg.cond(matrix) > 1e8:
        raise ValueError('Hai đường chéo marker suy biến.')
    fractions = np.linalg.solve(matrix, points[1]-points[0])
    if np.any(fractions < 0) or np.any(fractions > 1):
        raise ValueError('Đường chéo không giao bên trong marker.')
    return points[0]+fractions[0]*(points[2]-points[0])


def map_marker(frame, candidate, plane, marker_height_mm, target_id=900, camera_epoch=None, black_side_mm=50):
    """Unique ArUco -> plane XY and fixed-height camera XYZ, without marker PnP.

    All four marker corners must physically lie in the stated parallel plane.
    Height is marker TOP surface above floor, not its cardboard thickness alone
    when mounted on a robot. Caller must pass epoch to detect camera restarts.
    """
    result = {'detected': False, 'reason': '', 'reason_code': 'invalid_plane_measurement',
              'ids': [], 'target_count': 0, 'method': METHOD, 'coordinate_frame': FRAME,
              'position_plane_mm': None, 'camera_xyz_mm': None, 'corners_px': None,
              'center_px': None, 'world_ready': False, 'accuracy_pass': None,
              'measurement_validation': 'not_performed', 'requires_fixed_camera': True,
              'position_units': 'mm', 'body_offset_applied': False}
    try:
        height = _height(marker_height_mm, 'Độ cao mặt marker')
        side = _finite_scalar(black_side_mm, 'Cạnh đen tham chiếu', positive=True)
        if isinstance(target_id, bool) or not isinstance(target_id, (int, np.integer)) or not 0 <= target_id < 1000:
            raise ValueError('ID marker phải là số nguyên trong 0..999.')
        gray = _gray(frame)
        calibration = validate_intrinsics(candidate, [gray.shape[1], gray.shape[0]])
        R, t = _validate_plane(plane, calibration, camera_epoch)
        result.update(marker_height_mm=height, black_side_mm=side, target_id=int(target_id), dictionary=DICTIONARY,
                      plane_id=plane['plane_id'], plane_fingerprint=plane['plane_fingerprint'],
                      camera_epoch=plane['camera_epoch'], camera_epoch_verified=camera_epoch is not None,
                      calibration_fingerprint=calibration['calibration_fingerprint'],
                      calibration_rms_px=calibration['calibration_rms_px'], image_size=calibration['image_size'],
                      board_height_mm=plane['board_height_mm'], plane_ambiguity_warning=plane.get('plane_ambiguity_warning', False))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), params)
        corners, ids, _ = detector.detectMarkers(gray)
        values = [] if ids is None else [int(v) for v in ids.ravel()]
        result.update(ids=values, target_count=values.count(int(target_id)))
        if result['target_count'] != 1:
            result.update(reason_code='target_not_found' if not result['target_count'] else 'duplicate_target',
                          reason='Cần đúng một marker ID mục tiêu trong ảnh.')
            return result
        pts = _corners(corners[values.index(int(target_id))].reshape(-1, 2), 4, calibration['image_size'])
        if not cv2.isContourConvex(pts.astype(np.float32)) or cv2.contourArea(pts.astype(np.float32)) < 16:
            raise ValueError('Bốn góc marker suy biến hoặc quá nhỏ.')
        plane_pts, _ = _intersect(pts, calibration, plane, height)
        center = _diagonal_center(plane_pts)
        camera_center = R @ np.array([*center, height-plane['board_height_mm']])+t
        rvec, _ = cv2.Rodrigues(R)
        center_px, _ = cv2.projectPoints(np.array([[*center, height-plane['board_height_mm']]]),
                                       rvec, t, np.asarray(calibration['camera_matrix']), np.asarray(calibration['distortion_coefficients']))
        edge = plane_pts[1]-plane_pts[0]
        edge_lengths = np.linalg.norm(np.roll(plane_pts, -1, axis=0)-plane_pts, axis=1)
        if edge_lengths.min() <= 1e-6:
            raise ValueError('Cạnh marker bằng không trên mặt phẳng.')
        hull = np.asarray(plane['board_hull_plane_mm'], np.float32)
        outside = any(cv2.pointPolygonTest(hull, tuple(map(float, p)), False) < 0 for p in plane_pts)
        result.update(detected=True, reason_code='plane_position_estimated',
                      reason='Đã quy đổi marker lên mặt phẳng; chưa xác nhận độ chính xác mm.',
                      position_plane_mm=center.tolist(), camera_xyz_mm=camera_center.tolist(),
                      corners_plane_mm=plane_pts.tolist(), corners_px=pts.tolist(),
                      center_px=center_px.reshape(2).tolist(), center_definition='diagonal_intersection_after_undistortion_and_plane_mapping',
                      yaw_rad=float(math.atan2(edge[1], edge[0])), yaw_definition='ArUco canonical corner0_to_corner1 relative plane +x',
                      measured_edge_lengths_mm=edge_lengths.tolist(), corner_refinement='SUBPIX',
                      mean_measured_edge_mm=float(edge_lengths.mean()),
                      mean_edge_error_mm=float(edge_lengths.mean()-side),
                      mean_edge_error_percent=float(100*(edge_lengths.mean()-side)/side),
                      marker_size_used_for_scale=False,
                      projected_mean_edge_px=float(np.mean(np.linalg.norm(np.roll(pts, -1, axis=0)-pts, axis=1))),
                      extrapolation_warning=bool(outside), parallel_plane_assumption=True)
    except (ValueError, TypeError, AttributeError, KeyError, np.linalg.LinAlgError, cv2.error) as exc:
        result['reason'] = str(exc)
    return result


_RECORD_KEYS = ('plane_id', 'plane_fingerprint', 'calibration_fingerprint', 'camera_epoch',
                'marker_height_mm', 'board_height_mm', 'black_side_mm', 'target_id', 'dictionary', 'method', 'coordinate_frame')


def summarize_plane_samples(records, min_samples=MIN_FRAMES, scatter_limit_mm=5.0):
    """Robust XY scatter; stability is only a screening criterion, not accuracy."""
    if isinstance(min_samples, bool) or not isinstance(min_samples, int) or min_samples < MIN_FRAMES:
        raise ValueError('Cần ít nhất 10 frame độc lập.')
    limit = _finite_scalar(scatter_limit_mm, 'Ngưỡng phân tán', positive=True)
    records = list(records)
    good, seen, duplicates, invalid = [], set(), 0, 0
    for r in records:
        try:
            if not isinstance(r, dict) or not r.get('detected') or r.get('target_count') != 1:
                raise ValueError('invalid detection')
            if r.get('camera_epoch_verified') is not True:
                raise ValueError('caller must check current camera epoch')
            fid = _identity(r.get('frame_id'), 'frame_id')
            key = (type(fid).__name__, fid)
            if key in seen:
                duplicates += 1
                continue
            p = np.asarray(r['position_plane_mm'], float)
            if p.shape != (2,) or not np.isfinite(p).all() or r.get('coordinate_frame') != FRAME or r.get('method') != METHOD:
                raise ValueError('invalid plane coordinates')
            if any(r.get(k) is None for k in _RECORD_KEYS) or not r.get('plane_fingerprint') or not r.get('calibration_fingerprint'):
                raise ValueError('missing provenance')
            _height(r['marker_height_mm'], 'marker height')
            _height(r['board_height_mm'], 'board height')
            _finite_scalar(r['black_side_mm'], 'black side', positive=True)
            _finite_scalar(r.get('calibration_rms_px'), 'calibration RMS')
            seen.add(key)
            good.append(r)
        except (ValueError, TypeError, KeyError):
            invalid += 1
    result = {'valid': False, 'quality': 'insufficient_frames', 'reason': 'Cần ít nhất 10 frame hợp lệ độc lập.',
              'count_received': len(records), 'count_unique_valid': len(good), 'duplicate_frames': duplicates,
              'invalid_records': invalid, 'min_samples': min_samples, 'scatter_limit_mm': limit,
              'position_plane_mm': None, 'scatter_rms_mm': None, 'scatter_p95_mm': None,
              'world_ready': False, 'accuracy_pass': None, 'coordinate_frame': FRAME, 'method': METHOD,
              'measurement_validation': 'not_performed'}
    if len(good) < min_samples:
        return result
    identities = {_hash({k: r[k] for k in _RECORD_KEYS}) for r in good}
    if len(identities) != 1:
        result.update(quality='mixed_configuration', reason='Không được trộn mặt phẳng, camera, độ cao hoặc ID marker.')
        return result
    points = np.asarray([r['position_plane_mm'] for r in good])
    center = np.median(points, axis=0)
    radii = np.linalg.norm(points-center, axis=1)
    p95 = float(np.percentile(radii, 95))
    ambiguous = any(r.get('plane_ambiguity_warning', False) for r in good)
    quality = 'broad_scatter' if p95 > limit else 'ambiguous_plane' if ambiguous else 'stable_for_validation'
    result.update({k: good[0][k] for k in _RECORD_KEYS})
    result.update(valid=quality == 'stable_for_validation', quality=quality,
                  reason='Nhóm ổn định để kiểm chứng khoảng cách; chưa xác nhận độ chính xác.' if quality == 'stable_for_validation' else 'Nhóm chưa đủ ổn định hoặc mặt phẳng còn mơ hồ.',
                  position_plane_mm=center.tolist(), scatter_rms_mm=float(np.sqrt(np.mean(radii*radii))),
                  scatter_p95_mm=p95, scatter_max_mm=float(radii.max()), frame_ids=[r['frame_id'] for r in good],
                  calibration_rms_px=float(good[0]['calibration_rms_px']),
                  extrapolation_warning=any(r.get('extrapolation_warning', False) for r in good),
                  plane_ambiguity_warning=ambiguous)
    return result


def plane_distance_validation(A, B, expected_mm, min_samples=MIN_FRAMES, scatter_limit_mm=5.0):
    """Distance in common parallel plane, requiring disjoint real frame IDs."""
    expected = _finite_scalar(expected_mm, 'Khoảng cách chuẩn', positive=True)
    a = summarize_plane_samples(A, min_samples, scatter_limit_mm)
    b = summarize_plane_samples(B, min_samples, scatter_limit_mm)
    result = {'valid': False, 'reason': 'Chưa đủ hai nhóm đo hợp lệ.', 'A': a, 'B': b,
              'expected_mm': expected, 'measured_mm': None, 'signed_error_mm': None,
              'absolute_error_mm': None, 'signed_error_percent': None, 'absolute_error_percent': None,
              'world_ready': False, 'accuracy_pass': None, 'coordinate_frame': FRAME, 'method': METHOD,
              'measurement_validation': 'independent_plane_distance_trial', 'requires_fixed_camera': True}
    if not a['valid'] or not b['valid']:
        return result
    if any(a[k] != b[k] for k in _RECORD_KEYS):
        result['reason'] = 'Hai nhóm không cùng mặt phẳng, camera, độ cao hoặc marker.'
        return result
    if set(map(str, a['frame_ids'])) & set(map(str, b['frame_ids'])):
        result['reason'] = 'A và B chứa frame trùng nhau.'
        return result
    delta = np.asarray(b['position_plane_mm'])-a['position_plane_mm']
    length = float(np.linalg.norm(delta))
    if not math.isfinite(length) or length <= 1e-9:
        result['reason'] = 'A và B chưa có độ dịch chuyển khác không.'
        return result
    error = length-expected
    result.update(valid=True, reason='Đã tính sai số khoảng cách trên mặt phẳng; cần kiểm chứng nhiều vị trí.',
                  measured_mm=length, delta_plane_mm=delta.tolist(), signed_error_mm=error,
                  absolute_error_mm=abs(error), signed_error_percent=100*error/expected,
                  absolute_error_percent=100*abs(error)/expected,
                  plane_fingerprint=a['plane_fingerprint'], calibration_fingerprint=a['calibration_fingerprint'],
                  calibration_rms_px=a['calibration_rms_px'],
                  extrapolation_warning=a['extrapolation_warning'] or b['extrapolation_warning'])
    return result
