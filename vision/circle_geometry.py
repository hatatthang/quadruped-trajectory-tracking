"""Hinh hoc thuan cho trang /circle. Khong mo camera, khong ghi tep.

1. combine_planes: gop NHIEU lan dat ban co (moi lan da khop rieng bang
   plane_geometry.fit_plane, file goc KHONG sua) thanh MOT mat san, xuat ra dung
   dinh dang mat phang goc de map_marker dung lai nguyen ven.
   - San la z = 0 (board_height_mm = 0), z huong ve camera.
   - Truc x = huong sang phai cua anh, y = huong len tren anh (do thi nhin
     cung chieu voi video, dung la hinh chieu tu tren xuong, khong bi lat guong).
   - Goc toa do = trong tam cac goc ban co da thu.
2. background_features / camera_shift: so dac trung nen (ORB) voi anh tham
   chieu de phat hien camera bi cham dich. camera_epoch chi doi khi mo lai
   camera nen KHONG bat duoc truong hop nay (vi du cap 22/9 14:31-14:32 dich 80 px).
3. trajectory_metrics / trajectory_svg: ban kinh, tam, goc quet, goc xoay than,
   quang duong, hinh quy dao ti le that.
"""
import math
import uuid

from measurement_geometry import cv2, np          # cung duong dan OpenCV voi file goc
from plane_geometry import FRAME, METHOD, _IDENTITY_KEYS, _hash, _object_points

_SAME_CAMERA = ('calibration_fingerprint', 'camera_epoch', 'image_size')
# Nguong danh gia tung lan dat: do lech VI TRI cac goc (mm) so voi mat san chung.
# Khong cham theo do nghieng rieng cua tung tam: huong phap tuyen uoc luong tu MOT ban co
# nho bi lech he thong vai do (25/9: ca 7 lan cung nghieng 2-3.7 do mot huong) - chinh
# la cai mat san gop khac phuc. Do nghieng chi hien de tham khao; do chinh xac kiem bang A-B.
GOOD_OFFSET_MM, BAD_OFFSET_MM = 3.0, 8.0
# Nguong dich camera (trung vi do dich diem nen, px anh goc).
SHIFT_OK_PX, SHIFT_BAD_PX, SHIFT_MIN_INLIERS = 1.5, 3.0, 40


# ---------------------------------------------------------------- mat san gop
def board_floor_points(board):
    """54 goc trong cua mot lan dat ban co, da ha xuong san, trong he camera (mm)."""
    R = np.asarray(board['R_plane_to_camera'], float)
    t = np.asarray(board['t_plane_to_camera_mm'], float)
    on_paper = _object_points(float(board['square_mm'])) @ R.T + t
    # Mat in cao board_height_mm so voi san; phap tuyen R[:,2] huong ve camera.
    return on_paper - R[:, 2] * float(board['board_height_mm'])


def _grade(mean_offset, rms):
    worst = max(abs(mean_offset), rms)
    return 'tot' if worst <= GOOD_OFFSET_MM else 'chap_nhan' if worst <= BAD_OFFSET_MM else 'khong_dung'


def combine_planes(boards, plane_id=None):
    """Tra ve (mat_san, chan_doan). Nem ValueError neu cac lan dat khong cung camera.

    plane_id: giu nguyen ma mat san khi khoi phuc (hinh hoc tinh lai y het)."""
    boards = [b for b in boards if isinstance(b, dict) and b.get('valid')]
    if not boards:
        raise ValueError('Chưa có lần đặt bàn cờ hợp lệ nào.')
    ref = boards[0]
    for b in boards:
        if any(b.get(k) != ref.get(k) for k in _SAME_CAMERA):
            raise ValueError('Các lần đặt bàn cờ không cùng phiên camera/nội tại/độ phân giải. Làm lại từ đầu.')
    floor = [board_floor_points(b) for b in boards]
    allp = np.vstack(floor)
    c = allp.mean(axis=0)
    _, sing, vt = np.linalg.svd(allp - c, full_matrices=False)
    n = vt[2] / np.linalg.norm(vt[2])
    if float(n @ c) > 0:            # camera o goc toa do: phap tuyen phai quay ve camera
        n = -n
    x = np.array([1., 0., 0.]) - n[0] * n    # truc ngang cua anh chieu len san
    if np.linalg.norm(x) < 1e-6:
        raise ValueError('Trục ngang ảnh gần vuông góc mặt sàn; không dựng được hệ trục.')
    x /= np.linalg.norm(x)
    R = np.column_stack((x, np.cross(n, x), n))
    t = c
    if float(R[:, 2] @ t) >= 0:
        raise ValueError('Mặt sàn gộp không quay về phía camera.')
    H = np.linalg.inv(np.column_stack((R[:, 0], R[:, 1], t)))
    H /= np.max(np.abs(H))

    # Vung da hieu chuan = bao loi cua tat ca cac ban co, trong he mat san chung.
    local = [((p - t) @ R)[:, :2] for p in floor]
    hull = cv2.convexHull(np.vstack(local).astype(np.float32)).reshape(-1, 2).astype(float)

    diag = []
    for i, (b, p, q) in enumerate(zip(boards, floor, local)):
        own_n = np.asarray(b['R_plane_to_camera'], float)[:, 2]
        tilt = math.degrees(math.acos(min(1., abs(float(own_n @ n)))))
        dist = (p - t) @ n
        rms = float(np.sqrt(np.mean(dist ** 2)))
        diag.append({'index': i, 'plane_id': b.get('plane_id'), 'board_height_mm': float(b['board_height_mm']),
                     'center_mm': [round(float(v), 1) for v in q.mean(axis=0)],
                     'tilt_vs_floor_deg': round(tilt, 3), 'mean_offset_mm': round(float(dist.mean()), 2),
                     'rms_offset_mm': round(rms, 2), 'grade': _grade(float(dist.mean()), rms),
                     'reprojection_rms_px': b.get('reprojection_rms_px')})

    identity = {'schema_version': 1, 'plane_id': plane_id or uuid.uuid4().hex,
                'calibration_fingerprint': ref['calibration_fingerprint'],
                'camera_epoch': ref['camera_epoch'], 'image_size': ref['image_size'],
                'board_height_mm': 0.0, 'square_mm': float(ref['square_mm']),
                'R_plane_to_camera': R.tolist(), 't_plane_to_camera_mm': t.tolist(),
                'H_normalized_to_plane': H.tolist(), 'coordinate_frame': FRAME,
                'board_hull_plane_mm': hull.tolist()}
    if set(identity) != set(_IDENTITY_KEYS):
        raise ValueError('Khóa nhận diện mặt phẳng không khớp plane_geometry.')
    span = np.ptp(hull, axis=0)
    plane = {**identity, 'valid': True, 'reason_code': 'combined_floor_plane',
             'reason': f'Mặt sàn gộp từ {len(boards)} lần đặt bàn cờ; chưa kiểm chứng A/B độc lập.',
             'plane_fingerprint': _hash(identity), 'method': METHOD,
             'calibration_rms_px': ref.get('calibration_rms_px'),
             'plane_ambiguity_warning': any(bool(b.get('plane_ambiguity_warning')) for b in boards),
             'combined_from': [b.get('plane_id') for b in boards],
             'axes': 'x_image_right_y_image_up_origin_board_centroid',
             'normal_camera': n.tolist(), 'camera_distance_to_floor_mm': float(-n @ t),
             'height_reference': 'physical_floor_is_z0',
             'world_ready': False, 'accuracy_pass': None, 'requires_fixed_camera': True,
             'measurement_validation': 'not_performed'}
    summary = {'boards': diag, 'board_count': len(boards), 'points': int(len(allp)),
               'fit_rms_mm': round(float(sing[2] / math.sqrt(len(allp))), 3),
               'worst_tilt_deg': max(d['tilt_vs_floor_deg'] for d in diag),
               'worst_rms_mm': max(d['rms_offset_mm'] for d in diag),
               'calibrated_span_mm': [round(float(v), 1) for v in span],
               'camera_distance_to_floor_mm': round(float(-n @ t), 1),
               'camera_tilt_from_vertical_deg': round(math.degrees(math.acos(min(1., abs(float(n[2]))))), 1),
               'usable': all(d['grade'] != 'khong_dung' for d in diag)}
    return plane, summary


# ------------------------------------------------------------ camera co dich?
def _mask(shape, polygons):
    """Mat na loai vung ban co (dich chuyen giua cac lan dat) khoi phep so nen."""
    mask = np.full(shape[:2], 255, np.uint8)
    for poly in polygons or ():
        pts = np.asarray(poly, np.float32).reshape(-1, 2)
        if len(pts) < 3:
            continue
        hull = cv2.convexHull(pts)
        cx, cy = pts.mean(axis=0)
        grow = 1.35 if len(pts) > 4 else 1.8      # ban co: noi rong ra vien; marker: rong hon
        cv2.fillConvexPoly(mask, ((hull - (cx, cy)) * grow + (cx, cy)).astype(np.int32), 0)
    return mask


def _gray(frame):
    return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def background_features(frame, polygons=()):
    """Dac trung ORB cua nen anh tham chieu (bo vung ban co)."""
    gray = _gray(frame)
    kp, des = cv2.ORB_create(3000).detectAndCompute(gray, _mask(gray.shape, polygons))
    if des is None or len(kp) < SHIFT_MIN_INLIERS:
        raise ValueError('Ảnh tham chiếu quá ít chi tiết nền để theo dõi camera có bị dịch.')
    return {'points': np.float32([k.pt for k in kp]), 'descriptors': des}


def camera_shift(reference, frame, polygons=()):
    """Trung vi do dich diem nen (px) giua anh tham chieu va anh hien tai."""
    gray = _gray(frame)
    kp, des = cv2.ORB_create(3000).detectAndCompute(gray, _mask(gray.shape, polygons))
    out = {'median_px': None, 'p90_px': None, 'inliers': 0, 'status': 'khong_du_dac_trung', 'ok': False}
    if des is None or len(kp) < SHIFT_MIN_INLIERS:
        return out
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(reference['descriptors'], des)
    if len(matches) < SHIFT_MIN_INLIERS:
        return out
    a = reference['points'][[m.queryIdx for m in matches]]
    b = np.float32([kp[m.trainIdx].pt for m in matches])
    _, inl = cv2.findHomography(a, b, cv2.RANSAC, 3.0)
    if inl is None or int(inl.sum()) < SHIFT_MIN_INLIERS:
        return out
    d = np.linalg.norm((b - a)[inl.ravel().astype(bool)], axis=1)
    med = float(np.median(d))
    out.update(median_px=round(med, 2), p90_px=round(float(np.percentile(d, 90)), 2), inliers=int(inl.sum()),
               status='on_dinh' if med <= SHIFT_OK_PX else 'dich_nhe' if med <= SHIFT_BAD_PX else 'da_dich',
               ok=med <= SHIFT_OK_PX)
    return out


# --------------------------------------------------------- chi so quy dao
def fit_circle(xy):
    """Khop duong tron hinh hoc (Kasa roi Gauss-Newton). Tra ve (cx, cy, R, rms)."""
    x, y = xy[:, 0], xy[:, 1]
    D, E, F = np.linalg.lstsq(np.column_stack((x, y, np.ones_like(x))), -(x * x + y * y), rcond=None)[0]
    cx, cy = -D / 2, -E / 2
    r = math.sqrt(max(cx * cx + cy * cy - F, 0.))
    for _ in range(30):
        d = np.maximum(np.hypot(x - cx, y - cy), 1e-9)
        J = np.column_stack(((cx - x) / d, (cy - y) / d, -np.ones_like(x)))
        step = np.linalg.lstsq(J, -(d - r), rcond=None)[0]
        cx, cy, r = cx + step[0], cy + step[1], r + step[2]
        if np.linalg.norm(step) < 1e-6:
            break
    res = np.hypot(x - cx, y - cy) - r
    return float(cx), float(cy), float(abs(r)), float(np.sqrt(np.mean(res ** 2)))


def _unwrap_deg(a):
    return np.degrees(np.unwrap(np.asarray(a, float)))


def trajectory_metrics(samples, bin_s=1.0, path_bin_s=5.0):
    """samples: [(elapsed_s, x_mm, y_mm, heading_rad, extrapolated), ...] chi mau nhan marker.

    Khop duong tron tren diem trung binh moi bin_s giay; quang duong tren diem trung
    binh moi path_bin_s giay (dai hon mot chu ky buoc) de nhieu va lac than khong cong don.
    Goc quet tinh quanh tam duong tron khop; duong > 0 la quay trai (nhin tu tren xuong).
    """
    if len(samples) < 5:
        return {'valid': False, 'reason': 'Quá ít mẫu nhận marker.'}
    s = np.asarray(samples, float)
    t, xy, head = s[:, 0], s[:, 1:3], s[:, 3]
    def binned(width):
        k = np.floor((t - t[0]) / width).astype(int)
        return np.array([xy[k == i].mean(axis=0) for i in np.unique(k)])
    smooth, coarse = binned(bin_s), binned(path_bin_s)
    path = float(np.sum(np.linalg.norm(np.diff(coarse, axis=0), axis=1))) if len(coarse) > 1 else 0.
    head_turn = _unwrap_deg(head)
    first, last = xy[t <= t[0] + 1].mean(axis=0), xy[t >= t[-1] - 1].mean(axis=0)
    out = {'valid': True, 'samples': int(len(s)), 'duration_s': round(float(t[-1] - t[0]), 2),
           'start_mm': [round(float(v), 1) for v in first], 'end_mm': [round(float(v), 1) for v in last],
           'closure_mm': round(float(np.linalg.norm(last - first)), 1),
           'path_length_mm': round(path, 1),
           'net_displacement_mm': round(float(np.linalg.norm(last - first)), 1),
           'heading_change_deg': round(float(head_turn[-1] - head_turn[0]), 1),
           'position_spread_mm': round(float(np.percentile(np.linalg.norm(xy - np.median(xy, axis=0), axis=1), 95)), 1),
           'extrapolated_fraction': round(float(s[:, 4].mean()), 3), 'circle': None}
    # Chi khop duong tron khi robot thuc su di cung (khong phai xoay tai cho).
    if len(smooth) >= 8 and out['position_spread_mm'] > 40:
        cx, cy, r, rms = fit_circle(smooth)
        swept = _unwrap_deg(np.arctan2(smooth[:, 1] - cy, smooth[:, 0] - cx))
        out['circle'] = {'center_mm': [round(cx, 1), round(cy, 1)], 'radius_mm': round(r, 1),
                         'rms_mm': round(rms, 1), 'swept_deg': round(float(swept[-1] - swept[0]), 1),
                         'turns': round(float(swept[-1] - swept[0]) / 360, 3),
                         'fit_on': f'trung_binh_{bin_s:g}s'}
    return out


# ---------------------------------------------------------------- hinh SVG
def trajectory_svg(trail, hull=None, metrics=None, title='', size=760):
    """Quy dao ti le that (1 mm x = 1 mm y), luoi 100 mm, dut net tai cho mat marker."""
    pts = [p for p in trail if p is not None]
    if not pts:
        return None
    arr = np.asarray([p[:2] for p in pts], float)
    extra = [arr]
    if hull is not None and len(hull):
        extra.append(np.asarray(hull, float))
    circ = (metrics or {}).get('circle')
    if circ:
        c, r = np.asarray(circ['center_mm']), circ['radius_mm']
        extra.append(np.array([c - r, c + r]))
    allp = np.vstack(extra)
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    span = float(max(hi - lo)) * 1.12 + 50
    mid = (lo + hi) / 2
    top, pad = 70, 50
    k = (size - 2 * pad) / span
    X = lambda v: pad + (v - mid[0] + span / 2) * k
    Y = lambda v: top + (mid[1] + span / 2 - v) * k
    H = int(top + span * k + 60)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{H}" viewBox="0 0 {size} {H}" '
         'font-family="Arial, sans-serif"><rect width="100%" height="100%" fill="#fff"/>']
    g0, g1 = np.floor((mid - span / 2) / 100) * 100, np.ceil((mid + span / 2) / 100) * 100
    for v in range(int(g0[0]), int(g1[0]) + 1, 100):
        o.append(f'<line x1="{X(v):.1f}" y1="{Y(g0[1]):.1f}" x2="{X(v):.1f}" y2="{Y(g1[1]):.1f}" stroke="#eee"/>')
    for v in range(int(g0[1]), int(g1[1]) + 1, 100):
        o.append(f'<line x1="{X(g0[0]):.1f}" y1="{Y(v):.1f}" x2="{X(g1[0]):.1f}" y2="{Y(v):.1f}" stroke="#eee"/>')
    if hull is not None and len(hull):
        o.append('<polygon points="' + ' '.join(f'{X(p[0]):.1f},{Y(p[1]):.1f}' for p in hull) +
                 '" fill="#e8f4f1" stroke="#9cc9bf" stroke-dasharray="4 3"/>')
    if circ:
        o.append(f'<circle cx="{X(c[0]):.1f}" cy="{Y(c[1]):.1f}" r="{r * k:.1f}" fill="none" stroke="#999" '
                 f'stroke-dasharray="6 5"/><circle cx="{X(c[0]):.1f}" cy="{Y(c[1]):.1f}" r="3" fill="#999"/>')
    seg, segs = [], []
    for p in trail:
        if p is None:
            if len(seg) > 1:
                segs.append(seg)
            seg = []
        else:
            seg.append(f'{X(p[0]):.1f},{Y(p[1]):.1f}')
    if len(seg) > 1:
        segs.append(seg)
    for s in segs:
        o.append(f'<polyline points="{" ".join(s)}" fill="none" stroke="#087e72" stroke-width="2"/>')
    o.append(f'<circle cx="{X(arr[0, 0]):.1f}" cy="{Y(arr[0, 1]):.1f}" r="6" fill="#2e7d32"/>')
    o.append(f'<circle cx="{X(arr[-1, 0]):.1f}" cy="{Y(arr[-1, 1]):.1f}" r="6" fill="#c62828"/>')
    bar = 100 * k
    o.append(f'<line x1="{pad}" y1="{H - 30}" x2="{pad + bar:.1f}" y2="{H - 30}" stroke="#333" stroke-width="3"/>'
             f'<text x="{pad}" y="{H - 12}" font-size="12" fill="#333">100 mm (ô lưới 100 mm)</text>')
    o.append(f'<text x="{pad}" y="26" font-size="16" font-weight="bold" fill="#193640">{title}</text>')
    m = metrics or {}
    line = (f'<tspan fill="#2e7d32">●</tspan> bắt đầu  <tspan fill="#c62828">●</tspan> kết thúc · {m.get("duration_s", "—")} s · thân xoay {m.get("heading_change_deg", "—")}°'
            + (f' · R khớp {circ["radius_mm"]} mm · quét {circ["swept_deg"]}°' if circ else ''))
    o.append(f'<text x="{pad}" y="50" font-size="12.5" fill="#444">{line}</text>')
    o.append(f'<text x="{size - pad}" y="{H - 12}" font-size="11" fill="#777" text-anchor="end">'
             'x: sang phải ảnh · y: lên trên ảnh · nhìn từ trên xuống</text></svg>')
    return '\n'.join(o)


# -------------------------------------------------- duong dat (theo lenh firmware)
# Firmware Reference_Pose: robot di toi theo tiep tuyen; tam duong tron o BEN TRAI than
# khi CCW (CIRCLE_DIR = +1), BEN PHAI khi CW (-1), cach dung R. Xoay tai cho: giu nguyen vi tri.
KINDS = {'cw': -1, 'ccw': 1, 'xoay_phai': -1, 'xoay_trai': 1}


def anchor_pose(samples, t_event=None, window_s=2.5):
    """Tu the dung yen ngay truoc luc bat dau buoc (hoac dau luot ghi neu khong danh dau)."""
    s = np.asarray(samples, float)
    if len(s) == 0:
        return None
    if t_event is None:
        t0, how = float(s[0, 0]), 'dau_luot_ghi'
        sel = s[s[:, 0] <= t0 + window_s]
    else:
        t0, how = float(t_event), 'nut_bat_dau_buoc'
        sel = s[(s[:, 0] <= t0) & (s[:, 0] >= t0 - window_s)]
    if len(sel) < 3:
        return None
    p0 = np.median(sel[:, 1:3], axis=0)
    h0 = math.atan2(float(np.mean(np.sin(sel[:, 3]))), float(np.mean(np.cos(sel[:, 3]))))
    return {'t0_s': round(t0, 3), 'p0_mm': [round(float(v), 2) for v in p0], 'heading0_deg': round(math.degrees(h0), 3),
            'samples': int(len(sel)), 'still_p95_mm': round(float(np.percentile(np.linalg.norm(sel[:, 1:3] - p0, axis=1), 95)), 2),
            'anchor': how}


def reference_center(anchor, kind, radius_mm):
    h = math.radians(anchor['heading0_deg'])
    left = np.array([-math.sin(h), math.cos(h)])
    return np.asarray(anchor['p0_mm'], float) + KINDS[kind] * radius_mm * left


def reference_errors(samples, anchor, kind, radius_mm=None, angle_deg=None):
    """So quy dao tam than (tu luc bat dau buoc) voi duong dat neo tai tu the dau."""
    if kind not in KINDS or not anchor:
        return None
    s = np.asarray(samples, float)
    s = s[s[:, 0] >= anchor['t0_s']]
    if len(s) < 5:
        return None
    p0, sign = np.asarray(anchor['p0_mm'], float), KINDS[kind]
    t, xy = s[:, 0], s[:, 1:3]
    end = xy[t >= t[-1] - 2].mean(axis=0)
    turn = float(np.degrees(np.unwrap(np.r_[math.radians(anchor['heading0_deg']), s[:, 3]]))[-1] - anchor['heading0_deg'])
    out = {'kind': kind, 'anchor': anchor, 'radius_set_mm': radius_mm, 'angle_set_deg': angle_deg,
           'heading_turn_deg': round(turn, 1), 'direction_ok': bool(turn * sign > 0)}
    if angle_deg:
        out['heading_turn_error_deg'] = round(abs(turn) - angle_deg, 1)
    if kind in ('cw', 'ccw') and radius_mm:
        c = reference_center(anchor, kind, radius_mm)
        e = np.hypot(xy[:, 0] - c[0], xy[:, 1] - c[1]) - radius_mm       # + : ra ngoai duong dat
        a0 = math.atan2(p0[1] - c[1], p0[0] - c[0])
        phi = np.unwrap(np.r_[a0, np.arctan2(xy[:, 1] - c[1], xy[:, 0] - c[0])])
        swept = float(np.degrees(phi[-1] - a0))
        out.update(center_set_mm=[round(float(v), 1) for v in c], swept_about_set_center_deg=round(swept, 1),
                   cross_track_mm={'mean': round(float(e.mean()), 1), 'rms': round(float(np.sqrt(np.mean(e ** 2))), 1),
                                   'p95_abs': round(float(np.percentile(np.abs(e), 95)), 1),
                                   'max_abs': round(float(np.abs(e).max()), 1), 'end': round(float(e[t >= t[-1] - 2].mean()), 1)})
        if angle_deg:
            a1 = a0 + sign * math.radians(angle_deg)
            goal = c + radius_mm * np.array([math.cos(a1), math.sin(a1)])
            out.update(end_set_mm=[round(float(v), 1) for v in goal], end_error_mm=round(float(np.linalg.norm(end - goal)), 1),
                       swept_error_deg=round(abs(swept) - angle_deg, 1))
    else:                                       # xoay tai cho: dat = dung yen tai p0
        d = np.linalg.norm(xy - p0, axis=1)
        out['drift_mm'] = {'mean': round(float(d.mean()), 1), 'p95': round(float(np.percentile(d, 95)), 1),
                           'max': round(float(d.max()), 1), 'end': round(float(np.linalg.norm(end - p0)), 1)}
    return out
