"""Hinh quy dao cua MOT luot /circle theo kieu hinh bao cao (SVG + PNG).

Chi doc cac mau da ghi cua chinh luot do (samples: elapsed_s, x, y, heading, ngoai suy),
nen hinh va video / samples.csv trong cung thu muc luot la cung mot lan chay.
  Trai      : quy dao tam than nhin tu tren xuong, ti le that, luoi 100 mm,
              vung ban co, duong tron khop, diem dau / cuoi.
  Phai tren : di vong -> do lech ban kinh r - R theo goc quet (sai so bam duong tron);
              xoay tai cho -> do troi tam than theo thoi gian.
  Phai duoi : goc than doi so voi luc dau theo thoi gian.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / 'docs'))
from ve_hinh import Canvas, Truc      # bo ve hinh bao cao (PIL + SVG)

XANH, CAM, XAM, DO, LUC = '#087e72', '#d2691e', '#9a9a9a', '#c62828', '#2e7d32'


def _when(run, summary=None):
    """Ngay gio bat dau ghi (dd/mm/yyyy hh:mm) de dat tieu de hinh; khong co thi dung ten thu muc."""
    import json
    try:
        summary = summary or json.loads((Path(run) / 'summary.json').read_text(encoding='utf-8'))
        t = summary['started_at']
        return f'{t[8:10]}/{t[5:7]}/{t[:4]} {t[11:16]}'
    except (OSError, ValueError, KeyError, TypeError):
        return Path(run).name


def _nice(span, target=5):
    """Buoc chia truc dep (1, 2, 5 x 10^k) cho khoang target vach."""
    raw = max(span, 1e-9) / target
    mag = 10 ** math.floor(math.log10(raw))
    return next(k * mag for k in (1, 2, 5, 10) if k * mag >= raw)


def _smooth(t, v, window_s=14.0):
    """Trung binh truot theo mot chu ky buoc de thay xu huong, bo dao dong lac than."""
    dt = float(np.median(np.diff(t))) if len(t) > 2 else 1.
    half = max(0, int(round(window_s / max(dt, 1e-3) / 2)))
    n = len(v)
    # Cua so doi xung, co lai o hai dau -> khong lech khi tin hieu tang/giam deu (goc than).
    i = np.arange(n)
    h = np.minimum(half, np.minimum(i, n - 1 - i))
    c = np.r_[0., np.cumsum(v, dtype=float)]
    return (c[i + h + 1] - c[i - h]) / (2 * h + 1)


def _segments(t, *cols, gap_s=2.0):
    """Tach thanh doan lien tuc; ngat khi mat marker lau hon gap_s giay."""
    cut = np.where(np.diff(t) > gap_s)[0] + 1
    return [tuple(c[a:b] for c in (t, *cols)) for a, b in zip(np.r_[0, cut], np.r_[cut, len(t)]) if b - a > 1]


def make(path_no_ext, samples, hull=None, metrics=None, title='', ref=None):
    """samples: [(elapsed_s, x_mm, y_mm, heading_rad, extrapolated), ...]. Tra ve (svg, png)."""
    s = np.asarray(samples, float)
    if len(s) < 5:
        return None
    t, x, y, hd = s[:, 0], s[:, 1], s[:, 2], s[:, 3]
    m = metrics or {}
    circ = m.get('circle')
    c = Canvas(600, 380)
    c.text(20, 20, title, 10.5, '#111', dam=True)

    # ---------------------------------------------------------- quy dao
    pts = [np.c_[x, y]]
    if hull is not None and len(hull):
        pts.append(np.asarray(hull, float))
    if circ:
        cc, r = np.asarray(circ['center_mm']), circ['radius_mm']
        pts.append(np.array([cc - r, cc + r]))
    rc = np.asarray(ref['center_set_mm']) if ref and ref.get('center_set_mm') else None
    if rc is not None:
        rr_set = ref['radius_set_mm']
        pts.append(np.array([rc - rr_set, rc + rr_set]))
    allp = np.vstack(pts)
    lo, hi = allp.min(0), allp.max(0)
    span = float(max(hi - lo)) * 1.08 + 40
    mid = (lo + hi) / 2
    T = Truc(c, 58, 38, 262, 262, (mid[0] - span / 2, mid[0] + span / 2), (mid[1] - span / 2, mid[1] + span / 2))
    step = _nice(span, 8)
    g = np.arange(math.ceil((mid.min() - span) / step) * step, mid.max() + span, step)
    T.khung(luoi_x=list(g), luoi_y=list(g), co=7.5)
    c.text(T.x + T.w / 2, T.y + T.h + 27, 'x (mm) · sang phải ảnh', 8.5, '#000', 'middle')
    c.text(T.x - 40, T.y + T.h / 2, 'y (mm) · lên trên ảnh', 8.5, '#000', 'middle', xoay=-90)
    if hull is not None and len(hull):
        c.polygon([(T.X(p[0]), T.Y(p[1])) for p in hull], to='#cfe8e2', mau='#9cc9bf', day=.6, mo=.6)
    if circ:
        a = np.linspace(0, 2 * math.pi, 181)
        c.polyline([(T.X(cc[0] + r * math.cos(v)), T.Y(cc[1] + r * math.sin(v))) for v in a], XAM, .9, nét='4 3')
        c.circle(T.X(cc[0]), T.Y(cc[1]), 1.8, to=XAM)
    if ref:                              # duong dat neo tai tu the luc bat dau buoc
        q0, h0 = np.asarray(ref['anchor']['p0_mm']), math.radians(ref['anchor']['heading0_deg'])
        if rc is not None:
            a = np.linspace(0, 2 * math.pi, 241)
            c.polyline([(T.X(rc[0] + rr_set * math.cos(v)), T.Y(rc[1] + rr_set * math.sin(v))) for v in a], CAM, 1.1, nét='5 3')
            c.line(T.X(rc[0]) - 4, T.Y(rc[1]), T.X(rc[0]) + 4, T.Y(rc[1]), CAM, 1)
            c.line(T.X(rc[0]), T.Y(rc[1]) - 4, T.X(rc[0]), T.Y(rc[1]) + 4, CAM, 1)
        c.mui_ten(T.X(q0[0]), T.Y(q0[1]), T.X(q0[0]) + 16 * math.cos(h0), T.Y(q0[1]) - 16 * math.sin(h0), CAM, 1.2, 5)
    for seg in _segments(t, x, y):
        c.polyline([(T.X(a), T.Y(b)) for a, b in zip(seg[1], seg[2])], XANH, 1.1)
    c.circle(T.X(x[0]), T.Y(y[0]), 3.4, to=LUC, mau='#fff', day=.6)
    c.circle(T.X(x[-1]), T.Y(y[-1]), 3.4, to=DO, mau='#fff', day=.6)
    lx, ly = T.x + 6, T.y + 12
    c.rect(lx - 3, ly - 9, 122, 27 + 11 * bool(circ) + 11 * bool(rc is not None), to='#ffffff', mau='#ccc', day=.5)
    c.circle(lx + 4, ly - 3, 2.8, to=LUC); c.text(lx + 11, ly, 'điểm đầu', 7.5)
    c.circle(lx + 60, ly - 3, 2.8, to=DO); c.text(lx + 67, ly, 'điểm cuối', 7.5)
    c.line(lx, ly + 9, lx + 12, ly + 9, XANH, 1.3); c.text(lx + 16, ly + 12, 'tâm thân đo được', 7.5)
    if circ:
        c.line(lx, ly + 20, lx + 12, ly + 20, XAM, .9, nét='4 3'); c.text(lx + 16, ly + 23, 'đường tròn khớp', 7.5)
    if rc is not None:
        yy = ly + 20 + 11 * bool(circ)
        c.line(lx, yy, lx + 12, yy, CAM, 1.1, nét='5 3'); c.text(lx + 16, yy + 3, f'đường đặt R {rr_set:.0f} mm', 7.5)

    # ----------------------------------------------- phai tren: sai lech
    if rc is not None:                   # lech khoi DUONG DAT theo thoi gian (nhu lech Y o /line)
        ys = np.hypot(x - rc[0], y - rc[1]) - rr_set
        xs, xl, yl = t / 60, 'thời gian (phút)', 'lệch (mm)'
        ct = ref['cross_track_mm']
        note = f'Lệch khỏi đường đặt (+ ra ngoài) · RMS {ct["rms"]:.1f}, lớn nhất {ct["max_abs"]:.0f} mm'
    elif ref:                            # xoay tai cho: dat = dung yen tai diem dau
        q0 = np.asarray(ref['anchor']['p0_mm'])
        xs, ys, xl, yl = t / 60, np.hypot(x - q0[0], y - q0[1]), 'thời gian (phút)', 'trôi (mm)'
        note = f'Tâm thân trôi khỏi điểm đầu · lớn nhất {ref["drift_mm"]["max"]:.0f} mm'
    elif circ:
        rr = np.hypot(x - cc[0], y - cc[1]) - r
        sw = np.degrees(np.unwrap(np.arctan2(y - cc[1], x - cc[0])))
        xs, ys, xl, yl = sw - sw[0], rr, 'góc quét quanh tâm khớp (°)', 'r − R (mm)'
        note = f'Lệch khỏi đường tròn khớp · RMS {circ["rms_mm"]:.1f} mm'
    else:
        med = np.median(np.c_[x, y], axis=0)
        xs, ys = t / 60, np.hypot(x - med[0], y - med[1])
        xl, yl, note = 'thời gian (phút)', 'độ trôi (mm)', f'Tâm thân trôi khỏi vị trí giữa · 95% ≤ {m.get("position_spread_mm", 0):.0f} mm'

    def panel(y0, xs, ys, xl, yl, note):
        x0, x1 = float(xs.min()), float(xs.max())
        if x1 - x0 < 1e-6:
            x1 = x0 + 1
        lo, hi = min(float(np.percentile(ys, .5)), 0.), max(float(np.percentile(ys, 99.5)), 0.)
        pad = max(hi - lo, 2.) * .12
        y_lo, y_hi = lo - pad, hi + pad
        P = Truc(c, 385, y0, 195, 100, (x0, x1), (y_lo, y_hi))
        sx, sy = _nice(x1 - x0), _nice(y_hi - y_lo, 4)
        P.khung(luoi_x=list(np.arange(math.ceil(x0 / sx) * sx, x1 + 1e-9, sx)),
                luoi_y=list(np.arange(math.ceil(y_lo / sy) * sy, y_hi + 1e-9, sy)), co=7)
        if y_lo < 0 < y_hi:
            c.line(P.x, P.Y(0), P.x + P.w, P.Y(0), '#666', .6)
        for tt, a_, b_ in _segments(t, xs, ys):      # ngat tai cho mat marker, khong noi bu
            c.polyline([(P.X(a), P.Y(b)) for a, b in zip(a_, np.clip(b_, y_lo, y_hi))], '#9fd3cb', .5)
            c.polyline([(P.X(a), P.Y(b)) for a, b in zip(a_, np.clip(_smooth(tt, b_), y_lo, y_hi))], XANH, 1.2)
        c.text(P.x + P.w / 2, P.y + P.h + 24, xl, 8, '#000', 'middle')
        c.text(P.x - 32, P.y + P.h / 2, yl, 8, '#000', 'middle', xoay=-90)
        c.text(P.x, P.y - 5, note, 7.5, '#333')
    panel(48, xs, ys, xl, yl, note)
    turn = np.degrees(np.unwrap(hd) - hd[0])
    panel(212, t / 60, turn, 'thời gian (phút)', 'góc thân (°)',
          f'Góc thân đổi so với lúc đầu · tổng {m.get("heading_change_deg", float(turn[-1])):.1f}°')
    c.text(385, 372, 'Ô phải · nhạt: từng mẫu, đậm: trung bình mỗi 14 s (một chu kỳ bước)', 7, '#666')

    # ------------------------------------------------------------ so lieu
    line = (f'{m.get("duration_s", t[-1] - t[0]):.0f} s · {len(s)} mẫu nhận marker · thân xoay {m.get("heading_change_deg", 0):.1f}°'
            + (f' · R khớp {circ["radius_mm"]:.1f} mm · quét {circ["swept_deg"]:.1f}° ({circ["turns"]:.2f} vòng)' if circ else '')
            + f' · điểm cuối cách điểm đầu {m.get("closure_mm", 0):.0f} mm')
    c.text(20, 358, line, 8, '#333')
    files = c.luu(str(path_no_ext))
    if rc is not None:
        try:
            make_unrolled(Path(path_no_ext).with_name('duong_dat'), samples, ref, title)
        except Exception as exc:          # hinh phu: loi thi bo qua, quy_dao van con
            print('duong_dat loi:', exc)
    return files


def make_unrolled(path_no_ext, samples, ref, title=''):
    """Kieu do thi trang /final cho duong tron: trai thang duong dat ra thanh truc ngang.

    Truc ngang = quang duong doc duong dat (goc quet quanh TAM DAT x R dat), giong 'X doc line'.
    Truc doc   = lech khoi duong dat: + ra ngoai, - vao trong. Duong dut ngang 0 = duong dat.
    """
    s = np.asarray(samples, float)
    t, x, y = s[:, 0], s[:, 1], s[:, 2]
    rc, R = np.asarray(ref['center_set_mm'], float), float(ref['radius_set_mm'])
    q0 = np.asarray(ref['anchor']['p0_mm'], float)
    sign = -1. if ref['kind'] == 'cw' else 1.
    a0 = math.atan2(q0[1] - rc[1], q0[0] - rc[0])
    phi = np.unwrap(np.r_[a0, np.arctan2(y - rc[1], x - rc[0])])[1:]
    arc = sign * (phi - a0) * R                       # mm doc duong dat, duong theo chieu lenh
    e = np.hypot(x - rc[0], y - rc[1]) - R
    goal = R * math.radians(ref['angle_set_deg']) if ref.get('angle_set_deg') else None
    x0, x1 = min(0., float(arc.min())), max(float(arc.max()), goal or 0.) * 1.03 + 1
    q = max(100., float(np.percentile(np.abs(e), 99.5)) * 1.15)       # nhu /final: toi thieu +-100 mm
    c = Canvas(600, 300)
    c.text(20, 20, 'Lệch khỏi đường đặt (trải thẳng) · ' + title, 10.5, '#111', dam=True)
    P = Truc(c, 62, 44, 516, 186, (x0, x1), (-q, q))
    sx, sy = _nice(x1 - x0, 8), _nice(2 * q, 6)
    P.khung(luoi_x=list(np.arange(math.ceil(x0 / sx) * sx, x1 + 1e-9, sx)),
            luoi_y=list(np.arange(math.ceil(-q / sy) * sy, q + 1e-9, sy)), co=7)
    for k in range(1, 12):                            # moc goc quet 90, 180, 270... quanh tam dat
        v = R * math.pi / 2 * k
        if v > x1:
            break
        c.line(P.X(v), P.y, P.X(v), P.y + P.h, '#d9d9d9', .6, nét='2 3')
        c.text(P.X(v), P.y - 4, f'{90 * k}°', 7, '#888', 'middle')
    if goal:
        c.line(P.X(goal), P.y, P.X(goal), P.y + P.h, CAM, .9, nét='4 3')
        c.text(P.X(goal) - 3, P.y + 10, f'cuối lệnh {ref["angle_set_deg"]:.0f}°', 7, CAM, 'end')
    c.line(P.X(x0), P.Y(0), P.X(x1), P.Y(0), CAM, 1.2, nét='6 4')
    c.text(P.x + 6, P.Y(0) - 5, f'đường đặt R {R:.0f} mm', 7.5, CAM)
    for tt, aa, ee in _segments(t, arc, e):
        c.polyline([(P.X(a), P.Y(b)) for a, b in zip(aa, np.clip(ee, -q, q))], XANH, 1.0)
    c.circle(P.X(arc[0]), P.Y(np.clip(e[0], -q, q)), 3, to=LUC, mau='#fff', day=.5)
    c.circle(P.X(arc[-1]), P.Y(np.clip(e[-1], -q, q)), 3, to=DO, mau='#fff', day=.5)
    c.text(P.x + P.w / 2, P.y + P.h + 24, 'quãng đường dọc đường đặt (mm) · vạch mờ: góc quét quanh tâm đặt', 8, '#000', 'middle')
    c.text(P.x - 36, P.y + P.h / 2, 'lệch (mm)', 8, '#000', 'middle', xoay=-90)
    ct = ref.get('cross_track_mm') or {}
    c.text(20, 282, f'+ ra ngoài, − vào trong đường đặt · lệch TB {ct.get("mean", 0):+.1f} mm, RMS {ct.get("rms", 0):.1f} mm, '
           f'lớn nhất {ct.get("max_abs", 0):.0f} mm · đi được {abs(ref.get("swept_about_set_center_deg", 0)):.0f}° quanh tâm đặt'
           + (f' (lệnh {ref["angle_set_deg"]:.0f}°)' if goal else '') + ' · neo tại tư thế đứng yên ' + ('lúc bấm bắt đầu bước' if ref['anchor']['anchor'] == 'nut_bat_dau_buoc' else 'đầu lượt ghi'), 7.5, '#333')
    return c.luu(str(path_no_ext))


# ====================================================== hinh danh gia sai so
def _load_run(run):
    """Doc mau nhan marker cua luot tu samples.csv (nguon duy nhat) + context.json."""
    import csv, json
    rows = [r for r in csv.DictReader(open(run / 'samples.csv', encoding='utf-8-sig')) if r['detected'] == 'True']
    f = lambda r, k: float(r[k]) if r.get(k) not in (None, '') else float('nan')
    arr = np.array([[f(r, 'elapsed_s'), f(r, 'x_mm'), f(r, 'y_mm'), math.radians(f(r, 'heading_deg')),
                     float(r.get('extrapolation_warning') == 'True'), f(r, 'mean_edge_error_percent'),
                     f(r, 'outside_board_mm')] for r in rows]).reshape(-1, 7)
    ctx = json.loads((run / 'context.json').read_text(encoding='utf-8'))
    return arr, ctx


def _series(c, x, y, w, h, t, v, xl, yl, note, band=None, floor0=False, smooth=True):
    """O thoi gian: nhat = tung mau, dam = trung binh moi chu ky buoc; ngat tai cho mat marker."""
    ok = np.isfinite(v)
    t, v = t[ok], v[ok]
    if len(t) < 2:
        c.text(x, y + h / 2, 'Không có dữ liệu', 8, '#777')
        return
    lo, hi = float(np.percentile(v, .5)), float(np.percentile(v, 99.5))
    if band:
        lo, hi = min(lo, -band), max(hi, band)
    if floor0:
        lo = min(lo, 0.)
    pad = max(hi - lo, 1.) * .12
    lo, hi = lo - pad, hi + pad
    tm = t / 60
    P = Truc(c, x, y, w, h, (float(tm[0]), max(float(tm[-1]), float(tm[0]) + 1e-3)), (lo, hi))
    sx, sy = _nice(tm[-1] - tm[0]), _nice(hi - lo, 4)
    P.khung(luoi_x=list(np.arange(math.ceil(tm[0] / sx) * sx, tm[-1] + 1e-9, sx)),
            luoi_y=list(np.arange(math.ceil(lo / sy) * sy, hi + 1e-9, sy)), co=7)
    if band:
        c.polygon([(P.x, P.Y(band)), (P.x + P.w, P.Y(band)), (P.x + P.w, P.Y(-band)), (P.x, P.Y(-band))],
                  to='#cfe8e2', mo=.55)
    if lo < 0 < hi:
        c.line(P.x, P.Y(0), P.x + P.w, P.Y(0), '#666', .6)
    for tt, vv in _segments(t, v):
        c.polyline([(P.X(a / 60), P.Y(b)) for a, b in zip(tt, np.clip(vv, lo, hi))], '#9fd3cb' if smooth else XANH, .5 if smooth else 1.1)
        if smooth:
            c.polyline([(P.X(a / 60), P.Y(b)) for a, b in zip(tt, np.clip(_smooth(tt, vv), lo, hi))], XANH, 1.2)
    c.text(P.x + P.w / 2, P.y + P.h + 24, xl, 8, '#000', 'middle')
    c.text(P.x - 32, P.y + P.h / 2, yl, 8, '#000', 'middle', xoay=-90)
    c.text(P.x, P.y - 5, note, 7.5, '#333')


def _histogram(c, x, y, w, h, v, xl, note):
    v = v[np.isfinite(v)]
    lo, hi = float(np.percentile(v, .5)), float(np.percentile(v, 99.5))
    if hi - lo < 1e-6:
        lo, hi = lo - 1, hi + 1
    step = _nice(hi - lo, 18)
    edges = np.arange(math.floor(lo / step) * step, hi + step, step)
    cnt, edges = np.histogram(np.clip(v, edges[0], edges[-1]), edges)
    frac = 100 * cnt / max(cnt.sum(), 1)
    P = Truc(c, x, y, w, h, (float(edges[0]), float(edges[-1])), (0., float(frac.max()) * 1.15))
    sx, sy = _nice(edges[-1] - edges[0]), _nice(frac.max() * 1.15, 4)
    P.khung(luoi_x=list(np.arange(math.ceil(edges[0] / sx) * sx, edges[-1] + 1e-9, sx)),
            luoi_y=list(np.arange(0, frac.max() * 1.15 + 1e-9, sy)), co=7)
    for a, b, f in zip(edges, edges[1:], frac):
        if f > 0:
            c.rect(P.X(a) + .4, P.Y(f), P.X(b) - P.X(a) - .8, P.Y(0) - P.Y(f), to='#7cc2b7', mau=XANH, day=.4)
    if edges[0] < 0 < edges[-1]:
        c.line(P.X(0), P.y, P.X(0), P.y + P.h, '#666', .7, nét='3 2')
    c.text(P.x + P.w / 2, P.y + P.h + 24, xl, 8, '#000', 'middle')
    c.text(P.x - 32, P.y + P.h / 2, 'tỉ lệ mẫu (%)', 8, '#000', 'middle', xoay=-90)
    c.text(P.x, P.y - 5, note, 7.5, '#333')


def make_errors(run_dir, metrics=None, ref=None):
    """Hinh sai_so.svg/.png cua mot luot: dat - do - sai lech, phan bo do lech, tu kiem he do."""
    from circle_geometry import trajectory_metrics
    run = Path(run_dir)
    a, ctx = _load_run(run)
    if len(a) < 5:
        return None
    t, x, y, hd, edge, out = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 5], a[:, 6]
    m = metrics or trajectory_metrics([tuple(r[:5]) for r in a])
    circ, cmd, ab = m.get('circle'), ctx.get('command') or {}, ctx.get('ab_check') or {}
    if ref is None and (run / 'summary.json').exists():
        import json
        ref = json.loads((run / 'summary.json').read_text(encoding='utf-8')).get('reference')

    # ---------------------------------------------- bang dat - do - sai lech
    def fmt(v, d=1):
        return '—' if v is None or not np.isfinite(v) else f'{v:.{d}f}'
    def row(name, set_v, meas, d=1):
        if set_v is None or meas is None:
            return (name, fmt(set_v, d), fmt(meas, d), '—')
        err = meas - set_v
        return (name, fmt(set_v, d), fmt(meas, d), f'{err:+.{d}f} ({100 * err / set_v:+.{2 if abs(100 * err / set_v) < 1 else 1}f} %)' if set_v else f'{err:+.{d}f}')
    R_set, A_set = cmd.get('radius_mm'), cmd.get('angle_deg')
    turn = abs(m.get('heading_change_deg', 0.))
    if ref and ref.get('center_set_mm'):
        ct = ref['cross_track_mm']
        rows = [('Robot so với đường đặt', 'đặt', 'đo', 'sai lệch'),
                row('Bán kính R (mm)', R_set, circ['radius_mm'] if circ else None),
                row('Góc quét quanh tâm đặt (°)', A_set, abs(ref['swept_about_set_center_deg'])),
                row('Góc thân xoay (°)', A_set, abs(ref['heading_turn_deg'])),
                ('Lệch khỏi đường đặt (mm)', '0', f'TB {ct["mean"]:+.1f}', f'RMS {ct["rms"]:.1f} · max {ct["max_abs"]:.0f}'),
                ('Điểm cuối cách điểm cuối đặt (mm)', '', fmt(ref.get('end_error_mm')), '')]
    elif ref:
        d = ref['drift_mm']
        rows = [('Robot so với lệnh xoay', 'đặt', 'đo', 'sai lệch'),
                row('Góc thân xoay (°)', A_set, abs(ref['heading_turn_deg'])),
                ('Tâm thân trôi khỏi điểm đầu (mm)', '0', f'TB {d["mean"]:.1f}', f'95% {d["p95"]:.0f} · max {d["max"]:.0f}')]
    else:
        rows = [('Robot so với lệnh', 'đặt', 'đo', 'sai lệch'),
                row('Bán kính R (mm)', R_set, circ['radius_mm'] if circ else None),
                row('Góc quét quanh tâm (°)', A_set if circ else None, abs(circ['swept_deg']) if circ else None),
                row('Góc thân xoay (°)', A_set, turn)]
    rows += [('Chiều quay', '', 'trái (CCW)' if m.get('heading_change_deg', 0) > 0 else 'phải (CW)',
              '' if not ref else ('đúng lệnh' if ref['direction_ok'] else 'NGƯỢC lệnh')),
             ('Thời gian lượt (s)', '', fmt(m.get('duration_s'), 0), ''),
             ('Hệ đo (camera)', 'chuẩn', 'đo', 'sai lệch')]
    res = (ab or {}).get('result')
    rows.append(row('Kiểm tra A–B (mm)', res['expected_mm'], res['measured_mm']) if res else ('Kiểm tra A–B (mm)', '—', 'chưa đo', ''))
    ae = np.abs(edge[np.isfinite(edge)])
    rows.append(('Cạnh marker 50 mm (%)', '0', fmt(float(np.mean(edge[np.isfinite(edge)])), 2) if len(ae) else '—',
                 f'95% |.| ≤ {np.percentile(ae, 95):.2f}' if len(ae) else '—'))
    X0, Y0, cw = 20, 44, (128, 44, 58, 96)
    bottom = max(232, Y0 + len(rows) * 15.5 + 48)       # bang dai thi day hai o duoi xuong
    c = Canvas(600, int(bottom + 148))
    c.text(20, 20, 'Đánh giá sai số · ' + _when(run) + (' · ' + ctx['note'] if ctx.get('note') else ''), 10.5, '#111', dam=True)
    for i, r in enumerate(rows):
        yy = Y0 + i * 15.5
        head = r[1] in ('đặt', 'chuẩn')
        if head:
            c.rect(X0 - 4, yy - 11, sum(cw) + 8, 15, to='#e8f1f0')
        xx = X0
        for j, cell in enumerate(r):
            c.text(xx + (0 if j == 0 else cw[j] - 4), yy, str(cell), 7.8, '#111', 'start' if j == 0 else 'end', dam=head)
            xx += cw[j]
    c.text(X0, Y0 + len(rows) * 15.5 + 2, 'Góc đặt: đi vòng = N×STEP/R (hoặc 360° nếu chỉ so một vòng); xoay = ANGLE_DEG. So theo độ lớn.', 7, '#666')
    if ref:
        an = ref['anchor']
        c.text(X0, Y0 + len(rows) * 15.5 + 22, f'Đường đặt neo tại tư thế đứng yên {"lúc bấm bắt đầu bước" if an["anchor"] == "nut_bat_dau_buoc" else "đầu lượt ghi"} '
               f'(t = {an["t0_s"]:.1f} s, dao động {an["still_p95_mm"]:.1f} mm).', 7, '#666')
    c.text(X0, Y0 + len(rows) * 15.5 + 12, 'Sai số cạnh marker: marker không dùng để lấy thang đo, nên đây là phép tự kiểm độc lập.', 7, '#666')

    # ---------------------------------------------- phan bo do lech
    after = t >= (ref['anchor']['t0_s'] if ref else -1)
    if ref and ref.get('center_set_mm'):
        rc = np.asarray(ref['center_set_mm'])
        e = np.hypot(x[after] - rc[0], y[after] - rc[1]) - ref['radius_set_mm']
        _histogram(c, 385, 52, 195, 118, e, 'lệch khỏi đường đặt (mm)', 'Phân bố lệch khỏi đường đặt (+ ra ngoài)')
    elif ref:
        q0 = np.asarray(ref['anchor']['p0_mm'])
        _histogram(c, 385, 52, 195, 118, np.hypot(x[after] - q0[0], y[after] - q0[1]), 'trôi khỏi điểm đầu (mm)',
                   'Phân bố độ trôi tâm thân khi xoay')
    elif circ:
        cc, r = np.asarray(circ['center_mm']), circ['radius_mm']
        dev = np.hypot(x - cc[0], y - cc[1]) - r
        _histogram(c, 385, 52, 195, 118, dev, 'r − R khớp (mm)',
                   f'Lệch khỏi đường tròn khớp · RMS {float(np.sqrt(np.mean(dev ** 2))):.1f} mm')
    else:
        med = np.median(np.c_[x, y], axis=0)
        _histogram(c, 385, 52, 195, 118, np.hypot(x - med[0], y - med[1]), 'cách vị trí trung vị (mm)',
                   f'Tâm thân trôi khi xoay · 95% ≤ {m.get("position_spread_mm", 0):.0f} mm')

    # ---------------------------------------------- tu kiem he do theo thoi gian
    _series(c, 58, bottom, 250, 100, t, edge, 'thời gian (phút)', 'sai số cạnh (%)',
            'Sai số cạnh marker theo thời gian (vùng xanh ±1 %)', band=1.0)
    _series(c, 385, bottom, 195, 100, t, out, 'thời gian (phút)', 'ngoài vùng (mm)',
            f'Cách vùng bàn cờ · 95% ≤ {np.nanpercentile(out, 95):.0f} mm, xa nhất {np.nanmax(out):.0f} mm', floor0=True, smooth=False)
    c.text(20, bottom + 140, 'Ô sai số cạnh · nhạt: từng mẫu, đậm: trung bình mỗi 14 s (một chu kỳ bước). Mọi số liệu lấy từ samples.csv và context.json của chính lượt này.', 7, '#666')
    return c.luu(str(run / 'sai_so'))


if __name__ == '__main__':
    # Ve lai hinh tu mot thu muc luot da ghi:  python circle_figure.py <circle_runs/.../run_...>
    import csv, json
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tmp' / 'pdfs' / 'aruco_deps'))
    from circle_geometry import trajectory_metrics
    run = Path(sys.argv[1])
    rows = [r for r in csv.DictReader(open(run / 'samples.csv', encoding='utf-8-sig')) if r['detected'] == 'True']
    S = [(float(r['elapsed_s']), float(r['x_mm']), float(r['y_mm']), math.radians(float(r['heading_deg'])),
          float(r['extrapolation_warning'] == 'True')) for r in rows]
    ctx = json.loads((run / 'context.json').read_text(encoding='utf-8'))
    hull = ctx['context']['plane'].get('board_hull_plane_mm')
    summary = json.loads((run / 'summary.json').read_text(encoding='utf-8')) if (run / 'summary.json').exists() else {}
    title = 'Quỹ đạo tâm thân · ' + _when(run, summary) + (' · ' + ctx['note'] if ctx.get('note') else '')
    print(make(run / 'quy_dao', S, hull, trajectory_metrics(S), title, ref=summary.get('reference')))
    print(make_errors(run))
