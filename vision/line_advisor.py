"""Read-only shadow advice. No serial access, no mutation of a control target.

12s gait contract: one rolling cycle, 12 equal phase bins. Bin medians have
equal weight so slow/fast camera processing does not overweight a leg phase.
This suppresses repeatable sway; it is NOT a validated instantaneous body pose.
"""
from collections import deque
import bisect
import copy
import math
import statistics as st

PERIOD = 12.0
WALK = (3, 4, 5)
ON_Y, ON_H, OFF_Y, OFF_H = 10., 3., 5., 1.5
# Initial design values for shadow inspection, not fitted/validated gains.
BASE, LOOKAHEAD, GAIN = 3., 200., .5
# 23/09: 6 do cho huong ve line qua nhe. Do tren 55 chu ky ngay 23/09, huong
# dich chuyen thuc thap hon goc than khoang 5 do, nen goc than 6 do chi cho
# huong di ~1 do, tuong ung 0.4 mm/chu ky: khong the thu hoi lech lon.
# Noi tran len 10; chua bu 5 do vi con phai phan biet truot that voi moc huong.
MAX_DESIRED = 10.
# Khau tich phan cham tren lech ngang (23/09). Ly do: 10 luot vong ho cung ngay
# cho do loi 0.597 do/ck moi don vi trim, trim trung tinh 1.85 va huong dich
# chuyen thap hon goc than 5.0 do. Chi co P thi diem can bang la
# Y = -LOOKAHEAD*tan(5.0 - (BASE-1.85)/GAIN) ~ -9.6 mm, dung nhu cac luot Auto
# do duoc (-8..-9 mm). BIAS_START la phan bu con thieu da do; KI do not phan
# con lai nen khong phu thuoc do chinh xac cua ba hang so tren.
# KI = 0 tra ve dung hanh vi truoc ngay 23/09.
KI, KI_GATE, KI_LIMIT, BIAS_START = .02, 20., 6., 2.7
# San/tran trim Auto duoc phep gui. Truoc 23/09 la 0..6; vi trim trung tinh do
# duoc la 1.85 nen mien do lech han mot ben: quay trai toi da 0.597*(6-1.85)
# = +2.48 do/ck, quay phai chi 0.597*(0-1.85) = -1.10 do/ck. Luot lech 43.8 mm
# ngay 23/09 cham san 0 lien 16/18 chu ky va khong kip ve. Ha san xuong -5, la
# dung gioi han firmware (TRIM_MIN -5.0; duoi nua bi ACK 4 va web yeu cau dung).
AUTO_MIN, AUTO_MAX = -2., 6.


def wrap(deg):
    return (deg + 180.) % 360. - 180.


class LineAdvisor:
    def __init__(self):
        self.reset()

    def reset(self):
        self.rows = deque(maxlen=500)
        self.active = False
        self.bias = BIAS_START if KI else 0.
        self.last_step = 0.
        self.last_valid = None
        self.last_gait = None
        self.persistent_since = None
        self.setup_id = None
        self.estimate = None
        self.prepared = None
        self.message = 'Chờ camera tâm thân và telemetry trên /final.'

    def invalidate(self, message, keep_bias=False):
        # Mat mau thoang qua thi giu phan bu da hoc; doi luot/he truc thi hoc lai.
        if not keep_bias:
            self.bias = BIAS_START if KI else 0.
        self.last_step = 0.
        self.rows.clear()
        self.active = False
        self.persistent_since = None
        self.estimate = self.prepared = None
        self.last_gait = self.last_valid = None
        self.message = message

    def update(self, row, telemetry, setup_id):
        if setup_id != self.setup_id:
            self.invalidate('Hệ trục đổi; thu lại cửa sổ đủ pha.')
            self.setup_id = setup_id
        g, stamp = telemetry['gait_s'], row['mono']
        if telemetry['stage'] not in WALK or telemetry['fault']:
            self.invalidate('Robot chưa WALK hoặc đang dừng; chưa xét bù.')
            return
        self.last_step = min(g-self.last_gait,.8) if self.last_gait is not None else 0.
        if self.last_gait is not None:
            if g <= self.last_gait:
                return
            # Do not bridge marker outages or pauses with apparently good means.
            if stamp-self.last_valid > .8 or g-self.last_gait > .8:
                self.invalidate('Mẫu bị gián đoạn; thu lại cửa sổ đủ pha.', keep_bias=True)
        self.last_valid, self.last_gait = stamp, g
        self.rows.append({'g':g,'mono':stamp,'x':row.get('x_mm'),'y':row['y_mm'],'h':row['heading_deg']})
        while self.rows and self.rows[0]['g'] < g-2*PERIOD-.3:
            self.rows.popleft()
        window = [r for r in self.rows if g-PERIOD < r['g'] <= g]
        bins = [[] for _ in range(12)]
        for r in window:
            bins[int(r['g'] % PERIOD)].append(r)
        if g-window[0]['g'] < 11.5 or any(len(b)<2 for b in bins):
            self.estimate = None
            self.message = 'Đang gom đủ 12 s / 12 nhóm pha; chưa kết luận cần bù.'
            return
        origin = window[-1]['h']
        if max(abs(wrap(r['h']-origin)) for r in window) > 45:
            self.invalidate('Góc thay đổi quá lớn trong cửa sổ; cần kiểm tra phép đo.')
            return
        y = st.fmean(st.median(r['y'] for r in b) for b in bins)
        h = wrap(origin+st.fmean(st.median(wrap(r['h']-origin) for r in b) for b in bins))
        outside = abs(y)>ON_Y or abs(h)>ON_H
        if outside:
            if self.persistent_since is None: self.persistent_since = g
            if g-self.persistent_since >= .5: self.active = True
        else:
            self.persistent_since = None
            if abs(y)<OFF_Y and abs(h)<OFF_H: self.active = False
        # Chi tich khi dang hieu chinh va con o gan, de khong bao hoa luc lech lon.
        # KI tinh theo MOT CHU KY, khong theo tung mau: update() duoc goi khoang
        # 60 lan moi chu ky nen phai nhan ti le thoi gian giua hai mau.
        if self.active and KI and abs(y) < KI_GATE and self.last_step:
            self.bias = max(-KI_LIMIT,min(KI_LIMIT,self.bias-KI*y*self.last_step/PERIOD))
        desired = max(-MAX_DESIRED,min(MAX_DESIRED,math.degrees(-math.atan2(y,LOOKAHEAD))+self.bias))
        # Ngoai vung hieu chinh van giu phan bu: do la mo hinh ve do lech chuyen
        # dong cua robot, khong phai phan ung voi sai lech tuc thoi.
        hold = max(AUTO_MIN,min(AUTO_MAX,BASE+GAIN*(self.bias-h))) if KI else BASE
        proposed = max(AUTO_MIN,min(AUTO_MAX,BASE+GAIN*(desired-h))) if self.active else hold
        # This is a hypothetical next boundary value, never sent to the MCU.
        next_trim = telemetry['trim']+max(-3.,min(3.,proposed-telemetry['trim']))
        old = [r for r in self.rows if abs(r['g']-(g-PERIOD))<=.15]
        prior = min(old,key=lambda r:abs(r['g']-(g-PERIOD))) if old else None
        # Do lech cheo = huong DI CHUYEN tru goc THAN. Moi cap diem cung pha cach
        # nhau mot chu ky cho mot uoc luong; mot cap co sai so ~6 do vi nhieu vi
        # tri, nen lay trung vi cua het cap trong cua so, con khoang 0.7 do.
        crab, goc = None, []
        gs = [r['g'] for r in self.rows]
        for r in window:
            if r.get('x') is None:
                continue
            k = bisect.bisect_left(gs, r['g']-PERIOD)
            for j in (k-1, k):
                if 0 <= j < len(self.rows) and abs(self.rows[j]['g']-(r['g']-PERIOD)) <= .15:
                    p = self.rows[j]
                    if p.get('x') is not None and r['x']-p['x'] > 10.:
                        goc.append(math.degrees(math.atan2(r['y']-p['y'], r['x']-p['x'])))
                    break
        course = wrap(st.median(goc)) if len(goc) >= 12 else None
        if course is not None:
            crab = wrap(course-h)
        boundary = (math.floor(g/PERIOD)+1)*PERIOD
        time_left = boundary-g
        self.estimate = {'mono':stamp,'gait_s':g,'window_start_s':window[0]['g'],
            'window_end_s':g,'lag_s':(g-window[0]['g'])/2,'bins':12,'samples':len(window),
            'y_mm':y,'heading_deg':h,'y_span_mm':max(r['y'] for r in window)-min(r['y'] for r in window),
            'heading_span_deg':max(wrap(r['h']-origin) for r in window)-min(wrap(r['h']-origin) for r in window),
            'same_phase_dy_mm':row['y_mm']-prior['y'] if prior else None,
            'same_phase_dheading_deg':wrap(row['heading_deg']-prior['h']) if prior else None,
            'decision':'correct' if self.active else ('confirm' if outside else 'hold'),
            'proposed_trim':round(proposed,3),'next_trim':round(next_trim,3),
            'delta_from_base':round(proposed-BASE,3),'boundary_s':boundary,
            'deadline_s':boundary-.5,'seconds_to_boundary':time_left,
            'candidate_cycle':int(boundary/PERIOD)+1,'desired_heading_deg':desired,
            'bias_deg':round(self.bias,3),
            'crab_deg':round(crab,2) if crab is not None else None,
            'course_deg':round(course,2) if course is not None else None}
        # Prepare once in advance, not at/after the boundary. No late backdating.
        if 1.0 <= time_left <= 1.5 and (not self.prepared or self.prepared['boundary_s']!=boundary):
            if boundary < telemetry['cycles']*PERIOD:
                self.prepared = {**copy.deepcopy(self.estimate),'sent':False}
        self.message = ('Cần hiệu chỉnh (đề xuất)' if self.active else
                        'Chờ xác nhận vượt ngưỡng' if outside else 'Giữ mức nền đề xuất')

    def state(self, now, telemetry, fresh_body):
        fresh_tele = telemetry and 0<=now-telemetry['mono']<=.6
        valid = (fresh_body and fresh_tele and telemetry['stage'] in WALK and
                 not telemetry['fault'] and self.last_valid is not None and now-self.last_valid<=.6)
        prepared = copy.deepcopy(self.prepared)
        if prepared:
            prepared['eligible'] = bool(valid and telemetry['gait_s'] < prepared['boundary_s']-.15)
        return {'shadow_only':True,'valid':bool(valid and self.estimate),
                'message':self.message if valid else 'Chưa có mẫu tâm thân / pha gait mới hợp lệ.',
                'estimate':copy.deepcopy(self.estimate) if valid else None,
                'prepared':prepared if valid else None,
                'config':{'period_s':PERIOD,'on_y_mm':ON_Y,'on_heading_deg':ON_H,
                    'off_y_mm':OFF_Y,'off_heading_deg':OFF_H,'base_trim':BASE,
                    'lookahead_mm':LOOKAHEAD,'gain':GAIN,'prepare_margin_s':.5,
                    'max_desired_deg':MAX_DESIRED,'ki':KI,'ki_gate_mm':KI_GATE,
                    'ki_limit_deg':KI_LIMIT,'bias_start_deg':BIAS_START,
                    'bias_deg':round(self.bias,3)}}
