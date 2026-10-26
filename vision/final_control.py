"""Camera/COM coordinator. No port opened and no motion until explicit UI actions.

Manual mode renews an operator target. Auto is an opt-in experimental controller
using phase-balanced measurements; it never starts or resumes motion by itself.
"""
from collections import deque
from datetime import datetime
from pathlib import Path
import copy
import json
import math
import threading
import time
from line_advisor import LineAdvisor

BUILD = 202609230
MARKER_TIMEOUT = 2.0
WALK = (3, 4, 5)
STAGES = {0:'HOME',1:'POSE',2:'READY',3:'WALK',4:'WALK',5:'WALK',6:'DONE',
          7:'STAND',8:'WAIT_START',9:'STOPPING',10:'STOPPED'}


def frame(seq, kind, value):
    body = f'{seq},{kind},{value}'
    checksum = 0
    for c in body.encode('ascii'): checksum ^= c
    return f'${body}*{checksum:02X}\r\n'.encode('ascii')


def decode(raw):
    text = raw.decode('ascii').strip()
    body, check = text[1:].split('*')
    if not text.startswith('$') or len(check) != 2:
        raise ValueError('frame')
    checksum = 0
    for c in body.encode('ascii'): checksum ^= c
    if checksum != int(check, 16): raise ValueError('checksum')
    f = body.split(',')
    if f[1] == 'A' and len(f) == 3:
        return {'type':'A','seq':int(f[0]),'code':int(f[2])}
    if f[1] != 'D' or len(f) not in (10,13): raise ValueError('fields')
    nums = [int(x) for x in [f[0], *f[2:]]]
    if any(abs(x)>2147483647 for x in nums): raise ValueError('range')
    return {'type':'D','seq':nums[0], 'gait_s':nums[1]/100, 'x_ref_mm':nums[2]/10,
            'y_ref_mm':nums[3]/10, 'yaw_ref_deg':nums[4]/100, 'trim':nums[5]/1000,
            'stage':nums[6], 'fault':nums[7], 'alive':nums[8],
            'build':nums[9] if len(f)==13 else None,
            'cycles':nums[10] if len(f)==13 else None,
            'target':nums[11]/1000 if len(f)==13 else None}


class FinalControl:
    def __init__(self, observer, root, transport_factory=None):
        self.observer, self.root = observer, Path(root)
        self.factory = transport_factory
        self.lock = threading.RLock()
        self.thread = None
        self.quit = threading.Event()
        self.serial = None
        self.mode = 'manual'
        self.port = 'COM4'
        self.tele = None
        self.history = deque(maxlen=40)
        self.events = deque(maxlen=12)
        self.seq = 1
        self.pending = None
        self.commands = deque()
        self.authorized = False
        self.awaiting_start = False
        self.stop_latched = False
        self.target = 2.
        self.error = ''
        self.log = None
        self.log_path = None
        self.last_send = 0.
        self.last_frame = None
        self.valid_samples = deque(maxlen=3)
        self.last_valid = None
        self.setup_id = None
        self.bad_frames = 0
        self.advisor = LineAdvisor()
        self.advice_enabled = False
        self.last_sent_trim = None
        self.logged_setup = None
        self.manual_until = 0.
        self.manual_target = None
        self.manual_reached = None
        self.auto_pending_boundary = None
        self.auto_boundary = None
        self.marker_state = 'waiting'
        self.control_message = 'Điều khiển bằng tay.'

    def camera(self, now=None):
        now = time.monotonic() if now is None else now
        with self.observer.lock:
            row = copy.deepcopy(self.observer.latest or {})
            setup = copy.deepcopy(self.observer.setup)
        age = now-row.get('mono',0)
        fresh = bool(row.get('detected') and 0<=age<=.6 and
                     all(isinstance(row.get(k),(int,float)) and math.isfinite(row[k]) for k in ('x_mm','y_mm','heading_deg')))
        return row, setup, fresh, age

    def state(self):
        now=time.monotonic()
        row,setup,fresh,age=self.camera(now)
        with self.lock:
            t=copy.deepcopy(self.tele)
            if t: t['age_s']=max(0,now-t['mono']);t['stage_name']=STAGES.get(t['stage'],'?')
            body = fresh and row.get('body_offset_applied') and setup and setup.get('offset_confirmed')
            advice = self.advisor.state(now, self.tele, body) if self.advice_enabled else None
            return {'connected':self.serial is not None,'connecting':bool(self.thread and self.thread.is_alive() and self.serial is None),
                    'port':self.port,'mode':self.mode,'telemetry':t,'target':self.target,
                    'authorized':self.authorized,'error':self.error,'events':list(self.events),
                    'log_path':str(self.log_path) if self.log_path else None,'bad_frames':self.bad_frames,
                    'estimate':None,'advice_enabled':self.advice_enabled,'advice':advice,
                    'last_sent_trim':copy.deepcopy(self.last_sent_trim),
                    'control_message':self.control_message,'marker_state':self.marker_state,
                    'manual_until_gait_s':self.manual_until,'marker_timeout_s':MARKER_TIMEOUT,
                    'measurement':{'fresh':fresh,'detected':bool(row.get('detected')),
                         'age_s':age if row.get('mono') else None,'time_unix':row.get('time_unix'),
                         'body':bool(row.get('body_offset_applied')),
                         'x_mm':row.get('x_mm') if fresh else None,'y_mm':row.get('y_mm') if fresh else None,
                         'heading_deg':row.get('heading_deg') if fresh else None,
                         'reason':row.get('reason','')},'build_required':BUILD}

    def note(self, message):
        self.events.append(datetime.now().strftime('%H:%M:%S')+' '+message)
        self.record('event',{'message':message})

    def record(self, kind, data):
        if self.log:
            self.log.write(json.dumps({'time':time.time(),'mono':time.monotonic(),'type':kind,**data},ensure_ascii=False,allow_nan=False)+'\n')
            self.log.flush()

    def ready(self, now):
        t=self.tele
        return bool(t and 0<=now-t['mono']<.6 and t['fault']==0 and t['build']==BUILD)

    def request_stop(self, message):
        if not self.stop_latched:
            self.commands.clear()
            self.commands.appendleft(('G',0))
            self.stop_latched=True
            self.authorized=False;self.awaiting_start=False
            self.note(message)

    def action(self, action, payload):
        if action=='connect':
            with self.lock:
                if self.thread and self.thread.is_alive(): raise ValueError('UART đang kết nối.')
                import re
                port=str(payload.get('port','COM4')).upper()
                if not re.fullmatch(r'COM[1-9][0-9]*',port): raise ValueError('Tên cổng phải là COM4, COM5…')
                self.port=port;self.error='';self.tele=None;self.history.clear()
                self.pending=None;self.commands.clear();self.authorized=False;self.awaiting_start=False;self.stop_latched=False
                self.last_frame=None;self.valid_samples.clear();self.last_valid=None
                self.advisor.reset();self.last_sent_trim=None;self.logged_setup=None
                self.quit.clear()
                self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
            return 'Đang kết nối UART; chưa ra lệnh chuyển động.'
        if action=='disconnect':
            self.close()
            return 'Đã ngắt UART; đã yêu cầu dừng, cần kiểm tra robot thực tế.'
        row,setup,fresh,age=self.camera()
        with self.lock:
            now=time.monotonic()
            if action=='advice':
                if self.mode=='auto': raise ValueError('Chế độ tự động luôn cần phân tích theo pha.')
                enabled = payload.get('enabled')
                if not isinstance(enabled, bool): raise ValueError('Cần chọn bật/tắt phân tích.')
                if enabled and payload.get('period_s') != 12:
                    raise ValueError('Phân tích này chỉ dành cho GAIT_T=12 s; chưa hỗ trợ chu kỳ khác.')
                self.advice_enabled=enabled;self.advisor.reset()
                self.note('Bật phân tích chỉ đề xuất (12 s).' if enabled else 'Tắt phân tích đề xuất.')
                return 'Chỉ phân tích và ghi nhật ký; không gửi trim tự động.'
            if action=='mode':
                if self.authorized or (self.tele and self.tele['stage'] in (*WALK,9)):
                    self.note('Từ chối đổi chế độ: robot đang chạy hoặc đã nhận lệnh chạy.')
                    raise ValueError('Tạm dừng robot trước khi đổi chế độ.')
                mode=payload.get('mode')
                if mode=='auto' and payload.get('period_s')!=12:
                    self.note('Từ chối Auto: chưa xác nhận chu kỳ gait 12 giây.')
                    raise ValueError('Xác nhận GAIT_T = 12 giây trước khi chọn tự động.')
                if mode not in ('manual','auto'): raise ValueError('Chế độ không hợp lệ.')
                previous=self.mode
                self.mode=mode;self.valid_samples.clear()
                self.advisor.reset();self.auto_boundary=None;self.manual_until=0.
                self.manual_target=None;self.manual_reached=None;self.auto_pending_boundary=None
                self.marker_state='waiting'
                if mode=='auto':self.advice_enabled=True
                self.control_message='Điều khiển bằng tay.' if mode=='manual' else 'Auto đã chọn; chờ dữ liệu camera và lệnh bắt đầu.'
                self.note('Đổi chế độ '+previous+' -> '+mode+'.')
                return 'Đã chọn '+('trim bằng tay.' if mode=='manual' else 'vòng kín thử nghiệm; chưa chạy.')
            if not self.serial: raise ValueError('Kết nối UART trước.')
            if action=='stop':
                self.request_stop('Yêu cầu G 0: tạm dừng theo pha.')
                return 'Đã xếp lệnh G 0; chờ MCU báo STOPPED.'
            if not self.ready(now): raise ValueError('Cần telemetry mới, fault=0 và firmware build '+str(BUILD)+'.')
            if action=='start':
                if self.pending or self.commands: raise ValueError('Chờ phản hồi lệnh đang gửi rồi thử lại.')
                if self.tele['stage'] not in (8,10): raise ValueError('Chỉ bắt đầu/tiếp tục ở WAIT_START hoặc STOPPED; DONE cần reset.')
                if self.mode=='auto' and not (fresh and row.get('body_offset_applied') and setup and setup.get('offset_confirmed') and len(self.valid_samples)==3 and self.valid_samples[-1]-self.valid_samples[0]>=.3):
                    raise ValueError('Cần 3 mẫu tâm thân mới liên tiếp, đã xác nhận hệ trục và bù tâm.')
                self.target=self.tele['trim'] if self.tele['stage']==10 else 2.
                self.advisor.reset();self.auto_boundary=None;self.manual_until=0.
                self.manual_target=None;self.manual_reached=None;self.auto_pending_boundary=None
                self.last_valid=now;self.marker_state='tracking'
                self.setup_id=(setup or {}).get('created_at')
                self.authorized=True;self.awaiting_start=True;self.stop_latched=False
                self.commands.append(('G',1))
                return 'Đã xếp lệnh bắt đầu/tiếp tục; chờ ACK và trạng thái WALK.'
            if action=='trim':
                v=float(payload.get('trim',2))
                if not math.isfinite(v) or not -5<=v<=10: raise ValueError('Trim trong [-5,10].')
                if not self.authorized or self.tele['stage'] not in WALK: raise ValueError('Bắt đầu WALK từ trang này trước.')
                if self.tele['gait_s'] >= (self.tele['cycles']-1)*12:
                    return 'Chu kỳ cuối: giữ trim đang áp dụng đến DONE; không nhận mục tiêu mới.'
                if self.mode=='auto' and (not fresh or self.marker_state!='tracking'):
                    raise ValueError('Đang chờ marker ổn định; giữ trim đang áp dụng.')
                self.target=round(v,3)
                self.manual_target=self.target;self.manual_reached=None;self.auto_pending_boundary=None
                steps=max(1,math.ceil(abs(self.target-self.tele['trim'])/3))
                self.manual_until=(math.floor(self.tele['gait_s']/12)+steps+1)*12
                self.note(f'Lệnh tay ưu tiên: {self.target:.3f}; tự động xét lại sau khi đạt mục tiêu và đi thêm 12 s.')
                return 'Đã đổi mục tiêu T; MCU áp tại ranh giới chu kỳ.'
            raise ValueError('Thao tác UART không hợp lệ.')

    def receive(self, raw, now):
        try: d=decode(raw)
        except (ValueError,UnicodeError,IndexError):
            self.bad_frames+=1;return
        self.record('rx',{'frame':raw.decode('ascii').strip()})
        if d['type']=='A':
            if self.pending and self.pending['seq']==d['seq']:
                p=self.pending;self.pending=None
                acceptable=d['code']==0
                self.record('ack',{'seq':d['seq'],'code':d['code'],'latency_ms':(now-p['mono'])*1000})
                if p['kind']=='G' or not acceptable:self.note(f"ACK {p['kind']} {p['value']}: mã {d['code']}")
                if not acceptable and p['kind'] in ('T','G'):
                    self.request_stop('Lệnh điều khiển bị từ chối; yêu cầu dừng.')
            return
        old=self.tele
        if old and d['alive']<old['alive']:
            self.authorized=False;self.awaiting_start=False;self.pending=None;self.commands.clear()
            self.history.clear();self.valid_samples.clear();self.note('MCU đã khởi động lại; cần bắt đầu chủ động.')
            self.advisor.reset();self.last_sent_trim=None
        d['mono']=now;self.tele=d;self.history.append(d)
        if not old or old['stage']!=d['stage']:
            if not old or STAGES.get(old['stage'])!=STAGES.get(d['stage']):self.note('MCU: '+STAGES.get(d['stage'],'?'))
        if old and old['trim']!=d['trim']:self.note(f"Trim áp dụng: {old['trim']:.3f} → {d['trim']:.3f}")
        if d['stage'] in WALK: self.awaiting_start=False
        if d['stage'] in (6,9) or (d['stage']==10 and not self.awaiting_start) or d['fault']:
            self.authorized=False
        if d['stage'] not in WALK:
            self.advisor.invalidate('Robot chưa WALK hoặc đang dừng; thu lại cửa sổ khi đi tiếp.')
        if d['fault'] and not self.stop_latched:self.request_stop('MCU báo lỗi; ngừng gia hạn điều khiển.')

    def step(self, now):
        row,setup,fresh,age=self.camera(now)
        body=fresh and row.get('body_offset_applied') and setup and setup.get('offset_confirmed')
        key=row.get('frame_id', row.get('mono'))
        if key is not None and key!=self.last_frame:
            self.last_frame=key
            setup_id=(setup or {}).get('created_at')
            if setup_id!=self.logged_setup:
                self.logged_setup=setup_id
                self.record('camera_setup',{'setup':setup})
                self.advisor.reset()
            matches=[t for t in self.history if abs(t['mono']-row['mono'])<=.15]
            matched=copy.deepcopy(min(matches,key=lambda t:abs(t['mono']-row['mono']))) if matches else None
            if matched:
                dt=row['mono']-matched['mono']
                if matched['stage'] in WALK: matched['gait_s']+=dt
                matched['sample_rx_dt_s']=dt
                if matched['gait_s']<0: matched=None
            if body:
                self.last_valid=row['mono'];self.valid_samples.append(row['mono'])
                if self.advice_enabled and matched and matched['build']==BUILD:
                    self.advisor.update(row,matched,setup_id)
                elif self.advice_enabled:
                    self.advisor.invalidate('Chưa ghép được ảnh với telemetry mới đúng phiên bản.')
            else:self.valid_samples.clear()
            if not body and (self.last_valid is None or now-self.last_valid>.8):
                self.advisor.invalidate('Mất mẫu tâm thân; thu lại cửa sổ đủ pha.')
            with self.observer.lock:
                run=copy.deepcopy(getattr(self.observer,'run',None))
            self.record('camera',{'frame_id':row.get('frame_id'),'detected':bool(row.get('detected')),
                'mode':self.mode,'advice_enabled':self.advice_enabled,
                'fresh_body':bool(body),'reason':row.get('reason'),
                'x_mm':row.get('x_mm'),'y_mm':row.get('y_mm'),'heading_deg':row.get('heading_deg'),
                'sample_mono':row.get('mono'),'sample_time_unix':row.get('time_unix'),'age_s':age,
                'line_run_directory':run.get('directory') if run else None,
                'phase':{'gait_s':matched['gait_s'],'cycle':int(matched['gait_s']/12)+1,
                         'phase_s':matched['gait_s']%12,'stage':matched['stage'],
                         'trim_applied':matched['trim'],'sample_rx_dt_s':matched['sample_rx_dt_s'],
                         'clock_basis':'PC receive time; approximate alignment, GAIT_T=12s'} if matched else None,
                'advice':self.advisor.state(now,self.tele,body) if self.advice_enabled else None})
        if not body:self.valid_samples.clear()
        if self.authorized:
            if not self.ready(now):self.request_stop('Telemetry cũ, lỗi hoặc firmware không phù hợp.')
            elif self.mode=='auto':self.auto_step(now, row, setup, body)
        # Stop supersedes pending trim/start; never retry a start automatically.
        if self.commands and self.commands[0]==('G',0):self.pending=None
        if self.pending and now-self.pending['mono']>.5:
            p=self.pending;self.pending=None
            self.note('Quá hạn ACK '+p['kind'])
            if p['kind']!='P':self.request_stop('Không xác nhận được lệnh; ngừng điều khiển.')
        if self.pending:return
        if self.auto_pending_boundary is not None and self.tele:
            gait=self.tele['gait_s']+max(0,now-self.tele['mono'])
            if gait>self.auto_pending_boundary-.5:
                self.target=self.tele['trim'];self.auto_pending_boundary=None
                self.note('Lỡ hạn gửi mục tiêu tự động; giữ trim đang áp dụng.')
        if self.commands:kind,value=self.commands.popleft()
        elif now-self.last_send>=.8:
            if self.authorized and self.tele and self.tele['stage'] in WALK:
                kind,value='T',int(round(self.target*1000))
            else:kind,value='P',0
        else:return
        seq=self.seq;self.seq=(self.seq+1)%2147483647
        self.serial.write(frame(seq,kind,value));self.last_send=now
        if kind=='T':
            self.last_sent_trim={'value':value/1000.,'seq':seq,'mono':now,'time_unix':time.time()}
            self.auto_pending_boundary=None
        self.pending={'seq':seq,'kind':kind,'value':value,'mono':now}
        self.record('tx',{'seq':seq,'kind':kind,'value':value})

    def auto_step(self, now, row, setup, body):
        if (setup or {}).get('created_at')!=self.setup_id:
            self.request_stop('Hệ trục camera thay đổi; cần xác nhận và bắt đầu lại.');return
        gap=now-(self.last_valid if self.last_valid is not None else now-MARKER_TIMEOUT)
        if gap>=MARKER_TIMEOUT:
            self.marker_state='stopped';self.control_message='Mất marker quá 2 giây; dừng theo pha.'
            self.request_stop(self.control_message);return
        stable=body and len(self.valid_samples)==3 and self.valid_samples[-1]-self.valid_samples[0]>=.3
        if not stable:
            if self.marker_state!='holding':
                self.note('Marker thiếu/chưa ổn định: giữ trim đang áp dụng, hủy mục tiêu đang chờ.')
            self.marker_state='holding';self.target=self.tele['trim'];self.manual_until=0.
            self.manual_target=None;self.manual_reached=None;self.auto_pending_boundary=None
            self.control_message='Giữ trim; chờ 3 mẫu marker mới liên tiếp.';return
        self.marker_state='tracking'
        gait=self.tele['gait_s']+max(0,now-self.tele['mono'])
        if gait >= (self.tele['cycles']-1)*12:
            self.target=self.tele['trim'];self.control_message='Chu kỳ cuối: giữ trim đến DONE.';return
        if self.manual_target is not None:
            if abs(self.tele['trim']-self.manual_target)>.001:
                self.control_message=f'Ưu tiên lệnh tay: chờ MCU đạt {self.manual_target:.3f}.';return
            if self.manual_reached is None:self.manual_reached=gait;self.manual_until=gait+12
            if gait<self.manual_until:
                self.control_message=f'Ưu tiên lệnh tay đến {self.manual_until:.1f} s gait.';return
            self.manual_target=None;self.manual_reached=None
        advice=self.advisor.state(now,self.tele,True);p=advice.get('prepared')
        self.control_message='Chờ cửa sổ đủ pha; giữ mục tiêu hiện có.'
        if not (advice['valid'] and p and p['eligible'] and gait<=p['deadline_s'] and self.auto_boundary!=p['boundary_s']):return
        self.target=p['proposed_trim'];self.auto_boundary=p['boundary_s']
        self.auto_pending_boundary=p['boundary_s']
        self.control_message=f'Tự động: trim {self.target:.3f}, dự kiến chu kỳ {p["candidate_cycle"]}.'
        self.last_send=0. # Gui som sau ACK dang cho; khong doi moc ap tren MCU.
        self.record('auto_decision',{'target':self.target,'estimate':p})
        self.note(self.control_message)

    def run(self):
        try:
            if self.factory is None:
                from serial_win32 import SerialPort
                factory=SerialPort
            else:factory=self.factory
            ser=factory(self.port)
            with self.lock: self.serial=ser
            folder=self.root/'final_runs';folder.mkdir(exist_ok=True)
            with self.lock:
                self.log_path=folder/('uart_camera_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
                self.log=self.log_path.open('w',encoding='utf-8')
                self.serial=ser;self.note('Đã mở '+self.port+'; chưa điều khiển robot.')
            buf=b''
            while not self.quit.is_set():
                data=ser.read();buf+=data
                if len(buf)>16384:buf=b'';self.bad_frames+=1
                with self.lock:
                    while b'\n' in buf:
                        raw,buf=buf.split(b'\n',1)
                        if raw.strip():self.receive(raw,time.monotonic())
                    self.step(time.monotonic())
                self.quit.wait(.02)
        except Exception as exc:
            with self.lock:self.error=str(exc);self.authorized=False;self.note('UART: '+str(exc))
        finally:
            with self.lock:
                if self.serial:
                    try:self.serial.write(frame(self.seq,'G',0))
                    except OSError:pass
                    self.serial.close()
                self.serial=None;self.authorized=False;self.pending=None;self.commands.clear()
                if self.log:self.log.close();self.log=None

    def close(self):
        self.quit.set()
        worker=self.thread
        if worker and worker is not threading.current_thread():worker.join(2)
