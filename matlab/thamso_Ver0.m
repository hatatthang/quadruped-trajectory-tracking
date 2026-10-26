function P = thamso_Ver0(snapshot)
% THAMSO_VER0  Doc thong tin lan chay cho viewer/kiem tra.
% Tham so dieu khien da nam TRUC TIEP trong cac khoi Simulink.
% File nay khong cap tham so cho WPG, IK, FK hay COM.
% Giu ten file cu; cac so 0 ben duoi chi la khuon kich thuoc de doc ban ghi.
if nargin==0
    model = 'Spider_4leg_line_Ver0';
    if ~bdIsLoaded(model),load_system(model);end
    P = evalin(get_param(model,'ModelWorkspace'),'P');
    return;
end
% Chi khai bao KICH THUOC cac truong cua ban ghi, khong dat gia tri dieu khien.
P.FORMAT_VERSION = 1;
P.L = zeros(1,3);
P.HIP = zeros(4,2);
P.STANCE = zeros(1,2);
P.BODY_H = 0;
P.STEP = 0;
P.T = 0;
P.V = 0;
P.LIFT = 0;
P.SHIFT_X = 0;
P.SHIFT_Y = 0;
P.PHI = zeros(1,4);
P.PREP = 0;
P.SWING = 0;
P.TOUCH = 0;
P.HEADING_DEG = 0;
P.DT = 0;
P.N_CYCLES = 0;
P.MASS_BODY = 0;
P.MASS_LINK = zeros(1,3);
P.COM_OFFSET = zeros(1,3);
P.MARGIN_GUIDE = 0;
P.JOINT_TRIM = zeros(4,3);
P.SERVO_LO = zeros(1,3);
P.SERVO_HI = zeros(1,3);
assert(snapshot(1)==P.FORMAT_VERSION,'Ver0: snapshot version mismatch.');
names=fieldnames(P);offset=0;
for k=1:numel(names)
    shape=size(P.(names{k}));n=prod(shape);
    P.(names{k})=reshape(snapshot(offset+(1:n)),shape);
    offset=offset+n;
end
assert(offset==numel(snapshot),'Ver0: snapshot size mismatch.');
end
