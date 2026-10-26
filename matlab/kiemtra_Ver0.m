function R = kiemtra_Ver0(out,verbose)
% KIEMTRA_VER0  Kiem tra du lieu Simulink; khong tu thay doi model/tham so.
% Run model truoc, sau do: R=kiemtra_Ver0(out). Don vi bao cao: mm, deg, s.
if nargin<1,out=evalin('base','out');end
if nargin<2,verbose=true;end
assert(isa(out,'Simulink.SimulationOutput'),'Ver0: Run Spider_4leg_line_Ver0 truoc.');
D.t=out.tout(:);N=numel(D.t);
assert(N>1,'Ver0: simulation output has fewer than two samples.');
snap=series(out.parameter_snapshot);P=thamso_Ver0(snap(end,:));
D.P=P;D.pose=series(out.P_real);D.foot=series(out.Pfoot_r);
D.hip=series(out.Phip_r);D.body=series(out.Pc_body);
D.ref=series(out.com_ref);D.com=series(out.com_real);
D.contact=series(out.contact)>0.5;D.gait=series(out.gait);
D.q=cat(3,series(out.theta1),series(out.theta2),series(out.theta3));
D.servo=zeros(N,4,3);D.margin=zeros(N,1);D.bodyMargin=zeros(N,1);
D.previewMargin=zeros(N,1);D.toe=zeros(N,4,3);
fkErr=0;hipErr=0;lengthErr=0;comErr=0;minReach=inf;
for k=1:N
    foot=reshape(D.foot(k,:),4,3);hip=reshape(D.hip(k,:),4,4);
    points=reshape(D.pose(k,:),4,4,4); % coordinate x joint x leg
    toe=squeeze(points(1:3,4,:))';D.toe(k,:,:)=toe;
    actualHip=squeeze(points(1:3,1,:))';
    fkErr=max(fkErr,max(vecnorm(toe-foot,2,2)));
    hipErr=max(hipErr,max(vecnorm(actualHip-hip(:,1:3),2,2)));
    weighted=zeros(1,3);yaw=D.body(k,4);
    rot=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    for i=1:4
        p=points(1:3,:,i)';
        lengthErr=max(lengthErr,max(abs(vecnorm(diff(p),2,2)'-P.L)));
        weighted=weighted+sum(((p(1:3,:)+p(2:4,:))/2).*P.MASS_LINK',1);
        dxy=(rot'*(toe(i,1:2)-actualHip(i,1:2))')';
        mirrored=dxy.*sign(P.HIP(i,:));
        q=[atan2(mirrored(2),mirrored(1))*180/pi, ...
            D.q(k,i,2)*180/pi,D.q(k,i,3)*180/pi]+P.JOINT_TRIM(i,:);
        if i==1 || i==3,s=[q(1),90+q(2),-q(3)];
        else,s=[180-q(1),90-q(2),180+q(3)];end
        D.servo(k,i,:)=s;
        r=hypot(dxy(1),dxy(2));z=toe(i,3)-actualHip(i,3);
        c=((r-P.L(1))^2+z^2-P.L(2)^2-P.L(3)^2)/(2*P.L(2)*P.L(3));
        minReach=min(minReach,1-abs(c));
    end
    off=rot*P.COM_OFFSET(1:2)';
    bc=mean(actualHip,1)+[off',P.COM_OFFSET(3)];
    expectedCOM=(weighted+P.MASS_BODY*bc)/(P.MASS_BODY+4*sum(P.MASS_LINK));
    comErr=max(comErr,norm(expectedCOM-D.com(k,1:3)));
    D.margin(k)=signedMargin(toe(D.contact(k,:),1:2),D.com(k,1:2));
    D.bodyMargin(k)=signedMargin(toe(D.contact(k,:),1:2),D.body(k,1:2));
    preview=true(1,4);preview(D.gait(k,1))=false;
    D.previewMargin(k)=signedMargin(toe(preview,1:2),D.com(k,1:2));
end
% Drift is measured against the FIRST sample in each uninterrupted stance.
% A foot is allowed to acquire a new ground point AFTER its swing.
anchorDrift=0;pairDrift=0;groundErr=0;stanceSpeed=0;
for i=1:4
    toe=reshape(D.toe(:,i,:),N,3);mask=D.contact(:,i);anchor=[];
    groundErr=max(groundErr,max(abs(toe(mask,3))));
    for k=1:N
        if ~mask(k),anchor=[];continue;end
        if isempty(anchor),anchor=toe(k,:);end
        anchorDrift=max(anchorDrift,norm(toe(k,:)-anchor));
        if k>1 && mask(k-1)
            stanceSpeed=max(stanceSpeed,norm(toe(k,:)-toe(k-1,:))/(D.t(k)-D.t(k-1)));
        end
    end
end
for i=1:3
    for j=i+1:4
        dist=vecnorm(reshape(D.toe(:,i,:)-D.toe(:,j,:),N,3),2,2);
        mask=D.contact(:,i)&D.contact(:,j);anchor=[];
        for k=1:N
            if ~mask(k),anchor=[];continue;end
            if isempty(anchor),anchor=dist(k);end
            pairDrift=max(pairDrift,abs(dist(k)-anchor));
        end
    end
end
lift=zeros(1,4);events=[];
for i=1:4,lift(i)=max(D.toe(:,i,3));end
for k=2:N
    if D.gait(k,3)>0 && D.gait(k-1,3)==0,events(end+1)=D.gait(k,3);end %#ok<AGROW>
end
jointMin=reshape(min(D.servo,[],1),4,3);
jointMax=reshape(max(D.servo,[],1),4,3);
rates=abs(diff(D.servo,1,1)./reshape(diff(D.t),[],1,1));
finite=all(isfinite(D.pose),'all')&&all(isfinite(D.q),'all')&&isreal(D.q);
R.samples=N;R.duration_s=D.t(end)-D.t(1);R.cycles=R.duration_s/P.T;
R.fk_error_mm=fkErr*1000;R.hip_error_mm=hipErr*1000;
R.link_length_error_mm=lengthErr*1000;R.com_error_mm=comErr*1000;
R.stance_anchor_drift_mm=anchorDrift*1000;R.support_pair_drift_mm=pairDrift*1000;
R.stance_speed_mm_s=stanceSpeed*1000;R.ground_error_mm=groundErr*1000;
R.min_support_count=min(sum(D.contact,2));R.swing_sequence=events;
R.max_lift_mm=lift*1000;R.min_com_margin_mm=min(D.margin)*1000;
R.min_body_margin_mm=min(D.bodyMargin)*1000;
R.servo_min_deg=jointMin;R.servo_max_deg=jointMax;
R.max_servo_rate_deg_s=max(rates,[],'all');R.ik_cosine_clearance=minReach;
R.finite=finite;
R.kinematics_pass=finite && max([fkErr,hipErr,lengthErr,comErr,anchorDrift,pairDrift,groundErr])<1e-8 ...
    && R.min_support_count>=3 && minReach>0;
R.screening_pass=R.min_com_margin_mm>=1000*P.MARGIN_GUIDE ...
    && all(jointMin>=P.SERVO_LO,'all') && all(jointMax<=P.SERVO_HI,'all');
R.pass=R.kinematics_pass && R.screening_pass;
R.note='COM uses assumed masses; contacts are scheduled. This is not a force, motor or hardware test.';
R.data=D;
if verbose
    fprintf('\nVER0 | %.2f s | %d samples | %g cycles\n',R.duration_s,N,R.cycles);
    fprintf('Kinematics pass: %d | COM/servo screening pass: %d\n',R.kinematics_pass,R.screening_pass);
    fprintf('FK error %.3g mm | stance drift %.3g mm | pair drift %.3g mm\n',R.fk_error_mm,R.stance_anchor_drift_mm,R.support_pair_drift_mm);
    fprintf('COM margin min %.3f mm | body margin min %.3f mm | supports >= %d\n',R.min_com_margin_mm,R.min_body_margin_mm,R.min_support_count);
    fprintf('Lift max [FL FR RR RL]: %s mm\n',mat2str(R.max_lift_mm,4));
    fprintf('Servo preview range %.2f..%.2f deg | max rate %.2f deg/s\n',min(jointMin,[],'all'),max(jointMax,[],'all'),R.max_servo_rate_deg_s);
    fprintf('Swing order: %s\n%s\n\n',mat2str(events),R.note);
end
end

function x=series(ts)
n=numel(ts.Time);
if ts.IsTimeFirst,x=reshape(ts.Data,n,[]);else,x=reshape(ts.Data,[],n)';end
end

function d=signedMargin(p,c)
if size(p,1)<3,d=-inf;return;end
h=convhull(p(:,1),p(:,2));v=p(h(1:end-1),:);w=v([2:end 1],:);
% convhull returns a counterclockwise polygon: left normals point inside.
e=w-v;n=[-e(:,2),e(:,1)]./vecnorm(e,2,2);
d=min(sum(n.*(c-v),2));
end
