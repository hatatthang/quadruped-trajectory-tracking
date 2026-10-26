function S = in_bien_COM_Ver0(input)
% IN_BIEN_COM_VER0  Read-only COM margin report for the completed run.
% S=in_bien_COM_Ver0(out); or run without arguments after Simulink Run.
% The viewer can pass the existing kiemtra_Ver0 result to avoid recomputing it.
if nargin<1,input=evalin('base','out');end
if isa(input,'Simulink.SimulationOutput')
    R=kiemtra_Ver0(input,false);
else
    R=input;
end
assert(isstruct(R) && isfield(R,'data'),'Ver0: pass out or a kiemtra_Ver0 result.');
D=R.data;P=D.P;names={'FL','FR','RR','RL'};
[margin,k]=min(D.margin);
support=find(D.contact(k,:));
toe=reshape(D.toe(k,:,:),4,3);
xy=toe(support,1:2);hull=convhull(xy(:,1),xy(:,2));
first=hull(1:end-1);last=hull(2:end);
edges=xy(last,:)-xy(first,:);
normals=[-edges(:,2),edges(:,1)]./vecnorm(edges,2,2);
distances=sum(normals.*(D.com(k,1:2)-xy(first,:)),2);
[edgeMargin,j]=min(distances);
assert(abs(edgeMargin-margin)<1e-10,'Ver0: reported edge differs from the existing margin.');
S.min_margin_mm=margin*1000;S.time_s=D.t(k);S.sample_index=k;
S.com_xyz_mm=D.com(k,1:3)*1000;
S.support_legs=names(support);
S.limiting_edge_legs=names(support([first(j),last(j)]));
S.edge_endpoints_mm=xy([first(j),last(j)],:)*1000;
S.edge_signed_distances_mm=distances'*1000;
S.guide_mm=P.MARGIN_GUIDE*1000;
S.margin_guide_pass=S.min_margin_mm>=S.guide_mm;
S.total_mass_g=1000*(P.MASS_BODY+4*sum(P.MASS_LINK));
S.sample_count=numel(D.t);S.duration_s=D.t(end)-D.t(1);
S.min_3_support_mm=NaN;S.min_4_support_mm=NaN;
counts=sum(D.contact,2);
if any(counts==3),S.min_3_support_mm=min(D.margin(counts==3))*1000;end
if any(counts==4),S.min_4_support_mm=min(D.margin(counts==4))*1000;end
fprintf('\n========== BIEN COM / KET QUA LAN CHAY ==========' );
fprintf('\nT = %.6g s | h = %.6g mm | v = %.6g mm/s | STEP = v*T = %.6g mm\n', ...
    P.T,P.LIFT*1000,P.V*1000,P.STEP*1000);
if P.STEP==0 && P.V==0
    fprintf('Rotate: STEP=0 la tinh tien bang 0; do xoay do Angle va N quyet dinh.\n');
end
fprintf('BIEN COM NHO NHAT = %+.6f mm, tai t = %.6f s (mau %d/%d)\n', ...
    S.min_margin_mm,S.time_s,k,S.sample_count);
fprintf('Chan dang tru: %s | canh gioi han: %s - %s\n', ...
    strjoin(S.support_legs,', '),S.limiting_edge_legs{1},S.limiting_edge_legs{2});
fprintf('Min khi 3 chan tru = %+.6f mm | khi 4 chan tru = %+.6f mm\n', ...
    S.min_3_support_mm,S.min_4_support_mm);
fprintf('Nguong sang loc = %.3f mm | Dat rieng nguong COM: %d\n',S.guide_mm,S.margin_guide_pass);
fprintf('COM theo khoi luong gia dinh; min tren cac mau da ghi, khong phai phep do phan cung.\n');
fprintf('================================================\n\n');
end
