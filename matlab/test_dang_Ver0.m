function fig = test_dang_Ver0(out,options)
% TEST_DANG_VER0  Mot cua so: dang 3D, support/COM, 12 goc, chan/than.
% Cach 1: Run Simulink, sau do go test_dang_Ver0
% Cach 2: out=sim('Spider_4leg_line_Ver0'); test_dang_Ver0(out)
% Dung cung file nay sau khi Run model circular, lissajous hoac rotate.
% QA: f=test_dang_Ver0(out,struct('Visible','off','AutoPlay',false));
if nargin<1
    assert(evalin('base','exist(''out'',''var'')==1'), ...
        'Ver0: Run mot model Simulink truoc, sau do go test_dang_Ver0.');
    out=evalin('base','out');
end
if nargin<2,options=struct;end
if ~isfield(options,'Visible'),options.Visible='on';end
if ~isfield(options,'AutoPlay'),options.AutoPlay=true;end
if ~isfield(options,'KeepPrevious'),options.KeepPrevious=false;end
R=kiemtra_Ver0(out,false);D=R.data;P=D.P;t=D.t;N=numel(t);
% Read-only summary of this run; does not change gait, IK/FK or COM.
if ~isfield(options,'PrintComSummary') || options.PrintComSummary
    in_bien_COM_Ver0(R);
end
curved=ismember('path_snapshot',out.who);
circular=false;rotating=false;direction=1;pathFormat=1;legacyMirrored=false;shapeRatio=1;
directionNames={'CCW','CW'};
if curved
    pathData=out.path_snapshot;
    pathSamples=numel(pathData.Time);
    if pathData.IsTimeFirst,pathData=reshape(pathData.Data,pathSamples,[]);
    else,pathData=reshape(pathData.Data,[],pathSamples)';end
    pathData=pathData(end,:);
    circular=pathData(1)==1;
    rotating=pathData(1)==3;
    if ~circular,direction=-1;end
    if numel(pathData)>=5,direction=pathData(5);end
    if numel(pathData)>=6,pathFormat=pathData(6);end
    if pathData(1)==2 && pathFormat==3 && numel(pathData)>=7,shapeRatio=pathData(7);end
    assert(ismember(pathData(1),[1 2 3]) && ismember(direction,[-1 1]) && ismember(pathFormat,[1 2 3]) && ...
        isfinite(shapeRatio) && shapeRatio>0 && (~(pathFormat==3) || (pathData(1)==2 && numel(pathData)>=7)) && ...
        (~rotating || (pathFormat==1 && isfinite(pathData(2)) && pathData(2)>0)), ...
        'Ver0: invalid recorded path or direction.');
    legacyMirrored=circular && pathFormat<2 && direction<0;
    % Whole ground path, including when out contains only a partial run.
    u=linspace(0,2*pi,5001)';radius=pathData(2);
    angle=P.HEADING_DEG*pi/180;
    if rotating
        directionNames={'Left','Right'};
        pathXY=D.ref(1,1:2)*1000;
        guideRadius=max(165,1000*hypot(max(abs(P.HIP(:,1)))+P.STANCE(1), ...
            max(abs(P.HIP(:,2)))+P.STANCE(2))+25);
        guideAngle=linspace(angle,angle+direction*pathData(2),181)';
        rotationGuide=pathXY+guideRadius*[cos(guideAngle),sin(guideAngle)];
    elseif circular
        pathXY=radius*[sin(u),1-cos(u)];
        u0=direction*P.V*t(1)/radius;
        startXY=radius*[sin(u0),1-cos(u0)];
        if legacyMirrored
            % Older CW data followed a different circle; preserve its true geometry.
            pathXY(:,2)=-pathXY(:,2);
            u0=P.V*t(1)/radius;
            startXY=radius*[sin(u0),-(1-cos(u0))];
        end
    else
        pathXY=radius*[sin(u),shapeRatio*sin(2*u)];
        arc=cumtrapz(u,sqrt(cos(u).^2+4*shapeRatio^2*cos(2*u).^2));
        u0=interp1(arc,u,mod(-direction*P.V*t(1)/radius,arc(end)));
        startXY=radius*[sin(u0),shapeRatio*sin(2*u0)];
        angle=angle-atan2(2*shapeRatio,1);
    end
    if ~rotating
        pathRotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
        pathOrigin=D.ref(1,1:2)-startXY*pathRotation';
        pathXY=1000*(pathXY*pathRotation'+pathOrigin);
    end
else
    % The recorded straight reference stays on the ground in world coordinates.
    pathXY=D.ref(:,1:2)*1000;
end
old=findall(groot,'Type','figure','Tag','Ver0Viewer');
if ~options.KeepPrevious && ~isempty(old),close(old);end
colors=[0.05 .40 .85;.90 .36 .04;0 .58 .38;.64 .26 .73];
names={'FL','FR','RR','RL'};bg=[.965 .973 .985];ink=[.10 .16 .25];
fig=figure('Name','Ver0 | 4-leg gait + COM / support', ...
    'NumberTitle','off','Tag','Ver0Viewer','Color',bg,'Position',[70 70 1420 880], ...
    'Visible',options.Visible,'MenuBar','none','ToolBar','figure','CloseRequestFcn',@closeViewer);
tabs=uitabgroup(fig,'Units','normalized','Position',[.009 .108 .982 .883]);
poseTab=uitab(tabs,'Title','3D + COM','BackgroundColor',bg);
angleTab=uitab(tabs,'Title','Joint angles','BackgroundColor',bg);
traceTab=uitab(tabs,'Title','Feet / body / margin','BackgroundColor',bg);
servoTab=uitab(tabs,'Title','Servo angles','BackgroundColor',bg);
ax3=axes('Parent',poseTab,'Position',[.055 .355 .54 .55],'Tag','Ver0WorldView');hold(ax3,'on');
ax2=axes('Parent',poseTab,'Position',[.685 .35 .275 .56],'Tag','Ver0TopView');hold(ax2,'on');
% Fixed world frame for every gait; preserve the true physical geometry.
jointX=D.pose(:,1:4:end)*1000;jointY=D.pose(:,2:4:end)*1000;
framePath=pathXY;
if rotating,framePath=[framePath;rotationGuide];end
xlim3=[min([jointX(:);framePath(:,1)])-25,max([jointX(:);framePath(:,1)])+25];
ylim3=[min([jointY(:);framePath(:,2)])-25,max([jointY(:);framePath(:,2)])+25];
relativeXY=D.toe(:,:,1:2)-reshape(D.ref(:,1:2),N,1,2);
followHalfWidth=max(125,1000*max(vecnorm(relativeXY,2,3),[],'all')+25);
% Include lifted knees, not just body height, when framing the mechanism.
zAll=D.pose(:,3:4:end)*1000;zmax=max(130,max(zAll,[],'all')+25);
for ax=[ax3 ax2]
    grid(ax,'on');box(ax,'on');ax.FontName='Arial';ax.FontSize=11;
    ax.XColor=ink;ax.YColor=ink;xlim(ax,xlim3);ylim(ax,ylim3);
    xlabel(ax,'X (mm)  → forward');ylabel(ax,'Y (mm)  → left');
end
zlim(ax3,[-5 zmax]);zlabel(ax3,'Z (mm)');view(ax3,[-15 22]);daspect(ax3,[1 1 1]);
axis(ax2,'equal');xlim(ax2,xlim3);ylim(ax2,ylim3);
title(ax3,'Pre-shift → swing → post-return','FontSize',13);
title(ax2,'Top view | ground contacts / COM','FontSize',12);
ground=patch(ax3,'XData',[xlim3 fliplr(xlim3)], ...
    'YData',[ylim3(1) ylim3(1) ylim3(2) ylim3(2)],'ZData',[0 0 0 0], ...
    'FaceColor',[.90 .93 .96],'FaceAlpha',.18,'EdgeColor','none'); %#ok<NASGU>
ground.Visible='off';
hSupport3=patch(ax3,nan,nan,nan,[1 .75 .15],'FaceAlpha',.14,'EdgeColor',[.90 .42 .05],'LineWidth',1.7);
hBody=plot3(ax3,nan,nan,nan,'-','Color',[.88 .15 .18],'LineWidth',3);
hLeg=gobjects(1,4);hToe=gobjects(1,4);hLabel=gobjects(1,4);
for i=1:4
    hLeg(i)=plot3(ax3,nan,nan,nan,'-o','Color',colors(i,:),'LineWidth',2.8,'MarkerSize',5,'MarkerFaceColor',colors(i,:));
    hToe(i)=plot(ax2,nan,nan,'o','Color',colors(i,:),'MarkerSize',7,'LineWidth',1.8);
    hLabel(i)=text(ax2,0,0,names{i},'Color',colors(i,:),'FontWeight','bold','FontSize',11);
end
hCom3=plot3(ax3,nan,nan,nan,'o','MarkerFaceColor',[.08 .22 .88],'MarkerEdgeColor','w','MarkerSize',9);
hDrop=plot3(ax3,nan,nan,nan,':','Color',[.08 .22 .88],'LineWidth',1.4);
hProjection=plot(ax2,nan,nan,'k--','LineWidth',1.2);
hSupport=patch(ax2,nan,nan,[1 .78 .2],'FaceAlpha',.16,'EdgeColor',[.90 .42 .05],'LineWidth',2);
hPreview=plot(ax2,nan,nan,':','Color',[.60 .25 .75],'LineWidth',1.5);
hBody2=plot(ax2,nan,nan,'-','Color',[.88 .15 .18],'LineWidth',1.6);
hRef=plot(ax2,nan,nan,'+','Color',[.35 .42 .5],'MarkerSize',10,'LineWidth',1.5);
hCenter=plot(ax2,nan,nan,'d','Color',[.86 0 .61],'MarkerFaceColor',[.86 0 .61],'MarkerSize',6);
hCom2=plot(ax2,nan,nan,'o','Color',[.08 .22 .88],'MarkerFaceColor',[.08 .22 .88],'MarkerSize',7);
hMargin=plot(ax2,nan,nan,':','Color',[.08 .22 .88],'LineWidth',1.2);
poseLegend=legend(ax2,[hProjection hSupport hPreview hCom2 hCenter hRef], ...
    {'4-toe projection','Current support','Triangle: active leg excluded', ...
    'COM (mass model)','Body center (shifted)','Straight reference'}, ...
    'Location','southoutside','NumColumns',2,'FontSize',9);
% Reserve the legend area explicitly so it does not shrink the top view.
poseLegend.Units='normalized';poseLegend.Position=[.615 .185 .365 .063];
ax3.Position=[.035 .235 .545 .70];ax2.Position=[.615 .31 .365 .625];
xlabel(ax3,'World X (mm)');ylabel(ax3,'World Y (mm)');
xlabel(ax2,'World X (mm)');ylabel(ax2,'World Y (mm)');
if curved
    directionTitle=directionNames{1+(direction<0)};
    if legacyMirrored,directionTitle='CW | legacy mirrored circle';end
    title(ax3,['3D | full ground path | ' directionTitle],'FontSize',12);
    hRef.DisplayName='Path reference';
else
    title(ax3,'3D | full ground path | Line','FontSize',12);
end
view(ax3,[-20 48]);ax3.Projection='orthographic';
set(hLeg,'LineWidth',3.2,'MarkerSize',5.5);
if ~curved
    % Extend only a faint guide, not the recorded travel, to show the ground axis.
    lineExtent=2*max(diff(xlim3),diff(ylim3));lineAngle=P.HEADING_DEG*pi/180;
    lineGuide=pathXY(1,:)+[-lineExtent;lineExtent]*[cos(lineAngle),sin(lineAngle)];
    plot3(ax3,lineGuide(:,1),lineGuide(:,2),[0;0],'--','Color',[.65 .70 .77], ...
        'LineWidth',1,'HandleVisibility','off','Tag','Ver0GroundLineGuide');
    plot(ax2,lineGuide(:,1),lineGuide(:,2),'--','Color',[.65 .70 .77], ...
        'LineWidth',1,'HandleVisibility','off','Tag','Ver0TopLineGuide');
end
plot3(ax3,pathXY(:,1),pathXY(:,2),zeros(size(pathXY,1),1),'-', ...
    'Color',[.28 .37 .49],'LineWidth',1.6,'HandleVisibility','off','Tag','Ver0GroundPath');
plot(ax2,pathXY(:,1),pathXY(:,2),'--', ...
    'Color',[.35 .42 .5],'LineWidth',1.2,'HandleVisibility','off','Tag','Ver0TopPath');
hTrail=plot(ax2,nan,nan,'-','Color',[.08 .22 .88], ...
    'LineWidth',1,'HandleVisibility','off','Tag','Ver0COMTrail');
hHeading=quiver(ax2,0,0,0,0,0,'Color',[.88 .15 .18], ...
    'LineWidth',1.8,'MaxHeadSize',.8,'HandleVisibility','off','Tag','Ver0BodyHeading');
if ~curved
    plot3(ax3,pathXY(1,1),pathXY(1,2),0,'s','Color',[.28 .37 .49], ...
        'MarkerFaceColor','w','MarkerSize',6,'HandleVisibility','off');
    plot3(ax3,pathXY(end,1),pathXY(end,2),0,'d','Color',[.28 .37 .49], ...
        'MarkerFaceColor',[.28 .37 .49],'MarkerSize',6,'HandleVisibility','off');
    text(ax3,pathXY(1,1),pathXY(1,2)-18,0,'Start','FontSize',9,'Color',ink);
    text(ax3,pathXY(end,1),pathXY(end,2)+18,0,'End','FontSize',9,'Color',ink);
    plot(ax2,pathXY(1,1),pathXY(1,2),'s','Color',[.28 .37 .49], ...
        'MarkerFaceColor','w','MarkerSize',6,'HandleVisibility','off');
    plot(ax2,pathXY(end,1),pathXY(end,2),'d','Color',[.28 .37 .49], ...
        'MarkerFaceColor',[.28 .37 .49],'MarkerSize',6,'HandleVisibility','off');
elseif rotating
    title(ax3,sprintf('3D | fixed center | %s %.0f deg',directionTitle,pathData(2)*180/pi),'FontSize',12);
    hRef.DisplayName='Rotation center';
    plot3(ax3,pathXY(1),pathXY(2),0,'+','Color',[.28 .37 .49], ...
        'MarkerSize',10,'LineWidth',1.5,'HandleVisibility','off');
    plot3(ax3,rotationGuide(:,1),rotationGuide(:,2),zeros(size(guideAngle)), '--', ...
        'Color',[.28 .37 .49],'LineWidth',1.4,'HandleVisibility','off','Tag','Ver0RotationGuide');
    plot(ax2,rotationGuide(:,1),rotationGuide(:,2),'--','Color',[.28 .37 .49], ...
        'LineWidth',1.2,'HandleVisibility','off','Tag','Ver0TopRotationGuide');
    guideEnds=rotationGuide([1 end],:);
    for endpoint=1:2
        guideRay=[pathXY;guideEnds(endpoint,:)];
        plot3(ax3,guideRay(:,1),guideRay(:,2),[0;0],':','Color',[.35 .42 .5], ...
            'LineWidth',1.2,'HandleVisibility','off');
        plot(ax2,guideRay(:,1),guideRay(:,2),':','Color',[.35 .42 .5], ...
            'LineWidth',1.2,'HandleVisibility','off');
    end
    targetText=sprintf('Target %+.0f deg',direction*pathData(2)*180/pi);
    text(ax3,guideEnds(1,1),guideEnds(1,2),8,'Start +X','FontSize',9,'Color',ink);
    text(ax3,guideEnds(2,1),guideEnds(2,2),8,targetText,'FontSize',9,'Color',ink);
end
status=uicontrol(poseTab,'Style','text','Units','normalized','Position',[.025 .117 .95 .035], ...
    'BackgroundColor',bg,'ForegroundColor',ink,'HorizontalAlignment','left','FontSize',11,'FontWeight','bold','Tag','Ver0Status');
leftInfo=uicontrol(poseTab,'Style','text','Units','normalized','Position',[.025 .018 .465 .085], ...
    'BackgroundColor',[.93 .96 1],'ForegroundColor',ink,'HorizontalAlignment','left','FontName','Consolas','FontSize',9);
rightInfo=uicontrol(poseTab,'Style','text','Units','normalized','Position',[.515 .018 .465 .085], ...
    'BackgroundColor',[1 .98 .92],'ForegroundColor',ink,'HorizontalAlignment','left','FontName','Consolas','FontSize',9);

% The viewer contains every chart in tabs of ONE figure.
defaultTopMode=1;defaultChartMode=1;
topMode=defaultTopMode;chartMode=defaultChartMode;
chartCacheLimits=nan(3,2);lastChartLimits=[nan nan];
initialChartLimits=[t(1) t(end)];initialSamples=1:N;
if chartMode==1
    initialChartLimits(2)=min(t(end),t(1)+2*P.T);
    initialSamples=1:min(N,find(t<=initialChartLimits(2),1,'last')+1);
end
jointAx=gobjects(4,3);servoAx=gobjects(4,3);
jointLines=gobjects(4,3);servoLines=gobjects(4,3);
jointCursors=gobjects(4,3);servoCursors=gobjects(4,3);traceCursors=gobjects(1,6);
tl=tiledlayout(angleTab,4,3,'TileSpacing','compact','Padding','compact');
title(tl,'IK angles (deg) | q1: world azimuth | Charts selects time range');
ts=tiledlayout(servoTab,4,3,'TileSpacing','compact','Padding','compact');
title(ts,'Servo preview (deg) | fitted Y scale; limits always listed in titles | no clipping');
qplot=D.q*180/pi;qplot(:,:,1)=unwrap(D.q(:,:,1),[],1)*180/pi;
for i=1:4
    for j=1:3
        ax=nexttile(tl,(i-1)*3+j);jointAx(i,j)=ax;
        jointLines(i,j)=plot(ax,t(initialSamples),qplot(initialSamples,i,j),'Color',colors(i,:),'LineWidth',1.4);grid(ax,'on');
        title(ax,sprintf('%s | q%d',names{i},j));ylabel(ax,'deg');xlim(ax,initialChartLimits);
        ylim(ax,signalLimits(qplot(initialSamples,i,j),4));
        if i==4,xlabel(ax,'Time (s)');end
        jointCursors(i,j)=xline(ax,t(1),'k:','LineWidth',1,'HandleVisibility','off');
        ax=nexttile(ts,(i-1)*3+j);servoAx(i,j)=ax;
        servoLines(i,j)=plot(ax,t(initialSamples),D.servo(initialSamples,i,j),'Color',colors(i,:),'LineWidth',1.4);grid(ax,'on');
        yline(ax,P.SERVO_LO(j),'r--');yline(ax,P.SERVO_HI(j),'r--');
        servoTitle=title(ax,sprintf('%s | servo %d | limits %.0f..%.0f',names{i},j,P.SERVO_LO(j),P.SERVO_HI(j)));ylabel(ax,'deg');
        if any(D.servo(:,i,j)<P.SERVO_LO(j) | D.servo(:,i,j)>P.SERVO_HI(j))
            servoTitle.Color=[.8 .12 .08];servoTitle.String=[servoTitle.String ' | EXCEEDED'];
        end
        xlim(ax,initialChartLimits);ylim(ax,signalLimits(D.servo(initialSamples,i,j),6));if i==4,xlabel(ax,'Time (s)');end
        servoCursors(i,j)=xline(ax,t(1),'k:','LineWidth',1,'HandleVisibility','off');
    end
end
tt=tiledlayout(traceTab,3,2,'TileSpacing','compact','Padding','compact');
title(tt,'Foot X/Y relative to body | foot Z above ground | scheduled contacts');
relativeToe=D.toe;
dx=D.toe(:,:,1)-D.body(:,1);dy=D.toe(:,:,2)-D.body(:,2);
cy=cos(D.body(:,4));sy=sin(D.body(:,4));
relativeToe(:,:,1)=dx.*cy+dy.*sy;relativeToe(:,:,2)=-dx.*sy+dy.*cy;
dx=[];dy=[];cy=[];sy=[];
traceAx=gobjects(1,6);traceLines=cell(1,6);xyzLabels='XYZ';
for j=1:3
    ax=nexttile(tt,j);traceAx(j)=ax;hold(ax,'on');
    traceLines{j}=gobjects(1,4);
    for i=1:4,traceLines{j}(i)=plot(ax,t(initialSamples),relativeToe(initialSamples,i,j)*1000,'Color',colors(i,:),'LineWidth',1.3);end
    if j<3,frameName='relative to body';else,frameName='above ground';end
    grid(ax,'on');title(ax,sprintf('Foot %s | %s',xyzLabels(j),frameName));ylabel(ax,'mm');
    if j==1,legend(ax,names,'Location','best','NumColumns',4);end
end
ax=nexttile(tt,4);traceAx(4)=ax;
traceLines{4}=plot(ax,t(initialSamples),D.gait(initialSamples,4:5)*1000,'LineWidth',1.5);grid(ax,'on');title(ax,'Body offset from straight reference');ylabel(ax,'mm');legend(ax,{'X shift','Y shift'});
if curved,title(ax,'Body offset from path reference | body axes');end
ax=nexttile(tt,5);traceAx(5)=ax;
traceLines{5}=gobjects(1,2);
traceLines{5}(1)=plot(ax,t(initialSamples),D.margin(initialSamples)*1000,'Color',[.08 .22 .88],'LineWidth',1.4);hold(ax,'on');
traceLines{5}(2)=plot(ax,t(initialSamples),D.bodyMargin(initialSamples)*1000,'Color',[.86 0 .61],'LineWidth',1);yline(ax,0,'r-');yline(ax,P.MARGIN_GUIDE*1000,'k--');
grid(ax,'on');title(ax,'Signed distance to current support edge');ylabel(ax,'mm');legend(ax,{'COM','Body center','Edge','Guide'},'NumColumns',2);
ax=nexttile(tt,6);traceAx(6)=ax;hold(ax,'on');
traceLines{6}=gobjects(1,4);
for i=1:4,traceLines{6}(i)=stairs(ax,t(initialSamples),double(D.contact(initialSamples,i))*.7+(4-i),'Color',colors(i,:),'LineWidth',1.4);end
yticks(ax,(0:3)+.35);yticklabels(ax,fliplr(names));ylim(ax,[-.15 3.9]);grid(ax,'on');title(ax,'Contact schedule | upper = stance, lower = swing');
for chart=1:6
    ax=traceAx(chart);xlim(ax,initialChartLimits);xlabel(ax,'Time (s)');
    traceCursors(chart)=xline(ax,t(1),'k:','LineWidth',1,'HandleVisibility','off');
end
set([jointAx(:);servoAx(:);traceAx(:)],'FontName','Arial','FontSize',9);
ylim(traceAx(3),signalLimits(relativeToe(:,:,3)*1000,2));
ylim(traceAx(4),signalLimits(D.gait(:,4:5)*1000,2));
ylim(traceAx(5),signalLimits([D.margin;D.bodyMargin;0;P.MARGIN_GUIDE]*1000,5));

playButton=uicontrol(fig,'Style','pushbutton','String','Play','Units','normalized', ...
    'Position',[.015 .063 .055 .030],'FontSize',10,'Callback',@togglePlay,'Tag','Ver0Play');
resetButton=uicontrol(fig,'Style','pushbutton','String','Reset','Units','normalized', ...
    'Position',[.076 .063 .055 .030],'FontSize',10,'Callback',@(~,~) resetView(), ...
    'Tag','Ver0Reset','TooltipString','Pause, return to t = 0 and restore all views');
slider=uicontrol(fig,'Style','slider','Min',t(1),'Max',t(end),'Value',t(1), ...
    'Units','normalized','Position',[.14 .067 .28 .021], ...
    'SliderStep',[min(1,P.DT/(t(end)-t(1))),min(1,P.T/(t(end)-t(1)))], ...
    'Callback',@scrub,'Tag','Ver0Time');
timeLabel=uicontrol(fig,'Style','text','Units','normalized','Position',[.43 .062 .105 .029], ...
    'BackgroundColor',bg,'HorizontalAlignment','left','FontSize',10);
speedList=[.25 .5 1 2 8 16 32 64];speed=1;speedIndex=3;
if curved,speed=8;speedIndex=5;end
speedMenu=uicontrol(fig,'Style','popupmenu','String',{'0.25x','0.5x','1x','2x','8x','16x','32x','64x'},'Value',speedIndex, ...
    'Units','normalized','Position',[.545 .066 .055 .027],'Callback',@changeSpeed,'Tag','Ver0Speed', ...
    'TooltipString','Playback speed only; simulation time and gait parameters stay unchanged.');
directionMenu=gobjects(0);directionButton=gobjects(0);rerunBusy=false;
if curved
    directionTip='CCW: counterclockwise. CW: clockwise. Choose, then press Select to rerun the model.';
    if rotating
        directionTip='Left: +90 deg toward +Y. Right: -90 deg toward -Y. Choose, then press Select to rerun from +X.';
    elseif ~circular
        directionTip=[directionTip ' Figure eight: direction names refer to the sin(u)>0 lobe; the other lobe turns oppositely.'];
    elseif legacyMirrored
        directionTip=[directionTip ' This older result uses a mirrored circle; rerun to use the current same-path convention.'];
    end
    directionMenu=uicontrol(fig,'Style','popupmenu','String',directionNames, ...
        'Value',1+(direction<0),'Units','normalized','Position',[.61 .066 .06 .027],'FontSize',9, ...
        'Tag','Ver0Direction','TooltipString',directionTip);
    directionButton=uicontrol(fig,'Style','pushbutton','String','Select', ...
        'Units','normalized','Position',[.678 .063 .06 .030],'FontSize',10, ...
        'Tag','Ver0RunDirection','Callback',@rerunDirection, ...
        'TooltipString','Rerun the selected direction; replace data and synchronize the model switch only after success.');
    setappdata(fig,'Ver0DisplayedDirection',direction);
    setappdata(fig,'Ver0PathFormat',pathFormat);
    setappdata(fig,'Ver0PathType',pathData(1));
end
uicontrol(fig,'Style','text','String','Top','Units','normalized','Position',[.745 .064 .024 .025], ...
    'BackgroundColor',bg,'HorizontalAlignment','right','FontSize',9);
topMenu=uicontrol(fig,'Style','popupmenu','String',{'Follow','Full'},'Value',topMode, ...
    'Units','normalized','Position',[.773 .066 .075 .027],'FontSize',9,'Tag','Ver0TopMode', ...
    'Callback',@changeTopMode,'TooltipString','Follow: close world-XY view around the path reference. Full: whole ground path.');
uicontrol(fig,'Style','text','String','Charts','Units','normalized','Position',[.853 .064 .039 .025], ...
    'BackgroundColor',bg,'HorizontalAlignment','right','FontSize',9);
chartMenu=uicontrol(fig,'Style','popupmenu','String',{'2 cycles','Full'},'Value',chartMode, ...
    'Units','normalized','Position',[.897 .066 .086 .027],'FontSize',9,'Tag','Ver0ChartMode', ...
    'Callback',@changeChartMode,'TooltipString','Two-cycle windows advance once per gait cycle; Full displays every recorded sample.');
title(ax2,['Top | COM / support | ' topMenu.String{topMode}],'FontSize',12);
footer=uicontrol(fig,'Style','text','Units','normalized','Position',[.015 .009 .97 .040], ...
    'BackgroundColor',bg,'ForegroundColor',[.34 .40 .48],'HorizontalAlignment','left','FontSize',9, ...
    'String',sprintf(['T = %.1f s | step = %.0f mm/cycle | lift = %.0f mm | body shift X/Y = %.0f/%.0f mm.   ' ...
    'COM: assumed masses, %.1f g total.\nContacts follow the WPG schedule; no ground forces, motor torque or measured hardware COM are simulated.'], ...
    P.T,P.STEP*1000,P.LIFT*1000,P.SHIFT_X*1000,P.SHIFT_Y*1000,1000*(P.MASS_BODY+4*sum(P.MASS_LINK))));
if rotating
    set(footer,'String',sprintf(['T = %.1f s | turn = %s %.0f deg / %.1f s | lift = %.0f mm | body shift X/Y = %.0f/%.0f mm.\n' ...
        'Ground arc shows heading only; the reference center stays fixed. COM uses assumed masses; no ground-force or servo-load simulation.'], ...
        P.T,directionNames{1+(direction<0)},pathData(2)*180/pi,pathData(4),P.LIFT*1000,P.SHIFT_X*1000,P.SHIFT_Y*1000));
elseif curved
    set(footer,'String',sprintf(['T = %.1f s | reference advance = %.0f mm/cycle | lift = %.0f mm | R = %.0f mm | path = %.0f mm / %.1f s.\n' ...
        'Playback speed affects display only. COM uses assumed masses; contacts are scheduled, with no ground-force or servo-load simulation.'], ...
        P.T,P.STEP*1000,P.LIFT*1000,pathData(2)*1000,pathData(3)*1000,pathData(4)));
end
frame=1;playing=false;elapsedBase=t(1);epoch=tic;
tm=timer('ExecutionMode','fixedSpacing','Period',.04,'BusyMode','drop', ...
    'TimerFcn',@tick,'Tag','Ver0Animation','ErrorFcn',@timerError);
setappdata(fig,'Ver0Report',rmfield(R,'data'));
setappdata(fig,'Ver0Controls',struct('Seek',@seek,'Play',@startPlay,'Pause',@pausePlay, ...
    'Reset',@resetView,'Timer',tm,'Tabs',tabs,'PoseTab',poseTab,'AngleTab',angleTab,'TraceTab',traceTab,'ServoTab',servoTab, ...
    'TopMenu',topMenu,'ChartMenu',chartMenu,'MainAxes',ax3,'TopAxes',ax2, ...
    'JointAxes',jointAx,'ServoAxes',servoAx,'TraceAxes',traceAx,'AdoptFigure',@adoptFigure, ...
    'RefreshCharts',@updateChartViews,'JointLines',jointLines,'ServoLines',servoLines,'TraceLines',{traceLines}));
if curved
    controls=getappdata(fig,'Ver0Controls');
    controls.RunDirection=@() rerunDirection([],[]);
    controls.DirectionMenu=directionMenu;
    setappdata(fig,'Ver0Controls',controls);
end
renderFrame(1);
% Save the initial camera and limits, including plots on the other tabs.
drawnow nocallbacks;
viewAxes=[ax3;ax2;jointAx(:);servoAx(:);traceAx(:)];
viewValues={'XLim','YLim','ZLim','DataAspectRatio','PlotBoxAspectRatio', ...
    'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','Projection'};
viewModes={'XLimMode','YLimMode','ZLimMode','DataAspectRatioMode','PlotBoxAspectRatioMode', ...
    'CameraPositionMode','CameraTargetMode','CameraUpVectorMode','CameraViewAngleMode'};
initialViews=get(viewAxes,viewValues);
initialModes=get(viewAxes,viewModes);
initialDirections=get(viewAxes,'View');
selectionListener=addlistener(tabs,'SelectedTab','PostSet',@changeTab);
setappdata(tabs,'Ver0SelectionListener',selectionListener);
if options.AutoPlay,startPlay();end

    function renderFrame(k)
        if ~isgraphics(fig),return;end
        frame=max(1,min(N,k));k=frame;
        points=reshape(D.pose(k,:),4,4,4)*1000;
        toe=reshape(D.toe(k,:,:),4,3)*1000;
        hip=squeeze(points(1:3,1,:))';com=D.com(k,1:3)*1000;
        mask=D.contact(k,:);h=convhull(toe(mask,1),toe(mask,2));support=toe(mask,:);support=support(h,:);
        set(hSupport3,'XData',support(:,1),'YData',support(:,2),'ZData',zeros(size(h)));
        set(hSupport,'XData',support(:,1),'YData',support(:,2));
        ring=[1 2 3 4 1];
        set(hBody,'XData',hip(ring,1),'YData',hip(ring,2),'ZData',hip(ring,3));
        set(hBody2,'XData',hip(ring,1),'YData',hip(ring,2));
        set(hProjection,'XData',toe(ring,1),'YData',toe(ring,2));
        for leg=1:4
            p=points(1:3,:,leg);
            set(hLeg(leg),'XData',p(1,:),'YData',p(2,:),'ZData',p(3,:));
            if mask(leg),face=colors(leg,:);else,face=[1 1 1];end
            set(hToe(leg),'XData',toe(leg,1),'YData',toe(leg,2),'MarkerFaceColor',face);
            set(hLabel(leg),'Position',[toe(leg,1)+6 toe(leg,2)+5 0]);
        end
        preview=true(1,4);active=D.gait(k,1);preview(active)=false;
        tri=toe(preview,:);tri=tri([1 2 3 1],:);set(hPreview,'XData',tri(:,1),'YData',tri(:,2));
        set(hCom3,'XData',com(1),'YData',com(2),'ZData',com(3));
        set(hDrop,'XData',[com(1) com(1)],'YData',[com(2) com(2)],'ZData',[0 com(3)]);
        set(hCom2,'XData',com(1),'YData',com(2));
        set(hCenter,'XData',D.body(k,1)*1000,'YData',D.body(k,2)*1000);
        set(hRef,'XData',D.ref(k,1)*1000,'YData',D.ref(k,2)*1000);
        if topMode==1
            trail=max(1,k-ceil(2*P.T/P.DT)):k;
        else
            trail=unique([1:10:k k]);
        end
        set(hTrail,'XData',D.com(trail,1)*1000,'YData',D.com(trail,2)*1000);
        set(hHeading,'XData',D.body(k,1)*1000,'YData',D.body(k,2)*1000, ...
            'UData',50*cos(D.body(k,4)),'VData',50*sin(D.body(k,4)));
        if topMode==1
            center=D.ref(k,1:2)*1000;
            set(ax2,'XLim',center(1)+[-followHalfWidth followHalfWidth], ...
                'YLim',center(2)+[-followHalfWidth followHalfWidth]);
        else
            set(ax2,'XLim',xlim3,'YLim',ylim3);
        end
        % Perpendicular to the closest support edge, for the signed margin.
        a=support(1:end-1,1:2);e=diff(support(:,1:2));n=[-e(:,2),e(:,1)]./vecnorm(e,2,2);
        ds=sum(n.*(com(1:2)-a),2);[d,edgeIndex]=min(ds);proj=com(1:2)-d*n(edgeIndex,:);
        set(hMargin,'XData',[com(1) proj(1)],'YData',[com(2) proj(2)]);
        stages={'PRE-SHIFT','SWING','POST-RETURN'};stage=stages{D.gait(k,2)};
        if D.margin(k)>=0,where='INSIDE';else,where='OUTSIDE';end
        set(status,'String',sprintf('t = %.2f s  |  %s %s  |  %d supports  |  COM margin = %+.2f mm', ...
            t(k),names{active},stage,sum(mask),D.margin(k)*1000));
        if rotating
            phaseText=[names{active} ' ' stage];
            if t(k)>=pathData(4)-P.DT/2,phaseText='HOLD';end
            set(status,'String',sprintf('t = %.2f s  |  yaw = %+.1f deg  |  %s  |  %d supports  |  COM margin = %+.2f mm', ...
                t(k),D.body(k,4)*180/pi,phaseText,sum(mask),D.margin(k)*1000));
        end
        set(leftInfo,'String',sprintf(['COM XYZ: %7.2f / %7.2f / %6.2f mm\n' ...
            'Shift XY: %+6.2f / %+6.2f mm | support SM %+.2f mm (%s)'], ...
            com,D.gait(k,4)*1000,D.gait(k,5)*1000,D.margin(k)*1000,where));
        set(rightInfo,'String',sprintf(['Exclude %s: COM SM %+.2f mm | guide %.1f mm\n' ...
            'Run: drift %.2g mm | IK/FK %.2g mm | min COM SM %.2f mm'], ...
            names{active},D.previewMargin(k)*1000,P.MARGIN_GUIDE*1000,R.stance_anchor_drift_mm,R.fk_error_mm,R.min_com_margin_mm));
        set(slider,'Value',t(k));set(timeLabel,'String',sprintf('%.2f / %.1f s',t(k),t(end)));
        setappdata(fig,'Ver0Frame',k);
        updateChartViews();
        drawnow limitrate nocallbacks;
    end
    function tick(~,~)
        if ~isvalid(tm),return;end
        if ~isgraphics(fig),stop(tm);return;end
        if ~playing,return;end
        target=elapsedBase+toc(epoch)*speed;
        k=find(t<=target,1,'last');if isempty(k),k=1;end
        renderFrame(k);if target>=t(end),pausePlay();end
    end
    function startPlay()
        if playing,return;end
        if frame==N,renderFrame(1);end
        elapsedBase=t(frame);epoch=tic;playing=true;
        setappdata(fig,'Ver0PlaybackStartTime',elapsedBase);
        set(playButton,'String','Pause');start(tm);
    end
    function pausePlay()
        playing=false;if isvalid(tm) && strcmp(tm.Running,'on'),stop(tm);end
        if isgraphics(playButton),set(playButton,'String','Play');end
    end
    function togglePlay(~,~)
        if playing,pausePlay();else,startPlay();end
    end
    function seek(value)
        pausePlay();[~,k]=min(abs(t-value));renderFrame(k);
    end
    function scrub(src,~),seek(src.Value);end
    function changeSpeed(src,~)
        elapsedBase=t(frame);epoch=tic;speed=speedList(src.Value);
    end
    function changeTopMode(src,~)
        topMode=src.Value;
        title(ax2,['Top | COM / support | ' src.String{topMode}],'FontSize',12);
        renderFrame(frame);
    end
    function changeChartMode(src,~)
        chartMode=src.Value;chartCacheLimits=nan(3,2);lastChartLimits=[nan nan];
        updateChartViews();drawnow nocallbacks;
    end
    function changeTab(~,~)
        updateChartViews();drawnow nocallbacks;
    end
    function updateChartViews()
        limits=[t(1) t(end)];
        if chartMode==1
            width=min(2*P.T,t(end)-t(1));
            cycle=floor((t(frame)-t(1))/P.T+1e-10);
            first=max(t(1),min(t(1)+max(0,cycle-1)*P.T,t(end)-width));
            limits=[first first+width];
        end
        lastChartLimits=limits;setappdata(fig,'Ver0ChartLimits',limits);
        selected=tabs.SelectedTab;
        if selected==angleTab
            group=1;set(jointCursors(:),'Value',t(frame));
        elseif selected==servoTab
            group=2;set(servoCursors(:),'Value',t(frame));
        elseif selected==traceTab
            group=3;set(traceCursors,'Value',t(frame));
        else
            return;
        end
        if isequal(limits,chartCacheLimits(group,:)),return;end
        first=find(t>=limits(1),1,'first');last=find(t<=limits(2),1,'last');
        rows=max(1,first-1):min(N,last+1);
        if group==1
            set(jointAx(:),'XLim',limits);
            for leg=1:4
                for joint=1:3
                    values=qplot(rows,leg,joint);
                    set(jointLines(leg,joint),'XData',t(rows),'YData',values);
                    ylim(jointAx(leg,joint),signalLimits(values,4));
                end
            end
        elseif group==2
            set(servoAx(:),'XLim',limits);
            for leg=1:4
                for joint=1:3
                    values=D.servo(rows,leg,joint);
                    set(servoLines(leg,joint),'XData',t(rows),'YData',values);
                    ylim(servoAx(leg,joint),signalLimits(values,6));
                end
            end
        else
            set(traceAx,'XLim',limits);
            for coord=1:3
                for leg=1:4
                    set(traceLines{coord}(leg),'XData',t(rows),'YData',relativeToe(rows,leg,coord)*1000);
                end
            end
            for coord=1:2
                set(traceLines{4}(coord),'XData',t(rows),'YData',D.gait(rows,coord+3)*1000);
            end
            set(traceLines{5}(1),'XData',t(rows),'YData',D.margin(rows)*1000);
            set(traceLines{5}(2),'XData',t(rows),'YData',D.bodyMargin(rows)*1000);
            for leg=1:4
                set(traceLines{6}(leg),'XData',t(rows),'YData',double(D.contact(rows,leg))*.7+(4-leg));
            end
        end
        chartCacheLimits(group,:)=limits;
    end
    function limits=signalLimits(values,minWidth)
        lo=min(values,[],'all');hi=max(values,[],'all');
        width=max(minWidth,hi-lo);middle=(lo+hi)/2;
        limits=middle+[-.6 .6]*width;
    end
    function rerunDirection(~,~)
        if ~curved || rerunBusy || ~isgraphics(fig),return;end
        pausePlay();rerunBusy=true;
        rerunControls=[playButton resetButton slider speedMenu directionMenu directionButton topMenu chartMenu];
        set(rerunControls,'Enable','off');
        cleanup=onCleanup(@finishRerun); %#ok<NASGU>
        choice=directionMenu.Value;
        set(status,'String',['Simulating ' directionNames{choice} ' ...']);
        drawnow nocallbacks;
        newFig=gobjects(0);
        keptViewers=findall(groot,'Type','figure','Tag','Ver0Viewer');
        try
            model='Spider_4leg_circular_Ver0';
            if rotating,model='Spider_4leg_rotate_Ver0';
            elseif ~circular,model='Spider_4leg_lissajous_Ver0';end
            modelFile=fullfile(fileparts(mfilename('fullpath')),[model '.slx']);
            if bdIsLoaded(model)
                loadedFile=get_param(model,'FileName');
                assert(strcmpi(strrep(loadedFile,'/','\'),strrep(modelFile,'/','\')), ...
                    'Ver0:DirectionModelPath','A different copy of this model is loaded. Close it before running this viewer model.');
            else
                load_system(modelFile);
            end
            assert(strcmp(get_param(model,'SimulationStatus'),'stopped'), ...
                'Ver0:DirectionModelBusy','This model is already running. Wait for it to stop.');
            simInput=Simulink.SimulationInput(model);
            simInput=simInput.setBlockParameter([model '/Direction'],'sw',num2str(choice==1));
            newOut=sim(simInput);
            assert(isempty(newOut.ErrorMessage),'Ver0:DirectionSimulation','%s',newOut.ErrorMessage);
            nextOptions=options;nextOptions.KeepPrevious=true;nextOptions.Visible='off';nextOptions.AutoPlay=false;
            newFig=test_dang_Ver0(newOut,nextOptions);
            assert(getappdata(newFig,'Ver0DisplayedDirection')==3-2*choice, ...
                'Ver0:DirectionMismatch','The simulated direction does not match the selected direction.');
            expectedFormat=2+~circular;
            if rotating,expectedFormat=1;end
            assert(getappdata(newFig,'Ver0PathFormat')==expectedFormat && getappdata(newFig,'Ver0PathType')==pathData(1), ...
                'Ver0:DirectionFormat','The model did not produce the requested path with the updated direction convention.');
            set_param([model '/Direction'],'sw',num2str(choice==1));
            nextControls=getappdata(newFig,'Ver0Controls');
            nextControls.AdoptFigure(fig);
            assignin('base','out',newOut);
        catch errorInfo
            openedViewers=findall(groot,'Type','figure','Tag','Ver0Viewer');
            for opened=openedViewers'
                if ~any(opened==keptViewers),close(opened);end
            end
            if isgraphics(fig)
                setappdata(fig,'Ver0DirectionError',errorInfo.message);
                if isgraphics(status),set(status,'String',['Run failed; current data kept. ' errorInfo.message]);end
                if strcmpi(options.Visible,'on'),errordlg(errorInfo.message,'Run direction failed','non-modal');end
            end
            warning('Ver0:DirectionRunFailed','%s',errorInfo.message);
        end
    end
    function finishRerun()
        rerunBusy=false;
        if isgraphics(fig)
            rerunControls=[playButton resetButton slider speedMenu directionMenu directionButton topMenu chartMenu];
            set(rerunControls(isgraphics(rerunControls)),'Enable','on');
        end
    end
    function adoptFigure(target)
        % Replace the contents, while keeping the existing visible window.
        assert(isgraphics(target,'figure') && target~=fig,'Ver0: invalid target viewer.');
        source=fig;sourceChildren=allchild(source);sourceData=getappdata(source);
        targetVisibility=target.Visible;targetUnits=target.Units;
        targetPosition=target.Position;targetWindowState=target.WindowState;
        previous=getappdata(target,'Ver0Controls');previous.Pause();
        if isvalid(previous.Timer),delete(previous.Timer);end
        delete(allchild(target));
        for child=sourceChildren',child.Parent=target;end
        target.Name=source.Name;target.Color=source.Color;target.CloseRequestFcn=source.CloseRequestFcn;
        oldKeys=fieldnames(getappdata(target));
        for key=1:numel(oldKeys)
            if startsWith(oldKeys{key},'Ver0'),rmappdata(target,oldKeys{key});end
        end
        newKeys=fieldnames(sourceData);
        for key=1:numel(newKeys),setappdata(target,newKeys{key},sourceData.(newKeys{key}));end
        fig=target;options.Visible=targetVisibility;options.AutoPlay=true;options.KeepPrevious=false;
        delete(source);
        target.Visible=targetVisibility;
        renderFrame(1);
        target.Units=targetUnits;
        if strcmp(targetWindowState,'normal')
            target.WindowState='normal';target.Position=targetPosition;
        else
            target.WindowState=targetWindowState;
        end
        drawnow nocallbacks;
        if strcmp(targetWindowState,'normal'),target.Position=targetPosition;end
        startPlay();
    end
    function resetView()
        pausePlay();
        zoom(fig,'off');pan(fig,'off');rotate3d(fig,'off');
        elapsedBase=t(1);epoch=tic;
        set(viewAxes,viewValues,initialViews);
        set(viewAxes,{'View'},initialDirections);
        set(viewAxes,viewModes,initialModes);
        topMode=defaultTopMode;chartMode=defaultChartMode;
        topMenu.Value=topMode;chartMenu.Value=chartMode;chartCacheLimits=nan(3,2);lastChartLimits=[nan nan];
        title(ax2,['Top | COM / support | ' topMenu.String{topMode}],'FontSize',12);
        renderFrame(1);
        drawnow nocallbacks;
    end
    function timerError(~,event)
        pausePlay();warning('Ver0:ViewerTimer','Animation stopped: %s',event.Data.Message);
    end
    function closeViewer(~,~)
        if exist('tm','var') && isvalid(tm),stop(tm);delete(tm);end
        delete(fig);
    end
end
