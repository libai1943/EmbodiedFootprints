function figures = PlotResults(traj,guess,path,cfg)
%PLOTRESULTS Two static figures; all plotting source is available for study.
blue=[0.00,0.48,0.72]; red=[0.82,0.20,0.17]; green=[0.12,0.60,0.42];
figures(1)=figure('Color','w','Name','Embodied footprints','Position',[80 100 1180 570]);
layout=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(layout); hold(ax,'on'); axis(ax,'equal'); box(ax,'on');
DrawMap(ax,cfg);
for index=1:numel(traj.x)-1
    vertices=EmbodiedBox(traj.x(index),traj.y(index),traj.theta(index), ...
        traj.v(index)*traj.dt(index),tan(traj.phi(index))/cfg.vehicle.lw,cfg.vehicle);
    patch(ax,vertices(:,1),vertices(:,2),blue,'FaceAlpha',0.025,'EdgeColor',blue,'EdgeAlpha',0.12,'HandleVisibility','off');
end
for index=unique(round(linspace(1,numel(traj.x),22)))
    vertices=EmbodiedBox(traj.x(index),traj.y(index),traj.theta(index),0,0,cfg.vehicle);
    plot(ax,vertices([1:4,1],1),vertices([1:4,1],2),'Color',[0.25 0.55 0.65],'LineWidth',0.5,'HandleVisibility','off');
end
p1=plot(ax,path.x,path.y,'--','Color',[0.75 0.45 0.10],'LineWidth',1.1);
p2=plot(ax,traj.x,traj.y,'k-','LineWidth',1.6);
p3=scatter(ax,traj.x,traj.y,9,red,'filled');
plot(ax,traj.x(1),traj.y(1),'o','MarkerSize',8,'MarkerFaceColor',green,'MarkerEdgeColor','w','HandleVisibility','off');
plot(ax,traj.x(end),traj.y(end),'s','MarkerSize',8,'MarkerFaceColor',red,'MarkerEdgeColor','w','HandleVisibility','off');
xlim(ax,[0 30]); ylim(ax,[0 30]); xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
title(ax,sprintf('Case %d | T = %.2f s | %d nodes',cfg.case_id,sum(traj.dt),numel(traj.x)));
legend(ax,[p1 p2 p3],{'Hybrid A*','Optimized trajectory','Collocation nodes'},'Location','southoutside');
ax=nexttile(layout); hold(ax,'on'); axis(ax,'equal'); box(ax,'on');
[~,index]=max(abs(tan(traj.phi)/cfg.vehicle.lw).*traj.v.*traj.dt);
kappa=tan(traj.phi(index))/cfg.vehicle.lw; s=traj.v(index)*traj.dt(index);
box_vertices=EmbodiedBox(0,0,0,s,kappa,cfg.vehicle);
hbox=patch(ax,box_vertices(:,1),box_vertices(:,2),blue,'FaceAlpha',0.08,'EdgeColor',red,'LineWidth',2);
for fraction=linspace(0,1,16)
    theta=kappa*s*fraction;
    if abs(kappa)<1e-9, x=s*fraction; y=0; else, x=sin(theta)/kappa; y=(1-cos(theta))/kappa; end
    vertices=EmbodiedBox(x,y,theta,0,0,cfg.vehicle);
    hsweep=patch(ax,vertices(:,1),vertices(:,2),blue,'FaceAlpha',0.03,'EdgeColor',blue,'EdgeAlpha',0.35);
end
nominal=EmbodiedBox(0,0,0,0,0,cfg.vehicle);
hnominal=plot(ax,nominal([1:4,1],1),nominal([1:4,1],2),'Color',green,'LineWidth',1.8);
plot(ax,0,0,'k.','MarkerSize',15,'HandleVisibility','off');
legend(ax,[hbox,hsweep,hnominal],{'Embodied footprint','Intermediate nominal footprints','Nominal footprint at t_k'},'Location','southoutside');
xlabel(ax,'Local longitudinal coordinate (m)'); ylabel(ax,'Local lateral coordinate (m)');
title(ax,sprintf('Interval %d | v = %.2f m/s | duration = %.3f s',index,traj.v(index),traj.dt(index)));
xlim(ax,[min(box_vertices(:,1))-0.3,max(box_vertices(:,1))+0.3]);
ylim(ax,[min(box_vertices(:,2))-0.5,max(box_vertices(:,2))+0.5]);
sgtitle(layout,'Embodied Footprints: trajectory and interval coverage','FontWeight','bold');
figures(2)=figure('Color','w','Name','States, controls and time intervals','Position',[110 80 1150 670]);
layout=tiledlayout(2,4,'TileSpacing','compact','Padding','compact');
fields={'v','theta','phi','a','w'};
labels={'Speed (m/s)','Heading (rad)','Steering angle (rad)','Acceleration (m/s^2)','Steering rate (rad/s)'};
for index=1:numel(fields)
    ax=nexttile(layout); plot(ax,traj.time,traj.(fields{index}),'Color',blue,'LineWidth',1.5);
    xlabel(ax,'Time (s)'); ylabel(ax,labels{index}); grid(ax,'on');
end
ax=nexttile(layout); plot(ax,1:numel(traj.dt)-1,guess.dt(1:end-1),'--','Color',[0.6 .6 .6]); hold(ax,'on');
plot(ax,1:numel(traj.dt)-1,traj.dt(1:end-1),'Color',blue,'LineWidth',1.4);
xlabel(ax,'Interval index'); ylabel(ax,'Duration (s)'); legend(ax,{'Initial','Optimized'},'Location','best'); grid(ax,'on');
ax=nexttile(layout); plot(ax,traj.time,tan(traj.phi)/cfg.vehicle.lw,'Color',blue,'LineWidth',1.5);
xlabel(ax,'Time (s)'); ylabel(ax,'Curvature (1/m)'); grid(ax,'on');
ax=nexttile(layout); plot(ax,traj.time,traj.v.*traj.dt,'Color',blue,'LineWidth',1.5);
xlabel(ax,'Time (s)'); ylabel(ax,'Interval travel s_k (m)'); grid(ax,'on');
sgtitle(layout,'Optimized states, controls and nonuniform time intervals');
end

function DrawMap(ax,cfg)
for index=1:cfg.environment.num_obs
    obstacle=cfg.environment.obs(index);
    patch(ax,obstacle.x,obstacle.y,[0.68 0.70 0.71],'EdgeColor','none','HandleVisibility','off');
end
end
