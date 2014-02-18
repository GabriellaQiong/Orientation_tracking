function [h] = plot_data(rots, Racc, tsVicon, tsImu, outputDir, verbose)
% PLOT_DATA() plots the data 
% 
% Written by Qiong Wang at University of Pennsylvania
% 02/10/2014

% Synchronize time and data length
offset = sync_time(tsImu, tsVicon);
endIdx = min(size(Racc, 3), size(rots, 3));
if offset < 0
    RaccNew = Racc(:, :, 1 : endIdx + offset + 1);
    rotsNew = rots(:, :, -offset : endIdx);
    t       = tsImu(1 : endIdx + offset + 1) - tsImu(1);
elseif offset > 0
    RaccNew = Racc(:, :, offset : endIdx);
    rotsNew = rots(:, :, 1 : endIdx - offset + 1);
    t       = tsVicon(1 : endIdx - offset + 1) - tsVicon(1);
end

assert(size(RaccNew, 3) == size(rotsNew, 3), 'Dimension error, please check the last step : )');
datNum = length(t);

% Plot out rotation according to time
if verbose
    h1 = figure(1);
    set(gcf,'units','normalized','position',[0 .4 .6 .6]);
    ht = suptitle({'Sanity Check for Data Conversion'; sprintf('Time = %05f s',t(1))});
else
    ht = [];
end
rpyVicon = zeros(datNum, 3);
rpyAcc   = zeros(datNum, 3);

% % Initialize video writer
% writerObj = VideoWriter(fullfile(outputDir, 'rotplot_qiong.avi'));
% writerObj.FrameRate = 10;
% open(writerObj);

for i = 1 : datNum
    subplot(1, 2, 1);
    rotplot(rotsNew(:, :, i), 'Ground Truth', verbose);
    rpyVicon(i, :) = rot2rpy(rotsNew(:, :, i));
    subplot(1, 2, 2); 
    rotplot(RaccNew(:, :, i), 'Accelerometer Value', verbose);
    rpyAcc(i, :)   = rot2rpy(RaccNew(:, :, i));
    set(ht, 'String', {'Sanity Check'; sprintf('Time = %05f s',t(i))});
%     writeVideo(writerObj, getframe(gcf));
end
close(gcf);
% close(writerObj);
% clear writerObj;

% Plot roll pitch yaw angles
h = figure(3);
subplot(3,1,1);
plot(t, rpyVicon(:, 1),'r','LineWidth',1.2); hold on;
plot(t, rpyAcc(:, 1),'g','LineWidth',1.2); 
xlim([t(1), t(end)]); xlabel('t/s'); ylabel('roll/rad')
obj1= title('Plot of roll-angle');obj2 = legend('$Vicon$','$UKF$');
set(obj1,'Interpreter','Latex'); set(obj2,'Interpreter','Latex'); clear obj1 obj2;

subplot(3,1,2);
plot(t, rpyVicon(:, 2),'r','LineWidth',1.2); hold on;
plot(t,rpyAcc(:, 2),'g','LineWidth',1.2);
xlim([t(1), t(end)]); xlabel('t/s'); ylabel('pitch/rad');
obj1= title('Plot of pitch-angle'); obj2 = legend('$Vicon$','$UKF$');
set(obj1,'Interpreter','Latex'); set(obj2,'Interpreter','Latex'); clear obj1 obj2;

subplot(3,1,3);
plot(t, rpyVicon(:, 3),'r','LineWidth',1.2); hold on;
plot(t,rpyAcc(:, 3),'g','LineWidth',1.2);
xlim([t(1), t(end)]); xlabel('t/s'); ylabel('yaw/rad');
obj1= title('Plot of yaw-angle'); obj2 = legend('$Vicon$','$UKF$');
set(obj1,'Interpreter','Latex'); set(obj2,'Interpreter','Latex'); clear obj1 obj2;

end