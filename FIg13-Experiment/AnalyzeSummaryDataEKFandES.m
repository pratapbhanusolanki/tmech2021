clear all;
close all;
clc;

load('Data3DExperiments/TMech2020/DataSummaryEKFandES.mat');
n_ekf = 1;
n_es = 1;
n_te = 1;


Algorithms = {'EKF','ExSeeking'};
AlgorithmData = [];
numC = 13;
for i=1:2
    data.algorithm_name = Algorithms{i};
    data.entries = [];
    data.clustered_error_mean = cell(1,numC);
    data.clustered_error_std = cell(1,numC);
    data.clustered_Y_mean = cell(1,numC);
    data.clustered_Y_std = cell(1,numC);
    
    data.meanE = 0;
    data.meanY = 0;
    data.stdE = 0;
    data.stdY = 0;
    
    AlgorithmData = [AlgorithmData,data];
end

for n=1:length(summary)
    Index = find(contains(Algorithms,summary(n).Algorithm));
    AlgorithmData(Index).entries = [AlgorithmData(Index).entries,summary(n)];
end

figure();
start_data = [0;0.002;0.0035;0.005;0.0068;0.008;0.009;0.0105;0.0123;0.016;0.0185;0.0214;0.0245];
for i=1:2
    rover_speeds = [AlgorithmData(i).entries.speed];
    [idx,C] = kmeans(rover_speeds',numC,'Start',start_data);
    disp(C)
    C_all(:,i) = C;
    for k=1:length(idx)
       m = idx(k);
       AlgorithmData(i).clustered_error_mean{m} = [AlgorithmData(i).clustered_error_mean{m},AlgorithmData(i).entries(k).meanError];
       AlgorithmData(i).clustered_error_std{m} = [AlgorithmData(i).clustered_error_std{m},AlgorithmData(i).entries(k).stdError];
       AlgorithmData(i).clustered_Y_mean{m} = [AlgorithmData(i).clustered_Y_mean{m},AlgorithmData(i).entries(k).meanY];
       AlgorithmData(i).clustered_Y_std{m} = [AlgorithmData(i).clustered_Y_std{m},AlgorithmData(i).entries(k).stdY];
    end
    
    for m=1:numC
        %The number 80 is used to signify 
        [meanE,stdE] = compute_array_mean_and_std(AlgorithmData(i).clustered_error_mean{m},AlgorithmData(i).clustered_error_std{m},80);   
        [meanY,stdY] = compute_array_mean_and_std(AlgorithmData(i).clustered_Y_mean{m},AlgorithmData(i).clustered_Y_std{m},80);
        AlgorithmData(i).meanE(m) = meanE;
        AlgorithmData(i).stdE(m) = stdE;
        AlgorithmData(i).meanY(m) = meanY;
        AlgorithmData(i).stdY(m) = stdY;
    end
%     subplot(2,1,1)
%     errorbar(C*100,AlgorithmData(i).meanE,AlgorithmData(i).stdE);
%     xticks(C*100);
%     xtickformat('%.2f');
%     hold on;
%     ax = gca;
%     ax.FontSize = 14; 
%     ylabel('Mean Error $(^\circ)$', 'interpreter','latex', 'FontSize', 24);
%     
%     subplot(2,1,2)
%     errorbar(C*100,AlgorithmData(i).meanY,AlgorithmData(i).stdY);
%     xticks(C*100);
%     xtickformat('%.2g');
%     hold on;
%     ax = gca;
%     ax.FontSize = 14; 
%     xlabel('Combined speed $v\ (cm/s)$ ', 'interpreter','latex', 'FontSize', 24);
%     ylabel('Average intensity measurement (V)', 'interpreter','latex', 'FontSize', 24);
    
end



C = mean(C_all');
[Csorted, idx] = sort(C);
idx = idx(1:9)
C = Csorted(1:9);
styles = {'-r','--b'}
for i=1:2
%     subplot(2,1,1)
    errorbar(C*100,AlgorithmData(i).meanE(idx),AlgorithmData(i).stdE(idx), styles{i});
    %xticks(C*100);
    xtickformat('%.2f');
    hold on;
    ax = gca;
    ax.FontSize = 18; 
    ylabel('Mean Error $\textbf{E} (^\circ)$', 'interpreter','latex', 'FontSize', 24);
    %set(gca,'xtick',[])
    
%     subplot(2,1,2)
%     errorbar(C*100,AlgorithmData(i).meanY(idx),AlgorithmData(i).stdY(idx), styles{i});
%     xticks(C*100);
%     xtickformat('%.2g');
%     hold on;
%     ax = gca;
%     ax.FontSize = 18; 
    xlabel('Relative speed $v\ (cm/s)$ ', 'interpreter','latex', 'FontSize', 24);
    %ylabel('Average Intensity $\textbf{I}$ (V)', 'interpreter','latex', 'FontSize', 24);
%xlim([0 5]);
end

%subplot(2,1,1)
legend({'EKF','ES'},'FontSize', 24);

