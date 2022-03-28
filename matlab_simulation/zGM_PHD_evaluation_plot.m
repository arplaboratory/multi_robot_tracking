% 
% %% RMSE plots
% %NUM_DRONES 3
% %false positive nClutter, false negative pDetection, noiseScalar
% % control RMSE: 0.1646 MAX: 0.6667
% 
% %false positive nClutter = 1,5,10 = [0.1646,0.6667], [1.3542, 5.1059], [2.9198,15.3674]
% 
% %false negative prob_detection = 0.97,0.95, 0.9 = [1.58, 10.789], [32.173, 55.489], [65.71,104.906]
% 
% %noise clutter = 0.25,0.5,0.75 = [1.538, 2.33],[3.407,11.64],[41.07,57.56]
% 
% %mixed = 1,0.97, 0.25 = [63.01, 105.63]
% 
% %mixed = 3,0.99, 0.5 = [3.609, 9.366] 
% %% plot evaluation
% 
% FP = [1, 5, 10];
% FN = [0.97, 0.95, 0.9];
% FPrmse = [0.1646, 1.3542, 2.9198];
% FPmax = [0.6667, 5.1059, 15.3674];
% FNrmse = [1.58, 32.173, 65.71];
% FNmax = [10.789, 55.489, 104.906];
% Nrmse = [1.538, 3.407, 41.07];
% Nmax = [2.33, 11.64, 57.56];
% 
% 
% figure(4)
% 
% FPdata = [FPrmse(1), FPmax(1); FPrmse(2), FPmax(2); FPrmse(3), FPmax(3)];
% h = bar(FPdata)
% h(1).FaceColor = [0.4 0.4 0.4];
% h(2).FaceColor = [0.85 0.85 0.85];
% 
% hold on
% ylim([0 120]);
% xlabel('False Positive (# of Clutter)');
% ylabel('Error (pixel/drone)');
% title('Effect of False Positive on Tracking Error');
% 
% legend({'RMSE','Max'});
% name = {'1','5','10'};
% set(gca,'xticklabel',name);
% 
% %%
% figure(5)
% 
% FNdata = [FNrmse(1), FNmax(1); FNrmse(2), FNmax(2); FNrmse(3), FNmax(3)];
% m = bar(FNdata)
% m(1).FaceColor = [0.4 0.4 0.4];
% m(2).FaceColor = [0.85 0.85 0.85];
% 
% hold on
% ylim([0 120]);
% xlabel('False Negative (P(detection))');
% ylabel('Error (pixel/drone)');
% title('Effect of False Negative on Tracking Error');
% 
% legend({'RMSE','Max'});
% name = {'0.97','0.95','0.93'};
% set(gca,'xticklabel',name);
% 
% %%
% figure(6)
% 
% Ndata = [Nrmse(1), Nmax(1); Nrmse(2), Nmax(2); Nrmse(3), Nmax(3)];
% n = bar(Ndata)
% n(1).FaceColor = [0.4 0.4 0.4];
% n(2).FaceColor = [0.85 0.85 0.85];
% 
% hold on
% ylim([0 120]);
% xlabel('Noise (% of std. dev)');
% ylabel('Error (pixel/drone)');
% title('Effect of Varying Noise on Tracking Error');
% 
% legend({'RMSE','Max'});
% name = {'0.25','0.50','0.75'};
% set(gca,'xticklabel',name);

%%

figure(9)

FPdata = [1.1, 1.2 ; 20, 6 ; 550, 18; 7200, 23];
h = bar(FPdata)
h(1).FaceColor = [0.4 0.4 0.4];
h(2).FaceColor = [0.85 0.85 0.85];

set(gca,'YScale','log')
hold on
box off

% ylim([5 10000]);
xlabel('Number of Drones Tracking');
ylabel('Processing Time (msec)');
% title('Effect of False Positive on Tracking Error');

legend({'JPDAF','PHD'});
pbaspect([2 1 1])

name = {'3','6','7', '8'};
set(gca,'xticklabel',name);
