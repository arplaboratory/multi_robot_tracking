Asynch = readmatrix('/home/marklee/Desktop/RAL2021/Asynch_fig8_v2.csv');
Synch = readmatrix('/home/marklee/Desktop/RAL2021/Synch_fig8v2.csv');

GND_x1 = Asynch(:,3);
est_x1 = Asynch(:,7);

GND_syn_x1 = Synch(:,3);
est_syn_x1 = Synch(:,7);

hold on 
plot(GND_x1)
hold on
plot(est_x1)

% hold on 
% plot(GND_syn_x1)
% hold on
% plot(est_syn_x1)



