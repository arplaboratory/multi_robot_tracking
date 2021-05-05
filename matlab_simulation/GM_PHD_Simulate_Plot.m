%GM_PHD_Simulate_Plot
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 
%error_ellipse is by AJ Johnson, taken from Matlab Central http://www.mathworks.com.au/matlabcentral/fileexchange/4705-errorellipse

%This file plots the current measurements, true target position and estimated target
%position, as well as a history of estimated target positions.


%Measurements
figure(1);
clf;%Only plot the most recent measurement.
hold on;


%Plot all measurements, including clutter, as black 'x'
if(~isempty(Z))
    plot(Z(1,:), Z(2,:), 'xk');
end
%Plot noisy measurements of true target position(s), as black 'o'
if(~isempty(zTrue))
    plot(zTrue(1,:), zTrue(2,:), 'ok');
end

% Plot target 1 true position as red dots
plot(simTarget1History(1,:), simTarget1History(2,:), 'xk');
%Plot target 2 true position as blue dots
plot(simTarget2History(1,:), simTarget2History(2,:), 'xk');
%Plot target 3 true position as green dot
plot(simTarget3History(1,:), simTarget3History(2,:), 'xk');

%Plot target 1 track position as red dots
plot(X_k_history(1,1:3:end), X_k_history(2,1:3:end), '.r');
%Plot target 2 track position as blue dots
plot(X_k_history(1,2:3:end), X_k_history(2,2:3:end), '.b');
%Plot target 3 track position as green dots
plot(X_k_history(1,3:3:end), X_k_history(2,3:3:end), '.g');

% 
% %Plot tracked targets as magenta circles
% if(~isempty(X_k_history))
%     plot(X_k_history(1,:), X_k_history(2,:), 'om');
% end
xlabel('X position');
ylabel('Y position');
title('Simulated targets and measurements');
axis square;

%For extracted targets, plot latest target(s) as cyan triangle, and draw an
%error ellipse to show uncertainty
if(~isempty(X_k))
%     plot(X_k(1,:), X_k(2,:), '^c');
    [nRows, nCols] = size(X_k);
    for c = 1:nCols
       thisMu = X_k(1:2, c);
       covRange = calculateDataRange4(c);
       thisCov = X_k_P(:,covRange);
       thisCov = thisCov(1:2, 1:2); %We only care about position
       error_ellipse(thisCov, thisMu);
    end

end

%Individual X and Y components of measurements
figure(2);
hold on
subplot(2,1,1);

% plot(k, Z(1,:), 'xk');%X coord of clutter measurements
if(~isempty(zTrue))
    plot(k, zTrue(1,:), 'xk');
end
if(~isempty(X_k))
    plot(k, X_k(1,1), '.r');
    plot(k, X_k(1,2), '.b');
    plot(k, X_k(1,3), '.g');
end
% legend(L, {'noise','x1','x2','x3'});

subplot(2,1,2);
% plot(k, Z(2,:), 'xk');%Y coord of clutter measurements
if(~isempty(zTrue))
    plot(k, zTrue(2,:), 'xk');
end
if(~isempty(X_k))
    plot(k, X_k(2,1), '.r');
    plot(k, X_k(2,2), '.b');
    plot(k, X_k(2,3), '.g');
end
% legend('noise','y1','y2','y3');

%% error
if(~USE_REAL_DATA)
    if(k>=3)
        figure (3)
        hold on;
        axis([0 DATA_SIZE*DOWN_SAMPLE 0 100]);
        xlim([0 DATA_SIZE*DOWN_SAMPLE]);
        ylim([0 100]);
        xlabel('Time step (n)');
        ylabel('Avg Error (pixel/drone)');
        title('Error over Time');
        
        total_error = 0;
        for n = 1:NUM_DRONES
            total_error = total_error + sqrt(power(floor((zTrue(1,n)) - floor(X_k(1,n))),2) + power(floor((zTrue(2,n)) - floor(X_k(2,n))),2));
        end
        
        avg_error = total_error / NUM_DRONES;
        
        errorHistory = [errorHistory avg_error];
        kHistory = [kHistory k*DOWN_SAMPLE];
        plot(kHistory, errorHistory, 'r');
    end
end
% %% get RMSE
% if(k==DATA_SIZE)
%     xerrorArray = (X_k_history(1,1:end) - zTrueHistory(1,NUM_DRONES+1:end)).^2;
%     yerrorArray = (X_k_history(2,1:end) - zTrueHistory(2,NUM_DRONES+1:end)).^2;
%     xerrorRMSE = sqrt(sum(xerrorArray,2) / (DATA_SIZE * NUM_DRONES));
%     yerrorRMSE = sqrt(sum(yerrorArray,2) / (DATA_SIZE * NUM_DRONES));
%     RMSE = sqrt(xerrorRMSE^2 + yerrorRMSE^2)
%     
%     maxError = max(errorHistory)
% end



%% plot real data from rosbag
% axis([0 1000 0 1000]);
% xlim([0 1000]);
% ylim([0 1000]);   

%Plot target 1 track position as red dots
% plot(real_pos_array(1,k).x,  real_pos_array(1,k).y, '.r');
% %Plot target 2 track position as blue dots
% plot(real_pos_array(2,k).x, real_pos_array(2,k).y, '.b');
% %Plot target 3 track position as green dots
% plot(real_pos_akrray(3,k).x, real_pos_array(3,k).y, '.g');

