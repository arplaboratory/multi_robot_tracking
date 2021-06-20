%GM_PHD_Estimate
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file estimates the positions of targets tracked by the PHD filter.
%We need to extract the likely target positions from the PHD (i.e. we need to find the peaks of the PHD).
%This is actually fairly tricky. The naive approach is to pull out targets with the
%highest weights, but this is FAR from the best approach. A large covariance will pull down
%the peak size, and when targets are close together or have high covariances, there can be
%superposition effects which shift the peak.

%This just implements the method in Vo&Ma, which is pulling out every target with a weight over  
%weightThresholdToBeExtracted (defined in GM_PHD_Initialisation). There is
%the option of repeatedly printing out targets with rounded weights greater
%than 1 (i.e. if two or more strong targets are mergde and the weight
%rounds to 2/3/4/etc, display the target at that point multiple times when
%VERBOSE is set to 1). This will NOT change filter performance as the
%extracted state estimate is not fed back into the filter.
s = sprintf('Step 6: Estimate target states');
disp(s);

%% simple version but fails for clutter, missed detection
X_k = [];
X_k_P = [];
X_k_w = [];

i = find(w_bar_k_fixed > weightThresholdToBeExtracted);
X_k = m_bar_k_fixed(:,i);
X_k_w = w_bar_k_fixed(:,i);
for j = 1:length(i)
    thisI = i(j);
    P_range = calculateDataRange4(thisI);
    
    thisP = P_bar_k_fixed(:,P_range);
    X_k_P = [X_k_P, thisP];
end

if k>=80
    stopHere = 0;
end

%% extended version to handle clutter, missed detection

if k>=2
    X_k = zeros(4,NUM_DRONES);
    X_k_P = zeros(4,4*NUM_DRONES);
    X_k_w = zeros(1,NUM_DRONES);
    
    for index = 1:length(w_bar_k_fixed)
        %if only 1 is noise in [1 2 3], add predicted state to 1
        if(w_bar_k_fixed(index) <= weightThresholdToBeExtracted)
            X_k(:,index) = mk_minus_1(:,index);
            X_k_w(index) = wk_minus_1(index);
            
            P_range = calculateDataRange4(index);
            X_k_P(:,P_range) = Pk_minus_1(:,P_range);
            
        else %extract data above threshold
            X_k(:,index) = m_bar_k_fixed(:,index);
            X_k_w(index) = w_bar_k_fixed(index);
            
            P_range = calculateDataRange4(index);
            X_k_P(:,P_range) = P_bar_k_fixed(:,P_range);
        end
    end
end



if(VERBOSE == 1)
    s = sprintf('\t%d targets beleived to be valid:', size(X_k, 2));
    disp(s);
    for i = 1:size(X_k, 2)
        P_range = calculateDataRange4(i);
       s = sprintf('\t\tTarget %d at %3.4f %3.4f, P %3.4f %3.4f, W %3.5f', i, X_k(1, i), X_k(2,i), X_k_P(1, P_range(1)), X_k_P(2, P_range(2)), X_k_w(i));
       disp(s);
    end
end


if k >= 30
    stopHere = 0;
end


%update values for next iteration (moved from prune to use in extracting
%values below threshold
wk_minus_1 = w_bar_k_fixed; %Weights from this iteration
mk_minus_1 = m_bar_k_fixed; %Means from this iteration
Pk_minus_1 = P_bar_k_fixed; %Covariances from this iteration

X_k_P = abs(X_k_P);
Pk_minus_1 = abs(Pk_minus_1);

%Store history for plotting.
X_k;
X_k_history = [X_k_history, X_k];

if k>2
    velocity = (Z - zTrueHistory(:,3*k-5:3*k-3))/dt;
%     velocity = (X_k_history(1:2,3*k-2:3*k) - X_k_history(1:2,3*k-5:3*k-3))/dt;
    %% update velocities
    for i = 1:length(w_bar_k_fixed)
        mk_minus_1(3:4,i) = velocity(:,i);
    end

end
