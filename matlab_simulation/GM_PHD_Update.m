%GM_PHD_Update
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file performs a PHD filter update on the targets
%This is basically a brute-force Kalman update of every target with every
%measurement and creating a new target from the update results.

s = sprintf('Step 4: Performing update.');
disp(s);

%1. Set up matrices for post-update filter state
w_k = zeros(1, numTargets_Jk_k_minus_1 * size(Z, 2) + numTargets_Jk_k_minus_1);
m_k = zeros(4, numTargets_Jk_k_minus_1 * size(Z, 2) + numTargets_Jk_k_minus_1);
P_k =  zeros(4, 4 * (numTargets_Jk_k_minus_1 * size(Z, 2) + numTargets_Jk_k_minus_1));

%2. First we assume that we failed to detect all targets.
%We scale all weights by probability of missed detection
%We already did the prediction step for these so their position &
%covariance will have been updated. What remains is to rescale their
%weight.
for j = 1:numTargets_Jk_k_minus_1
    w_k(j) = (1 - prob_detection) * wk_k_minus_1(j); 
    m_k(:,j) = mk_k_minus_1(:,j);    
    P_range = calculateDataRange4(j);
    newP = Pk_k_minus_1(:,P_range);
    P_k(:,P_range) = newP;
end

%3. Now we update all combinations of matching every observation with every target in the
%map. 
%First we expand the observation to include velocity (by simple v = delta_x/delta_t for 
%delta_x = measured_position - position_before_prediction. 
%That is, how fast would the target have needed
%to move to get from where it was to where it was seen now?
L = 0; 
for zi = 1:size(Z,2)
    L = L + 1;%L is used to calculate an offset from previous updates. It maxes out at L = number_of_measurements. A more elegant solution would be to set L = zi but I retain this method for compatibility with Vo&Ma
    
    for j = 1:numTargets_Jk_k_minus_1

        m_j = mk_k_minus_1(:,j);
        %Augment the measurement of position with calculated velocity
        thisZ = Z(:,zi);%This consists of only the observed position. But we need to extract the equivalent velocity observation
%         prevX = mk_k_minus_1_before_prediction(1:2,j);%Get the old (pre-prediction) position of the target
%         thisV = (thisZ - prevX) / dt;%velocity = dx / dt. Since Z and x are 2d, V = [Vx Vy]
        thisV = [0;0];
        thisZ = [thisZ; thisV];%So we pretend to observe velocity as well as position

        %If this is the first time a target has been observed, it will have
        %no velocity stored.
        %Therefore it will be missing a prediction update, which will
        %impact both its position and its covariance.
        thisIndex = L * numTargets_Jk_k_minus_1 + j;
        
        old_P_range = calculateDataRange4(j);%Returns 4 columns
        new_P_range = 4 * L * numTargets_Jk_k_minus_1 + old_P_range;

        %Recalculate weight.
        %weightDataRange is used to control which dimensions we want to
        %reweight over. In my experience, reweighting over all four
        %produces unacceptably low weights most of the time, so we reweight
        %over 2D.
        
       
        w_new = prob_detection * wk_k_minus_1(j) * mvnpdf(thisZ(weightDataRange), eta(weightDataRange,j), S(weightDataRange,old_P_range(weightDataRange)));%Hoping normpdf is the function I need
        w_k(thisIndex) = w_new;

        %Update mean
        delta = thisZ - eta(:,j);
        K_range = calculateDataRange4(j);
        m_new = m_j;
        m_new(1:2) = m_j(1:2) +  K(1:2,K_range(1:2)) * delta(1:2);
        m_k(:,thisIndex) = m_new;

        %this is original velocity
%         m_new = m_j +  K(:,K_range) * delta;
%         m_k(:,thisIndex) = m_new;
        
        %Update covariance
        P_new = P_k_k(:,old_P_range);
        P_k(:,new_P_range) = P_new;
        


    end

    %Sum up weights for use in reweighting
    weight_tally = 0;
    for i = 1:numTargets_Jk_k_minus_1
        thisIndex = L * numTargets_Jk_k_minus_1 + i;
        weight_tally = weight_tally + w_k(thisIndex);
    end
    

    for j = 1:numTargets_Jk_k_minus_1
        old_weight = w_k(L * numTargets_Jk_k_minus_1 + j);
        measZ = [Z(1,zi), Z(2,zi)];
        new_weight = old_weight / (clutter_intensity(measZ) + weight_tally);%Normalise

        w_k(L * numTargets_Jk_k_minus_1 + j) = new_weight;
        

    end

end


s= sprintf('\t Jk_targets: %d , Jk_k-1: %d  ',numTargets_Jk, numTargets_Jk_k_minus_1);
disp(s);


