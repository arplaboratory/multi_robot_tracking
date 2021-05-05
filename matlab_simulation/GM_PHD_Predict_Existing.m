%GM_PHD_Predict_Existing
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file performs prediction for existing targets
s = sprintf('Step 2: Prediction for existing targets.');
disp(s);

mk_k_minus_1_before_prediction = mk_minus_1;

for j = 1:size(mk_minus_1,2)
    wk_minus_1(j) = prob_survival * wk_minus_1(j);
    mk_minus_1(:,j) = F * mk_minus_1(:,j); %Assume constant velocity.

    P_range = calculateDataRange4(j);
    P_i = Q + F * Pk_minus_1(:,P_range) * F';
    
    prevState = mk_k_minus_1_before_prediction(:,j);
    newState = mk_minus_1(:,j);
    
    Pk_minus_1(:,P_range) = P_i;
    

end


%% Now we combine the birthed targets with the existing ones.
%Append newly birthed targets (in m_k_minus_1) to back of old ones
wk_k_minus_1 = [wk_minus_1, w_birth ];
mk_k_minus_1 = [mk_minus_1, m_birth ];
Pk_k_minus_1 = [Pk_minus_1, P_birth ];
numTargets_Jk_k_minus_1 = numTargets_Jk_minus_1 + numBirthedTargets ; 
%Create a backup to allow for augmenting the measurement in the update
mk_k_minus_1_before_prediction = [mk_k_minus_1_before_prediction, m_birth_before_prediction];%m_birth_before_prediction also contains the spawned targets before prediction
