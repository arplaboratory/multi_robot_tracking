%GM_PHD_Predict_Existing
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file performs prediction for existing targets
s = sprintf('Step 2: Prediction for existing targets.');
disp(s);

mk_k_minus_1_before_prediction = mk_minus_1;



%create B matrix for rotation velocity
for i = 1: size(mk_minus_1,2)
    x = (Z(1,i) );
    y = (Z(2,i) );
    bcol(1,:) = [ (x-cx)*(y-cy)/f, -(x-cx)^2/f-f, y-cy ];
    bcol(2,:) = [ f+((y-cy)^2)/f,  -(x-cx)*(y-cy)/f , -x+cx];
    bcol(3,:) = [0 0 0 ];
    bcol(4,:) = [0 0 0 ];
    
    B = [B, bcol];
end


rotm_imu_cam = [0 -1 0 ; 0 0 -1; 1 0 0];
%u = [0 0 0]'; %no imu testing here
u = rotm_imu_cam*simCameraVel * dt ;
% u = simCameraVel;
u_history = [u_history u];

if k >5
    stophere =0;
end


for j = 1:size(mk_minus_1,2)
    wk_minus_1(j) = prob_survival * wk_minus_1(j);
%     mk_minus_1(:,j) = F * mk_minus_1(:,j); %without rotation velocity.
    
    %include rotation velocity (imu reading) as well
    % X_k+1 = A*X_k + B*u  
    index = (i-1)*3 +1;
    B_ang = B(:,index:index+2);
    angVelB = B(:,index:index+2)*u;
    mk_minus_1(:,j) = F * mk_minus_1(:,j) + angVelB;
%     

    P_range = calculateDataRange4(j);
    P_i = Q + F * Pk_minus_1(:,P_range) * F';
    
    prevState = mk_k_minus_1_before_prediction(:,j);
    newState = mk_minus_1(:,j);
    
    Pk_minus_1(:,P_range) = P_i;
    

end

B = [];

%% Now we combine the birthed targets with the existing ones.
%Append newly birthed targets (in m_k_minus_1) to back of old ones
wk_k_minus_1 = [wk_minus_1, w_birth ];
mk_k_minus_1 = [mk_minus_1, m_birth ];
Pk_k_minus_1 = [Pk_minus_1, P_birth ];
numTargets_Jk_k_minus_1 = numTargets_Jk_minus_1 + numBirthedTargets ; 
%Create a backup to allow for augmenting the measurement in the update
mk_k_minus_1_before_prediction = [mk_k_minus_1_before_prediction, m_birth_before_prediction];%m_birth_before_prediction also contains the spawned targets before prediction
