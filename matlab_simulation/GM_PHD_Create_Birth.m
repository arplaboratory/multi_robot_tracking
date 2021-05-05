%GM_PHD_Create_Birth
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file performs the processing on measurements to extract new targets
%and populates the birth lists (mean, weight, covariance) needed to instantiate them next iteration.

%This is not formally spelt out in Vo&Ma so I have used my best judgement to
%come up with a method to initialise targets. Targets are initialised as
%static (using the position of current measurements) and/or dynamic (using
%two generations of measurements and calculating the movement between).
%If only static targets are added, the number of new targets = the number
%of measurements. 
%If only dynamic targets are added, the number of new targets = the number of
%measurements this teration * the number of measurements last iteration
%If both static and dynamic targets are added, the number of new targets is
%equal to the sum of these.
disp('Step 7: Creating new targets from measurements, for birthing next iteration');
w_birth = [];
m_birth = [];
P_birth = [];
w_spawn = [];
m_spawn = [];
P_spawn = [];
numBirthedTargets = 0;
numSpawnedTargets = 0;

%We create targets using two generations of measurements.
%The first generation is used to calculate the velocity (dx/dt) to the
%second generation.
%The second generation gives the position.
%We also add a set of targets from the second generation but with velocity
%zero, since some may be unlinked to the first generation, but we have no
%velocity information for them.
if(k >= 2)%If we want to add targets with initial velocities.If only one iteration complete, cannot calculate velocity
    %Each measurement consists of 2 rows
    thisMeasRowRange = k;
    prevMeasRowRange = k-1;

    thisMeas = simMeasurementHistory{thisMeasRowRange};
    prevMeas = simMeasurementHistory{prevMeasRowRange};
    
    %% update birth mean from estimated states
    birth_mean1 = X_k(:,1);
    birth_mean2 = X_k(:,2);
    birth_mean3 = X_k(:,3);
    
    for j_this = 1:size(thisMeas,2) 
        
        m_this = thisMeas(:,j_this);
%         m_prev = prevMeas(:, j_this);
        
%         birthWeight = birth_intensity(m_thi1s);
        birthWeight = 1e-5;    
        w_i = birthWeight;
        %Initialise the covariance
        P_i = covariance_birth;
        
%         thisV = (m_this(1:2) - m_prev(1:2)) / dt;
        thisV = [0;0];
        m_i = [m_this; thisV];
        
        w_birth = [w_birth, w_i];
        m_birth = [m_birth m_i];
        P_birth = [P_birth, P_i];
        
        numBirthedTargets = size(thisMeas,2);
        
    end
    
   
    %% 
    
%     for j_this = 1:size(thisMeas,2)            
%         for j_prev = 1:1:size(prevMeas,2)%Match every pair from previous to current
%             m_this = thisMeas(:,j_this);
%             m_prev = prevMeas(:, j_prev);
%             %Calculate and add the velocity.
%             m_i = m_this;
%             thisV = (m_this(1:2) - m_prev(1:2)) / dt;
%             if(abs(thisV(1)) > MAX_V) || (abs(thisV(2)) > MAX_V)
%                 continue;%To reduce the number of targets added, we filter out the targets with unacceptable velocities.
%             end
% 
%             m_i = [m_i; thisV];
% 
%             %Decide if the target is birthed (from birth position)
%             %or spawned (from an existing target)
%             %Initialise the weight to birth
%             birthWeight = birth_intensity(m_i);
% 
%                    
%             %Birth the target
%             w_i = birthWeight;
%             %Initialise the covariance
%             P_i = covariance_birth;
%             w_birth = [w_birth, w_i];
%             m_birth = [m_birth m_i];
%             P_birth = [P_birth, P_i];
%             numBirthedTargets = numBirthedTargets + 1;
%             
%         end
%     end
end



