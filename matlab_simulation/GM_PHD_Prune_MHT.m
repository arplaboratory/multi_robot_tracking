
%GM_PHD_Prune
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file performs merging and pruning after the PHD update.
%The PHD update creates a combinatorial explosion in the number of targets.
%We prune the low-weight ones and merge the close-together ones. The weight
%threshold T and distance threshold L that control this process are set in
%GM_PHD_Initialisation.
s = sprintf('Step 5: Prune and merge targets.');
disp(s);


%% Prune out the low-weight targets
I = find(w_k >= T);%Find targets with high enough weights
if(VERBOSE == 1)
    s = sprintf('The only tracks with high enough weights are:');
    disp(s);
    disp(I);
end

ICopy = [];
ICopy = I;

%% Merge the close-together targets
l = 0;%Counts number of features
w_bar_k = [];
m_bar_k = [];
P_bar_k = [];
indexOrder = [];

Z;
w_k;
m_k;

if k >= 115
    stopHere = 0;
end

while isempty(I) == false %We delete from I as we merge
    l = l + 1;
    %Find j, which is i corresponding to highest w_k for all i in I
    if isempty(ICopy)
        break;
    end
    
    if l >=4
       break; 
    end
    highWeights = w_k(ICopy);
    [maxW, j] = max(highWeights);
    j = j(1); %In case of two targets with equal weight
    %j is an index of highWeights (i.e. a position in I)
    %We want the index in w_k
    j = ICopy(j);
    
    %Find all points with Mahalanobis distance less than U from point
    %m_k(j)
    L = [];%A vector of indexes of w_k
    for iterateI = 1:length(I)
        thisI = I(iterateI);

%                 delta_m = m_k(:,thisI) - m_k(:,j); %use all of state
        %         include pos&vel
        delta_m = m_k(1:2,thisI) - m_k(1:2,j);
        P_range = calculateDataRange2(thisI);
        mahal_dist = delta_m' * (P_k(1:2,P_range) \ delta_m);%Invert covariance via left division
        if(mahal_dist <= mergeThresholdU)
            L = [L, thisI];
            
            %see if other weighted sum is used for distance 
            if (thisI - j ~= 0)
                weightedSumHere = 0;
            end
        end
    end
    
    
%     s = sprintf('\tMerging target %d with these targets:', j);
%     disp(s);
%     disp(L);
    
    %The new weight is the sum of the old weights
    w_bar_k_l = sum(w_k(L));
    
    %The new mean is the weighted average of the merged means
    m_bar_k_l = 0;
    for i = 1:length(L)
        thisI = L(i);
        m_bar_k_l = m_bar_k_l + 1 / w_bar_k_l *  (w_k(thisI) * m_k(:,thisI));
    end
   
    %Calculating covariance P_bar_k is a bit trickier
    P_val = zeros(4,4);
    for i = 1:length(L)
        thisI = L(i);
        delta_m = m_bar_k_l - m_k(:,thisI);  
        P_range = calculateDataRange4(thisI);
        P_val = P_val + w_k(thisI) * (P_k(:,P_range) + delta_m * delta_m');
        tmpP = P_k(:,P_range);
    end
    P_bar_k_l = P_val / w_bar_k_l;
    
    old_P_range = calculateDataRange4(j);
    oldP = P_k(:,old_P_range);
    
    %save order list to resort
%     [val, nonDuplicatedIndex] = max( w_k(L) );
    nonDuplicatedIndex = j;
    indexOrder = [indexOrder nonDuplicatedIndex]; 

    %Now delete the elements in L from I
    for i = 1:length(L)
        iToRemove = find(I == L(i));
        I(iToRemove) = [];
    end
    
    %delete every Nth of ICopy
    drone_index_to_delete = rem(j,numTargets_Jk_k_minus_1);
    for i = 1:length(ICopy)
        if rem(ICopy(i),numTargets_Jk_k_minus_1) == drone_index_to_delete
            ICopy(i) = 0;
        end
    end
    
    ICopy = nonzeros(ICopy)';
    
    
    
    
    %Append the new values to the lists
    w_bar_k = [w_bar_k, w_bar_k_l];
    m_bar_k = [m_bar_k, m_bar_k_l];
    P_bar_k = [P_bar_k, P_bar_k_l];
    
    
end

indexOrder;
%% sort out mixed association
if ( k>=1 )
    
    %bring down to index order
    newIndex = rem(indexOrder,numTargets_Jk_k_minus_1);
    
    %change remainder for 0 index
    for i = 1:length(newIndex)
        if ( newIndex(i) == 0 )
            newIndex(i) = NUM_DRONES;
        end
    end
    %change remainder for rem bigger than NUMDRONES but less than numTarg
    for i = 1:length(newIndex)
        if ( newIndex(i) > NUM_DRONES )
            newIndex(i) = rem(newIndex(i),NUM_DRONES);
        end
    end
    
    
    
    %resort
    for i = 1:length(newIndex) %in case missed detection
        if i > NUM_DRONES %length(Z) %in case len(newIndex)=4 and len(Z)=4, dont want to overwrite good with low prob noise 
            %len(newIndex)=6 and len(Z)=2, dont want to write more than we
            %have to
            continue;
        end
        
 
        
        
        sortedIndex = newIndex(i);
        w_bar_k_fixed(sortedIndex) = w_bar_k(i);
        m_bar_k_fixed(:,sortedIndex) = m_bar_k(:,i);
        
        index4 = calculateDataRange4(sortedIndex);
        index4_old = calculateDataRange4(i);
        
        P_bar_k_fixed(:,index4) = P_bar_k(:,index4_old);
        
    end  
end


%%


numTargets_J_pruned = size(w_bar_k_fixed,2);%The number of targets after pruning

%Here you could do some check to see if numTargets_J_pruned > maxGaussiansJ
%and if needed delete some of the weaker gaussians. I haven't bothered but
%it might be useful for your implementation.

numTargets_Jk_minus_1 = numTargets_J_pruned;%Number of targets in total, passed into the next filter iteration
%Store the weights, means and covariances for use next iteration.
% wk_minus_1 = w_bar_k; %Weights from this iteration
% mk_minus_1 = m_bar_k; %Means from this iteration
% Pk_minus_1 = P_bar_k; %Covariances from this iteration

%moved to updating after Estimate b/c good for using stored data if dropped
%estimate
% wk_minus_1 = w_bar_k_fixed; %Weights from this iteration
% mk_minus_1 = m_bar_k_fixed %Means from this iteration
% Pk_minus_1 = P_bar_k_fixed; %Covariances from this iteration
