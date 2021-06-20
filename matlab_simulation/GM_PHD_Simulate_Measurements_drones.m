%GM_PHD_Simulate_Measurements
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file generates simulated measurement data for  the simulation
%described in Vo&Ma.
%There will be gaussian noise on the measurement and Poisson-distributed clutter
%in the environment. 

%If you want to use this PHD filter implementation for another problem, you
%will need to replace this script with another one that populates Z,
%zTrue, and simMeasurementHistory (Z is used in a lot of the update code,
%zTrue and simMeasurementHistory are used in GM_PHD_Simulate_Plot)

%Note: It is possible to get no measurements if the target is not detected
%and there is no clutter
s = sprintf('Step Sim: Simulating measurements.');
disp(s);

if(USE_REAL_DATA)
    pos_array(1).x(k) = real_pos_array(1,k).x;
    pos_array(2).x(k) = real_pos_array(2,k).x;
    pos_array(3).x(k) = real_pos_array(3,k).x;
    
    pos_array(1).y(k) = real_pos_array(1,k).y;
    pos_array(2).y(k) = real_pos_array(2,k).y;
    pos_array(3).y(k) = real_pos_array(3,k).y;
end
%Simulate target movement
if k > 1
   simTarget1Vel = [(pos_array(1).x(k)-pos_array(1).x(k-1))/dt; (pos_array(1).y(k)-pos_array(1).y(k-1))/dt];
   simTarget2Vel = [(pos_array(2).x(k)-pos_array(2).x(k-1))/dt; (pos_array(2).y(k)-pos_array(2).y(k-1))/dt];
   simTarget3Vel = [(pos_array(3).x(k)-pos_array(3).x(k-1))/dt; (pos_array(3).y(k)-pos_array(3).y(k-1))/dt];   
else
    simTarget1Vel = [0;0];
    simTarget2Vel = [0;0];
    simTarget3Vel = [0;0];
end
simTarget1State = [pos_array(1).x(k); pos_array(1).y(k); simTarget1Vel]; 
simTarget2State = [pos_array(2).x(k); pos_array(2).y(k); simTarget2Vel]; 
simTarget3State = [pos_array(3).x(k); pos_array(3).y(k); simTarget3Vel]; 

length_imu_data = length(imu_array);
length_pose_data = length(pos_array(1).x);
data_index_imu = floor(length_imu_data/length_pose_data);

simCameraVel = [imu_array(data_index_imu*k,1); imu_array(data_index_imu*k,2); imu_array(data_index_imu*k,3)];

%Save target movement for plotting
simTarget1History = [simTarget1History, simTarget1State];
simTarget2History = [simTarget2History, simTarget2State];
simTarget3History = [simTarget3History, simTarget3State];

%First, we generate some clutter in the environment.
clutter = zeros(2,nClutter);%The observations are of the form [x; y]
for i = 1:nClutter
    clutterX = rand * (xrange(2) - xrange(1)) + xrange(1); %Random number between xrange(1) and xrange(2), uniformly distributed.
    clutterY = rand * (yrange(2) - yrange(1)) + yrange(1); %Random number between yrange(1) and yrange(2), uniformly distributed.
    
    clutter(1,i) = clutterX;
    clutter(2,i) = clutterY;
end

%Generate true measurement without noise
measX1 = simTarget1State(1);
measY1 = simTarget1State(2);
measX2 = simTarget2State(1);
measY2 = simTarget2State(2);
measX3 = simTarget3State(1);
measY3 = simTarget3State(2);
Z = [ [measX1 measX2 measX3]; [measY1 measY2 measY3] ];

zTrue = Z;%Store for plotting


%We are not guaranteed to detect the target - there is only a probability

detect1 = rand;
detect2 = rand;
detect3 = rand;

if(detect1 > prob_detection)
    measX1 = [];
    measY1 = [];
else
    measX1 = simTarget1State(1) + sigma_r * randn * noiseScaler;
    measY1 = simTarget1State(2) + sigma_r * randn * noiseScaler;
end
if(detect2 > prob_detection)
    measX2 = [];
    measY2 = [];
else
    measX2 = simTarget2State(1) + sigma_r * randn * noiseScaler;
    measY2 = simTarget2State(2) + sigma_r * randn * noiseScaler;
end

if(detect3 > prob_detection)
    measX3 = [];
    measY3 = [];
else
    measX3 = simTarget3State(1) + sigma_r * randn * noiseScaler;
    measY3 = simTarget3State(2) + sigma_r * randn * noiseScaler;
end


%Generate measurement
Z = [ [measX1 measX2 measX3]; [measY1 measY2 measY3] ];


%% drop measurements
% if ( rem(k,30) == 0 )
%     measX1 = [];
%     measY1 = [];
% end
%%

%Generate  measurement
Z = [ [measX1 measX2 measX3]; [measY1 measY2 measY3] ];
% if (~all(Z, 'all'))
%     stopHere = 0;
% end


%Append clutter
Z = [Z, clutter];
zTrueHistory = [zTrueHistory, zTrue];

%Store history
simMeasurementHistory{k} =  Z;