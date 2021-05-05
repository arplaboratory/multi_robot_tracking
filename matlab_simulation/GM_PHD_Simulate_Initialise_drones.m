%GM_PHD_Simulate_Initialise
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file initialises the simulation described in example 1 of Vo&Ma 2006.
%The velocity and starting position values are rough estimates obtained by visual
%inspection of the simulation. 
%They can probably be changed without things breaking.

%If you want to use this GM-PHD filter for your own problem, you will need
%to replace this script with your own.



%% Control parameters
noiseScaler = 0.0;       %Adjust the strength of the noise on the measurements by adjusting this. Useful for debugging.
nClutter = 0; %Assume constant 50 clutter measurements. Since clutter is Poisson distrbuted it might be more accurate to use nClutter = poissrnd(50) if you have the required Matlab toolbox. Constant 50 clutter works well enough for simulation purposes.

%I haven't included descriptions of every variable because their names are
%fairly self-explanatory
endTime = DATA_SIZE;%Duration of main loop

if(USE_REAL_DATA)
    simTarget1Start = [real_pos_array(1,1).x; real_pos_array(1,1).y; 0;0];
    simTarget2Start = [real_pos_array(2,1).x; real_pos_array(2,1).y; 0;0];
    simTarget3Start = [real_pos_array(3,1).x; real_pos_array(3,1).y; 0;0];
else
    simTarget1Start = [pos_array(1).x(1); pos_array(1).y(1); 0;0];
    simTarget2Start = [pos_array(2).x(1); pos_array(2).y(1); 0;0];
    simTarget3Start = [pos_array(3).x(1); pos_array(3).y(1); 0;0];

end

simTarget1Vel = [0; 0];
simTarget2Vel = [0; 0];
simTarget3Vel = [0; 0];

%History arrays are mostly used for plotting.
simTarget1History = simTarget1Start;
simTarget2History = simTarget2Start;
simTarget3History = simTarget3Start;

simMeasurementHistory = {};%We use a cell array so that we can have rows of varying length.

simTarget1State = simTarget1Start;
simTarget2State = simTarget2Start;
simTarget3State = simTarget2Start;

simTarget3SpawnTime = 0;%Target 3 is spawned from target 1 at t = 66s.

%Set up for plot
%Measurements and targets plot
figure(1);
clf;
hold on;
axis([0 1000 0 1000]);
xlim([0 1000]);
ylim([0 1000]);

errorHistory= [];
kHistory = [];
zTrueHistory = [];
maxError = 0;

%X and Y measurements plot
xlabel('X image');
ylabel('Y image');
title('Detected targets and measurements');
axis square;

axismax = 300;
if(USE_REAL_DATA)
    axismax = 1000;

end

figure(2);
subplot(2,1,1);
hold on;
axis([0 DATA_SIZE 0 axismax]);
xlabel('Simulation step');
ylabel('X measured (pixel)');
title('Measured X ');
subplot(2,1,2);
hold on;
axis([0 DATA_SIZE 0 axismax]);
xlabel('Simulation step');
ylabel('Y measured (pixel)');
title('Measured Y ');

