% Create variables to be used later in the code
cutoff = 6; % Cutoff frequency for low-pass filter for kinematic data 
cutoff_GRF = 20; % Cutoff frequency for low-pass filter for kinetic data 

% Load data
load('Baseline.mat')

% Create time vector for the kinematic data
time_kinematics  = linspace(0,Data.Frames/Data.FrameRate,Data.Frames);

% Create time vector for the force data
time_kinetics = linspace(0,Data.Force(1).NrOfSamples/Data.Force(1).Frequency,Data.Force(1).NrOfSamples);

% Create separate matrices for x, y, and z components of the marker data
for i=1:Data.Trajectories.Labeled.Count % i is the marker number
    Marker_x(:,i) = squeeze(Data.Trajectories.Labeled.Data(i,1,:));
    Marker_y(:,i) = squeeze(Data.Trajectories.Labeled.Data(i,2,:));
    Marker_z(:,i) = squeeze(Data.Trajectories.Labeled.Data(i,3,:));
end

%% Compute and plot the thigh angle
% Calculate thigh angle - see Biomechanics, Winter (CH3, p76)
R_Thigh_Sagittal = Cal_angle(Marker_x(:,1),Marker_z(:,1),Marker_x(:,2),Marker_z(:,2));
R_Shank_Sagittal = Cal_angle(Marker_x(:,2),Marker_z(:,2),Marker_x(:,3),Marker_z(:,3));

% Plot the thigh, shank, and knee angles
figure
subplot(1,3,1), plot(time_kinematics,R_Thigh_Sagittal), xlim([2 4]), xlabel('Time (s)'), ylabel('Thigh Angle (deg)')
subplot(1,3,2), plot(time_kinematics,R_Shank_Sagittal), xlim([2 4]), xlabel('Time (s)'), ylabel('Shank Angle (deg)')
subplot(1,3,3), plot(time_kinematics,R_Thigh_Sagittal - R_Shank_Sagittal), xlim([2 4]), xlabel('Time (s)'), ylabel('Knee Angle (deg)')



%--------------------------------------------------------------------------
function output = Cal_angle(x1,z1,x2,z2)
% This function calculates the angle in degrees (relative to horizontal) of
% a vector between the points(x1,z1) and (x2,z2). If the angle is greater
% than 180 degrees, atan2d gives a negative value, so we add 360 to make
% sure that the angle ranges from 0 to 360 deg

output = atan2d((z2-z1),(x2-x1));
output(output<0) = output(output<0)+360;

end