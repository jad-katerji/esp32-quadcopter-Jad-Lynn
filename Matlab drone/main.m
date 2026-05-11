clc; clear; close all;
addpath('C:\Users\HP\Documents\MATLAB\Matlab drone\library')
%% SECTION 1: INITIAL SETTINGS & PARAMETERS
simulationTime = 60; 
shakeTime = 7.5; 
dt = 0.01; 
R2D = 180/pi; 

% Physical Parameters
params = containers.Map({'Mass', 'armlength', 'Ixx', 'Iyy', 'Izz'}, ...
                        {0.866, 0.18, 0.0232, 0.0232, 0.0468});

% PID Gains
gains = containers.Map({'P_phi','I_phi','D_phi', 'P_thetta','I_thetta','D_thetta', ...
                        'P_psi','I_psi','D_psi', 'P_zdot','I_zdot','D_zdot'}, ...
                        {3.0, 0.01, 0.8, 3.0, 0.01, 0.8, 1.5, 0.0, 0.3, 8.0, 1.5, 4.0});

% Initialize Drone at -6m (Up)
drone1_initStates = [0, 0, -6, 0, 0, 0, 0, 0, 0, 0, 0, 0]'; 
drone1 = drone(params, drone1_initStates, [0,0,0,0]', gains, simulationTime);

% Define Drone Body for Plotting
drone1_body = [1.8, 0, 0, 1; 0,-1.8, 0, 1; -1.8, 0, 0, 1; 0, 1.8, 0, 1; 0, 0, 0, 1; 0, 0, 0.15, 1]';

%% SECTION 2: FIGURES SETUP
fig1 = figure(1); set(fig1, 'Name', '3D Visualizer', 'pos', [50 200 600 600]);
view(3); axis equal; grid on; hold on;
set(gca, 'ZDir', 'Reverse', 'YDir', 'Reverse'); 
xlim([-5,5]); ylim([-5,5]); zlim([-10,0]);
h_a13 = plot3(0,0,0, '-ro', 'LineWidth', 2);
h_a24 = plot3(0,0,0, '-bo', 'LineWidth', 2);
h_pay = plot3(0,0,0, '-k', 'LineWidth', 4);

fig2 = figure(2); set(fig2, 'Name', 'Telemetry', 'pos', [670 200 800 500]);
titles = {'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]', 'X [m]', 'Y [m]', 'Z [m]'};
for k=1:6, ax(k) = subplot(2,3,k); title(titles{k}); grid on; hold on; end

%% SECTION 3: PHASE 1 - SHAKING (Noise Injection)
noise_lvl = 0.5; 
for i = 1:floor(shakeTime/dt)
    t = i*dt;
    
    % Try to stay at -6 while being pushed
    refSig = drone1.PositionCtrl([0, 0, -6]); 
    drone1.AttitudeCtrl(refSig);
    
    % Random disturbance
    dist = randn(3,1) * noise_lvl; 
    drone1.UpdateState(dist);
    st = drone1.GetState();
    
    % Visualizer Update
    wHb = [RPY2Rot(st(7:9))', st(1:3); 0 0 0 1]; 
    pts = wHb * drone1_body; 
    set(h_a13, 'XData', pts(1,[1 3]), 'YData', pts(2,[1 3]), 'ZData', pts(3,[1 3]));
    set(h_a24, 'XData', pts(1,[2 4]), 'YData', pts(2,[2 4]), 'ZData', pts(3,[2 4]));
    set(h_pay, 'XData', pts(1,[5 6]), 'YData', pts(2,[5 6]), 'ZData', pts(3,[5 6]));
    
    % Plotting
    plot(ax(1), t, st(7)*R2D, '.r'); plot(ax(2), t, st(8)*R2D, '.g');
    plot(ax(3), t, st(9)*R2D, '.b'); plot(ax(4), t, st(1), '.k');
    plot(ax(5), t, st(2), '.k');     plot(ax(6), t, st(3), '.m');
    drawnow limitrate; 
end

%% SECTION 4: PHASE 2 - STABILIZATION (Hover)
fprintf('\nPhase 1 Complete. Type "hover" to stabilize.\n');
cmd = input('Command: ', 's');

if strcmpi(cmd, 'hover')
    % Reset integrators for a fresh start
    drone1.phi_err_sum = 0; drone1.thetta_err_sum = 0; drone1.z_err_sum = 0;
    
    for i = (floor(shakeTime/dt) + 1):floor(simulationTime/dt)
        t = i*dt;
        
        % Target: Return to origin and stay at -6m
        refSig = drone1.PositionCtrl([0, 0, -6]); 
        drone1.AttitudeCtrl(refSig);
        
        % Update Physics (Zero noise)
        drone1.UpdateState([0;0;0]); 
        st = drone1.GetState();
       
        % Visualizer Update
        wHb = [RPY2Rot(st(7:9))', st(1:3); 0 0 0 1];
        pts = wHb * drone1_body;
        set(h_a13, 'XData', pts(1,[1 3]), 'YData', pts(2,[1 3]), 'ZData', pts(3,[1 3]));
        set(h_a24, 'XData', pts(1,[2 4]), 'YData', pts(2,[2 4]), 'ZData', pts(3,[2 4]));
        set(h_pay, 'XData', pts(1,[5 6]), 'YData', pts(2,[5 6]), 'ZData', pts(3,[5 6]));
        
        % Plotting (Blue for Stabilization)
        plot(ax(1), t, st(7)*R2D, '.b'); plot(ax(2), t, st(8)*R2D, '.b');
        plot(ax(3), t, st(9)*R2D, '.b'); plot(ax(4), t, st(1), '.b');
        plot(ax(5), t, st(2), '.b');     plot(ax(6), t, st(3), '.b');
        drawnow limitrate;
    end
end