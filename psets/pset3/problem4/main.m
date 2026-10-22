% Particle filter localisation of a differential-drive robot
% Copyright 2015 The MathWorks, Inc.  Modified 2026.

clear;
clear classes;
clc;
close all;

particleCounts = [10 100 1000 10000];
rmsePose = zeros(4,3);
rmsePosition = zeros(4,1);

for trial = 1:numel(particleCounts)

    rng('default'); % for repeatable result
    dt = 0.05;      % time step
    tEnd = 22;      % simulation length [s]
    
    initialPose = [0  0  0]';   % [X; Y; Theta]
    carbot = HelperCarBot(initialPose, dt);
    
    GPS_sd = 0.5;  % standard deviation of the GPS. Try varying this parameter!
    
    pf = stateEstimatorPF;
    num_particles = 1000;  % part d
    
    % Particle state is the pose, X = [X Y Theta]
    initialize(pf, num_particles, initialPose', eye(3), 'CircularVariables',[0 0 1]);
    pf.StateEstimationMethod = 'mean';
    pf.ResamplingMethod = 'systematic';
    
    policy = resamplingPolicyPF;
    policy.TriggerMethod = 'interval';
    policy.SamplingInterval = 20;   % part c
    pf.ResamplingPolicy = policy;
    
    % defines how particles evolve without measurement
    pf.StateTransitionFcn = @HelperCarBotStateTransition;
    
    % defines how measurement affect the estimation
    pf.MeasurementLikelihoodFcn = @HelperCarBotMeasurementLikelihood;
    
    % Last best estimation for X, Y and Theta
    lastBestGuess = [0 0 0];
    
    r = rateControl(1/dt);
    reset(r);
    
    simulationTime = 0;
    
    % store simulation results
    N = ceil(tEnd/dt) + 1;
    tLog     = nan(N,1);
    truePose = nan(N,3);     % [X Y Theta] ground truth
    estPose  = nan(N,3);     % [X Y Theta] PF estimate
    covLog   = nan(3,3,N);   % PF covariance
    k = 0;
    
    while simulationTime < tEnd % if time is not up
    
        vCmd = 0.7*abs(sin(simulationTime)) + 0.1;   % intended forward velocity
        wCmd = 0.20*sin(simulationTime);             % intended angular velocity
    
        uCmd(1) = (vCmd + wCmd*carbot.b/2)*dt;       % ds_R, right wheel turn
        uCmd(2) = (vCmd - wCmd*carbot.b/2)*dt;       % ds_L, left  wheel turn
    
        % NOTE there will be some discrepancy between the commanded wheel turns
        % and the ones the robot actually executes, because the wheels slip.
        drive(carbot, uCmd);
    
        % Propagate the carbot pose based on the motion model and command
        [statePred, covPred] = predict(pf, dt, uCmd);
    
        % Get GPS reading
        measurement = HelperCarBotGetGPSReading(carbot, GPS_sd);
    
        % If GPS measurement is available, update
        if ~isempty(measurement)
            [stateCorrected, covCorrected] = correct(pf, measurement', GPS_sd);
        else
            stateCorrected = statePred;
            covCorrected = covPred;
        end
    
        lastBestGuess = stateCorrected(1:3);
    
        k = k + 1;
        tLog(k)       = simulationTime + dt;
        truePose(k,:) = reshape(carbot.Pose(1:3), 1, 3);
        estPose(k,:)  = reshape(stateCorrected(1:3), 1, 3);
        covLog(:,:,k) = covCorrected(1:3,1:3);
    
        % Update plot
        if ~isempty(get(groot,'CurrentFigure')) % if figure is not prematurely killed
            updatePlot(carbot, pf, lastBestGuess, simulationTime);
        else
            break
        end
    
        waitfor(r);
    
        % Update simulation time
        simulationTime = simulationTime + dt;
    end

    err = estPose(1:k,:) - truePose(1:k,:);
    err(:,3) = atan2(sin(err(:,3)), cos(err(:,3)));
    rmsePose(trial,:) = sqrt(mean(err.^2, 1));
    rmsePosition(trial) = sqrt(mean(sum(err(:,1:2).^2, 2)));
end

t = tLog(1:k);
truth = truePose(1:k,:);
estimate = estPose(1:k,:);

figure;
hold on;
hTrue = plot3(truth(:,1), truth(:,2), truth(:,3),'k-', 'LineWidth', 1.5);
hEst = plot3(estimate(:,1), estimate(:,2), estimate(:,3), 'b--', 'LineWidth', 1.5);

[sx, sy, sz] = sphere(20);
S = [sx(:)'; sy(:)'; sz(:)'];

for time = 1:floor(t(end))
    [~, idx] = min(abs(t - time));

    P = covLog(:,:,idx);
    P = (P + P')/2;  
    [V, D] = eig(P);

    E = estimate(idx,:)' + V * diag(sqrt(max(diag(D),0))) * S;

    surf(reshape(E(1,:), size(sx)), ...
         reshape(E(2,:), size(sy)), ...
         reshape(E(3,:), size(sz)), ...
         'FaceColor', [0.2 0.7 0.3], ...
         'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
         'HandleVisibility', 'off');
end

xlabel('x (m)');
ylabel('y (m)');
zlabel('\theta (rad)');
legend([hTrue, hEst], {'True pose', 'PF estimate'});
title('Pose trajectories and 1\sigma ellipsoids every second');
grid on;
view(3);

figure;
tiledlayout(2,2);

labels = {'x RMSE (m)', 'y RMSE (m)', '\theta RMSE (rad)'};

for j = 1:3
    nexttile;
    semilogx(particleCounts, rmsePose(:,j), '-o', ...
        'LineWidth', 1.5);
    xlabel('Number of particles');
    ylabel(labels{j});
    xticks(particleCounts);
    grid on;
end

nexttile;
semilogx(particleCounts, rmsePosition, '-o', 'LineWidth', 1.5);
xlabel('Number of particles');
ylabel('Position RMSE (m)');
xticks(particleCounts);
grid on;

sgtitle('RMSE versus number of particles');

results = table(particleCounts', ...
    rmsePose(:,1), rmsePose(:,2), rmsePose(:,3), rmsePosition, ...
    'VariableNames', {'Particles', 'RMSE_x', 'RMSE_y', ...
                     'RMSE_theta', 'RMSE_position'});
disp(results);