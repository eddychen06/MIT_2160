%% Function Definitions
f = @(x1,x2) [x2.^2 .* cos(x1); x2 .* sin(x1)];

%% Generate Dataset
x1_bounds = [-pi, pi];
x2_bounds = [-pi, pi];

% Reset RNG (Gives Consistent Results)
rng("default");
data_set.N = 1e5;
data_set.x = ...
    [(x1_bounds(2)-x1_bounds(1))*rand(1,data_set.N)+x1_bounds(1);...
    (x2_bounds(2)-x2_bounds(1))*rand(1,data_set.N)+x2_bounds(1)];
data_set.y = f(data_set.x(1,:),data_set.x(2,:));

%% Separate Training Set, Validation Set, and Test Set
% What Fraction of Your Dataset do you want to use for validation?
holdout.validation = 0.15;
% What Fraction of Your Dataset do you want to use for testing?
holdout.test = 0.15;

holdout.validation_lim = round(data_set.N*holdout.validation);
holdout.test_lim = holdout.validation_lim + round(data_set.N*holdout.test);
holdout.validation_idx = 1:holdout.validation_lim;
holdout.test_idx = (holdout.validation_lim+1):holdout.test_lim;
holdout.training_idx = (holdout.test_lim+1):data_set.N;
% NOTE: Splitting up the dataset in a non-random way is only OK because the
% dataset itself was constructed randomly.  If you have a dataset that has
% some non-random structure to it, then you need to account for this by
% performing you splitting randomly.  There are also automatic tools that
% do this.
holdout.XTrain = data_set.x(:,holdout.training_idx).';
holdout.XTest = data_set.x(:,holdout.test_idx).';
holdout.XValid = data_set.x(:,holdout.validation_idx).';
holdout.YTrain = data_set.y(:,holdout.training_idx).';
holdout.YTest = data_set.y(:,holdout.test_idx).';
holdout.YValid = data_set.y(:,holdout.validation_idx).';

%% Design Neural Network Model
layers = [
    featureInputLayer(2)
    fullyConnectedLayer(5)
    tanhLayer
    fullyConnectedLayer(10)
    tanhLayer
    fullyConnectedLayer(15)
    reluLayer
    fullyConnectedLayer(10)
    reluLayer
    fullyConnectedLayer(5)
    tanhLayer
    fullyConnectedLayer(2)
    regressionLayer
    ];

%net = dlnetwork(layers);

%figure(1);clf;
%plot(net);

%% Training Options
options = trainingOptions("adam",...
    "Plots","training-progress",...
    'ValidationData',{holdout.XValid, holdout.YValid}, ...
    'ValidationFrequency',30);
%lossFcn = "mse";

%% Train Model
net = trainNetwork(holdout.XTrain,holdout.YTrain,layers,options);

%% Evaluate Results
testPredictions = predict(net,holdout.XTest);
testRMSE = sqrt(mean(vecnorm(testPredictions-holdout.YTest,2,2).^2))

%% Evaluating Predictions for Plot
plot_res = 101;
x1_plot = linspace(x1_bounds(1),x1_bounds(2),plot_res);
x2_plot = linspace(x2_bounds(1),x2_bounds(2),plot_res);
[X1_PLOT, X2_PLOT] = meshgrid(x1_plot,x2_plot);
X1_PLOT_FLAT = reshape(X1_PLOT,1,[]);
X2_PLOT_FLAT = reshape(X2_PLOT,1,[]);
X_PLOT_FLAT = [X1_PLOT_FLAT;X2_PLOT_FLAT];

Y_PLOT_FLAT = f(X1_PLOT_FLAT,X2_PLOT_FLAT);
Y_PRED_PLOT_FLAT = predict(net,X_PLOT_FLAT.').';

Y1_PLOT_FLAT = Y_PLOT_FLAT(1,:);
Y2_PLOT_FLAT = Y_PLOT_FLAT(2,:);
Y1_PRED_PLOT_FLAT = Y_PRED_PLOT_FLAT(1,:);
Y2_PRED_PLOT_FLAT = Y_PRED_PLOT_FLAT(2,:);

Y1_PLOT = reshape(Y1_PLOT_FLAT,plot_res,plot_res);
Y2_PLOT = reshape(Y2_PLOT_FLAT,plot_res,plot_res);
Y1_PRED_PLOT = reshape(Y1_PRED_PLOT_FLAT,plot_res,plot_res);
Y2_PRED_PLOT = reshape(Y2_PRED_PLOT_FLAT,plot_res,plot_res);

Y1_ERROR_PLOT = Y1_PRED_PLOT - Y1_PLOT;
Y2_ERROR_PLOT = Y2_PRED_PLOT - Y2_PLOT;
Y_RMSE_PLOT = sqrt(Y1_ERROR_PLOT.^2 + Y2_ERROR_PLOT.^2);

%% Plotting
figure(1);clf;
subplot(1,2,1);
contourf(X1_PLOT,X2_PLOT,Y1_PLOT,20);
colorbar;
subplot(1,2,2);
contourf(X1_PLOT,X2_PLOT,Y1_PRED_PLOT,20);
colorbar;

figure(2);clf;
subplot(1,2,1);
contourf(X1_PLOT,X2_PLOT,Y2_PLOT,20);
colorbar;
subplot(1,2,2);
contourf(X1_PLOT,X2_PLOT,Y2_PRED_PLOT,20);
colorbar;

figure(3);clf;
subplot(1,2,1);
contourf(X1_PLOT,X2_PLOT,Y1_ERROR_PLOT,20);
colorbar;
subplot(1,2,2);
contourf(X1_PLOT,X2_PLOT,Y2_ERROR_PLOT,20);
colorbar;

figure(4);clf;
contourf(X1_PLOT,X2_PLOT,Y_RMSE_PLOT,20);
colorbar;

figure(5);clf;
subplot(1,2,1);
plot(holdout.YTest(:,1),testPredictions(:,1),'k.');
subplot(1,2,2);
plot(holdout.YTest(:,2),testPredictions(:,2),'k.');