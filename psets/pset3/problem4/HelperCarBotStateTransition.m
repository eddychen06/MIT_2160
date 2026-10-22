function predictParticles = HelperCarBotStateTransition(pf, prevParticles, dT, u) %#ok<INUSL>
% HelperCarBotStateTransition  Propagation step of the particle filter.
%
%   Inputs : wheel turns       u_t = [ds_R ; ds_L]
%   States : robot pose        X_t = [X ; Y ; Theta]
%   Dynamics:  X_{t+1} = f(X_t,u_t) + G(X_t,u_t) w_t
%   Process noise covariance:
%       Q_t = E[w_t w_t'] = [ kR*|ds_R|      0       ;
%                                 0      kL*|ds_L| ]
%   Each particle gets its own draw of w_t, so the cloud spreads.
%   prevParticles is n x 3; predictParticles is n x 3.

    % make sure these parameters match those in HelperCarBot.m
    b  = 0.2;      % track width [m]      
    kR = 1e-4;     % right-wheel slip coefficient [m]   
    kL = 1e-4;     % left-wheel  slip coefficient [m]   -

    n     = size(prevParticles,1);
    x     = prevParticles(:,1);
    y     = prevParticles(:,2);
    theta = prevParticles(:,3);

    dsR = u(1);
    dsL = u(2);

    % f(X_t,u_t)
    v      = 0.5*(dsR + dsL);       % forward displacement over the step
    dTheta = (dsR - dsL)/b;         % rotation over the step
    mid    = theta + 0.5*dTheta;    % heading half a step ahead

    xf  = x     + v*cos(mid);
    yf  = y     + v*sin(mid);
    thf = theta + dTheta;

    % G(X_t,u_t) w_t 
    % G = df/d(ds_R,ds_L), evaluated at this particle's pose.
    c = cos(mid);  
    s = sin(mid);  
    g = v/(2*b);

    G11 = 0.5*c - g*s;   G12 = 0.5*c + g*s;
    G21 = 0.5*s + g*c;   G22 = 0.5*s - g*c;
    %  third row is constant: [ 1/b , -1/b ]


    wR = sqrt(kR*abs(dsR))*randn(n,1);
    wL = sqrt(kL*abs(dsL))*randn(n,1);

    predictParticles      = zeros(n,3);
    predictParticles(:,1) = xf  + G11.*wR + G12.*wL;
    predictParticles(:,2) = yf  + G21.*wR + G22.*wL;
    predictParticles(:,3) = thf + (wR - wL)/b;

end
