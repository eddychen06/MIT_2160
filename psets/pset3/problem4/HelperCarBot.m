classdef HelperCarBot < handle
%HELPERCARBOT Create a simulated differential-drive (two-wheeled) robot.
%
%   The robot follows the vehicle kinematics used in lecture:
%
%       v      = (ds_r + ds_l)/2            forward displacement
%       dTheta = (ds_r - ds_l)/b            rotation
%       x_{t+1}     = x_t     + v*cos(theta_t + dTheta/2)
%       y_{t+1}     = y_t     + v*sin(theta_t + dTheta/2)
%       theta_{t+1} = theta_t + dTheta
%
%   The control input is the pair of wheel turns,
%       u_t = [ds_R , ds_L]
%   and b is the track width. Pose is X_t = [X; Y; Theta].
%

% Copyright 2015 The MathWorks, Inc.
% Modified 2026 for differential-drive kinematics.

    properties(SetAccess = public)
        %Dt Time step for simulation of the robot
        Dt

        %Trace Pose history of the robot
        Trace = []

        %BGs Best estimated (guessed) robot trace
        BGs = []

        %Pose Current pose of the robot, [x; y; theta]
        Pose

        %b Track width (distance between the two drive wheels)
        b = 0.2

        %kR Right-wheel slip coefficient. Q_t = diag(kR*|ds_R|, kL*|ds_L|)
        kR = 1e-4

        %kL Left-wheel slip coefficient
        kL = 1e-4

        %NoiseStream Random stream for the robot's own wheel slip.
        NoiseStream = RandStream('twister','Seed',5)

        %isCarInRoofedArea Indicator of whether the robot is in roofed area
        isCarInRoofedArea = 0

    end

    properties(Access = private)
        %HCenter Graphics handle for the midpoint of the wheel axle
        HCenter

        %HTrajectory Graphics handle for the trajectory of the robot
        HTrajectory

        %HRoofedArea Graphics handle for roofed area
        HRoofedArea

        %HChassis Graphics handle for the body outline
        HChassis

        %HParticles Graphics handle for particles
        HParticles

        %HCaster Graphics handle for the front caster wheel
        HCaster

        %HHeading Graphics handle for the heading indicator
        HHeading

        %HLw Graphics handle for the left drive wheel
        HLw

        %HRw Graphics handle for the right drive wheel
        HRw

        %HBestGuesses Graphics handle for best current pose estimation
        HBestGuesses

        %Cnt1 Internal counter 1
        Cnt1

        %FigureHandle the handle of the figure
        FigureHandle

    end


    methods
        function obj = HelperCarBot(currentPose, dt)
            %HelperCarBot Constructor. currentPose is [x; y; theta].

            obj.Dt = dt;
            obj.Pose = currentPose(:);
            obj.Trace = obj.Pose;
            obj.Cnt1 = 0;

            obj.FigureHandle = figure('Name', 'DiffDriveBot');
            % clear the figure
            ax = axes(obj.FigureHandle);
            cla(ax)

            % customize the figure
            obj.FigureHandle.Position = [100 100 1000 500];
            axis(ax, 'equal');
            xlim(ax, [-0.1,12]);
            ylim(ax, [-1,4]);
            grid(ax, 'on');
            box(ax, 'on');

            hold(ax, 'on')
            obj.HRoofedArea = rectangle(ax, 'position', [6,0,2.5,3],'facecolor',[0.5 0.5 0.5]); % roofed area (no measurement)

            obj.HParticles = scatter(ax, 0,0,'MarkerEdgeColor','g', 'Marker', '.');

            obj.HCenter = plot(ax, obj.Pose(1), obj.Pose(2),'o'); % midpoint of the wheel axle
            obj.HTrajectory = plot(ax, obj.Trace(1,:), obj.Trace(2,:),'r'); % robot trace

            obj.HChassis = plot(ax, 0,0,'b');                              % body outline
            obj.HLw      = plot(ax, 0,0,'linewidth',3, 'color','b');        % left drive wheel
            obj.HRw      = plot(ax, 0,0,'linewidth',3, 'color','b');        % right drive wheel
            obj.HCaster  = plot(ax, 0,0,'linewidth',3, 'color',[1 0.84 0]); % front caster
            obj.HHeading = plot(ax, 0,0,'linewidth',1, 'color',[1 0.84 0]); % heading indicator

            obj.HBestGuesses = plot(ax, 0,0,'ks-'); % best guess of pose

            legend(ax, [obj.HTrajectory, obj.HBestGuesses],'actual pose','estimated pose','Location','northwest');
            text(ax, 6.75,-0.3,'roofed area');
            title(ax, 't = 0');
            xlabel(ax, 'x (m)');
            ylabel(ax, 'y (m)');
            hold(ax, 'off');

            obj.drawRobot()

        end

        function drive(obj, u)
            %DRIVE Move the robot forward one time step.
            %   u = [ds_R, ds_L] are the commanded wheel turns (path lengths)
            %   for this step. Each wheel then slips: the executed turn is
            %   the commanded one plus noise whose variance grows with the
            %   distance travelled,  var(ds) = k*|ds|.

            dsRcmd = u(1);
            dsLcmd = u(2);

            dsR = dsRcmd + sqrt(obj.kR*abs(dsRcmd))*randn(obj.NoiseStream);
            dsL = dsLcmd + sqrt(obj.kL*abs(dsLcmd))*randn(obj.NoiseStream);

            % Forward kinematics
            v      = 0.5*(dsR + dsL);     % forward displacement
            dTheta = (dsR - dsL)/obj.b;   % rotation

            theta = obj.Pose(3);
            obj.Pose(1) = obj.Pose(1) + v*cos(theta + 0.5*dTheta);
            obj.Pose(2) = obj.Pose(2) + v*sin(theta + 0.5*dTheta);
            obj.Pose(3) = theta + dTheta;

            obj.Trace = [obj.Trace, obj.Pose];

            % determine if the robot is in the roofed area
            if obj.Pose(1)>6 && obj.Pose(1)<8.5 && obj.Pose(2)>0 && obj.Pose(2)<3
                obj.isCarInRoofedArea = 1;
            else
                obj.isCarInRoofedArea = 0;
            end

        end

        function drawRobot(obj)
            %DRAWROBOT Draw the differential-drive robot at its current pose.

            bb = obj.b;
            r  = bb/6;    % wheel radius (for drawing only)

            % Geometry in the body frame: x forward, y to the left
            body = [-0.35*bb,  0.45*bb;
                     0.75*bb,  0.45*bb;
                     0.75*bb, -0.45*bb;
                    -0.35*bb, -0.45*bb;
                    -0.35*bb,  0.45*bb];

            lw     = [-r,  bb/2;  r,  bb/2];           % left drive wheel
            rw     = [-r, -bb/2;  r, -bb/2];           % right drive wheel
            caster = [0.55*bb - r/2, 0; 0.55*bb + r/2, 0];
            head   = [0, 0; 0.70*bb, 0];               % heading indicator

            body   = obj.body2world(body);
            lw     = obj.body2world(lw);
            rw     = obj.body2world(rw);
            caster = obj.body2world(caster);
            head   = obj.body2world(head);

            obj.HChassis.XData = body(:,1);   obj.HChassis.YData = body(:,2);
            obj.HLw.XData      = lw(:,1);     obj.HLw.YData      = lw(:,2);
            obj.HRw.XData      = rw(:,1);     obj.HRw.YData      = rw(:,2);
            obj.HCaster.XData  = caster(:,1); obj.HCaster.YData  = caster(:,2);
            obj.HHeading.XData = head(:,1);   obj.HHeading.YData = head(:,2);
        end

        function updatePlot(obj, particleFilter, currentBestGuess, t)
            % updatePlot

            obj.Cnt1 = obj.Cnt1 + 1;

            % render particles
            particles = particleFilter.Particles;
            if obj.isCarInRoofedArea
                obj.HParticles.MarkerEdgeColor = [1, 0.5, 0]; % orange
            else
                obj.HParticles.MarkerEdgeColor = 'g';
            end
            obj.HParticles.XData = particles(1:end,1);
            obj.HParticles.YData = particles(1:end,2);

            if obj.Cnt1 == 8
                obj.BGs = [obj.BGs; currentBestGuess];
                obj.Cnt1 = 0;
                % draw best estimated robot trace (by observer)
                obj.HBestGuesses.XData = obj.BGs(:,1);
                obj.HBestGuesses.YData = obj.BGs(:,2);
            end

            % draw the axle midpoint
            obj.HCenter.XData = obj.Pose(1);
            obj.HCenter.YData = obj.Pose(2);

            % draw robot trajectory
            obj.HTrajectory.XData = obj.Trace(1,:);
            obj.HTrajectory.YData = obj.Trace(2,:);

            % draw the robot
            obj.drawRobot();

            ax = get(obj.FigureHandle, 'currentaxes');
            title(ax, ['t = ', num2str(t)]);

            % capture snapshots for time frame 101 and 265
            if floor(t/obj.Dt) == 101
                snapnow
            end

            if floor(t/obj.Dt) == 265
                snapnow
            end
        end

    end

    methods(Access = private)
        function pts = body2world(obj, local)
            %BODY2WORLD Rotate body-frame points by theta and translate to (x,y).
            th = obj.Pose(3);
            R  = [cos(th), -sin(th); sin(th), cos(th)];
            pts = (R*local.').';
            pts(:,1) = pts(:,1) + obj.Pose(1);
            pts(:,2) = pts(:,2) + obj.Pose(2);
        end
    end

end