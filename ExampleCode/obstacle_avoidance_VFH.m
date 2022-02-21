ipaddress = "http://192.168.1.101:11311";
rosinit(ipaddress)

laserSub = rossubscriber("/scan","DataFormat","struct");
[velPub, velMsg] = rospublisher("/cmd_vel","DataFormat","struct");


%% Set up VFH object for obstacle avoidance. 
vfh = controllerVFH;
vfh.UseLidarScan = true;
vfh.DistanceLimits = [0.05 1];
vfh.RobotRadius = 0.1;
vfh.MinTurningRadius = 0.2;
vfh.SafetyDistance = 0.1;

targetDir = 0;

% Set up a Rate object using rateControl (Navigation Toolbox) to track 
% the timing of your loop. 
rate = rateControl(10);

%% Create a loop that collects data, calculates steering direction, 
% and drives the robot.
while rate.TotalElapsedTime < 30
    % Collect information from laser scan
    scan = receive(laserSub);
    rosPlot(scan);

	% Get laser scan data and create a lidarScan object
	scanMsg = receive(laserSub);
    scan = rosReadLidarScan(scanMsg);
        
	% Call VFH object to computer steering direction
	steerDir = vfh(scan,targetDir);  
    
	% Calculate velocities
	if ~isnan(steerDir) % If steering direction is valid
		desiredV = 0.2;
		w = exampleHelperComputeAngularVelocity(steerDir,1);
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = 0.5;
	end

	% Assign and send velocity commands
	velMsg.Linear.X = desiredV;
	velMsg.Angular.Z = w;
	velPub.send(velMsg);
end

velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
velPub.send(velMsg)
