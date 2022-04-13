classdef TargetNavigator
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
        TargFig
        TargAnno
        TargPosPlot
        CurPosPlot
        InitPosPlot
        targ_x = 0;
        targ_y = 0;
        initial_x = 0;
        initial_y = 0;
  
        odomTrajFig
        odomTrajPlot
        relative_positions_axis = 0;
        relative_x ;
        relative_y ;

        lin_pid
        ang_pid

        stop_dist_thresh  = .2;
        
    end

    methods
        function obj = TargetNavigator(initial_odom_data)
            obj.ang_pid = AngularPIDController();
            obj.lin_pid = LinearPIDController();              

            obj.initial_x = initial_odom_data.Pose.Pose.Position.X;
            obj.initial_y = initial_odom_data.Pose.Pose.Position.Y;

            % Target Fig
            obj.TargFig = figure('Position',[[0, 0], 960, 540]);
            title("Robot Start, Target, and Current Position");
            obj.TargPosPlot = scatter(0,0,1000,'green','x'); hold on;
            obj.InitPosPlot = scatter(obj.initial_x,obj.initial_y,1000,'red','x');hold on;
            obj.CurPosPlot = scatter(obj.initial_x,obj.initial_y,1000,'yellow','x');hold on;
            legend("Target Position", "Starting Position","Robot Position"); grid on;
            
%             obj.odomTrajFig = figure('Position',[50, 50, 550, 550]);
%             obj.relative_x = [obj.initial_x];
%             obj.relative_y = [obj.initial_y];
%             obj.odomTrajPlot = scatter(obj.relative_x,obj.relative_y,[],'green','filled'); grid on;  hold on;
%             title("Robot Historical Trajectory");

            obj.TargAnno = annotation('textbox',[.0 .0 .5 .3],'String','Initializing','FitBoxToText','on');
        end

        function [lin_vel, ang_vel, dist_to_targ] = CalcWayPointNavVels(obj, odom_data, targ_pose, curr_time)
            [dist_to_targ, ang_to_targ] = obj.CalcDeltaPoseToTarget(odom_data, targ_pose);
            
            if dist_to_targ < obj.stop_dist_thresh
                lin_vel = 0;
                ang_vel = 0;
                display_string = "Target Reached! Stopping!";
            else
                lin_vel = obj.lin_pid.CalcLinVel(curr_time, dist_to_targ);
                ang_vel = obj.ang_pid.CalcAngVel(curr_time, ang_to_targ);
                display_string = strcat(["DistToTarg: " dist_to_targ, "AngToTarg: " ang_to_targ, "LinVel: " lin_vel, "AngVel:" ang_vel]);
            end
            set(obj.TargAnno,'String', display_string);
        end 
        
        function [delta_dist, delta_theta] = CalcDeltaPoseToTarget(obj, odom_data, targ_pose)
            curr_pose = odom_data.Pose.Pose;
            % Calculate Delta between Current Pose and Target Pose
            d_x = targ_pose.Position.X - curr_pose.Position.X;
            d_y = targ_pose.Position.Y - curr_pose.Position.Y;
        
            % Get current robot angle
            cur_angle_quat = quaternion([curr_pose.Orientation.X curr_pose.Orientation.Y curr_pose.Orientation.Z curr_pose.Orientation.W ]);
            cur_angle_mat = quat2rotm(cur_angle_quat);
            cur_angle = wrapToPi(atan2(cur_angle_mat(2,3), cur_angle_mat(3,3)));
        
            % Get angle from robot directly to the target
            targ_ang = atan2(d_y,d_x);
            delta_theta = targ_ang-cur_angle;
        
            delta_dist = sqrt(d_x^2 + d_y^2);
        end

        function PlotTarget(obj, odom_data, target_data)
            current_pose = odom_data.Pose.Pose;
            
            % Update Target Figure 
            set(obj.CurPosPlot,'XData',current_pose.Position.X,'YData',current_pose.Position.Y); % change the line data

            % Update Historical Odom Data
            %obj.relative_x = [obj.relative_x, obj.initial_x - current_pose.Position.X];
            %obj.relative_y = [obj.relative_y, obj.initial_y - current_pose.Position.Y];
            %set(obj.odomTrajPlot,'XData',obj.relative_x,'YData',obj.relative_x);

            % Update Target Odom Data
            obj.targ_x = target_data.Position.X;
            obj.targ_y = target_data.Position.Y;
            set(obj.TargPosPlot,'XData',obj.targ_x,'YData',obj.targ_y);
        end

    end
end