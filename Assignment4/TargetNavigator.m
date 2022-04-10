classdef TargetNavigator
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
    end

    methods
        
        function [delta_dist, delta_theta] = CalcDeltaPoseToTarget(obj,curr_pose, targ_pose)
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

    end
end