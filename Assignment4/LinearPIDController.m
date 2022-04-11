classdef LinearPIDController
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        % Params
        Kp = .08;
        Ki = 10;
        Kd = .0003;
        vel_low_lim = 0;
        vel_up_lim = .05;
        
        % Runtime Vars
        last_time = 0;
        last_dist_to_target = 0;
        error_sum = 0;
    end

    methods
        function lin_vel = CalcLinVel(obj, current_time, cur_dist_to_target)
            %Calculate Linear Velocity for controling the robot using a PID
            prop_err = obj.Kp * cur_dist_to_target;
            obj.error_sum = obj.error_sum + cur_dist_to_target;
            %int_err = (obj.error_sum + cur_dist_to_target)/ obj.Ki;
            der_err = obj.Kd *(cur_dist_to_target - obj.last_dist_to_target) / (current_time - obj.last_time);

            lin_vel = prop_err - der_err; % + int_err;

            lin_vel=min(max(lin_vel,obj.vel_low_lim),obj.vel_up_lim);

            obj.last_dist_to_target = cur_dist_to_target;
            obj.last_time = current_time;            
        end

    end
end