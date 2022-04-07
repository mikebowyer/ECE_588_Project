classdef LinearPIDController
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Kp = .001;
        Ki = 10;
        Kd = .0000000005;
        last_time = 0;
        last_dist_to_target = 0;
        error_sum = 0;
    end

    methods
        function lin_vel = CalcLinVel(obj, current_time, cur_dist_to_target)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            prop_err = obj.Kp * cur_dist_to_target;
            obj.error_sum = obj.error_sum + cur_dist_to_target;
            %int_err = (obj.error_sum + cur_dist_to_target)/ obj.Ki;
            der_err = obj.Kd *(cur_dist_to_target - obj.last_dist_to_target) / (current_time - obj.last_time);

            lin_vel = prop_err + der_err; % + int_err;

            obj.last_dist_to_target = cur_dist_to_target;
            obj.last_time = current_time;
        end

    end
end