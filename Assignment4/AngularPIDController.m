classdef AngularPIDController
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Kp = .1;
        Ki = 10;
        Kd = .000005;
        last_time = 0;
        last_ang_to_target = 0;
        error_sum = 0;
    end

    methods
        function ang_vel = CalcAngVel(obj, current_time, cur_ang_to_target)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            prop_err = obj.Kp * cur_ang_to_target;
            obj.error_sum = obj.error_sum + cur_ang_to_target;
            int_err = (obj.error_sum + cur_ang_to_target)/ obj.Ki;
            der_err = obj.Kd *(cur_ang_to_target - obj.last_ang_to_target) / (current_time - obj.last_time);
            
            ang_vel = (prop_err + int_err - der_err);

            %Brandon - for running on real robot
            if ang_vel > 0.1
                ang_vel = 0.1;
            end

            obj.last_ang_to_target = cur_ang_to_target;
            obj.last_time = current_time;
        end

    end
end