classdef ChargingRobot < handle
    
    properties
        id
        initial_x
        initial_y
        current_x
        current_y
        trajectory_x
        trajectory_y
        max_speed
        power_level
        meeting_locations
        meeting_times
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj=ChargingRobot()
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setID(obj, ID_number)
            obj.id=ID_number;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setpos(obj,xini,yini)
            obj.initial_x=xini;
            obj.initial_y=yini;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setspeed(obj,speed)
            obj.max_speed=speed;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plotTrajectory(obj)
            
            obj.trajectory_x=[obj.initial_x; obj.trajectory_x];
            
            obj.trajectory_y=[obj.initial_y; obj.trajectory_y];
            
            %average_speed=norm(obj.trajectory_x,obj.trajectory_y)./timeSorted(vertexSequenceFinal)

            location=[obj.trajectory_x(2:end),obj.trajectory_y(2:end)];
            %For time, the order of location is upside down
            disp('The ordered meeting locations are:')
            disp(location)
            obj.meeting_locations=location;
            %disp('The average speed for each meeting are:')
            %dip(average_speed)


            plot(obj.trajectory_x,obj.trajectory_y,'--k')
            hold on
            plot(1,1,'+')
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function move(obj, time_step)
            obj.current_x=something % To be added
            obj.current_y=something % To be added
        end
    end
    
end