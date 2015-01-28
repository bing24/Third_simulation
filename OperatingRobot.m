classdef OperatingRobot <handle
    
    properties
        id
        initial_x
        initial_y
        current_x
        current_y
        trajectory_x
        trajectory_y
        trajectory_power
        max_speed
        power_level
        battery_drain_rate
        recharge_window_max_level
        recharge_window_min_level
        charging
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj=OperatingRobot()
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj=setID(obj, ID_number)
            obj.id=ID_number;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj=setTrajectory(obj,setmap)
            if (strcmp(setmap,'random1'))
                travel_length=2*pi;
                number_of_trajectory_steps=100;
                number_of_divergences=15;
                radius=5+rand;
                divergence_range=1;
                divergence_steps=linspace(0,2*pi,number_of_divergences);
                discrete_steps=linspace(0,2*pi,number_of_trajectory_steps);
                random_values=radius+divergence_range*randn([1 number_of_divergences]);
                random_values(end)=random_values(1);
                fitted_values=spline(divergence_steps,[0 random_values 0],discrete_steps);
                x_array=fitted_values.*cos(discrete_steps);
                y_array=fitted_values.*sin(discrete_steps);
                normalization_coef=sum(sqrt(diff(x_array).^2+diff(y_array).^2));
                obj.trajectory_x=travel_length/normalization_coef*x_array;
                obj.trajectory_y=travel_length/normalization_coef*y_array;
                
                
                indexes = ceil(length(obj.trajectory_x)*rand); %Generate random initial position and current position
                indexes=1;
                obj.initial_x=obj.trajectory_x(indexes);
                obj.initial_y=obj.trajectory_y(indexes);
                indexes = ceil(length(obj.trajectory_x)*rand);
                obj.current_x=obj.trajectory_x(indexes);
                obj.current_y=obj.trajectory_y(indexes);
            elseif (strcmp(setmap,'random2'))
                travel_length=2*pi;
                number_of_trajectory_steps=100;
                number_of_divergences=15;
                radius=5+rand;
                divergence_range=1;
                divergence_steps=linspace(0,2*pi,number_of_divergences);
                discrete_steps=linspace(0,2*pi,number_of_trajectory_steps);
                random_values=radius+divergence_range*randn([1 number_of_divergences]);
                random_values(end)=random_values(1);
                fitted_values=spline(divergence_steps,[0 random_values 0],discrete_steps);
                x_array=fitted_values.*cos(discrete_steps);
                y_array=fitted_values.*sin(discrete_steps);
                normalization_coef=sum(sqrt(diff(x_array).^2+diff(y_array).^2));
                obj.trajectory_x=travel_length/normalization_coef*x_array+2;
                obj.trajectory_y=travel_length/normalization_coef*y_array;
                
                indexes = ceil(length(obj.trajectory_x)*rand); %Generate random initial position and current position
                indexes=1;
                obj.initial_x=obj.trajectory_x(indexes);
                obj.initial_y=obj.trajectory_y(indexes);
                indexes = ceil(length(obj.trajectory_x)*rand);
                obj.current_x=obj.trajectory_x(indexes);
                obj.current_y=obj.trajectory_y(indexes);
            elseif (strcmp(setmap,'random3'))
                travel_length=2*pi;
                number_of_trajectory_steps=100;
                number_of_divergences=15;
                radius=5+rand;
                divergence_range=1;
                divergence_steps=linspace(0,2*pi,number_of_divergences);
                discrete_steps=linspace(0,2*pi,number_of_trajectory_steps);
                random_values=radius+divergence_range*randn([1 number_of_divergences]);
                random_values(end)=random_values(1);
                fitted_values=spline(divergence_steps,[0 random_values 0],discrete_steps);
                x_array=fitted_values.*cos(discrete_steps);
                y_array=fitted_values.*sin(discrete_steps);
                normalization_coef=sum(sqrt(diff(x_array).^2+diff(y_array).^2));
                obj.trajectory_x=travel_length/normalization_coef*x_array+2;
                obj.trajectory_y=travel_length/normalization_coef*y_array+2;
                
                indexes = ceil(length(obj.trajectory_x)*rand); %Generate random initial position and current position
                indexes=1;
                obj.initial_x=obj.trajectory_x(indexes);
                obj.initial_y=obj.trajectory_y(indexes);
                indexes = ceil(length(obj.trajectory_x)*rand);
                obj.current_x=obj.trajectory_x(indexes);
                obj.current_y=obj.trajectory_y(indexes);
            else
                load setmap
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plotTrajectory(obj)
            plot(obj.trajectory_x,obj.trajectory_y,'LineWidth',3)
            hold on
            %plot(obj.initial_x,obj.initial_y,'c*')
            plot(obj.charging(1,:),obj.charging(2,:),'m','LineWidth',5)
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj=configWindow(obj, min,max)
            %Charging window 50%-75%
            
            
            index=round(length(obj.trajectory_x)*min);
            indexs=round(length(obj.trajectory_x)*max);
            obj.charging=[obj.trajectory_x(index:indexs);obj.trajectory_y(index:indexs)];
            obj.charging(3,:)=zeros(1,length(obj.charging)); % 
            obj.charging(3,1)=round(index);
            for i=2:length(obj.charging)
            obj.charging(3,i)=obj.charging(3,i-1)+1;
            end
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         function obj=setspeed(obj,speed)
             obj.max_speed=speed;
         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function move(obj, time_step)
            obj.current_x=something % To be added
            obj.current_y=something % To be added
            obj.power_level=obj.power_level-time_step*obj.battery_drain_rate
        end
    end
    
end
