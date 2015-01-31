classdef OperatingRobot <handle
    
    properties
        id
        initial_x
        initial_y
        current_x
        current_y
        trajectory_x
        trajectory_y
        all_trajectory_x
        all_trajectory_y
        trajectory_power
        max_speed % how to difine?
        power_level
        battery_drain_rate
        recharge_window_max_level
        recharge_window_min_level
        charging
        laps
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj=OperatingRobot()
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setID(obj, ID_number)
            obj.id=ID_number;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setTrajectory(obj,setmap,center_x,center_y,laps)
            
            switch nargin
            
            case 2
                
                if (strcmp(setmap,'random'))
                
                    disp('Not enough arguement')
                else
                load setmap;
                end
            case 5
                obj.laps=laps;
                if (strcmp(setmap,'random'))
                      
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
                
                obj.trajectory_x=travel_length/normalization_coef*x_array+center_x;
                obj.trajectory_y=travel_length/normalization_coef*y_array+center_y;

                obj.all_trajectory_x=repmat(obj.trajectory_x,1,obj.laps);
                obj.all_trajectory_y=repmat(obj.trajectory_y,1,obj.laps);

                %Generate random initial position and current position
                % indexes = ceil(length(obj.trajectory_x)*rand); 
                % indexes=1;
                obj.initial_x=obj.trajectory_x(1);
                obj.initial_y=obj.trajectory_y(1);
                % indexes = ceil(length(obj.trajectory_x)*rand);
                % obj.current_x=obj.trajectory_x(indexes);
                % obj.current_y=obj.trajectory_y(indexes);
                else 
                    disp('Too many inputs')
                end
                
                otherwise 
                    disp('Numer of input is wrong')
                  
                    end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plotTrajectory(obj)
            
            plot(obj.trajectory_x*100,obj.trajectory_y*100,'b','LineWidth',1)
            hold on
            %Configure the charging window before plot command
            plot(obj.charging(1,1:end/obj.laps)*100,obj.charging(2,1:end/obj.laps)*100,'r','LineWidth',1)
            scatter(obj.initial_x*100,obj.initial_y*100,'k')
            [workerimage,~,alpha]=imread('or.png');
            workerimage=imresize(workerimage,0.5);
            alpha=imresize(alpha,0.5);
            p1=[obj.trajectory_x(1) obj.trajectory_y(1)];
            p2=[obj.trajectory_x(2) obj.trajectory_y(2)];
            sita = atan2(p2(2)-p1(2),p2(1)-p1(1))*180/pi+20;

            workerrotate=imrotate(workerimage,-sita);
            alpharotate=imrotate(alpha,-sita);
            image_center=size(workerrotate)/2;
            bigimage=image(obj.initial_x*100-image_center(2),obj.initial_y*100-image_center(1),workerrotate);
            set(bigimage,'AlphaData',alpharotate);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function configWindow(obj,min,max)
            
            obj.recharge_window_min_level=min;
            obj.recharge_window_max_level=max;            
            index=round(length(obj.trajectory_x)*min);
            indexs=round(length(obj.trajectory_x)*max);
            obj.charging=repmat([obj.trajectory_x(index:indexs);obj.trajectory_y(index:indexs)],1,obj.laps);
                      
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setspeed(obj,speed)
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
