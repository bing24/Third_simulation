obj=med;

set=[];
            ChargingRobotx=[];
            iter=0;
            for i=1:length(obj.list_of_charging_robots)
                temp=[obj.list_of_charging_robots(i).initial_x;obj.list_of_charging_robots(i).initial_y]; 
                ChargingRobotx=cat(2,ChargingRobotx,temp);
                set=cat(1,set,i);
                iter=iter+1;
            end
            ChargingRobotx=ChargingRobotx';
            OperatingRobotx=[];
            tempSet=[];
            time=0;
            for i=1:length(obj.list_of_operating_robots)
                OperatingRobotx=cat(2,OperatingRobotx,obj.list_of_operating_robots(i).charging);
                time=cat(1,time,obj.list_of_operating_robots(i).charging(3,:)');
                for j=1:obj.list_of_operating_robots(i).laps
                    iter=iter+1;
                    tempSet=cat(1,tempSet,ones(length(obj.list_of_operating_robots(i).charging)/obj.list_of_operating_robots(i).laps,1)*iter);
                end
            end
            OperatingRobotx=[OperatingRobotx(1,:)' OperatingRobotx(2,:)'];
            
            x=cat(1,ChargingRobotx,OperatingRobotx);
            set=cat(1,set,tempSet);

