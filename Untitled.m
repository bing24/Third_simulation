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
            time=zeros(length(obj.list_of_charging_robots),1);
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
            
            numCities = max(set);
            xSorted = [];
            setSorted = [];
            timeSorted=[];
            
            for i = 1:numCities
                currentSet = find(set == i);
                
                for j = 1:length(currentSet)
                    setSorted = [setSorted; i];
                    xSorted = [xSorted; x(currentSet(j),:)];
                    timeSorted=[timeSorted;time(currentSet(j))];
                end
            end
            
            numElements = length(setSorted);
            A = zeros(numElements, numElements);
            
         
             
            for i = 1:numElements
                
                for j = 1:numElements
                   
                    time_matrix(i,j) = timeSorted(j) - timeSorted(i);
                    vertices_distance = norm(xSorted(i,:) - xSorted(j,:));
                    
                    if (vertices_distance/obj.list_of_charging_robots(1).max_speed+obj.charging_time<=time_matrix(i,j))
                        A(i,j)=vertices_distance;
                    else
                        A(i,j)=nan;
                    end
                    
                end
            end