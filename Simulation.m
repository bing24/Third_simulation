classdef Simulation < handle

	properties
		time_step
		total_steps
        charging_time
		list_of_operating_robots
		list_of_charging_robots
	end
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	methods
		function obj=Simulation()
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function setTimeStep(obj,time_step)
			obj.time_step=time_step
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function setChargingTime(obj,chargingtime)
			obj.charging_time=chargingtime;
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function addOperatingRobots(obj, number_of_robots_to_add)
			for i=1:number_of_robots_to_add
				obj.list_of_operating_robots=[obj.list_of_operating_robots, OperatingRobot()];
				obj.list_of_operating_robots(end).setID(length(obj.list_of_operating_robots));
			end
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function addChargingRobots(obj, number_of_robots_to_add)
			for i=1:number_of_robots_to_add
				obj.list_of_charging_robots=[obj.list_of_charging_robots, ChargingRobot()];
				obj.list_of_charging_robots(end).setID(length(obj.list_of_charging_robots));
			end
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setTotalSteps(obj)
            max()
        for i=1:length(obj.list_of_operating_robots)
            maxNumberSteps=ones()


            obj.total_steps=obj.time_step*[1:length(obj.list_of_operating_robots(1).all_trajectory_x)];
            for i=1:obj.
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setMeetingTime(obj)
            for i=1:length(obj.list_of_operating_robots)
                obj.list_of_operating_robots(i).charging(3,:)=zeros(1,length(obj.list_of_operating_robots.charging));
                index=round(length(obj.list_of_operating_robots(i).trajectory_x)*obj.list_of_operating_robots(i).recharge_window_min_level);
                indexs=round(length(obj.list_of_operating_robots(i).trajectory_x)*obj.list_of_operating_robots(i).recharge_window_max_level);
                for j=1:obj.list_of_operating_robots(i).laps
                obj.list_of_operating_robots(i).charging(3,1:)=obj.total_steps(index:indexs)


            %for i=2:length(obj.charging)
                %obj.charging(3,i)=obj.charging(3,i-1)+1;
            %end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function error=plan(obj,SOLVER,goal)
            error=0;
            %x and set
            tempx=[obj.initial_x;map.a.charging(1,:)';map.b.charging(1,:)';map.c.charging(1,:)'];
            tempy=[obj.initial_y;map.a.charging(2,:)';map.b.charging(2,:)';map.c.charging(2,:)'];
            x = [tempx,tempy];
            time=[0,map.a.charging(3,:),map.b.charging(3,:),map.c.charging(3,:)]';
            set = [1;(map.a.id+1)*ones(length(map.a.charging),1); (map.b.id+1)*ones(length(map.b.charging),1); ...
                (map.c.id+1)*ones(length(map.c.charging),1)];
            
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
            
            if (strcmp(goal,'Distance'))
             
            for i = 1:numElements
                
                for j = 1:numElements
                    
                   
                    time_matrix(i,j) = timeSorted(j) - timeSorted(i);
                    vertices_distance = norm(xSorted(i,:) - xSorted(j,:));
                    
                    
                    
                    if (vertices_distance/obj.max_speed<=time_matrix(i,j))
                        A(i,j)=vertices_distance;
                    else
                        A(i,j)=nan;
                    end
                    
                   
                end
            end
                
            elseif (strcmp(goal,'Time'))
              
            %--------------------------------------------------------------------------
     
            for i = 1:numElements
                
                for j = 1:numElements
                    
                    vertices_distance = norm(xSorted(i,:) - xSorted(j,:));
                     if (vertices_distance/obj.max_speed<=timeSorted(i,:) - timeSorted(j,:))
                        A(i,j)=timeSorted(i,:) - timeSorted(j,:);
                    else
                        A(i,j)=nan;
                    end
                    
                end
            end
                
            else disp('Unknown Objective');
                
            end
            
            
            
            % Transform GTSP to ATSP
            
            [atspMatrix infcost] = gtsp_to_atsp(A, setSorted);
            
            index=atspMatrix~=atspMatrix;
            
            for i=1:length(atspMatrix)
                for j=1:length(atspMatrix)
                if index(i,j)==1
                    
                    atspMatrix(i,j)=9999999;
                end
                end
            end
             atspMatrix(:,1)=0;

            % Transform ATSP to TSP if needed
            
            
            if (strcmp(SOLVER,'LinKern'))
                symtsp = atsp_to_tsp(atspMatrix, infcost);
            elseif (strcmp(SOLVER,'LKH'))
                symtsp = atspMatrix;
                %symtsp = transformATSP2TSP(atspMatrix, infcost);
            else
                disp('Unknown Solver');
            end
            
            % Solver TSP using SOLVER
            vertexSequence = [];
            
            if (strcmp(SOLVER,'LinKern'))
                vertexSequenceOrdered = get_linkern_result(symtsp,setSorted);
            elseif (strcmp(SOLVER,'LKH'))
                vertexSequenceOrdered = get_lkh_result(symtsp, setSorted);
            else
                disp(' Cannot solve using unknown solver ');
            end
            
            
            
            %% Prune ATSP solution to the final GTSP solution
            
            vertexSequenceFinal = [];
            currentVertexIdx = 1;
            setsFound = [];
            
            %while(setsFound < numCities)
            while(currentVertexIdx < length(setSorted))
                
                currentVertex = vertexSequenceOrdered(currentVertexIdx);
                vertexSequenceFinal = [vertexSequenceFinal currentVertex];
                thisSet = setSorted(currentVertex);
                setsFound = [setsFound; thisSet];
                indexes = find(setSorted == thisSet);
                currentVertexIdx = currentVertexIdx + (length(indexes));
            end
            
            
            % Solver Analysis
            
                        for i=1:numCities
            
                            if (isempty(find(setsFound == i)))
            
                                setsNotFound = [setsNotFound i];
                            end
                        end
                        if (strcmp(goal,'Distance'))
                            meet=timeSorted(vertexSequenceFinal);
                            judge=[];
                            for i=1:length(meet)-1
                                judge(i)=max(meet(i:i+1))~=meet(i+1);
                            end
                            if sum(judge)==0;
                            disp('Route found. The meeting time:')
                            disp((meet(2:end)))
                            obj.meeting_times=meet;
                            else disp((meet(2:end)))
                                disp('Mission impossible. Please speed up or assign more chargers.')
                                error=1;
                            end
                            
                            
                            
                            obj.trajectory_x= xSorted(vertexSequenceFinal,1);
                            obj.trajectory_y= xSorted(vertexSequenceFinal,2);
                        elseif (strcmp(goal,'Time'))
                            
                            
                            meet=timeSorted(vertexSequenceFinal);
                            meet(2:4)=flip(meet(2:4));
                            judge=[];
                            for i=1:length(meet)-1
                                judge(i)=max(meet(i:i+1))~=meet(i+1);
                            end
                            if sum(judge)==0;
                            	disp('Route found. The meeting time:')
                            	disp((meet(2:end)))
                            	obj.meeting_times=meet;
                            else 
                            	disp((meet(2:end)))
                                disp('Mission impossibe. Please speed up or assign more chargers.')
                                error=1;
                            end
                            
                           	
                            obj.trajectory_x= xSorted(vertexSequenceFinal,1);
                            obj.trajectory_y= xSorted(vertexSequenceFinal,2);
                        else
                            disp(' Cannot get result ');
                        end
                        
                          
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
		function simStep(obj)
			for i=1:length(obj.list_of_operatoring_robots)
				obj.list_of_operatoring_robots(i).move(obj.time_step)
			end
			for i=1:length(obj.list_of_charging_robots)
				obj.list_of_charging_robots(i).move(obj.time_step)
			end
		end
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		function simulate(obj)
			for i=1:obj.total_steps
				obj.simStep(obj.time_step)
			end
		end
	end

end