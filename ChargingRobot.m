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
        function error=plan(obj,map,SOLVER,goal)
            %     LinKern
            
            %% Test 1
            %--------------------------------------------------------------------------
            % Test Noon-bean transformation on GTSP instance
            % using various TSP solvers (LinKern, LKH)
            %--------------------------------------------------------------------------
            
            % Global Script Paramters
            %--------------------------------------------------------------------------
            % Load Data file
            %--------------------------------------------------------------------------
            % x and set
            error=0;
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
            %--------------------------------------------------------------------------
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
            atspMatrix(1,:)=0;
            % Transform ATSP to TSP if needed
            %--------------------------------------------------------------------------
            
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