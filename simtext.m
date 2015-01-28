clear all
close all
choice='No';


%% Define operating robots

alpha=OperatingRobot;
beta=OperatingRobot;
gamma=OperatingRobot;

alpha=setID(alpha,1);
beta=setID(beta,2);

gamma=setID(gamma,3);

alpha=setTrajectory(alpha,'random1');
beta=setTrajectory(beta,'random2');
gamma=setTrajectory(gamma,'random3');

alpha=configWindow(alpha, 0.5,.95);
beta=configWindow(beta, 0.5,.95);
gamma=configWindow(gamma, 0.5,.95);

% plotTrajectory(alpha)
% hold on
% plotTrajectory(beta)
% plotTrajectory(gamma)

%% Define charging robots
med=ChargingRobot;
setpos(med,1,1);
setspeed(med,.06);

%% Run the simulation

or.a=alpha;
or.b=beta;
or.c=gamma;



er=plan(med,or,'LKH','Distance')
if er==1
	return
end

dlg_string = sprintf('Do you want to record a video of results?\n\nThe name of the file will be the current date and time and it will be saved in the same directory.');
%choice=questdlg(dlg_string,'Video Recording','Yes','No','Cancel','No');
if(strcmp(choice,'Cancel')||isempty(choice)) 
    break
end
recordBool=strcmp(choice,'Yes');
if recordBool
    
    c=fix(clock);
    cstr=strcat(mat2str(c),'.avi');
    SimulationVideo = VideoWriter(cstr);
    open(SimulationVideo);
    axis tight
    set(gca,'nextplot','replacechildren');
    set(gcf,'Renderer','zbuffer');
end

FigHandle=figure(1);
plotTrajectory(alpha)
hold on
plotTrajectory(beta)
plotTrajectory(gamma)
plotTrajectory(med)

t=1;
simulation_time=100;
handle_1=[];
handle_2=[];
handle_3=[];
handle_4=[];

set(FigHandle, 'Position', [100, 100, 1049, 895]);

charging_robot_trajectory_x=[];
charging_robot_trajectory_y=[];
% feasibility check
number_of_meeting_points=size(med.meeting_locations,1);
for i=2:number_of_meeting_points
	distance=norm(med.meeting_locations(i,:)-med.meeting_locations(i-1,:));
	time_difference=(med.meeting_times(i)-med.meeting_times(i-1));
	required_speed=distance/time_difference;
	current_theta=atan2((med.meeting_locations(i,2)-med.meeting_locations(i-1,2)),(med.meeting_locations(i,1)-med.meeting_locations(i-1,1)));
	current_path=[1:time_difference]*required_speed;
	path_x=current_path*cos(current_theta)+med.meeting_locations(i-1,1);
	path_y=current_path*sin(current_theta)+med.meeting_locations(i-1,2);
	charging_robot_trajectory_x=[ charging_robot_trajectory_x, path_x];
	charging_robot_trajectory_y=[ charging_robot_trajectory_y, path_y];
	fprintf('For path #%i the distance is %.3f and minimum required speed is %.3f.\n',i-1,distance,required_speed)
	if(required_speed>med.max_speed)||(time_difference<0)
		disp('ERROR: Speed criteria is not met.')
		return
	end
end

charging_robot_trajectory_x=[charging_robot_trajectory_x,ones(1,simulation_time-length(charging_robot_trajectory_y))*charging_robot_trajectory_x(end)];
charging_robot_trajectory_y=[charging_robot_trajectory_y,ones(1,simulation_time-length(charging_robot_trajectory_y))*charging_robot_trajectory_y(end)];

while t<simulation_time
	if ishandle(handle_1) delete(handle_1); end
	handle_1=plot(alpha.trajectory_x(t),alpha.trajectory_y(t),'k.','MarkerSize',30);
	if ishandle(handle_2) delete(handle_2); end
	handle_2=plot(beta.trajectory_x(t),beta.trajectory_y(t),'k.','MarkerSize',30);
	if ishandle(handle_3) delete(handle_3); end
	handle_3=plot(gamma.trajectory_x(t),gamma.trajectory_y(t),'k.','MarkerSize',30);
	if ishandle(handle_4) delete(handle_4); end
	handle_4=plot(charging_robot_trajectory_x(t),charging_robot_trajectory_y(t),'r.','MarkerSize',35);
	t=t+1;
	drawnow;
	if recordBool
        frame = getframe;
        writeVideo(SimulationVideo,frame);
    end
    pause(.1)
end

if recordBool
    close(SimulationVideo);
end