clear all
close

load image_use.mat


 plotMap(med)

% obj=med;

% index=find(obj.meeting_time==0);
%             color=[0 0 0;1 0 1;0 1 1; 0 1 0 ];
%             hold on
%             for i=1:length(index)
%                 if i==length(index)
%                     plot(obj.chargingTrajectory_x(index(i):end)*100,obj.chargingTrajectory_y(index(i):end)*100,'--','Color',color(i,:))
%                 else
%                     plot(obj.chargingTrajectory_x(index(i):index(i+1)-1)*100,obj.chargingTrajectory_y(index(i):index(i+1)-1)*100,'--','Color',color(i,:))
%                 end
%             end
%             for i=1:length(obj.list_of_operating_robots)
%                 plotTrajectory(obj.list_of_operating_robots(i))
%             end
%             scatter(obj.chargingTrajectory_x*100,obj.chargingTrajectory_y*100,'g')
%             [chargerimage,~,alpha]=imread('cr.png');
%             chargerimage=imresize(chargerimage,0.5);
%             alpha=imresize(alpha,0.5);
            
%             for i=1:length(obj.list_of_charging_robots)
%             	p1=[obj.chargingTrajectory_x(index(i)) obj.chargingTrajectory_y(index(i))];
%             	p2=[obj.chargingTrajectory_x(index(i)+1) obj.chargingTrajectory_y(index(i)+1)];
            	
%    				sita = atan2(p2(2)-p1(2),p2(1)-p1(1))*180/pi-15;

%             	chargerrotate=imrotate(chargerimage,-sita);
%             alpharotate=imrotate(alpha,-sita);
%             image_center=size(chargerrotate)/2;
%             bigimage=image(obj.list_of_charging_robots(i).initial_x*100-image_center(2),obj.list_of_charging_robots(i).initial_y*100-image_center(1),chargerrotate);
%             set(bigimage,'AlphaData',alpharotate);
%             end
%             axis equal


% plot(obj.trajectory_x*100,obj.trajectory_y*100,'b','LineWidth',1)
%             hold on
%             %Configure the charging window before plot command
%             plot(obj.charging(1,1:end/obj.laps)*100,obj.charging(2,1:end/obj.laps)*100,'r','LineWidth',1)
%             scatter(obj.initial_x*100,obj.initial_y*100,'k')
%             workerimage=imread('or.png');
%             workerimage=imresize(workerimage,0.5);
%             workerimage=imrotate(workerimage,-90);
%             image(obj.initial_x*100,obj.initial_y*100,workerimage);
