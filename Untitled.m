i=2;
j=1;
obj=med;
temp=[];
obj.list_of_operating_robots(i).charging(3,:)=zeros(1,length(obj.list_of_operating_robots(i).charging));
index=round(length(obj.list_of_operating_robots(i).trajectory_x)*obj.list_of_operating_robots(i).recharge_window_min_level);
indexs=round(length(obj.list_of_operating_robots(i).trajectory_x)*obj.list_of_operating_robots(i).recharge_window_max_level);
eachChargingTime(j,:)=obj.total_time(length(obj.list_of_operating_robots(i).trajectory_x)*(j-1)+obj.charging_time*(j-1)+index: ...
    length(obj.list_of_operating_robots(i).trajectory_x)*(j-1)+obj.charging_time*(j-1)+indexs);
temp=cat(2,temp,eachChargingTime(j,:));


j=2;
eachChargingTime(j,:)=obj.total_time(length(obj.list_of_operating_robots(i).trajectory_x)*(j-1)+obj.charging_time*(j-1)+index: ...
    length(obj.list_of_operating_robots(i).trajectory_x)*(j-1)+obj.charging_time*(j-1)+indexs);
temp=cat(2,temp,eachChargingTime(j,:));
obj.list_of_operating_robots(i).charging(3,:)=zeros(1,length(obj.list_of_operating_robots(i).charging));