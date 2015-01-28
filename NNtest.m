clear all
close all
%% Motion planning
%First random trajactory

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
trajectory_x=travel_length/normalization_coef*x_array;
trajectory_y=travel_length/normalization_coef*y_array;
%Generate random initial position for working robot

indexes = ceil(length(trajectory_x)*rand); 
initial_x=trajectory_x(indexes);
initial_y=trajectory_y(indexes);
% Second random trajectory

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
x=travel_length/normalization_coef*x_array;
y=travel_length/normalization_coef*y_array;
indexes = ceil(length(trajectory_x)*rand); 
initial_x2=x(indexes);
initial_y2=y(indexes);
%Charging window 50%-75%

chwinb=.5;
chwine=0.75;
index=round(length(trajectory_x)*chwinb/2);
indexs=round(length(trajectory_x)*chwine/2);
cha1=[trajectory_x(index:indexs);trajectory_y(index:indexs)];
cha1(3,:)=zeros(1,length(cha1));
cha1(3,:)=cha1(3,:)+round(100*rand);
index=round(34+length(trajectory_x)*chwinb/2);
indexs=round(34+length(trajectory_x)*chwine/2);
cha2=[x(index:indexs) ;y(index:indexs)];
cha2(3,:)=zeros(1,length(cha2));
cha2(3,:)=cha1(3,:)+round(rand*100);
%Initial pos for charging robot

x0=0;
y0=0;
%% Applying NN algorithm
%Current pos is initial pos

xx=x0;
yy=y0;
%Find the next charging window

next=min(cha1(3,:),cha2(3,:));
%Calculate distance

diss=(xx-cha1(1,:)).^2+(yy-cha1(2,:)).^2;
ind=find(diss==min(diss));
xx(1,2)=cha1(1,ind);
yy(1,2)=cha1(2,ind);
diss=(xx(1,2)-cha2(1,:)).^2+(yy(1,2)-cha2(2,:)).^2;
ind=find(diss==min(diss));
xx(1,3)=cha2(1,ind);
yy(1,3)=cha2(2,ind);

%Plot trajectory
plot(trajectory_x,trajectory_y,'-.k*')
hold on 
plot(x,y,'-.or')
plot(initial_x,initial_y,'x','linewidth',3,'MarkerSize',10)
plot(initial_x2,initial_y2,'x','linewidth',3,'MarkerSize',10)
plot(cha1(1,:),cha1(2,:),'g')
plot(cha2(1,:),cha2(2,:),'y')
plot(xx,yy,'--mo')
