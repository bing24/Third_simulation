
travel_length=2*pi;
number_of_trajectory_steps=1000;
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
plot(trajectory_x,trajectory_y)
