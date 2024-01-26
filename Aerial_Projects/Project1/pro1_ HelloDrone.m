function[]=mobility(number_of_lines_parameter,velocity_parameter)
 
num_lines = number_of_lines_parameter;

start_points = rand(1, 3);
 
x = zeros(num_lines, 2);
y = zeros(num_lines, 2);
z = zeros(num_lines, 2);
 
x(1,1) = start_points(1);
y(1,1) = start_points(2);
z(1,1) = start_points(3);
 
for i = 2:num_lines
    x(i,1) = x(i-1,2);
    y(i,1) = y(i-1,2);
    z(i,1) = z(i-1,2);    
     
    x(i,2) = rand();
    y(i,2) = rand();
    z(i,2) = rand();
end

figure;
for i = 1:num_lines
    plot3(x(i,:), y(i,:), z(i,:));
    hold on
end
 
plot3([x(1,1), x(num_lines,2)], [y(1,1), y(num_lines,2)], [z(1,1), z(num_lines,2)], 'k-', 'LineWidth', 2);
 
point_pos = [x(1,1) y(1,1) z(1,1)];
plot3(point_pos(1), point_pos(2), point_pos(3), 'ro');
 
velocity = velocity_parameter;  
time_step = 0.1;  

total_distance = 0;  
 
for i = 1:num_lines     
    dist = norm([x(i,2)-x(i,1) y(i,2)-y(i,1) z(i,2)-z(i,1)]);
    total_distance = total_distance + dist;  
    
    time_needed = dist / velocity;
    
    num_steps = ceil(time_needed / time_step);
     
    step_vector = [x(i,2)-x(i,1) y(i,2)-y(i,1) z(i,2)-z(i,1)] / num_steps;
     
    for j = 1:num_steps     
        point_pos = point_pos + step_vector;
        
        plot3(point_pos(1), point_pos(2), point_pos(3), 'ro');
        hold on; 
        pause(0.1);
    end
end

dist_black = norm([x(1,1)-x(num_lines,2) y(1,1)-y(num_lines,2) z(1,1)-z(num_lines,2)]);
total_distance = total_distance + dist_black; % add distance to total
 
num_steps_black = ceil(dist_black / velocity / time_step);
 
step_vector_black = ([x(num_lines,2)-x(1,1) y(num_lines,2)-y(1,1) z(num_lines,2)-z(1,1)] / num_steps_black) * -1; % multiply by -1 to move in opposite direction
 
for j = 1:num_steps_black     
    point_pos = point_pos + step_vector_black;
    
    plot3(point_pos(1), point_pos(2), point_pos(3), 'ro');
     
    pause(0.1);
end

fprintf('Total distance covered: %f meter\n', total_distance);
 
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('A mobility model of Drone');
 
view(3);
rotate3d on;

end