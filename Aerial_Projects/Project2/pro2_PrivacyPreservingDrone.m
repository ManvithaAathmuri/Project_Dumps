function mobility(mobility_model, velocity, pausing_time, num_dummy_locations)
    % Network dimensions
    network_width = 1000;
    network_height = 1000;

    % Initialize drone's starting location
    drone_x = randi([0, network_width]);
    drone_y = randi([0, network_height]);

    % Initialize variables to store total flying distance and time
    total_flying_distance = 0;
    total_flying_time = 0;

    time_step = 1;  
    clf('reset')
    figure(1);
    plot(drone_x, drone_y, 'rd');
    hold on;
    simulation = 2;
    % Start the simulation
    while simulation>0
        % Generate random destination within the network
        dest_x = randi([0, network_width]);
        dest_y = randi([0, network_height]);

        if mobility_model == 0 % Random Waypoint (RWP)
            % Calculate distance to the destination
            distance = norm([(dest_x - drone_x)  (dest_y - drone_y)]);

            % Calculate time taken to reach the destination
            flying_time = distance / velocity;

            % Update total flying distance and time
            total_flying_distance = total_flying_distance + distance;
            total_flying_time = total_flying_time + flying_time;
    
            num_steps = ceil(flying_time / time_step);
             
            step_vector = [dest_x-drone_x dest_y-drone_y] / num_steps;
            point_pos = [drone_x drone_y];
            x1 = drone_x;
            y1 = drone_y;
            for j = 1:num_steps     
                point_pos = point_pos + step_vector;
                % display(point_pos);
                plot([x1 point_pos(1)], [y1 point_pos(2)], 'b-');
                hold on; 
                plot(point_pos(1), point_pos(2), 'g*');
                hold on; 
                
                x1 = point_pos(1);
                y1 = point_pos(2);

                pause(0.1);
            end
            
            % Move the drone to the destination and pause
            drone_x = dest_x;
            drone_y = dest_y;
            pause(pausing_time);

        elseif mobility_model == 1 % Privacy-Preserving Random (PPR)
            % Calculate distance to the destination
            distance_to_dest = 0;

            % Generate dummy locations
            if dest_x>drone_x
                dummy_locations_x = randi([drone_x dest_x], 1,num_dummy_locations+2);
            else
                dummy_locations_x = randi([dest_x drone_x], 1,num_dummy_locations+2);
            end
            if dest_y>drone_y
                dummy_locations_y = randi([drone_y dest_y], 1,num_dummy_locations+2);
            else
                dummy_locations_y = randi([dest_y drone_y], 1,num_dummy_locations+2);
            end
            dummy_locations_x = dummy_locations_x(2:end-1);
            dummy_locations_y = dummy_locations_y(2:end-1);
            dummy_locations_x = [dummy_locations_x dest_x];
            dummy_locations_y = [dummy_locations_y dest_y];
            % display([drone_x ,drone_y])
            % display([dest_x,dest_y])
            % display([dummy_locations_x,dummy_locations_y])

            % Move the drone through the dummy locations and pause at the destination
            for i = 1:num_dummy_locations+1
                
                distances_to_dummy = sqrt((dummy_locations_x(i) - drone_x).^2 + (dummy_locations_x(i) - drone_y).^2);
    
                % Calculate time taken to reach the destination through the dummy locations
                flying_time = sum(distances_to_dummy) / velocity;
    
                % Update total flying distance and time
                total_flying_distance = total_flying_distance + distances_to_dummy;% + sum(distances_to_dummy);
                total_flying_time = total_flying_time + flying_time;

                num_steps = ceil(flying_time / time_step);
                 
                step_vector = [dummy_locations_x(i)-drone_x dummy_locations_y(i)-drone_y] / num_steps;
                point_pos = [drone_x drone_y];
                x1 = drone_x;
                y1 = drone_y;
                for j = 1:num_steps     
                    point_pos = point_pos + step_vector;
                    % display(point_pos);
                    plot([x1 point_pos(1)], [y1 point_pos(2)], 'b-');
                    hold on; 
                    plot(point_pos(1), point_pos(2), 'mo');
                    hold on; 
                    x1 = point_pos(1);
                    y1 = point_pos(2);
    
                    pause(0.1);
                end
              
                drone_x = dummy_locations_x(i);
                drone_y = dummy_locations_y(i);
                pause(1);
            end
            drone_x = dest_x;
            drone_y = dest_y;
            pause(pausing_time);
        end

        % Visualization
        if mobility_model == 1
            plot(dummy_locations_x, dummy_locations_y, 'mo');
        end
        plot([drone_x, dest_x], [drone_y, dest_y], 'b-');
        axis([0, network_width, 0, network_height]);
        xlabel('X-axis (m)');
        ylabel('Y-axis (m)');
        title('Drone Mobility Simulation');
        drawnow;
        
        simulation = simulation-1;
        % display(simulation);
    end

    % Display total flying distance and time
    disp(['Total Flying Distance: ', num2str(total_flying_distance), ' meters']);
    disp(['Total Flying Time: ', num2str(total_flying_time), ' seconds']);
end