% -------------------------
% CS5331: Aerial Computing
% Project #3 
% Summer II, 2023
% -------------------------
function scale_free_only(path)
    % network size: width (w) x height (h)
    % e.g., 1000 (m) x 1000 (m) by default
    
    w_begin= 0;
    w_end = 1000;
    h_begin = 0;
    h_end = 1000;
    
    % number of cells (subareas): 5-by-5, by default
    n_cell = 5;
    tot_cell = n_cell * n_cell;
    size_cell = w_end / n_cell;
    
    % n number of target points: 
    n = 100;
    
    % a set of rectangular subareas: 5-by-5
    x_ = linspace(w_begin, w_end - size_cell, n_cell);
    ux = [];
    for i = 1:n_cell
        ux = [ux, x_]; 
    end 
    ux = ux'
    
    y_ = ones(1, n_cell);  
    uy = [];
    for i = 1:n_cell
        uy = [uy, y_ .* (size_cell * (i - 1))];
    end 
    uy = uy'
    
    % n number of weights: w, n-by-1, uniform
    uw = ones(n, 1);
    
    % n number of weights: w, n-by-1, uniform
    % -- between the interval (w_begin, w_end) 
    w_begin = 0;
    w_end = 10;
    w = w_begin + (w_end - w_begin) .* rand(n, 1);
    
    % coverage area with radius, r (m), by default 100
    r = 100;
    
    % -----------------------
    % scale-free distribution 
    % -----------------------
    
    % clustering exponent, alpha
    alpha = 1.4;
    
    % population, pop  
    % -- initialize to zero
    pop = ones(tot_cell, 1) - 1;
    
    % probability, prob
    % -- initialize to zero
    prob = ones(tot_cell, 1) - 1;
    
    % a set of rectangular subareas, 25-by-5
    subarea_loc = [ux, uy]
    
    % the first target point is randomly assigned to one of cells
    pos_subarea = randi(tot_cell)
    
    pos_x = randi(size_cell) + ux(pos_subarea)
    pos_y = randi(size_cell) + uy(pos_subarea)
    pop(pos_subarea) = pop(pos_subarea) + 1
    
    % the first target point - randomly assigned
    loc(1, 1) = pos_x
    loc(1, 2) = pos_y
    
    % generate all scale-free target points (x, y)
    for i = 2:n
        % calculate probabilities
        % -- sigma_pop = sum(pop, "all")
        sigma_pop = 0;
        for j = 1: tot_cell
            sigma_pop = sigma_pop + power(pop(j) + 1, alpha);
        end
        for j = 1: tot_cell
            prob(j) = power(pop(j) + 1, alpha) / sigma_pop; %power(sigma_pop, alpha);
            %prob(j) = power(pop(j), alpha) / power(sigma_pop, alpha)
        end
        % sanity check: if total probabilities are one
        %tot_prob = sum(prob, "all")
    
        % randomly choose one of subareas
        % -- pos_subarea = randi(tot_cell);
        
        % choose one of subareas based on the probability
        % -- generate a random and compare with cumulative probabilities 
        rand_prob = rand(1, 1); % generate between 0 to 1
        cumu_prob = 0; 
        for j = 1: tot_cell
            cumu_prob = cumu_prob + prob(j);
            if (cumu_prob >= rand_prob)
                pos_subarea = j;
                break
            end
        end
    
        % generate a position within the chosen subarea
        pos_x = randi(size_cell) + ux(pos_subarea);
        pos_y = randi(size_cell) + uy(pos_subarea);
        % increment the population of subarea
        pop(pos_subarea) = pop(pos_subarea) + 1;
    
        % add a new target point's (x, y) into a row
        loc = [loc; [pos_x, pos_y]];
    end    
    
    
    minpts=1;
    epsilon=50;
    [idx, corepts] = dbscan(loc,epsilon,minpts);
    number_of_clusters = sum(unique(idx)>0);
    % remove ouliers
    core_data = loc(corepts, :);
    core_idx = idx(corepts);
    % Clusters are saved in a cell array
    clusters = splitapply(@(x) {x}, core_data, core_idx);
    plot(loc(:, 1), loc(:, 2), "rx")
    hold on
    title("POI's after Clustering")
    % colors = lines(number_of_clusters);
    cluster_centers = [];
    for i = 1:number_of_clusters
        cluster_points = clusters{i};
        % plot(cluster_points(:, 1), cluster_points(:, 2), '.', 'Color', colors(i, :), 'MarkerSize', 10)
        plot(cluster_points(:, 1), cluster_points(:, 2), '.', 'MarkerSize', 10)
        hold on
        
        % Calculate the centroid of the current cluster
        centroid = mean(cluster_points, 1);
        
        % Plot the centroid as a blue 'o'
        plot(centroid(1), centroid(2), '^', 'MarkerSize', 8, 'LineWidth', 1,'Color', 'b');
        cluster_centers = [cluster_centers; [centroid(1), centroid(2),length(cluster_points)]];
        % Calculate the radius of the circle based on the maximum distance from the centroid to any point in the cluster
        radius = max(max(pdist2(cluster_points, centroid, 'euclidean')),50);
        
        % Draw the circle around the centroid
        % viscircles(centroid, radius, 'EdgeColor', colors(i, :), 'LineWidth', 2);
        viscircles(centroid, radius, 'Color', 'b','LineWidth', 1);
    end
    
    set(gca, 'FontSize', 16);
    set(gca, 'xTick', [-200:200:1200]);
    set(gca, 'yTick', [-200:200:1200]);
    xlabel('X', 'FontSize', 16);
    ylabel('Y', 'FontSize', 16); 
    axis([-200 1200 -200 1200]);
    print -depsc sf_topo;
    savefig('sf_topo.fig');
    

    if path==0
        %RAND
        total_dist = 0;
        temp_clusters = cluster_centers;
        ind = randi(length(temp_clusters));
        point_pos = [temp_clusters(ind,1),temp_clusters(ind,2)];
        figure
        plot(0, 0, '^', 'MarkerSize', 8, 'LineWidth', 1,'Color', 'b');
        hold on;
        title("Random Path Planning(RAND)")
        x1 = 0;%temp_clusters(ind,1);
        y1 = 0;%temp_clusters(ind,2);
        % temp_clusters(ind,:)=[];
        % for j = 1:length(cluster_centers)-1     
        for i =1:length(cluster_centers)
            ind = randi(length(temp_clusters));
            if i==length(cluster_centers)-1
                point_pos = [temp_clusters(1) , temp_clusters(2)];
            else
                point_pos = [temp_clusters(ind,1),temp_clusters(ind,2)];
                temp_clusters(ind,:)=[];
            end
            plot(point_pos(1), point_pos(2), '^', 'MarkerSize', 8, 'LineWidth', 1,'Color', 'b');
            
            % display(point_pos);
            plot([x1 point_pos(1)], [y1 point_pos(2)], 'b-');
            hold on; 
            plot(point_pos(1), point_pos(2), 'g*');
            total_dist = total_dist + norm([(point_pos(1) - x1)  (point_pos(2) - y1)]);
            hold on; 
            
            x1 = point_pos(1);
            y1 = point_pos(2);
        
            pause(0.1);
        end
        display('Distance while tracng the RAND path');
        display(total_dist);
    elseif path==1
        
        %NNF
        total_dist = 0;
        temp_clusters = cluster_centers;
        % point_pos = [temp_clusters(ind,1),temp_clusters(ind,2)];
        figure
        plot(0, 0, '^', 'MarkerSize', 8, 'LineWidth', 1,'Color', 'b');
        hold on; 
        title("Distance Based Planning(NNF)")
        x1 = 0;%temp_clusters(ind,1);
        y1 = 0;%temp_clusters(ind,2);
        
        for j = 1:length(cluster_centers)
            ind=1;
            minDist = norm([(temp_clusters(1,1) - x1)  (temp_clusters(1,2) - y1)]);
            if j<length(cluster_centers)-1
                for i = 2:length(temp_clusters)
                    dist = norm([(temp_clusters(i,1) - x1)  (temp_clusters(i,2) - y1)]);
                    if dist<minDist
                        minDist = dist;
                        ind = i;
                    end
                end
            
                point_pos = [temp_clusters(ind,1),temp_clusters(ind,2)];
                temp_clusters(ind,:)=[];
            else
                point_pos = [temp_clusters(1),temp_clusters(2)];
            end
            plot(point_pos(1), point_pos(2), '^', 'MarkerSize', 8, 'LineWidth', 1,'Color', 'b');
            
            % display(point_pos);
            plot([x1 point_pos(1)], [y1 point_pos(2)], 'b-');
            hold on; 
            plot(point_pos(1), point_pos(2), 'g*');
            hold on; 
            total_dist = total_dist + norm([(point_pos(1) - x1)  (point_pos(2) - y1)]);
            x1 = point_pos(1);
            y1 = point_pos(2);
        
            pause(0.1);
        end
        display('Distance while tracng the NNF path');
        display(total_dist);
    elseif path==2
        %DF
        total_dist = 0;
        temp_clusters = sortrows(cluster_centers,-3);
        figure
        plot(0, 0, '^', 'MarkerSize', 8, 'LineWidth', 1,'Color', 'b');
        hold on;
        title("Density Based Planning(DF)")
        x1 = 0;
        y1 = 0;
        for j = 1:length(cluster_centers)-1
            point_pos = [temp_clusters(j,1),temp_clusters(j,2)];
            plot(point_pos(1), point_pos(2), '^', 'MarkerSize', 8, 'LineWidth', 1,'Color', 'b');
            % display(point_pos);
            plot([x1 point_pos(1)], [y1 point_pos(2)], 'b-');
            hold on; 
            plot(point_pos(1), point_pos(2), 'g*');
            hold on; 
            total_dist = total_dist + norm([(point_pos(1) - x1)  (point_pos(2) - y1)]);
            x1 = point_pos(1);
            y1 = point_pos(2);
        
            pause(0.1);
        end
        display('Distance while tracng the DF path');
        display(total_dist);
    else
        display('Path undefined')
    end
    % draw target points
    clear;
end    
    

