clear variables
close all
clc

n_birds = 200;
dt = 0.1;
x_lim = 200;
y_lim = 200;
z_lim = 200;
spawn_radius = 100;

% Weights for the simulation (arbitrary units)
centre_w = 0.5;  % How far the bird is pulled to the origin (the flock's roost)
vel_w = 20;  % How much the bird wants to keep its normal velocity
dist_w = 100;  % How much bird wants to keep away from nearby birds
swarm_w = 0.0001;  % How much bird is pulled towards other birds
vel_diff_w = 1;  % How much bird wants to adapt to velocity of surrounding birds
pred_w = 400;  % How much bird wants to evade predators

norm_vel = 25;  % Typical bird velocity
norm_dist = 10;  % Typical distance between birds
swarm_vis_dist = 50;
vel_debye_dist = 10;  % Typical distance for which bird adapts velocity of surrounding birds
pred_danger_dist = 40;  % Distance that bird wants to keep from predator

max_acc = 100;  % Limit on how quickly a bird can accelerate

predator_prob = 0.005;  % Predator spawn probability every frame
pred_spawn_radius = 200;
pred_norm_vel = 25;  % Predators fly at fixed velocity


% Pos [3] and Vel [3]
% Randomly initialized
bird_states = rand(n_birds, 6);
bird_states(:, 1) = (bird_states(:, 1) .* 2 - 1) .* spawn_radius;
bird_states(:, 2) = (bird_states(:, 2) .* 2 - 1) .* spawn_radius;
bird_states(:, 3) = (bird_states(:, 3) .* 2 - 1) .* spawn_radius;
bird_states(:, 4) = (bird_states(:, 4) .* 2 - 1) .* norm_vel;
bird_states(:, 5) = (bird_states(:, 5) .* 2 - 1) .* norm_vel;
bird_states(:, 6) = (bird_states(:, 6) .* 2 - 1) .* norm_vel;

% Predator
n_pred = 5;
predators = nan(n_pred, 6);

% Plot birds
fig = figure();
fig.Position = [400, 50, 700, 700];

bird_plots = [];
for i = 1:n_birds
    bird_plots(i) = plot3(bird_states(i, 1), bird_states(i, 2), bird_states(i, 3), 'k.', 'MarkerSize', 20);
    if (i == 1)
        hold on;
    end
end

pred_plots = [];
for i = 1:n_pred
    pred_plots(i) = plot3(predators(i, 1), predators(i, 2), predators(i, 3), 'r.', 'MarkerSize', 30);
end

grid on;
axis equal;
xlim([-x_lim, x_lim]);
ylim([-y_lim, y_lim]);
zlim([-z_lim, z_lim]);

% For saving the video
flock_video = VideoWriter('flock3d.mp4', 'MPEG-4');
flock_video.FrameRate = ceil(1/dt);
flock_video.Quality = 100;
open(flock_video);

% Simulate
try
while true
    tic  % Measure time

    % Spawn random predator
    if (any(isnan(predators)))
        if (rand() < predator_prob)
            % Randomly initialize predator
            pred_ang = 2*pi * rand();
            % Todo make this work not only in the x-y-plane
            pred_pos = pred_spawn_radius * [cos(pred_ang), sin(pred_ang), 0];
            pred_vel = - pred_norm_vel * [cos(pred_ang), sin(pred_ang), 0];

            % Find empty slot in predator array
            empty_pred_idx = find(isnan(predators(:, 1)));
            % Add 
            predators(empty_pred_idx(1), :) = [pred_pos, pred_vel];
        end
    end

    new_bird_states = zeros(n_birds, 6);
    for i = 1:n_birds
        bird_pos = reshape(bird_states(i, 1:3), 3, 1);
        bird_vel = reshape(bird_states(i, 4:6), 3, 1);
        bird_pos_abs = norm(bird_pos);
        bird_vel_abs = norm(bird_vel);

        % Find closest bird neighbour
        % bird_dists = vecnorm(bird_states(:, 1:3) - bird_pos', 2, 2);
        % bird_dists_non_z_idx = find(bird_dists > 0);
        % [min_dist, min_idx] = min(bird_dists(bird_dists_non_z_idx));
        % min_dist_bird_idx = bird_dists_non_z_idx(min_idx);
        % 
        % % Closest neighbour dist and position
        % min_dist_pos = reshape(bird_states(min_dist_bird_idx, 1:3), 3, 1);

        debye_acc = [0; 0; 0];
        swarm_pull = [0; 0; 0];
        vel_pull = [0; 0; 0];
        for j = 1:n_birds
            if (i == j)  % Bird can't influence itself
                continue;
            end

            other_bird_pos = reshape(bird_states(j, 1:3), 3, 1);
            other_bird_vel = reshape(bird_states(j, 4:6), 3, 1);

            % Todo: Find closest other bird and compute acceleration
            bird_to_other_dist = other_bird_pos - bird_pos;
            bird_to_other_vel = other_bird_vel - bird_vel;

            debye_acc = debye_acc + bird_to_other_dist / norm(bird_to_other_dist) * exp(-norm(bird_to_other_dist) / norm_dist);
            swarm_pull = swarm_pull - bird_to_other_dist * norm(bird_to_other_dist) * exp(-norm(bird_to_other_dist) / swarm_vis_dist);

            vel_pull = vel_pull + bird_to_other_vel * exp(-norm(bird_to_other_dist) / vel_debye_dist);
        end

        pred_push = [0; 0; 0];
        for j = 1:n_pred
            if (any(isnan(predators(j, :))))
                continue;
            end

            pred_pos = reshape(predators(j, 1:3), 3, 1);

            bird_to_pred_dist = pred_pos - bird_pos;
            pred_push = pred_push + bird_to_pred_dist / norm(bird_to_pred_dist) * exp(-norm(bird_to_pred_dist) / pred_danger_dist);
        end

        % Accelerations
        % Gravity to centre
        centre_acc = - centre_w * bird_pos;
        % Keep velocity
        vel_acc = - vel_w * log(bird_vel_abs / norm_vel) * bird_vel;
        % Keep distance to closest neighbour
        %dist_acc = - dist_w * log(min_dist / norm_dist) * (bird_pos - min_dist_pos);
        % Keep distance to surrounding neighbours
        dist_acc = - dist_w * debye_acc;
        % Go back towards flock / swarm
        swarm_acc = - swarm_w * swarm_pull;
        % Adapt velocity to that of surrounding birds
        vel_diff_acc = vel_diff_w * vel_pull;
        % Flee from predator
        pred_acc = - pred_w * pred_push;

        % Update bird motion
        bird_acc = centre_acc + vel_acc + dist_acc + swarm_acc + vel_diff_acc + pred_acc;

        if (any(isnan(bird_acc)) || any(isinf(bird_acc)))
            bird_acc = [0; 0; 0];
        end
        if (norm(bird_acc) > max_acc)
            bird_acc = bird_acc / norm(bird_acc) * max_acc;
        end

        new_bird_pos = bird_pos + bird_vel * dt;
        new_bird_vel = bird_vel + bird_acc * dt;
        new_bird_states(i, :) = [reshape(new_bird_pos, 1, 3), reshape(new_bird_vel, 1, 3)];
    end

    % Update all bird states
    bird_states = new_bird_states;

    % Update all predators
    for i = 1:n_pred
        if (any(isnan(predators(i, :))) || norm(predators(i, 1:3)) > pred_spawn_radius+1)
            predators(i, :) = nan(1, 6);
            continue;
        end

        predators(i, 1:3) = predators(i, 1:3) + predators(i, 4:6) * dt;
    end

    % Update animation
    for i = 1:n_birds
        bird_pos = bird_states(i, 1:3);
        set(bird_plots(i), 'XData', bird_pos(1), 'YData', bird_pos(2), 'ZData', bird_pos(3));
    end
    % Update predator animation
    for i = 1:n_pred
        pred_pos = predators(i, 1:3);
        set(pred_plots(i), 'XData', pred_pos(1), 'YData', pred_pos(2), 'ZData', pred_pos(3));
    end
    drawnow;
    
    % Save frame to video file
    frame = getframe(gcf);
    writeVideo(flock_video, frame);

    % Pause to get right frame rate
    elapsed_t = toc;
    pause_t = 0;
    if (elapsed_t < dt)
        pause_t = dt - elapsed_t;
    end
    pause(pause_t);
end
catch
    % Save video
    close(flock_video);
end