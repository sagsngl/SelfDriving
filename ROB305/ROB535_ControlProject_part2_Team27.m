function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team27(TestTrack,Xobs_seen,curr_state)

    % Interpolate to expand number of cline points
    new_cline = interp_cline(TestTrack.cline,2);
    
    centerLineTheta = TestTrack.theta;
    FLAG_terminate = 0;
    
    % Times in seconds
    total_time = .51;
    dt = .01;
    
    % Speed cannot be too fast
    max_speed = [11,16]; % 11,16
    steps_forward = 2;
    centerline_strength = 0.1;

    % Gains
    Gain.P(1) = 5.12;
    Gain.I(1) = 0;
    Gain.D(1) = 0.3;
    Gain.P(2) = 10005.376;
    Gain.I(2) = 0;
    Gain.D(2) = 0;
    
    
    % Sum Error Initialize
    sum_error = zeros(1,6);
    prev_error = zeros(1,6);
    U = [0, 0];

    n_elements = size(TestTrack.cline, 2);
    
    % d_theta
    [d_theta,d_theta_ext] = get_turn_angle(TestTrack.theta,n_elements);
      
    
    % Number of time steps
    nsteps = total_time/dt;
    
    %Get initial line to track from obstacles
    closest_obs = get_next_obs(curr_state,Xobs_seen,new_cline);
    centerLine = get_lane(closest_obs, TestTrack);
    Cnormals = get_Cnormals(centerLine);
    
    for i=1:nsteps
        
        closest_obs_center_old = closest_obs;
        cline_old = centerLine;
        
        %next step        
        closest_obs = get_next_obs(curr_state,Xobs_seen,new_cline);
        
        if closest_obs_center_old ~= closest_obs
            centerLine = get_lane( closest_obs, TestTrack);
            if cline_old ~= centerLine
                Cnormals = get_Cnormals(centerLine);
            end
        end

        
        nearest_goal_id = knnsearch(centerLine', [curr_state(1), curr_state(3)]);
        future_goal = nearest_goal_id + steps_forward;
        
        if future_goal > size(centerLine, 2)
            future_goal = size(centerLine, 2);
        end
        nearest_goal = centerLine(:, nearest_goal_id);
        length_scale = (d_theta(:, nearest_goal_id) - d_theta_ext(1))/(d_theta_ext(2) - d_theta_ext(1));
        goal_speed = length_scale * (max_speed(2) - max_speed(1)) + max_speed(1);
        goal_state = [nearest_goal(1), goal_speed, nearest_goal(2), 0, 0, 0];
        error = goal_state - curr_state;
        goal_state(5) = centerLineTheta(:, future_goal) -centerline_strength*dot(Cnormals(:, nearest_goal_id), [error(1);error(3)], 1); %atan2(error(3),error(1));
        error = goal_state - curr_state;
        sum_error = sum_error + error;
        d_error = (error - prev_error)/0.01;
        delta_f = Gain.P(1) * error(5) + Gain.I(1) * sum_error(5) + Gain.D(1) * d_error(5);
        Fx = Gain.P(2) * error(2) + Gain.I(2) * sum_error(2) + Gain.D(2) * d_error(2);
        if delta_f > 0.5
            delta_f = 0.5;
        end
        if delta_f < -0.5
            delta_f = -0.5;
        end
        %Solve for trajectory
        T=(i-1)*0.01:0.01:i*0.01;
        U = [U;[delta_f, Fx]];
        
        U_sim = U(end-1:end, :);
        if mod(i,1000) == 999
            [Y,~]=forwardIntegrateControlInput(U,x0);
        else
            [~,Y]=ode45(@(t,x)bike(t,x,T,U_sim),T,curr_state);
        end
        curr_state = Y(end,:);
        prev_error = error;
        if ((curr_state(1) > TestTrack.cline(1,end)) && (curr_state(3) > TestTrack.cline(2,end))) == 1
            FLAG_terminate = 1;
        end
        
        
    end
    
    sol_2 = U;

end


%% bike model
function dzdt=bike(t,x,T,U)
    %constants
    Nw=2;
    f=0.01;
    Iz=2667;
    a=1.35;
    b=1.45;
    By=0.27;
    Cy=1.2;
    Dy=0.7;
    Ey=-1.6;
    Shy=0;
    Svy=0;
    m=1400;
    g=9.806;

    %generate input functions
    %delta_f=interp1(T,U(:,1),t,'previous','extrap');
    %F_x=interp1(T,U(:,2),t,'previous','extrap');
    delta_f = U(end,1);
    F_x = U(end,2);

    %slip angle functions in degrees
    a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
    a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));

    %Nonlinear Tire Dynamics
    phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
    phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

    F_zf=b/(a+b)*m*g;
    F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

    F_zr=a/(a+b)*m*g;
    F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

    F_total=sqrt((Nw*F_x)^2+(F_yr^2));
    F_max=0.7*m*g;

    if F_total>F_max

        F_x=F_max/F_total*F_x;

        F_yr=F_max/F_total*F_yr;
    end

    %vehicle dynamics
    dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
              (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
              x(2)*sin(x(5))+x(4)*cos(x(5));...
              (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
              x(6);...
              (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end

%% function to get normal unit vectors for each step
function unit_vec = get_Cnormals(centerLine)
    n_elements = size(centerLine, 2);
    unit_vec = (centerLine(:,2:n_elements) - centerLine(:,1:n_elements-1));
    unit_vec = unit_vec ./ vecnorm(unit_vec);
    %rotate by 90deg
    unit_vec = [0, 1; -1, 0] * unit_vec;
    unit_vec(:,n_elements) = unit_vec(:,n_elements - 1);
end

function [diff_theta,diff_theta_ext] = get_turn_angle(centerLineTheta,n)
    diff_theta = 1./ abs(centerLineTheta(:, 2:end) - centerLineTheta(:, 1:end-1));
    diff_theta(:,n) = diff_theta(:,n - 1);
    diff_theta_ext = [min(diff_theta), max(diff_theta)];
end

%function to find next closest obstacle
function closest_obstacle_center = get_closest_obstacle(curr_state, Xobs_seen)
    num_cells = numel(Xobs_seen);
    if num_cells == 0
        closest_obstacle_center = [0;0];
        %disp("NO CELLS!");
    else
        min_dist = 999999;
        for i = 1:num_cells
            obstacle = Xobs_seen{i};
            obstacle_center = mean(obstacle)';
            obstacle_dist = norm(obstacle_center - [curr_state(1); curr_state(3)]);
            if obstacle_dist < min_dist
                min_dist = obstacle_dist;
                closest_obstacle_center = obstacle_center;
            end
        end
        
        %hold on;
        %scatter(closest_obstacle_center(1), closest_obstacle_center(2));
    end
end

function next_obstacle_center = get_next_obs(curr_state, Xobs_seen, cline_interp)
    num_cells = numel(Xobs_seen);
    next_obstacle_center = [0;0];
    if num_cells ~= 0
            %centerLine = TestTrack.cline;
        centerLine = cline_interp;
        nearest_id_state = knnsearch(centerLine', [curr_state(1), curr_state(3)]);%closest point
        min_diff = 10000000; %initialize to very high value
        for i = 1:num_cells
            obstacle = Xobs_seen{i};
            obstacle_center = mean(obstacle)';
            nearest_id_obs = knnsearch(centerLine', [obstacle_center(1), obstacle_center(2)]);
            id_diff = nearest_id_obs - nearest_id_state;
            % loop to find the closest obstacle to avoid first
            if id_diff >= 0 && id_diff < min_diff
                next_obstacle_center = obstacle_center;
                min_diff = id_diff;
            end
        end
    end

end

function centerLine = get_lane(closest_obs, TestTrack)
    if closest_obs == [0;0]
        %disp("Center")
        centerLine = TestTrack.cline;
    else
        Cnormals = get_Cnormals(TestTrack.cline);
        % needs to take in centerLine and Cnormals
        % pos_obs needs to be row for knn to work
        cline_idx = knnsearch(TestTrack.cline', closest_obs');
        obs_vec = TestTrack.cline(:,cline_idx) - closest_obs;
        norm_vec = Cnormals(:,cline_idx);
        sign_check = dot(obs_vec, norm_vec);
        % positive = RHS
        % negative = LHS
        if sign_check >= 0
            %disp("Right")
            centerLine = (TestTrack.br + TestTrack.cline)./2;
        else
            %disp("Left")
            centerLine = (TestTrack.bl + TestTrack.cline)./2;
        end
    end
end

function new_cline = interp_cline(cline, div)
    N = size(cline,2);
    step = 1/div;
    
    cline_x = cline(1,:);
    cline_y = cline(2,:);
    
    new_x = interp1(1:N, cline_x, 1:0.5:N);
    new_y = interp1(1:N, cline_y, 1:0.5:N);
    
    new_cline = [new_x;new_y];
end