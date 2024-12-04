function [res] = refine(positions, num_fine_waypoints)
    num_positions = size(positions, 1);

    diffs = diff(positions, 1, 1);
    sums = sum(diffs .^ 2, 2);
    dists = sqrt(sums);
    timestamps = cumsum(dists);
    timestamps = timestamps';
    timestamps = [0, timestamps];
    timestamps = timestamps ./ timestamps(end);

    % timestamps = linspace(0, execution_time_sec, num_positions);
    
    position_derivatives = diff(positions, 1, 1);
    position_derivatives_with_boundaries = zeros(size(position_derivatives, 1) + 2, size(position_derivatives, 2));
    position_derivatives_with_boundaries(2:end-1, :) = position_derivatives;
    position_derivatives_with_boundaries(1, :) = zeros(1, size(position_derivatives_with_boundaries, 2));
    position_derivatives_with_boundaries(end, :) = zeros(1, size(position_derivatives_with_boundaries, 2));
    average_speeds = zeros(size(positions, 1), size(positions, 2));
    for i = 1:size(average_speeds, 1)
        average_speeds(i, :) = (position_derivatives_with_boundaries(i, :) + position_derivatives_with_boundaries(i+1, :)) ./ double(2);
    end
    accelerations = zeros(size(positions));
    
    coefficients = zeros(size(positions, 1) - 1, size(positions, 2), size(positions, 2));
    for i = 1:size(positions, 1) - 1
        for j = 1:size(positions, 2)
            p0 = positions(i, j);          % Position at start
            p1 = positions(i+1, j);        % Position at end
            v0 = average_speeds(i, j);            % Velocity at start
            v1 = average_speeds(i+1, j);          % Velocity at end
            a0 = 0;                           % Acceleration at start
            a1 = 0;                           % Acceleration at end
    
            t0 = timestamps(i);
            t1 = timestamps(i+1);
            t2 = t1 - t0;
    
            % Solve the system Ax = b using the backslash operator: x = A \ b
            % Construct the system of equations
            % Coefficients: [a5, a4, a3, a2, a1, a0]
            A = [
                t2^5, t2^4, t2^3, t2^2, t2, 1;         % p(t1) = p1
                5*t2^4, 4*t2^3, 3*t2^2, 2*t2, 1, 0;   % p'(t1) = v1
                20*t2^3, 12*t2^2, 6*t2, 2, 0, 0;     % p''(t1) = a1
                0, 0, 0, 0, 0, 1;                 % p(t0) = p0
                0, 0, 0, 0, 1, 0;                 % p'(t0) = v0
                0, 0, 0, 2, 0, 0;                 % p''(t0) = a0
            ];
    
            b = [p1; v1; a1; p0; v0; a0];
    
            % Solve the system
            coefficients(i, j, :) = A \ b;
        end
    end

    fine_timestamps = linspace(0, 1, num_fine_waypoints);
    fine_positions = zeros(num_fine_waypoints, size(positions, 2));
    for i = 1:size(fine_timestamps, 2)
        t = fine_timestamps(i);
        segment_ix = -1;
        for k = 1:size(timestamps, 2)
            t2 = timestamps(k);
            if t2 > t
                segment_ix = k-1;
                break;
            elseif t2 == t
                segment_ix = k-1;
                if segment_ix == 0
                    segment_ix = 1;
                end
                break;
            end
        end
        assert(segment_ix ~= -1);
        t0 = timestamps(segment_ix);
        t2 = t - t0;
        % Ax = b
        A = [t2^5, t2^4, t2^3, t2^2, t2, 1];
        for j = 1:size(positions, 2)
            x = squeeze(coefficients(segment_ix, j, :));
            b = A * x;
            fine_positions(i, j) = b;
        end
    end
    
    res = fine_positions;
end