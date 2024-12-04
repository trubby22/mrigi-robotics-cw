function result = interpolate_vectors(data, target_dist)
    % interpolate_vectors - Interpolates between consecutive vectors with uniform Euclidean spacing.
    %
    % Syntax:
    % result = interpolate_vectors(data, target_dist)
    %
    % Inputs:
    %   data - n x m array, where n is the dimensionality of each vector
    %          and m is the number of vectors.
    %   target_dist - Desired Euclidean distance between consecutive vectors.
    %
    % Outputs:
    %   result - n x p array containing original vectors and interpolated points.
    
    [n, m] = size(data);
    
    if m < 2
        result = data;
        return;
    end
    
    % Initialize result with the first vector
    result = data(:, 1);
    
    for i = 1:(m-1)
        % Current vector
        v1 = data(:, i);
        % Next vector
        v2 = data(:, i+1);
        
        % Compute the Euclidean distance between v1 and v2
        total_dist = norm(v2 - v1);
        
        % Number of intermediate points required
        num_points = max(1, round(total_dist / target_dist));
        
        % Interpolation parameter for each intermediate point
        t = linspace(0, 1, num_points + 1);
        % Remove the last endpoint as it will overlap with the next vector
        t = t(1:end-1);
        
        % Generate interpolated points
        for j = 2:length(t) % Skip v1 as it's already in the result
            interp_vec = (1-t(j)) * v1 + t(j) * v2;
            result = [result, interp_vec];
        end
        
        % Append v2 to the result
        result = [result, v2];
    end
end