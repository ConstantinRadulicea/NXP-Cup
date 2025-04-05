function result = line_coordinates_to_matrix(vector)
    % Converts a vector [x1, y1, x2, y2] to a matrix [x1, y1; x2, y2]
    
    if length(vector) == 4
        result = reshape(vector, 2, 2)';  % Reshape to [x1, y1; x2, y2]
    else
        error('Input vector must have exactly 4 elements.');
    end
end
