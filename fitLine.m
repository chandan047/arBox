function [unit_vector,distance] = fitLine(A,B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Given 2 points, find the direction of vector between them %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    direction   = B-A;
    distance    = norm(direction);
    unit_vector = direction/distance;
end