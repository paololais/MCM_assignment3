function [h,theta] = RotToAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.

    
    % Check matrix R to see if its size is 3x3
    
    if (((size(R, 1)==3 && size(R, 2)== 3) && norm(R.'*R - eye(3))<1e-6) && abs(det(R) - 1) < 1e-6 )
        theta = acos((trace(R)-1)/2);
        if theta <1e-6
            h = [0; 0; 0];  % Arbitrary axis when no rotation
   
        elseif abs(theta - pi) < 1e-6
            % Initialize the rotation axis h
            h = zeros(3, 1);
            
            % Loop to find the diagonal element with its abs > 0 and use it to determine h
            for i = 1:3
                h(i) = sqrt((R(i,i) + 1) / 2);
                if h(i) > 0
                    break; % Pick the first positive component
                end
            end
            
            % Assign signs to other components based on off-diagonal elements
            if i == 1
                h(2) = sign(R(1,2)) * sqrt((R(2,2) + 1) / 2);
                h(3) = sign(R(1,3)) * sqrt((R(3,3) + 1) / 2);
            elseif i == 2
                h(1) = sign(R(2,1)) * sqrt((R(1,1) + 1) / 2);
                h(3) = sign(R(2,3)) * sqrt((R(3,3) + 1) / 2);
            else
                h(1) = sign(R(3,1)) * sqrt((R(1,1) + 1) / 2);
                h(2) = sign(R(3,2)) * sqrt((R(2,2) + 1) / 2);
            end
           
        else
            S = (R - R.') / 2;
            h = vex(S) /sin(theta);
        end
        
      

    else
        error('R is not a rotational matrix');
    end
end


function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)

a = 1/2 * [S_a(3,2)-S_a(2,3);S_a(1,3)-S_a(3,1);S_a(2,1)-S_a(1,2)];
end
