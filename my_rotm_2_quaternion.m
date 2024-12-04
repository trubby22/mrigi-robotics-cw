function [q] = my_rotm_2_quaternion(R)
r11 = R(1, 1);  % First row, first column
r12 = R(1, 2);  % First row, second column
r13 = R(1, 3);  % First row, third column

r21 = R(2, 1);  % Second row, first column
r22 = R(2, 2);  % Second row, second column
r23 = R(2, 3);  % Second row, third column

r31 = R(3, 1);  % Third row, first column
r32 = R(3, 2);  % Third row, second column
r33 = R(3, 3);  % Third row, third column

% Define the trace
traceR = r11 + r22 + r33;

% Compute s using the trace, but ensure it's always calculated in the right way
s = sqrt(1 + traceR) * 2;

% Calculate the components of the quaternion
q1 = 0.25 * s;
q2 = (r32 - r23) / s;
q3 = (r13 - r31) / s;
q4 = (r21 - r12) / s;

% Handle the case when traceR <= 0 using max and sign functions for a branch-less approach
s1 = sqrt(1 + r11 - r22 - r33) * 2;
s2 = sqrt(1 + r22 - r11 - r33) * 2;
s3 = sqrt(1 + r33 - r11 - r22) * 2;

% Use the maximum function to select the appropriate s value
max_s = max([s, s1, s2, s3]);

% Update q1, q2, q3, and q4 using max_s, ensuring correct assignment
q1 = max_s / 4;
q2 = (r32 - r23) / max_s;
q3 = (r13 - r31) / max_s;
q4 = (r21 - r12) / max_s;

% The quaternion q is [q1, q2, q3, q4]
q = [q1, q2, q3, q4];
end