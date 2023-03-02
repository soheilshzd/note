clear all
clc

# Computing IK for the changed orientation
```
dist1 = 0.0164;
dist2 = 0.054;
dist3 = 0.0384;
dist4 = 0.0408;
dist5 = 0.01335;

% Link num.  1      2       3       4       5       6 
    a  =[    0      dist2   dist3   dist4   dist5   0   ];   
    d  =[    dist1  0       0       0       0       0   ];
t_offs =[    0      -pi/2    0       0       pi/2   pi/2];  
alpha  =[    -pi/2   0       0       0       0      pi/2];

syms q1 q2 q3 q4;
assume(q1,'real');
assume(q2,'real');
assume(q3,'real');
assume(q4,'real');

syms qi ai di alphai
assume(qi,'real')
assume(ai,'real')
assume(di,'real')
assume(alphai,'real')

A01i = [cos(qi) -sin(qi)*cos(alphai)  sin(qi)*sin(alphai) ai*cos(qi);...
       sin(qi)  cos(qi)*cos(alphai) -cos(qi)*sin(alphai) ai*sin(qi);...
          0     sin(alphai)          cos(alphai)         di;...
          0    0     0    1];
A01 = subs(A01i,{ai,di,alphai,qi},{ a(1), d(1), alpha(1), q1+t_offs(1) });
A12 = subs(A01i,{ai,di,alphai,qi},{ a(2), d(2), alpha(2), q2+t_offs(2) });
A23 = subs(A01i,{ai,di,alphai,qi},{ a(3), d(3), alpha(3), q3+t_offs(3) });
A34 = subs(A01i,{ai,di,alphai,qi},{ a(4), d(4), alpha(4), q4+t_offs(4) });
rotation = rotm2tform(rotz(90));      % For Matlab in house
% rotation = rotm2tform(rotz(pi/2));      % For Matlab in lab
A_aux = rotation*transl(dist5,0,0);
% A45 = A_aux * rotm2tform(rotz(pi/2)) * rotm2tform(rotx(pi/2));  % To have Z axis of the final orientation normal to the robot
A45 = A_aux * rotm2tform(rotz(90)) * rotm2tform(rotx(90));  % To have Z axis of the final orientation normal to the robot

A05=A01*A12*A23*A34*A45;
```
# Computing IK for the changed orientation. Stage 1 of IK.
## Define a real cartesian position in space (X,Y,Z) and its desired orientation eul angles 
```
load('fingerIK_1ene','eul_angles','p_e_testing','bot','q_test', 'A05_testing_toolbox','Tbase')

eul_angles
T_desired = rotm2tform(eul2rotm(eul_angles,'ZYZ'));
T_desired(1:3,4) = p_e_testing;
T_desired;
hold on
trplot(T_desired, 'color', 'b')

% % Converting the desired transform into the frame of the base of the finger
% aux = T_desired*rotm2tform(roty(90));
aux = T_desired*rotm2tform(roty(pi/2));
trplot(aux, 'color', 'r')
hold on


res = inv(Tbase)*aux;

trplot(res, 'color', 'g')
hold on


eul_angles = rotm2eul(res(1:3,1:3), "ZYZ");
p_e_testing= res(1:3,4);

point_in_ws = checkPointInsideWs(p_e_testing(1), p_e_testing(2), p_e_testing(3));
if(point_in_ws==1)
    reorientation = 0.05;
    
    T_desired = rotm2tform(eul2rotm(eul_angles,'ZYZ'));
    T_desired(1:3,4) = p_e_testing;
    T_desired;
    
%     T_desired_in_aux = T_desired*rotm2tform(rotx(-pi/2))*rotm2tform(rotz(-pi/2));        % aux is the frame that has X axis as normal to TCP. From this frame we can extract the eul angles, specifically the eul_angle(3), and it is always = sum of q_test 3 last joints
    T_desired_in_aux = T_desired*rotm2tform(rotx(-90))*rotm2tform(rotz(-90));

    new_eul_angles = rotm2eul(T_desired_in_aux(1:3,1:3), "ZYZ");
%     hold on
%     trplot(T_desired_in_aux, 'color', 'b')
%     
    theta_testing = new_eul_angles(3);              % This is equal to pi/2 plus the sum of the last 3 joint values of q_test
    % theta_testing =  3.0668;                      % This is equal to pi/2 plus the sum of the last 3 joint values of q_test
    theta_testing =  theta_testing-pi/2;             % The pi/2 addition is due to the offset turn in Z that we do for the 5th link DH parameters.
    extra_angle = atan2(dist5,dist4);               % Angle from the last joint to the tip of the robot. It exists because of the translation offset of the TCP of the finger
   ``` 
    
# Inverse kinematics solving (symbolic)
```
    syms a2 a3 a4 a5 px py pz theta;
    a2 = a(2);
    a3 = a(3);
    a4 = a(4);
    a5 = a(5);
    px = p_e_testing(1);
    py = p_e_testing(2);
    pz = p_e_testing(3);
    theta = theta_testing;
    
    theta1_I = atan2(py,px);
    theta1_II = atan2(-py,-px);
    
    % Considering only one solution of theta1 as valid. The angle that is
    % positive and smaller than pi.
    if abs(theta1_I) < 0.001
        theta1_I = 0;
        theta1_sol = theta1_I;
    else
        if abs(theta1_I) >= 0 && abs(theta1_I) < pi/2
            theta1_sol = theta1_I;
        end
    end
    if abs(theta1_II) < 0.001
        theta1_II = 0;
        theta1_sol = theta1_II;
    else
        if abs(theta1_II) >= 0 && abs(theta1_II) < pi/2
            theta1_sol = theta1_II;
        end
    end
    
    theta1_sol;
    
    A01_subs = subs(A01,{q1,q2,q3,q4},{theta1_sol, 0, 0, 0});
%     trplot(double(A01_subs), 'color', 'g')
    
    hold on;
%     goal_frame = double(A01_subs)*rotm2tform(rotz(theta_testing));      % To show the goal frame after turning theta_testing+delta_orientation angle
    goal_frame = double(A01_subs)*rotm2tform(rotz(theta_testing*180/pi))      % To show the goal frame after turning theta_testing+delta_orientation angle
%     goal_frame = goal_frame*rotm2tform(rotz(pi/2))*rotm2tform(rotx(pi/2));   % These rotations are used for matching the aux frame with the final TCP frame, the one with Z perpendicular to the finger tip plane. For Matlab in lab
    goal_frame = goal_frame*rotm2tform(rotz(90))*rotm2tform(rotx(90))   % These rotations are used for matching the aux frame with the final TCP frame, the one with Z perpendicular to the finger tip plane. For Matlab in house 
    goal_frame(:,4) = [p_e_testing;1];
    trplot(goal_frame, 'color', 'b')
    
    
    
    A01_vect_to_goal = inv(A01_subs)*[p_e_testing;1];
    
    % pwx and pwy are the position of the 4th joint of the arm 
    pwx = A01_vect_to_goal(1) - sqrt(a5^2+a4^2)*cos(theta+extra_angle-pi/2);
    pwy = A01_vect_to_goal(2) - sqrt(a5^2+a4^2)*sin(theta+extra_angle-pi/2);
    
    % Rounding pwx and pwy to zero if these are very small
    if abs(pwx) < 0.001
        pwx = 0;
    end
    if abs(pwy) < 0.001
        pwy = 0;
    end
    
    c3 = (pwx^2+pwy^2-a2^2-a3^2)/(2*a2*a3);
    subtraction = 1-c3^2;
    % Rounding subtraction to zero if very small
    if abs(subtraction) < 0.001
        subtraction = 0;
    end
    
    if(subtraction<0)
        delta_orientation = 0;
        delta_orientation = delta_orientation-reorientation;
        stage_IK_solver = 1;
        arm_solution = compute_IKvector(new_eul_angles, delta_orientation, A01_subs, p_e_testing, A01_vect_to_goal, a(5), a(4), a(3), a(2), theta1_sol,stage_IK_solver, reorientation)
        ik_solutions_testing_grad = double(arm_solution)*180/pi;
    else    
        s3plus = sqrt(subtraction);
        s3minus = -sqrt(subtraction);
        theta3_I = atan2(s3plus,c3);
        theta3_II = atan2(s3minus,c3);
        
        s2_I = ((a2+a3*c3)*pwy - a3*s3plus*pwx) / (pwx^2+pwy^2);
        s2_II = ((a2+a3*c3)*pwy - a3*s3minus*pwx) / (pwx^2+pwy^2);
        c2_I = ((a2+a3*c3)*pwx + a3*s3plus*pwy) / (pwx^2+pwy^2);
        c2_II = ((a2+a3*c3)*pwx + a3*s3minus*pwy) / (pwx^2+pwy^2);
        
        theta2_I = atan2(s2_I,c2_I);
        theta2_II = atan2(s2_II,c2_II);
        
        theta4_I = theta - theta2_I - theta3_I;
        theta4_II = theta - theta2_II - theta3_II;
        
        arm_solution(1,:) = [theta1_sol, theta2_I+pi/2, theta3_I, theta4_I-pi/2];       %The addition to theta2 is because the computation of IK is performed assuming the finger starts layed down (horizontal)
        arm_solution(2,:) = [theta1_sol, theta2_II+pi/2, theta3_II, theta4_II-pi/2];    %And the subtraction from theta4 is because of the alpha DH parameter of the 5th link.
        
        ik_solutions_testing_grad = arm_solution*180/pi;
        ik_solutions_testing_grad = double(ik_solutions_testing_grad)
        
    end
    T_with_ik_sols = bot.fkine([ik_solutions_testing_grad(1,:) 0 0]*pi/180)
    check_vect = check_feasibility(arm_solution)
    jv_not_valid=verify_all_joint_values(check_vect)
    bot.plot([ik_solutions_testing_grad(plot_robot_conf(check_vect,jv_not_valid),:) 0 0]*pi/180);
else
    disp("POint is outside the finger workspace")
end
```

# Computing IK for the changed orientation. Stage 2.
```
if(point_in_ws==1)
    new_A05 = double(subs(A05,{q1,q2,q3,q4},{ik_solutions_testing_grad(1,:)*pi/180}));      % Having found the IK already computes the transform from base to TCP
    orntn_eeframe_wrt_A0 = rotm2tform(T_desired(1:3,1:3));
    orntn_eeframe_wrt_endpoint = inv(new_A05) * orntn_eeframe_wrt_A0;
    orntn_eeframe_wrt_endpoint(abs(orntn_eeframe_wrt_endpoint)<0.001)=0.0;                  % Obtaining the ee rotation with respect to the current endpoint frame of the robot
    theta_testing_was_neg=0;
    theta_testing = double(arm_solution(1,2)+arm_solution(1,3)+arm_solution(1,4));
    % theta_testing = new_eul_angles(3);
    % theta_testing = theta_testing -pi/2;             % The pi/2 subtraction is due to the offset turn in Z that we do for the 5th link DH parameters.
    
    % hold on;
    % goal_frame = double(A01_subs)*rotm2tform(rotz(theta_testing));      % To show the goal frame after turning theta_testing+delta_orientation angle
    % % goal_frame = double(A01_subs)*rotm2tform(rotz(theta_testing*180/pi))      % To show the goal frame after turning theta_testing+delta_orientation angle
    % goal_frame = goal_frame*rotm2tform(rotz(pi/2))*rotm2tform(rotx(pi/2))   % For Matlab in lab
    % % goal_frame = goal_frame*rotm2tform(rotz(90))*rotm2tform(rotx(90))  % For Matlab in house 
    % goal_frame(:,4) = new_A05(:,4);
    % trplot(goal_frame, 'color', 'r')
    
    
    delta_orientation = atan2(orntn_eeframe_wrt_endpoint(1,3), orntn_eeframe_wrt_endpoint(3,3));
    if(theta_testing< 0)
        theta_testing = theta_testing+2*pi;  
        theta_testing_was_neg = 1;
    end
    if(orntn_eeframe_wrt_endpoint(1,3) < 0 && theta_testing_was_neg==0)
        delta_orientation = -delta_orientation; 
    end
    
    ```
    
    % if(theta_testing ~= 0)
    %     if(theta_testing<0 && delta_orientation>0 )
    %         delta_orientation = -1*sign(theta_testing)*abs(delta_orientation);    
    %     else
    %         delta_orientation = sign(theta_testing)*abs(delta_orientation);    
    %     end
    % end
    theta_testing = theta_testing + delta_orientation;       % To make Z axis coincide with the projection of Z axis of the desired goal into the plane of motion (ZX)  
    % theta_testing = theta_testing + delta_orientation+pi/2;       % To make Y axis coincide with the projection of X axis of the desired goal  
    % theta_testing = theta_testing + delta_orientation+pi;       % To make Y axis coincide with the projection of X axis of the desired goal  
    hold on;
%     goal_frame = double(A01_subs)*rotm2tform(rotz(theta_testing));      % To show the goal frame after turning theta_testing+delta_orientation angle
    goal_frame = double(A01_subs)*rotm2tform(rotz(theta_testing*180/pi))      % To show the goal frame after turning theta_testing+delta_orientation angle
%     goal_frame = goal_frame*rotm2tform(rotz(pi/2))*rotm2tform(rotx(pi/2));   % For Matlab in lab
    goal_frame = goal_frame*rotm2tform(rotz(90))*rotm2tform(rotx(90))  % For Matlab in house 
    goal_frame(:,4) = new_A05(:,4);
    trplot(goal_frame, 'color', 'r')
    
 ```
 # Inverse kinematics solving
 ```
    syms theta;
    
    theta = theta_testing;
    
    pwx = A01_vect_to_goal(1) - sqrt(a5^2+a4^2)*cos(theta+extra_angle-pi/2);
    pwy = A01_vect_to_goal(2) - sqrt(a5^2+a4^2)*sin(theta+extra_angle-pi/2);
    
    % Rounding pwx and pwy to zero if these are very small
    if abs(pwx) < 0.001
        pwx = 0;
    end
    if abs(pwy) < 0.001
        pwy = 0;
    end
    
    c3 = (pwx^2+pwy^2-a2^2-a3^2)/(2*a2*a3);
    subtraction = 1-c3^2;
    % Rounding subtraction to zero if very small
    if abs(subtraction) < 0.001
        subtraction = 0;
    end
    
    if(subtraction<0)
        theta_start_with = [0 0 theta_testing-delta_orientation+pi/2];
        delta_orientation = delta_orientation+reorientation;
        stage_IK_solver = 2;
        arm_solution = compute_IKvector(theta_start_with, delta_orientation, A01_subs, p_e_testing, A01_vect_to_goal, a(5), a(4), a(3), a(2), theta1_sol, stage_IK_solver, reorientation)
        ik_solutions_testing_grad = double(arm_solution)*180/pi;
    
    else
    
        s3plus = sqrt(subtraction);
        s3minus = -sqrt(subtraction);
        theta3_I = atan2(s3plus,c3);
        theta3_II = atan2(s3minus,c3);
        
        s2_I = ((a2+a3*c3)*pwy - a3*s3plus*pwx) / (pwx^2+pwy^2);
        s2_II = ((a2+a3*c3)*pwy - a3*s3minus*pwx) / (pwx^2+pwy^2);
        c2_I = ((a2+a3*c3)*pwx + a3*s3plus*pwy) / (pwx^2+pwy^2);
        c2_II = ((a2+a3*c3)*pwx + a3*s3minus*pwy) / (pwx^2+pwy^2);
        
        theta2_I = atan2(s2_I,c2_I);
        theta2_II = atan2(s2_II,c2_II);
        
        theta4_I = theta - theta2_I - theta3_I;
        theta4_II = theta - theta2_II - theta3_II;
    
        arm_solution(1,:) = [theta1_sol, theta2_I+pi/2, theta3_I, theta4_I-pi/2];       %The addition to theta2 is because the computation of IK is performed assuming the finger starts layed down (horizontal)
        arm_solution(2,:) = [theta1_sol, theta2_II+pi/2, theta3_II, theta4_II-pi/2];    %And the subtraction from theta4 is because of the alpha DH parameter of the 5th link.
        
        ik_solutions_testing_grad = arm_solution*180/pi;
        ik_solutions_testing_grad = double(ik_solutions_testing_grad)
    
    end
    
    
    disp('Transformation matrix found with IK:')
    T_with_ik_sols = bot.fkine([ik_solutions_testing_grad(1,:) 0 0]*pi/180)
    
    disp('Obtained with the Robotic toolbox for q_test:')
    A05_testing_toolbox = bot.fkine(q_test)
    
    check_vect = check_feasibility(arm_solution)
    jv_not_valid=verify_all_joint_values(check_vect)
    bot.plot([ik_solutions_testing_grad(plot_robot_conf(check_vect,jv_not_valid),:) 0 0]*pi/180);
else
    disp("POint is outside the finger workspace")
end



%% Correcting unfeasible joint values, if any
if(point_in_ws==1)
    jv_not_valid=verify_all_joint_values(check_vect)
    if(jv_not_valid)
        delta_orientation = 0;
        
        stage_IK_solver = 3;
        complete_turn = 0;
        while (jv_not_valid)
            if complete_turn == 0
                delta_orientation = delta_orientation -reorientation;
            else
                delta_orientation = delta_orientation +reorientation;
            end
    
            theta_start_with = [0 0 theta_testing+pi/2];
            [IK_joint_values, complete_turn, delta_orientation]= compute_IKvector(theta_start_with, delta_orientation, A01_subs, p_e_testing, A01_vect_to_goal, a(5), a(4), a(3), a(2), theta1_sol, stage_IK_solver, reorientation);
            check_vect = check_feasibility(IK_joint_values)
    %         count_check_validation= count_check_validation+1;
            jv_not_valid=verify_all_joint_values(check_vect);
        end
        ik_solutions_testing_grad = double(IK_joint_values)*180/pi;
        bot.plot([ik_solutions_testing_grad(plot_robot_conf(check_vect,jv_not_valid),:) 0 0]*pi/180);
        disp('FINALLY, IK solutions in rad are:')
        arm_solution = double(IK_joint_values)
        disp('FINALLY, IK solutions in grad are:')
        ik_solutions_testing_grad
        disp('FINALLY, the check vector is:')
        check_vect
        save('finger4nov','arm_solution','ik_solutions_testing_grad');
    
    else
        disp('FINALLY, IK solutions in rad are:')
        arm_solution = double(arm_solution)
        disp('FINALLY, IK solutions in grad are:')
        ik_solutions_testing_grad
        disp('FINALLY, the check vector is:')
        check_vect
        save('finger4nov','arm_solution', 'ik_solutions_testing_grad');
    end
else
    disp("POint is outside the finger workspace")
end



function [IK_values, turn_complete, new_delta_orientation] = compute_IKvector(euler_angles, delta_orientation_, A01_subs_, desired_pos, A01_vect_to_goal_, a5, a4, a3, a2, q1_solved, from_stage_flag, reorientation_)
    turn_complete=0;
    new_delta_orientation = delta_orientation_;

    if(abs(delta_orientation_)>=2*pi)
        turn_complete = 1;
    end
    theta_testing = euler_angles(3);
    theta_testing = theta_testing -pi/2;             % The pi/2 subtraction is due to the offset turn in Z that we do for the 5th link DH parameters.
    if(theta_testing ~= 0)
        if (from_stage_flag==3 && turn_complete==1)
            if(theta_testing<0 && delta_orientation_>0 )
                delta_orientation_aux = sign(theta_testing)*abs(delta_orientation_);    
            else
                delta_orientation_aux = -1*sign(theta_testing)*abs(delta_orientation_);    
            end
        else
            if(theta_testing<0 && delta_orientation_>0 )

                delta_orientation_aux = sign(theta_testing)*abs(delta_orientation_);   
            else % THis means theta_testing is positive, so make delta_orientation_aux negative, to subtract from it

                delta_orientation_aux = -1*sign(theta_testing)*abs(delta_orientation_);
            end
        end
    else
        delta_orientation_aux = 0;
    end

    extra_angle = atan2(a5,a4); 
    theta_testing = theta_testing + delta_orientation_aux;       % To make Z axis coincide with the projection of Z axis of the desired goal into the plane of motion (ZX)  
    % theta_testing = theta_testing + delta_orientation+pi/2;       % To make Y axis coincide with the projection of X axis of the desired goal  
    % theta_testing = theta_testing + delta_orientation+pi;       % To make Y axis coincide with the projection of X axis of the desired goal  
    hold on;
%     goal_frame = double(A01_subs_)*rotm2tform(rotz(theta_testing));      % To show the goal frame after turning theta_testing+delta_orientation angle
    goal_frame = double(A01_subs_)*rotm2tform(rotz(theta_testing*180/pi));      % To show the goal frame after turning theta_testing+delta_orientation angle
%     goal_frame = goal_frame*rotm2tform(rotz(pi/2))*rotm2tform(rotx(pi/2));   % For Matlab in lab
    goal_frame = goal_frame*rotm2tform(rotz(90))*rotm2tform(rotx(90));  % For Matlab in house 

    goal_frame(:,4) = [desired_pos;1];

%     if(from_stage_flag==3)
%         trplot(goal_frame, 'color', 'r')
%     end

    % Inverse kinematics solving 
    syms theta;
    
    theta = theta_testing;
    
    pwx = A01_vect_to_goal_(1) - sqrt(a5^2+a4^2)*cos(theta+extra_angle-pi/2);
    pwy = A01_vect_to_goal_(2) - sqrt(a5^2+a4^2)*sin(theta+extra_angle-pi/2);
    
    % Rounding pwx and pwy to zero if these are very small
    if abs(pwx) < 0.001
        pwx = 0;
    end
    if abs(pwy) < 0.001
        pwy = 0;
    end
    
    c3 = (pwx^2+pwy^2-a2^2-a3^2)/(2*a2*a3);
    subtraction = 1-c3^2;
    % Rounding subtraction to zero if very small
    if abs(subtraction) < 0.001
        subtraction = 0;
    end

    if(subtraction<0)
        if (from_stage_flag==1)
            delta_orientation_ = delta_orientation_-reorientation_;
        end
        if (from_stage_flag==2)
            delta_orientation_ = delta_orientation_+reorientation_;
        end
        if (from_stage_flag==3)
            if(turn_complete == 0)
                delta_orientation_ = delta_orientation_-reorientation_;
            else
                delta_orientation_ = delta_orientation_+reorientation_;
            end
            new_delta_orientation = delta_orientation_;
        end
       
        [IK_joint_values, turn_complete, new_delta_orientation]= compute_IKvector(euler_angles, delta_orientation_, A01_subs_, desired_pos, A01_vect_to_goal_, a5, a4, a3, a2, q1_solved, from_stage_flag, reorientation_);
        IK_values = IK_joint_values;

    else  
        s3plus = sqrt(subtraction);
        s3minus = -sqrt(subtraction);
        theta3_I = atan2(s3plus,c3);
        theta3_II = atan2(s3minus,c3);
        
        s2_I = ((a2+a3*c3)*pwy - a3*s3plus*pwx) / (pwx^2+pwy^2);
        s2_II = ((a2+a3*c3)*pwy - a3*s3minus*pwx) / (pwx^2+pwy^2);
        c2_I = ((a2+a3*c3)*pwx + a3*s3plus*pwy) / (pwx^2+pwy^2);
        c2_II = ((a2+a3*c3)*pwx + a3*s3minus*pwy) / (pwx^2+pwy^2);
        
        theta2_I = atan2(s2_I,c2_I);
        theta2_II = atan2(s2_II,c2_II);
        
        theta4_I = theta - theta2_I - theta3_I;
        theta4_II = theta - theta2_II - theta3_II;
        
        arm_solution(1,:) = [q1_solved, theta2_I+pi/2, theta3_I, theta4_I-pi/2];       %The subtraction from theta2 is because the computation of IK is performed assuming the finger starts layed down (horizontal)
        arm_solution(2,:) = [q1_solved, theta2_II+pi/2, theta3_II, theta4_II-pi/2];    %And the addition to theta4 is because of the alpha DH parameter of the 6th ficticious link.
    
        IK_values = arm_solution;
    end
    
end


function res = check_feasibility(IK_solutions)
    max_limits = [0.47 1.61 1.71 1.62];
    min_limits = [-0.47 -0.2 -0.17 -0.23];
    res = zeros(2,4);
    for i=1:2
        for j=1:4
            if(IK_solutions(i,j)<=max_limits(j) && IK_solutions(i,j)>=min_limits(j))
                res(i,j) = 1;
            else
                res(i,j) = -1;
            end
        end
    end
    
    for i=1:2
        for j=1:4
            if(res(i,j)==-1)
                if(IK_solutions(i,j)<0)
                    if(IK_solutions(i,j)<-pi)
                        aux = IK_solutions(i,j)+2*pi;   
                        if(aux<=max_limits(j) && aux>=min_limits(j))
                            res(i,j) = 1;
                        else
                            res(i,j) = -1;
                        end 
                    end


                end
                if(IK_solutions(i,j)>0)
                    if(IK_solutions(i,j)>2*pi)
                        aux = IK_solutions(i,j)-2*pi;   
                        if(aux<=max_limits(j) && aux>=min_limits(j))
                            res(i,j) = 1;
                        else
                            res(i,j) = -1;
                        end
                    end
                end
            end
        end
    end
end

function res = verify_all_joint_values(feasibility_vec)
    if(ismember(-1, feasibility_vec(1,:)))
        res = 1;
    else
        res = 0;
    end

    if((ismember(-1, feasibility_vec(2,:))==0) && res==1)
        res = 0;
    end
end
function index = plot_robot_conf(feasibility_vec, not_feasible)
    if(not_feasible)
        index = 1;
    else
        if(ismember(-1, feasibility_vec(1,:))==0)
            index = 1;
        else
            if(ismember(-1, feasibility_vec(2,:))==0)
                index = 2;
            end
        end
    end
end
function point_in_ws = checkPointInsideWs(px, py, pz)
    load ('limit_equations','p1','p2','p3','p4');
    if(px == 0 && py==0 && pz == 0)
        point_in_ws = 0;
    else
        if(px<0)
            XY_feasible = false;
        else
            if( (polyval(p1,px)-py >= 0) && (polyval(p2,px)-py <= 0) )
                XY_feasible = true;
            else
                XY_feasible = false;
            end
        end
        x_on_XZ_plane = sqrt(px^2+py^2);
        if((polyval(p3,x_on_XZ_plane)-pz >= 0) && (polyval(p4,x_on_XZ_plane)-pz <= 0) )
            XZ_feasible = true;
        else
            XZ_feasible = false;
        end
    
        if(XZ_feasible && XY_feasible)
            point_in_ws = 1;
        else
            point_in_ws = 0;
        end
    end

end
```
