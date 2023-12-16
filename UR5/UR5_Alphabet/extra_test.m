%% matlab test file
ur5 = ur5_interface();
K = 0.75;
signal = 0;

%% initalization
% the start point should be at left_buttom 
while signal == 0
    prompt = "Is this the initial writing position? 1 or 0: ";
    ur5.swtich_to_pendant_control() 
    X = input(prompt);
    if X == 1
        signal = 1;
        left_start = ur5.get_current_joints();
        disp('initial position recorded!');
    end
end
signal = 0;

% the end point should be at right_buttom 
while signal == 0
    prompt = "Is this the final writing position? 1 or 0: ";
    X = input(prompt);
    if X == 1
        signal = 1;
        right_end = ur5.get_current_joints();
        disp('final position recorded!');
    end
end
ur5.swtich_to_ros_control();
Word = alphabet('RDKDC',ur5,K,left_start,right_end);
Word.draw('rviz');

