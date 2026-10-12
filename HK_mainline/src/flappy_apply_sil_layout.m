function flappy_apply_sil_layout(modelName)
%FLAPPY_APPLY_SIL_LAYOUT Presentation-only layout for the top-level SIL model.
% This function changes block rectangles and orthogonal line routing only.

modelName = string(modelName);
core = {
    "World",                    [  30 230  70 270]
    "world",                    [ 120 230 160 270]
    "torso_to_world",           [ 220 230 260 270]
    "torso",                    [ 320 230 360 270]
    "right_leading_edge",       [ 500  75 540 115]
    "right_stroke",             [ 600  75 640 115]
    "right_rotate",             [ 700  75 740 115]
    "right_wing",               [ 800  75 840 115]
    "right_aero_wrench",        [ 900  60 990 125]
    "left_leading_edge",        [ 500 405 540 445]
    "left_stroke",              [ 600 405 640 445]
    "left_rotate",              [ 700 405 740 445]
    "left_wing",                [ 800 405 840 445]
    "left_aero_wrench",         [ 900 390 990 455]
    "ground_plane",             [1110 225 1200 285]
    "foot_front_left_frame",    [1050  65 1110 115]
    "foot_front_left_point",    [1140  67 1190 113]
    "contact_front_left",       [1230  55 1345 135]
    "foot_front_right_frame",   [1050 205 1110 255]
    "foot_front_right_point",   [1140 207 1190 253]
    "contact_front_right",      [1230 195 1345 275]
    "foot_rear_left_frame",     [1050 345 1110 395]
    "foot_rear_left_point",     [1140 347 1190 393]
    "contact_rear_left",        [1230 335 1345 415]
    "foot_rear_right_frame",    [1050 485 1110 535]
    "foot_rear_right_point",    [1140 487 1190 533]
    "contact_rear_right",       [1230 475 1345 555]
    "sil_clock",                [  40 760  70 780]
    "sil_total_normal",         [ 250 920 280 970]
    "sil_truth_mux",            [ 340 735 365 930]
    "sil_sensor_sample",        [ 420 805 480 835]
    "sil_sensor_delay",         [ 530 805 590 835]
    "sil_hover_controller",     [ 650 800 785 840]
    "sil_controller_demux",     [ 815 720 820 885]
    "sil_actuator_input",       [ 850 795 880 845]
    "sil_virtual_actuator",     [ 920 800 1050 840]
    "sil_actuator_demux",       [1080 720 1085 855]
    "sil_actual_torque_mux",    [1120 865 1145 915]
    "ps_sil_virtual_torque",    [1185 875 1255 905]
    "sil_virtual_torque_wrench",[1295 855 1400 925]
    "sil_world_velocity_mux",   [1120 635 1145 680]
    "sil_wing_state",           [1180 635 1205 680]
    "sil_wing_state_sample",    [1245 640 1310 675]
    "sil_phase_rate",           [1120 545 1175 575]
    "sil_phase",                [1215 545 1245 575]
    "sil_wing_input",           [1350 590 1380 650]
    "sil_wing_loads",           [1430 600 1560 640]
    "sil_wing_demux",           [1600 560 1605 680]
    "sil_left_wing_frame_sensor", [1910  75 2010 135]
    "sil_right_wing_frame_sensor",[1910 405 2010 465]
    "pslog_sil_left_wing_quaternion", [2050  90 2150 120]
    "log_sil_left_wing_quaternion",   [2190  90 2370 120]
    "pslog_sil_right_wing_quaternion",[2050 420 2150 450]
    "log_sil_right_wing_quaternion",  [2190 420 2370 450]
    "log_sil_truth_state",       [ 390 1000 475 1028]
    "log_sil_sensor_state",      [ 585 1000 670 1028]
    "log_sil_controller_output", [ 790 1000 885 1028]
    "log_sil_actuator_output",   [1060 1000 1145 1028]
    "log_sil_phase_rad",         [1215 700 1300 728]
    "log_sil_wing_output",       [1580 700 1665 728]};
for index = 1:size(core,1)
    set_position_if_present(modelName,core{index,1},core{index,2});
end

% Aerodynamic force/torque converters form compact input panels beside each wing.
set_position_if_present(modelName,"ps_right_stroke",[1010  45 1080  75]);
set_position_if_present(modelName,"ps_right_rotate",[1010  90 1080 120]);
set_position_if_present(modelName,"ps_right_force", [1010 135 1080 170]);
set_position_if_present(modelName,"ps_right_torque",[1010 185 1080 220]);
set_position_if_present(modelName,"ps_left_stroke", [1010 375 1080 405]);
set_position_if_present(modelName,"ps_left_rotate", [1010 420 1080 450]);
set_position_if_present(modelName,"ps_left_force",  [1010 465 1080 500]);
set_position_if_present(modelName,"ps_left_torque", [1010 515 1080 550]);

% Contact sensing is arranged as four identical left-to-right rows.
contactNames = ["front_left","front_right","rear_left","rear_right"];
for index = 1:numel(contactNames)
    y = 40+(index-1)*145;
    labels = ["flag","penetration_m","normal_N","friction_N"];
    for outputIndex = 1:numel(labels)
        outputY = y+(outputIndex-1)*42;
        set_position_if_present(modelName,"pslog_contact_"+contactNames(index)+"_"+labels(outputIndex), ...
            [1395 outputY 1510 outputY+28]);
        set_position_if_present(modelName,"log_contact_"+contactNames(index)+"_"+labels(outputIndex), ...
            [1550 outputY 1765 outputY+28]);
        set_font_if_present(modelName,"pslog_contact_"+contactNames(index)+"_"+labels(outputIndex),8);
        set_font_if_present(modelName,"log_contact_"+contactNames(index)+"_"+labels(outputIndex),8);
    end
end

% State-log panel: converters and sinks are paired and never overlap.
stateNames = ["x_m","y_m","z_m","vx_mps","vy_mps","vz_mps", ...
    "quaternion","omega_rad_s"];
for index = 1:numel(stateNames)
    column = 0;
    if index > 4, column = 1; end
    row = mod(index-1,4);
    x = 1480+column*290;
    y = 760+row*45;
    set_position_if_present(modelName,"pslog_body_"+stateNames(index),[x y x+110 y+28]);
    set_position_if_present(modelName,"log_body_"+stateNames(index),[x+150 y x+275 y+28]);
    set_font_if_present(modelName,"pslog_body_"+stateNames(index),8);
    set_font_if_present(modelName,"log_body_"+stateNames(index),8);
end

function set_font_if_present(modelName,blockName,fontSize)
handle = getSimulinkBlockHandle(modelName+"/"+blockName);
if handle ~= -1
    set_param(handle,"FontSize",num2str(fontSize));
end
end

% Recompute line paths after moving blocks.  Routing does not change ports or
% connectivity; failed automatic routes retain their original valid path.
lineHandles = find_system(modelName,"FindAll","on","Type","line");
for lineIndex = 1:numel(lineHandles)
    try
        Simulink.BlockDiagram.routeLine(lineHandles(lineIndex));
    catch
    end
end
end

function set_position_if_present(modelName,blockName,position)
handle = getSimulinkBlockHandle(modelName+"/"+blockName);
if handle ~= -1
    set_param(handle,"Position",position);
end
end
