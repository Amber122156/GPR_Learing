function flappy_apply_pitch_axis_offsets(modelName,p)
%FLAPPY_APPLY_PITCH_AXIS_OFFSETS Insert/update physical pitch-axis offsets.
% The offset is a Rigid Transform between each leading-edge link and its
% revolute pitch joint.  It therefore moves the actual rotation axis without
% moving the CAD wing visual mesh.  Positive Z is the local leading-edge
% frame's +Z direction; both sides use the same signed physical offset.

arguments
    modelName (1,1) string
    p (1,1) struct
end

assert(bdIsLoaded(char(modelName)),"Model is not loaded: %s",modelName);
offsetZ = p.vehicle.pitch_axis_offset_z_m;
assert(isscalar(offsetZ) && isfinite(offsetZ), ...
    "pitch_axis_offset_z_m must be one finite scalar.");

jointNames = ["left_rotate"; "right_rotate"];
for jointIndex = 1:numel(jointNames)
    jointPath = modelName+"/"+jointNames(jointIndex);
    transformPath = modelName+"/"+jointNames(jointIndex)+"_axis_offset";

    if getSimulinkBlockHandle(transformPath) == -1
        jointPorts = get_param(jointPath,"PortHandles");
        % Imported SIL joints expose additional physical-signal ports.  LConn1
        % is the multibody base frame; later LConn entries are actuation ports.
        basePort = jointPorts.LConn(1);
        baseLine = get_param(basePort,"Line");
        if iscell(baseLine)
            assert(numel(baseLine)==1, ...
                "Expected one base-frame connection for %s.",jointPath);
            baseLine = baseLine{1};
        end
        assert(baseLine ~= -1, ...
            "Expected a connected base frame for %s.",jointPath);
        sourcePort = get_param(baseLine,"SrcPortHandle");

        delete_line(baseLine);
        add_block("sm_lib/Frames and Transforms/Rigid Transform",transformPath, ...
            "Position",localOffsetBlockPosition(jointPath));
        transformPorts = get_param(transformPath,"PortHandles");
        add_line(modelName,sourcePort,transformPorts.LConn,"autorouting","on");
        add_line(modelName,transformPorts.RConn,basePort,"autorouting","on");
    end

    set_param(transformPath,"TranslationMethod","Cartesian", ...
        "TranslationCartesianOffset",sprintf("[0 0 %.16g]",offsetZ), ...
        "TranslationCartesianOffsetUnits","m");
end
end

function position = localOffsetBlockPosition(jointPath)
position = get_param(jointPath,"Position");
width = position(3)-position(1);
position = position + [-2*width 0 -2*width 0];
end
