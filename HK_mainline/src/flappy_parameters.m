function p = flappy_parameters(projectRoot)
%FLAPPY_PARAMETERS Load the frozen Purdue Flappy nominal vehicle parameters.

arguments
    projectRoot (1,1) string = string(fileparts(fileparts(mfilename('fullpath'))))
end

configFile = fullfile(projectRoot,"third_party","flappy","flappy", ...
    "envs","fwmav","config","mav_config_list.json");
simFile = fullfile(projectRoot,"third_party","flappy","flappy", ...
    "envs","fwmav","config","sim_config.json");
vehicle = jsondecode(fileread(configFile));
p.vehicle = vehicle(1);
p.sim = jsondecode(fileread(simFile));
p.project_root = projectRoot;
% Preserve the upstream URDF as a read-only baseline.  The active HK copy
% retains its kinematics and is the sole edit point for rigid-body properties.
p.urdf_source = fullfile(projectRoot,"assets","flappy_urdf", ...
    "hk_flapper_import.urdf");
p.urdf_import = p.urdf_source;
p.mesh_root = fullfile(projectRoot,"assets","flappy_urdf","meshes");
p.frame = "Flappy URDF body frame; do not apply HK or Guo frame transforms.";
p.hk.rigid = flappy_hk_rigid_parameters();
p.hk.aero = flappy_hk_wing_geometry(projectRoot);
% Retain the upstream field names so the ported Flappy aerodynamic kernel
% receives HK CAD geometry without changing its public calling convention.
p.vehicle.wing_length = p.hk.aero.wing_length_m;
p.vehicle.mean_chord = p.hk.aero.mean_chord_m;
p.vehicle.r00 = p.hk.aero.r00;
p.vehicle.r11 = p.hk.aero.r11;
p.vehicle.r22 = p.hk.aero.r22;
p.vehicle.r33 = p.hk.aero.r33;
p.vehicle.z_cp0 = p.hk.aero.z_cp0;
p.vehicle.z_cp1 = p.hk.aero.z_cp1;
p.vehicle.z_cp2 = p.hk.aero.z_cp2;
p.vehicle.z_rd = p.hk.aero.z_rd;
p.vehicle.aero_scale = p.hk.aero.aero_scale;
p.vehicle.air_density_kg_m3 = p.hk.aero.air_density_kg_m3;
p.vehicle.pitch_axis_offset_z_m = 0;
% Shared visual-frame correction for the supplied HK CAD frame.  Keep this
% value here rather than in a plotting script: ground-contact registration
% uses the same value so visual and contact geometry remain co-located.
p.vehicle.hk_frame_visual_offset_m = [0; 0; 0.020];
p.nominal_action = struct("voltage_amplitude_V",9.3, ...
    "differential_voltage_V",0.0,"mean_voltage_V",0.0,"split_cycle",0.0);
end
