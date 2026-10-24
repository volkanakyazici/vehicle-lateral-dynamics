function p = parameters()
% Read the shared SI-unit configuration, independent of the current folder.
root = fileparts(fileparts(mfilename('fullpath')));
p = jsondecode(fileread(fullfile(root,'config.json')));
assert(p.mass_kg>0 && p.yaw_inertia_kgm2>0,'Mass and inertia must be positive.');
assert(p.front_distance_m>0 && p.rear_distance_m>0,'Axle distances must be positive.');
assert(p.front_axle_stiffness_Nprad>0 && p.rear_axle_stiffness_Nprad>0,'Stiffness must be positive.');
assert(p.sample_time_s>0,'Sample time must be positive.');
end
