function mass_kg = flappy_vehicle_mass(p)
%FLAPPY_VEHICLE_MASS Sum all rigid-link masses from the frozen Flappy URDF.

text = fileread(p.urdf_source);
tokens = regexp(text,'<mass value = "([^"]+)"','tokens');
mass_kg = sum(cellfun(@(token) str2double(token{1}),tokens));
assert(isfinite(mass_kg) && mass_kg > 0,"flappy:vehicleMass", ...
    "Could not determine a positive total mass from the source URDF.");
end
