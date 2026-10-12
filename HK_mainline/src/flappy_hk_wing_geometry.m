function geometry = flappy_hk_wing_geometry(projectRoot)
%FLAPPY_HK_WING_GEOMETRY Derive whole-wing Flappy constants from HK CAD.
% The left CAD mesh is the geometry authority: X is normal/thickness, +Y is
% span, and Z is chord.  Returned values use metres and the nondimensional
% integral definitions expected by Purdue Flappy's Wing.cpp.

arguments
    projectRoot (1,1) string = string(fileparts(fileparts(mfilename('fullpath'))))
end

meshFile = fullfile(projectRoot,"CAD","left_wing.STL");
assert(isfile(meshFile),"flappy:hkWingMesh", ...
    "Missing HK left-wing CAD mesh: %s",meshFile);
persistent cachedRoot cachedSignature cachedGeometry
meshInfo = dir(meshFile);
meshSignature = string(meshInfo.datenum)+"_"+string(meshInfo.bytes);
if ~isempty(cachedGeometry) && cachedRoot == projectRoot && ...
        cachedSignature == meshSignature
    geometry = cachedGeometry;
    return
end
mesh = stlread(meshFile);
vertices = double(mesh.Points)*1e-3;
faces = mesh.ConnectivityList;

span = vertices(:,2);
chordCoordinate = vertices(:,3);
spanRoot = min(span);
spanTip = max(span);
wingLength = spanTip-spanRoot;
assert(wingLength > 1e-4,"flappy:hkWingSpan", ...
    "The HK wing mesh has no usable span along local +Y.");

% Intersect the closed mesh with equally spaced span stations.  The maximum
% minus minimum chord coordinate gives a robust planform chord even for a
% thin solid STL containing both upper and lower surfaces.
station = linspace(spanRoot+wingLength*1e-5,spanTip-wingLength*1e-5,401).';
chord = zeros(size(station));
for stationIndex = 1:numel(station)
    z = localChordBounds(vertices,faces,station(stationIndex),chordCoordinate);
    chord(stationIndex) = z(2)-z(1);
end
assert(all(isfinite(chord)) && all(chord >= 0),"flappy:hkWingChord", ...
    "Could not extract a finite nonnegative chord distribution from %s.",meshFile);

normalisedSpan = (station-spanRoot)/wingLength;
area = trapz(station,chord);
meanChord = area/wingLength;
assert(area > 1e-8 && meanChord > 1e-5,"flappy:hkWingArea", ...
    "HK wing planform area is not positive.");
chordNormalised = chord/meanChord;
geometry = struct();
geometry.mesh_file = meshFile;
geometry.wing_length_m = wingLength;
geometry.planform_area_m2 = area;
geometry.mean_chord_m = meanChord;
geometry.span_station_m = station;
geometry.chord_distribution_m = chord;
geometry.r00 = trapz(normalisedSpan,chordNormalised);
geometry.r11 = trapz(normalisedSpan,normalisedSpan.*chordNormalised);
geometry.r22 = trapz(normalisedSpan,normalisedSpan.^2.*chordNormalised);
geometry.r33 = trapz(normalisedSpan,normalisedSpan.^3.*chordNormalised);
% Flappy's moment coefficients use chord-squared and chord-fourth span
% integrals.  This reproduces its dimensional scaling without changing the
% empirical C_N(alpha), d_cp(alpha), or rotational-damping formula.
geometry.z_cp0 = trapz(normalisedSpan,chordNormalised.^2);
geometry.z_cp1 = trapz(normalisedSpan,normalisedSpan.*chordNormalised.^2);
geometry.z_cp2 = trapz(normalisedSpan,normalisedSpan.^2.*chordNormalised.^2);
geometry.z_rd = trapz(normalisedSpan,chordNormalised.^4);
geometry.span_cp_m = wingLength*geometry.r33/geometry.r22;
geometry.chord_min_m = min(chord);
geometry.chord_max_m = max(chord);
geometry.aero_scale = 1.0;
geometry.air_density_kg_m3 = 1.18009482370369;

cachedRoot = projectRoot;
cachedSignature = meshSignature;
cachedGeometry = geometry;
end

function bounds = localChordBounds(vertices,faces,station,unusedChordCoordinate) %#ok<INUSD>
triangleVertices = reshape(vertices(faces.',:).',3,3,[]);
y = squeeze(triangleVertices(2,:,:));
z = squeeze(triangleVertices(3,:,:));
zIntersections = zeros(0,1);
edgePairs = [1 2;2 3;3 1];
for edgeIndex = 1:size(edgePairs,1)
    first = edgePairs(edgeIndex,1);
    second = edgePairs(edgeIndex,2);
    y1 = y(first,:); y2 = y(second,:);
    crosses = (y1 <= station & y2 >= station) | (y2 <= station & y1 >= station);
    denominator = y2(crosses)-y1(crosses);
    valid = abs(denominator) > eps;
    if any(valid)
        selected = find(crosses);
        selected = selected(valid);
        lambda = (station-y1(selected))./(y2(selected)-y1(selected));
        zIntersections = [zIntersections; z(first,selected).'+ ...
            lambda.'.*(z(second,selected).'-z(first,selected).')]; %#ok<AGROW>
    end
end
assert(numel(zIntersections) >= 2,"flappy:hkWingSlice", ...
    "A CAD span station did not intersect the wing mesh.");
bounds = [min(zIntersections);max(zIntersections)];
end
