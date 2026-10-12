function rigid = flappy_hk_rigid_parameters()
%FLAPPY_HK_RIGID_PARAMETERS Single source for the supplied HK rigid-body data.
% All centres of mass and inertia tensors are expressed in the existing
% Flappy link frames.  The wing and leading-edge axes are treated as their
% principal axes until full CAD inertia products are supplied.

% Torso inertia about its CoM, in kg*m^2, resolved in the torso link axes.
% User CAD data (2026-09-29), in g*mm^2, at CoM with output-axis alignment:
% [552203.47 -1627.17 44.76; -1627.17 532212.21 -25839.64;
%  44.76 -25839.64 280114.70].
% CAD axes: +X up, +Y forward, +Z left.  Model axes: +X forward,
% +Y left, +Z up, so R_BC = [0 1 0; 0 0 1; 1 0 0].
% Assume the earlier SolidWorks POSITIVE product-of-inertia convention:
% negate CAD off-diagonals first, then I_B = R_BC*I_C*R_BC.'*1e-9.
% This assumption needs review if the CAD output convention was changed.
% Iyy is the second diagonal entry below (2.801147e-4).
% Rebuild the HK mechanical chain after confirmation; saved SLX models
% retain their previously imported inertia until rebuilt.
rigid.torso = localPart(0.200, [0;0;-0.010], ...
    [5.3221221e-4 2.583964e-5 1.62717e-6; ...
     2.583964e-5 2.8011470e-4 -4.476e-8; ...
     1.62717e-6 -4.476e-8 5.5220347e-4]);

% User-requested 2x wing-system mass case.  The CAD geometry and each
% link's CoM are unchanged, therefore mass and inertia scale together.
% This preserves the same density distribution instead of introducing an
% inconsistent mass-only change.
wingMassScale = 2.0;
leadingEdgeInertia = wingMassScale*diag([1.25e-6 2.0e-8 1.23e-6]);
wingInertia = wingMassScale*diag([1.09e-6 1.3e-7 9.6e-7]);
rigid.left_leading_edge = localPart(wingMassScale*0.0005, [0;0.0486;0.0025], ...
    leadingEdgeInertia);
rigid.right_leading_edge = localPart(wingMassScale*0.0005, [0;-0.0486;0.0025], ...
    leadingEdgeInertia);
rigid.left_wing = localPart(wingMassScale*0.0003, [0;0.0702;0.0240], wingInertia);
rigid.right_wing = localPart(wingMassScale*0.0003, [0;-0.0702;0.0240], wingInertia);

rigid.total_mass_kg = rigid.torso.mass_kg + rigid.left_leading_edge.mass_kg + ...
    rigid.right_leading_edge.mass_kg + rigid.left_wing.mass_kg + ...
    rigid.right_wing.mass_kg;
assert(abs(rigid.total_mass_kg-(0.200+2*wingMassScale*(0.0005+0.0003))) < 1e-12, ...
    "flappy:hkMass", "HK rigid-body mass no longer matches the configured wing mass scale.");
end

function part = localPart(massKg,comM,inertiaKgM2)
part = struct("mass_kg",massKg,"com_m",comM(:), ...
    "inertia_kg_m2",inertiaKgM2);
end
