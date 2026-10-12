function report=validate_mainline()
%VALIDATE_MAINLINE Check physical consistency, paths and existing core tests.
root=startup_mainline();rigid=flappy_hk_rigid_parameters();p=flappy_parameters(root);
assert(abs(flappy_vehicle_mass(p)-rigid.total_mass_kg)<1e-12,'mainline:mass','URDF/source mass mismatch.');
assert(abs(rigid.total_mass_kg-.2032)<1e-12,'mainline:baseline','Expected packaged 2x-wing baseline.');
rows=struct('model',{},'total_mass_kg',{},'geometry_count',{},'analytic_motion',{});
files=dir(fullfile(root,'models','*.slx'));
for k=1:numel(files)
 file=fullfile(string(files(k).folder),string(files(k).name));[~,name]=fileparts(file);name=string(name);
 if name~="flappy_hk_gui_hover",continue;end
 assert(~bdIsLoaded(name),'hk:modelBusy','Close GUI hover model before validation.');
 if bdIsLoaded(name)
  assert(strcmpi(string(get_param(name,'FileName')),file),'mainline:modelConflict','Open model belongs to another tree.');
 end
 load_system(file);cleanup=onCleanup(@()close_system(name,0));total=0;
 for part=["torso","left_leading_edge","left_wing","right_leading_edge","right_wing"]
  expected=rigid.(part);block=name+'/'+part+'/Inertia';
  mass=str2num(get_param(block,'Mass')); %#ok<ST2NM>
  moments=str2num(get_param(block,'MomentsOfInertia')); %#ok<ST2NM>
  products=str2num(get_param(block,'ProductsOfInertia')); %#ok<ST2NM>
  I=expected.inertia_kg_m2;
  assert(abs(mass-expected.mass_kg)<1e-12,'mainline:mass','Stale mass in %s',block);
  assert(max(abs(moments(:)-diag(I)))<1e-12,'mainline:inertia','Stale inertia in %s',block);
  assert(max(abs(products(:)-[I(2,3);I(1,3);I(1,2)]))<1e-12,'mainline:inertia','Stale products in %s',block);
  total=total+mass;
 end
 visuals=find_system(name,'LookUnderMasks','all','FollowLinks','on','Name','Visual');n=0;
 for block=string(visuals).'
  f=replace(string(get_param(block,'ExtGeomFileName')),'\','/');
  assert(startsWith(f,'assets/')&&isfile(fullfile(root,f)),'mainline:geometry','External/missing geometry in %s',block);n=n+1;
 end
 analytic=~isempty(find_system(name,'SearchDepth',1,'Name','sil_wing_kinematics'));
 if contains(name,'sil_')||contains(name,'flappy_hk_')
  assert(analytic,'mainline:motion','Expected analytic motion in %s',name);
  assert(strcmp(get_param(name+'/ps_left_stroke','FilteringAndDerivatives'),'provide'));
 end
 rows(end+1)=struct('model',name,'total_mass_kg',total,'geometry_count',n,'analytic_motion',analytic); %#ok<AGROW>
 clear cleanup
end
report=struct('matlab',version,'total_mass_kg',rigid.total_mass_kg, ...
 'wing_geometry',p.hk.aero,'models',rows,'tests',struct());
report.tests.quadrotor=test_quadrotor_core();
report.tests.gui=test_hover_gui();
report.passed=true;
writetable(struct2table(rows),fullfile(root,'validation','model_consistency.csv'));
fid=fopen(fullfile(root,'validation','validation_report.json'),'w');assert(fid>=0);
cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));
disp('MAINLINE_VALIDATION_PASSED');
end
