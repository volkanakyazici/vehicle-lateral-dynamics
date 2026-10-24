function report = test_math()
% Analytic and physical checks; requires only base MATLAB.
p=parameters(); checks=0;
for U=p.speeds_mps(:)'
    M=model_matrices(p,U); [K,design]=design_controller(M,p);
    assert(all(real(M.lateral_poles)<0),'Lateral subsystem is unstable.'); checks=checks+1;
    assert(design.spectral_radius<1 && design.residual<1e-9); checks=checks+1;
    x=[0.1;0.02;0.03;0.4]; d=0.004;
    a=p.front_distance_m; b=p.rear_distance_m;
    Ff=p.front_axle_stiffness_Nprad*(d-(x(1)+a*x(2))/U);
    Fr=p.rear_axle_stiffness_Nprad*(-(x(1)-b*x(2))/U);
    direct=[(Ff+Fr)/p.mass_kg-U*x(2);(a*Ff-b*Fr)/p.yaw_inertia_kgm2;x(2);x(1)+U*x(3)];
    assert(max(abs(M.A*x+M.B*d-direct))<1e-12,'Force/matrix mismatch.'); checks=checks+1;
    equilibrium=-M.A(1:2,1:2)\(M.B(1:2)*d);
    analytic=U*d/(a+b+M.Kus*U^2);
    assert(abs(equilibrium(2)-analytic)<1e-12,'Steady yaw gain mismatch.'); checks=checks+1;
    c=make_case('zero',U,p); x0=reference_loop(c,M,K);
    assert(max(abs(x0(:)))==0); checks=checks+1;
    c=make_case('step',U,p); positive=reference_loop(c,M,K);
    c.open=-c.open; negative=reference_loop(c,M,K);
    assert(max(abs(positive(:)+negative(:)))<1e-12); checks=checks+1;
    c=make_case('track',U,p); coarse=reference_loop(c,M,K);
    finep=p; finep.sample_time_s=p.sample_time_s/2;
    finec=make_case('track',U,finep); fineM=model_matrices(finep,U);
    fineK=design_controller(fineM,finep); fine=reference_loop(finec,fineM,fineK);
    assert(max(abs(coarse(:,4)-fine(1:2:end,4)))<0.02,'Sample refinement differs by >=0.02 m.'); checks=checks+1;
end
report=struct('status','PASS','number_of_assertions',checks,'matlab_version',version);
fprintf('PASS: %d base-MATLAB mathematical checks.\n',checks);
end
