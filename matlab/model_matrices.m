function M = model_matrices(p,U,Ts)
% State order: [lateral velocity; yaw rate; body heading; global lateral position].
% Cornering stiffnesses are TOTAL AXLE values, not per-tire values.
if nargin<3, Ts=p.sample_time_s; end
assert(isscalar(U) && isfinite(U) && U>=5,'This model requires U >= 5 m/s.');
assert(Ts>0 && isfinite(Ts),'Invalid sample time.');
m=p.mass_kg; Iz=p.yaw_inertia_kgm2;
a=p.front_distance_m; b=p.rear_distance_m;
Cf=p.front_axle_stiffness_Nprad; Cr=p.rear_axle_stiffness_Nprad;
M.A=[-(Cf+Cr)/(m*U), (b*Cr-a*Cf)/(m*U)-U, 0, 0;
     (b*Cr-a*Cf)/(Iz*U), -(a*a*Cf+b*b*Cr)/(Iz*U), 0, 0;
     0, 1, 0, 0;
     1, 0, U, 0];
M.B=[Cf/m; a*Cf/Iz; 0; 0];
% Exact ZOH discretization of the linear plant, using base MATLAB expm.
E=expm([M.A M.B; zeros(1,5)]*Ts);
M.Ad=E(1:4,1:4); M.Bd=E(1:4,5); M.Ts=Ts; M.U=U;
M.Kus=m/(a+b)*(b/Cf-a/Cr);
M.lateral_poles=eig(M.A(1:2,1:2));
end
