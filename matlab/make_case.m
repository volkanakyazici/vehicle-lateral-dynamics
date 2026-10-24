function c = make_case(kind,U,p)
% Spatial path uses a quintic lane shift. All angles are radians.
if nargin<3, p=parameters(); end
assert(any(strcmp(kind,{'zero','step','sine','uncontrolled','track','offset','limited'})),'Unknown case.');
c.kind=kind; c.U=U; c.Ts=p.sample_time_s;
isOpen=any(strcmp(kind,{'zero','step','sine'}));
duration=10; if ~isOpen, duration=p.path_end_m/U; end
c.t=(0:round(duration/c.Ts))'*c.Ts; N=numel(c.t);
c.xref=zeros(N,4); c.ff=zeros(N,1); c.yref=zeros(N,1);
c.slope=zeros(N,1); c.curvature=zeros(N,1); c.open=zeros(N,1);
amp=p.steering_amplitude_deg*pi/180;
if strcmp(kind,'step')
    c.open(c.t>=1 & c.t<5)=amp;
elseif strcmp(kind,'sine')
    active=c.t>=1 & c.t<7;
    c.open(active)=amp*sin(2*pi*0.5*(c.t(active)-1));
end
if ~isOpen
    Lp=p.path_length_m; H=p.path_shift_m;
    q=min(max((U*c.t-p.path_start_m)/Lp,0),1);
    c.yref=H*(10*q.^3-15*q.^4+6*q.^5);
    c.slope=H/Lp*(30*q.^2-60*q.^3+30*q.^4);
    % Small-heading curvature approximation, consistent with Ydot=vy+U*psi.
    c.curvature=H/Lp^2*(60*q-180*q.^2+120*q.^3);
    a=p.front_distance_m; b=p.rear_distance_m; L=a+b;
    m=p.mass_kg; Cf=p.front_axle_stiffness_Nprad; Cr=p.rear_axle_stiffness_Nprad;
    Kus=m/L*(b/Cf-a/Cr);
    betaRef=(b-m*a*U^2/(L*Cr))*c.curvature;
    c.xref=[U*betaRef, U*c.curvature, c.slope-betaRef, c.yref];
    c.ff=(L+Kus*U^2)*c.curvature;
end
c.tracking=any(strcmp(kind,{'track','offset','limited'}));
c.limit=p.steering_limit_deg*pi/180;
c.rate=p.steering_rate_limit_degps*pi/180;
if strcmp(kind,'limited'), c.limit=0.15*pi/180; c.rate=3*pi/180; end
c.x0=zeros(4,1);
if strcmp(kind,'offset'), c.x0(4)=0.5; end
end
