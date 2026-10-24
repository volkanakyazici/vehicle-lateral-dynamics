function [T,report] = run_case(kind,U,outdir)
% Run the Simulink plant, compare it with the loop, and export actual results.
if nargin<3
    outdir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','simulink');
end
if ~exist(outdir,'dir'), mkdir(outdir); end
p=parameters(); c=make_case(kind,U,p); M=model_matrices(p,U); [K,design]=design_controller(M,p);
model='lateral_bicycle';
modelFile=fullfile(fileparts(mfilename('fullpath')),[model '.slx']);
assert(isfile(modelFile),'Missing Simulink model: %s',modelFile);
load_system(modelFile); configure_model(model,c,M,K);
out=sim(model);
x=readlog(out.get('state_log'),4);
request=readlog(out.get('request_log'),1);
delta=readlog(out.get('steering_log'),1);
[xr,ur,dr]=reference_loop(c,M,K);
stateError=max(abs(x(:)-xr(:))); requestError=max(abs(request-ur)); steerError=max(abs(delta-dr));
assert(stateError<1e-8 && requestError<1e-9 && steerError<1e-9, ...
    'Simulink mismatch: state=%g; request=%g; steering=%g',stateError,requestError,steerError);
T=result_table(c,p,x,request,delta);
assert(all(isfinite(T{:,:}),'all'),'Nonfinite result.');
assert(max(abs(delta))<=c.limit+1e-12,'Steering angle limit violated.');
assert(max(abs(diff([0;delta])))<=c.rate*c.Ts+1e-12,'Steering rate limit violated.');
if any(strcmp(kind,{'track','offset'}))
    assert(sqrt(mean(T.ey_m(1:end-1).^2))<0.15,'Tracking RMSE exceeds 0.15 m.');
    assert(abs(T.ey_m(end))<0.02,'Final path error exceeds 0.02 m.');
end
if strcmp(kind,'step')
    eq=-M.A(1:2,1:2)\(M.B(1:2)*p.steering_amplitude_deg*pi/180);
    [~,i]=min(abs(c.t-4.99));
    assert(max(abs(x(i,1:2)'-eq))<1e-5,'Step response differs from steady equilibrium.');
end
if strcmp(kind,'limited')
    assert(any(abs(request)>c.limit+1e-9),'Constrained experiment did not activate the angle limit.');
end
screen=max(abs(T.alpha_f_rad))<=p.screen_slip_deg*pi/180 && ...
    max(abs(T.alpha_r_rad))<=p.screen_slip_deg*pi/180 && ...
    max(abs(T.psi_rad))<=p.screen_heading_deg*pi/180 && ...
    max(abs(T.ay_mps2))<=p.screen_lateral_accel_g*p.gravity_mps2;
if ~screen, warning('Model screening limits exceeded in %s at %g m/s.',kind,U); end
name=sprintf('%s_%g',kind,U);
writetable(T,fullfile(outdir,[name '.csv']));
report=struct('case',kind,'speed_mps',U,'source','SIMULINK', ...
    'status','PASS','sample_time_s',c.Ts,'matlab_version',version, ...
    'max_state_difference',stateError,'max_request_difference_rad',requestError, ...
    'max_steering_difference_rad',steerError,'small_angle_screen_passed',screen, ...
    'steering_limit_rad',c.limit,'steering_rate_limit_radps',c.rate, ...
    'config',p,'gain',K,'controller_design',design);
write_json(fullfile(outdir,[name '.json']),report);
fprintf('PASS %s: state difference %.3g; steering difference %.3g rad\n',name,stateError,steerError);
    function data=readlog(ts,width)
        assert(numel(ts.Time)==numel(c.t),'Unexpected logged sample count.');
        assert(max(abs(ts.Time(:)-c.t))<1e-9,'Logged time grid differs.');
        data=squeeze(ts.Data);
        if width==1, data=data(:); end
        if width>1 && isequal(size(data),[width,numel(c.t)]), data=data'; end
        assert(isequal(size(data),[numel(c.t),width]),'Unexpected logged signal shape.');
    end
end
