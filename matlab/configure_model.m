function configure_model(model,c,M,K)
% Model-workspace variables keep the model independent of base workspace data.
w=get_param(model,'ModelWorkspace');
values=struct('Ad',M.Ad,'Bd',M.Bd,'K',K,'Ts',c.Ts,'x0',c.x0, ...
    'ref_input',[c.t c.xref],'ff_input',[c.t c.ff], ...
    'open_input',[c.t c.open],'tracking_enabled',double(c.tracking), ...
    'delta_limit',c.limit,'delta_increment',c.rate*c.Ts);
names=fieldnames(values);
for i=1:numel(names), assignin(w,names{i},values.(names{i})); end
set_param(model,'FixedStep',num2str(c.Ts,17),'StopTime',num2str(c.t(end),17));
end
