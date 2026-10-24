function [K,info] = design_controller(M,p)
% Discrete LQR for nominal, unconstrained state-error dynamics.
% Riccati iteration avoids a Control System Toolbox dependency.
Q=diag(1./p.lqr_state_scales(:).^2); R=1/p.lqr_input_scale_rad^2;
A=M.Ad; B=M.Bd; P=Q; converged=false;
for n=1:20000
    K=(R+B'*P*B)\(B'*P*A);
    next=Q+A'*P*A-A'*P*B*K; next=(next+next')/2;
    if norm(next-P,'fro')<=1e-12*max(1,norm(next,'fro'))
        P=next; converged=true; break
    end
    P=next;
end
assert(converged,'Riccati iteration did not converge.');
K=(R+B'*P*B)\(B'*P*A);
residual=norm(P-(Q+A'*P*A-A'*P*B*K),'fro')/max(1,norm(P,'fro'));
rho=max(abs(eig(A-B*K)));
assert(residual<1e-9 && rho<1,'LQR residual or nominal stability check failed.');
info=struct('Q',Q,'R',R,'P',P,'iterations',n,'residual',residual,'spectral_radius',rho);
end
