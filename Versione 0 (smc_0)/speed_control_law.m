function [Te,reset_signal,s] = speed_control_law(omega_ref_dot,speed_error,omega_r,position_error,tuning_parameters)

a1 = tuning_parameters.a1;
ro = tuning_parameters.ro;
epsilon = tuning_parameters.epsilon;
Te_max=tuning_parameters.Te_max;
%Te_max=single(0.035); %saturated
Te_min=tuning_parameters.Te_min;
B=tuning_parameters.B;
J=tuning_parameters.J;
  
%sliding surface
s = speed_error + a1*position_error;

%sat(s)
if abs(s) > epsilon
    v = ro*sign(s);
else
    v = ro*(s/epsilon);                 %% Chiedere se ro è negativo
end

%control law
u_eq = B*omega_r + J*(omega_ref_dot+a1*(speed_error));
u = u_eq + v;

%sat(Te)
if u >= Te_max
    u = Te_max;
    reset_signal=-1;
elseif u <= Te_min
    u = Te_min;
    reset_signal=-1;
else
    reset_signal=1;
end
Te=u;
