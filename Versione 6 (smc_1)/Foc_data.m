%% ************************************************************************
% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_hall_S32K144EVB_data.m
% Copyright 2020 NXP.

%% Parameters needed for Offset computation are
% target.PWM_Counter_Period  - PWM counter value for epwm blocks
% target.CPU_frequency       - CPU frequency of the microcontroller
% Ts                         - Control sample time
% PU_System.N_base           - Base speed for per unit conversion
% pmsm.p                     - Number pole pairs in the motor

% Other parameters are not mandatory for offset computation
%% Set PWM Switching frequency
PWM_frequency 	= 20e3;    %Hz          // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s  // PWM switching time period
 
%% Set Sample Times
Ts          	= T_pwm * 2;  %sec        // simulation time step for controller
Ts_simulink     = T_pwm;      %sec        // simulation time step for model simulation
Ts_motor        = T_pwm;      %Sec        // Simulation sample time
Ts_inverter     = T_pwm;      %sec        // simulation time step for average value inverter
Ts_speed        = 10*Ts;      %Sec        // Sample time for speed controller

%% Set data type for controller & code-gen
dataType = 'single';            % Floating point code-generation

%% System Parameters // Hardware parameters 

pmsm.model  = 'LINIX-45ZVN24';  %           // Manufacturer Model Number
pmsm.sn     = '40';             %           // Manufacturer Model Number
pmsm.p      = 2;                %           // Pole Pairs for the motor
pmsm.Rs     = 0.56;             %Ohm        // Shunt Resistor
pmsm.Ld     = 0.375e-3;         %H          // D-axis inductance value
pmsm.Lq     = 0.435e-3;         %H          // Q-axis inductance value
pmsm.J      = 0.12e-4;          %Kg-m2      // Inertia in SI units
pmsm.B      = 0.1e-6;           %Kg-m2/s    // Friction Co-efficient
pmsm.Ke     = 5.8;        %Bemf Const	// Vline_peak/krpm
pmsm.Kt     = 0.097;         %Nm/A       // Torque constant
pmsm.I_rated= 2.3;              %A      	// Rated current (phase-peak)
pmsm.N_max  = 2000;             %rpm        // Max speed
pmsm.QEPIndexOffset = 0.1712;   %0.6879/4;	%PU position// QEP Offset
pmsm.QEPSlits       = 1000;     %           // QEP Encoder Slits
pmsm.PositionOffset = 0.7061;
pmsm.FluxPM     = (pmsm.Ke)/(sqrt(3)*2*pi*1000*4/60);               %PM flux computed from Ke
pmsm.T_rated    = (3/2)*pmsm.p*pmsm.FluxPM*pmsm.I_rated;   %Get T_rated from I_rated

Target.model                = 'EVB_S32K144';% Manufacturer Model Number
Target.CPU_frequency        = 80e6;    %(Hz)   // Clock frequency
Target.PWM_frequency        = PWM_frequency;   %// PWM frequency
Target.PWM_Counter_Period   = round(Target.CPU_frequency/Target.PWM_frequency); %(PWM timer counts)
%% Parameters below are not mandatory for offset computation

% inverter = mcb_SetInverterParameters('MOTORGD');
inverter.model          = 'MOTORGD3000-KIT'; % Manufacturer Model Number
inverter.sn             = 'rev1';         % Manufacturer Serial Number
inverter.V_dc           = 12;       %V      // DC Link Voltage of the Inverter
inverter.I_max          = 31.25;        %Amps   // Max current that can be measured by ADC
inverter.I_trip         = 31.25;       %Amps   // Max current for trip
inverter.Rds_on         = 35e-3;    %Ohms   // Rds ON
inverter.Rshunt         = 0.010;    %Ohms   // Rshunt
inverter.MaxADCCnt      = 4095;     %Counts // ADC Counts Max Value
inverter.CtSensAOffset  = 2040;     %Counts // ADC Offset for phase-A
inverter.CtSensBOffset  = 2040;     %Counts // ADC Offset for phase-B
inverter.ADCGain        = 1;        %       // ADC Gain factor scaled by SPI
inverter.R_board        = inverter.Rds_on + inverter.Rshunt/3;  %Ohms
%% Derive Characteristics
Rs          = pmsm.Rs + inverter.R_board;

%Get base speed
Iq          = pmsm.I_rated;
Lq          = pmsm.Lq;
Flux_PM     = pmsm.FluxPM;
Vph_max     = inverter.V_dc/sqrt(3);
ConvRPM     = 60/(2*pi*pmsm.p);

a           = ((Lq*Iq)^2 + Flux_PM^2);
b           = 2*Iq*Rs*Flux_PM;
c           = (Iq*Rs)^2 - (Vph_max)^2;

equation    = [a b c];

speeds      = roots(equation);
speeds_rpm  = ConvRPM*speeds;

%find the positive root
if speeds_rpm(1) > speeds_rpm(2)
    pmsm.N_base = double(uint32(speeds_rpm(1)));
else
    pmsm.N_base = double(uint32(speeds_rpm(2)));
end

%% PU System details // Set base values for pu conversion

% Get Per-Unit System parameters
PU_System.V_base   = (inverter.V_dc/sqrt(3));
PU_System.I_base   = inverter.I_max;
PU_System.N_base   = pmsm.N_base;

PU_System.T_base   = (3/2)*pmsm.p*pmsm.FluxPM*PU_System.I_base;
PU_System.P_base   = (3/2)*PU_System.V_base*PU_System.I_base;

%% Controller design // Get ballpark values!
% Sensor Delays
pmsm.Rs = pmsm.Rs + inverter.R_board;

PI_params.T1 = Ts;                          %Current Sensor Delay
PI_params.T2 = Ts_speed;           	%Speed Sensor Delay

%% Delay calculations

% Current Controllers
PI_params.sigma   = 10*Ts;

PI_params.Ti_i    = pmsm.Lq/pmsm.Rs;                % Current loop integrator time constant

PI_params.Kp_i    = (pmsm.Lq*PU_System.I_base)/(2*PI_params.sigma);   % Kp // Current Loop        
PI_params.Ki_i    = (pmsm.Rs*PU_System.I_base)/(2*PI_params.sigma);   % Ki // Current Loop

% Speed Controller
PI_params.delta       = 2*PI_params.sigma - 0.5*Ts;     %sec // delay for current loop to settle down
PI_params.delay_IIR   = 0.05;

PI_params.delta       = 10*(2*PI_params.sigma + Ts) + PI_params.delay_IIR + Ts_speed + PI_params.T2; % 10x delay for current loop to reach steady-state

PI_params.x       = 1.2;                    % Speed controller delay factor 1< x <20

PI_params.Ti_speed          = (PI_params.x^2)*PI_params.delta;                          % Speed loop integrator time constant
PI_params.Kp_speed          = (pmsm.J/(PI_params.x*PI_params.delta))*PU_System.N_base;  % Kp // Speed Loop
PI_params.Ki_speed          = (PI_params.Kp_speed/PI_params.Ti_speed);                  % Ki // Speed Loop    
PI_params.Ki_speed_texas    = PI_params.Ki_speed*Ts_speed/PI_params.Kp_speed;           % Ki // Speed Loop for TI based controller


% Field Weakening Controller
PI_params.gamma = 0.1*PI_params.sigma;
PI_params.Kp_fwc  = (pmsm.Lq*PU_System.I_base)/(2*PI_params.gamma);   % Kp // Current Loop        
PI_params.Ki_fwc  = (pmsm.Rs*PU_System.I_base)/(2*PI_params.gamma);   % Ki // Current Loop
% %updating for debug
 PI_params.Kp_i = 0.5;
 PI_params.Ki_i = 5;


%Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Position    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);
PI_params.delay_Speed1      = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;

%% VARIABILI DI CONTROLLO

% SMC Velocità
lambda = 20;                 %0.2   |    20
rho = 0.05;                %0.012   |    0.05
epsilon = 20;                %0.05  |    20


% SMC Correnti
lambda_I = 0.2;                 %0.2   |    20
rho_I = 0.05;                %0.012   |    0.05
epsilon_I = 0.05;                %0.05  |    20

% Cotrollo PI Correnti
zeta1 = 0.707;      
zeta2 = 1;
gamma1 = 0.65;                           %0.6


%% TUNING PID CONTROLLER
pmsm.Rs = pmsm.Rs + inverter.R_board;

wnq = (1/(1-gamma1))*(pmsm.Rs/pmsm.Lq);
Kcq = (2*zeta1*wnq*pmsm.Lq)-pmsm.Rs;
t_iq = (2*zeta1*wnq*pmsm.Lq-pmsm.Rs)/(wnq^2*pmsm.Lq);
Kiq = Kcq/t_iq;

wnd = (1/(1-gamma1))*(pmsm.Rs/pmsm.Ld);
Kcd = (1/pmsm.p)*(2*zeta1*wnd*pmsm.Ld)-pmsm.Rs;
t_id = (1/pmsm.p)*(2*zeta1*wnd*pmsm.Ld-pmsm.Rs)/(wnd^2*pmsm.Ld);
Kid = Kcd/t_id;


%% CALCOLO INDICI INTEGRALI
error = SpeedError.signals.values;

%Calcolo degli indici di prestazione
[ISE, IAE, ITAE] = computeIndices(error, Ts_speed);

ok = carica_dati(gamma1, lambda, rho, epsilon, ISE, IAE, ITAE, error);

fprintf('ISE: %.4f\n', ISE);
fprintf('IAE: %.4f\n', IAE);
fprintf('ITAE: %.4f\n', ITAE);
