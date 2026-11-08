clc; clear; close all;

%% Simulation Time
T = 100;                          % s
tspan = 0:0.05:T;

%% Spacecraft and CMG Parameters
J = diag([0.38, 0.37, 0.22]);       % kg m^2, representative inertia
Iw = 42e-6;                       % kg m^2
Omega = 838;                      % rad/s
cmg.h0 = Iw * Omega;              % N m s
cmg.beta = deg2rad(54.74);         % rad
cmg.a_min = 0.20;

%% Reference Motion
slew.periodic = false;
slew.axis = [1; 1; 1] / sqrt(3);
slew.angle = deg2rad(25);         % rad
slew.slew_time = 30;              % s

principal_slew = slew;
principal_slew.axis = [1; 0; 0];

periodic.periodic = true;
periodic.amplitude = 0.035;       % rad/s per axis
periodic.frequency = 0.40;        % rad/s

%% Initial Conditions
q_d0 = [0; 0; 0; 1];
omega0 = zeros(3, 1);             % rad/s
theta_hat0 = [0.26; 0.26; 0.26];   % kg m^2
delta0 = zeros(4, 1);             % rad

offset_axis = [1; -2; 1] / norm([1; -2; 1]);
offset_angles = deg2rad([10; 20]);  % rad

%% Controller Gains
gains.k_e = 0.05;
K_nominal = 0.30;
Gamma_nominal = 40000;
alpha_nominal = 0.45;

% Gain cases change one gain from the nominal oblique slew.
case_name = {'Nominal'; 'Lower K'; 'Lower Gamma'; 'Lower Alpha'};
K_values = [K_nominal; 0.18; K_nominal; K_nominal];
Gamma_values = [Gamma_nominal; Gamma_nominal; 350; Gamma_nominal];
alpha_values = [alpha_nominal; alpha_nominal; alpha_nominal; 0.30];
case_ref = repmat({slew}, 4, 1);
case_q0 = repmat({q_d0}, 4, 1);

% Initial error and motion cases use the nominal gains.
for ii = 1:numel(offset_angles)
    case_name{end+1, 1} = sprintf('%d deg Offset', round(rad2deg(offset_angles(ii))));
    case_ref{end+1, 1} = slew;
    case_q0{end+1, 1} = axis_angle_quat(offset_axis, offset_angles(ii));
end
case_name = [case_name; {'Principal-Axis Slew'; 'Periodic Motion'}];
case_ref = [case_ref; {principal_slew; periodic}];
case_q0 = [case_q0; {q_d0; q_d0}];

added_cases = numel(case_name) - numel(K_values);
K_values = [K_values; repmat(K_nominal, added_cases, 1)];
Gamma_values = [Gamma_values; repmat(Gamma_nominal, added_cases, 1)];
alpha_values = [alpha_values; repmat(alpha_nominal, added_cases, 1)];

gain_cases = 1:4;
offset_cases = 5:4+numel(offset_angles);
motion_cases = [1, numel(case_name)-1, numel(case_name)];

%% Main Simulation Loop
case_results = cell(numel(case_name), 1);
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);

for ii = 1:numel(case_name)
    gains.K = K_values(ii) * eye(3);
    gains.Gamma = Gamma_values(ii) * eye(3);
    gains.alpha = alpha_values(ii);
    ref = case_ref{ii};
    y0 = [case_q0{ii}; omega0; theta_hat0; delta0; q_d0];

    odefun = @(t, y) attitude_dynamics(t, y, J, cmg, ref, gains);
    [t, y] = ode45(odefun, tspan, y0, options);
    case_results{ii} = simulation_results(t, y, J, cmg, ref, gains);
end

%% Results
results = [case_results{:}];
fprintf('K and Gamma are multiples of the identity. k_e = %.2f for all cases.\n', gains.k_e);
fprintf('Peak errors are over each simulated interval. Final values are at EndTime_s.\n\n');

performance = table(case_name, K_values, Gamma_values, alpha_values, [results.end_time]', ...
    rad2deg([results.peak_angle]'), rad2deg([results.peak_rate_error]'), ...
    [results.settling_time]', [results.final_inertia_error]', ...
    'VariableNames', {'Case', 'K', 'Gamma', 'Alpha', 'EndTime_s', 'PeakAngle_deg', ...
    'PeakRateError_deg_s', 'SettlingTime_s', 'FinalInertiaError_kg_m2'});
disp(performance);

theta_final = reshape([results.final_theta_hat], 3, [])';
estimates = table(case_name, theta_final(:, 1), theta_final(:, 2), theta_final(:, 3), ...
    'VariableNames', {'Case', 'J1_hat', 'J2_hat', 'J3_hat'});
disp(estimates);

actuator = table(case_name, 1000 * [results.peak_torque]', ...
    rad2deg([results.peak_gimbal_rate]'), [results.min_sigma]', ...
    [results.peak_torque_error]', ...
    'VariableNames', {'Case', 'PeakTorque_mNm', 'PeakGimbalRate_deg_s', ...
    'MinSigma', 'PeakTorqueResidual_Nm'});
disp(actuator);

checks = table(case_name, [results.max_q_norm_error]', [results.max_momentum_drift]', ...
    [results.max_lyapunov_residual]', ...
    'VariableNames', {'Case', 'MaxQuaternionNormError', 'MomentumDrift_Nms', 'LyapunovResidual'});
disp(checks);

%% Plots
plot_column_figures(results, case_name, gain_cases, offset_cases, motion_cases, ...
             diag(J), cmg, slew.slew_time, T);
plot_results(results, case_name, gain_cases, offset_cases, motion_cases, ...
             diag(J), cmg, slew.slew_time, T);
