function plot_column_figures(results, case_name, gain_cases, offset_cases, motion_cases, theta, cmg, slew_time, T)
    nominal = results(gain_cases(1));
    gain_results = results(gain_cases);
    oblique = results(motion_cases(1));
    principal = results(motion_cases(2));
    periodic = results(motion_cases(3));
    offset_results = results(offset_cases);
    t = nominal.t;
    lw = 1.2;
    colors = [0.00, 0.45, 0.74;
              0.85, 0.33, 0.10;
              0.13, 0.60, 0.33;
              0.49, 0.18, 0.56];
    gray = [0.55, 0.55, 0.55];
    series_styles = {'-'; '--'; '-.'; ':'};
    component_styles = {'-'; '-'; '-'};

    fig_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures', 'column');
    if ~exist(fig_dir, 'dir')
        mkdir(fig_dir);
    end

    %% Nominal Tracking Errors
    [fig, layout] = column_figure(2, 3.0);
    nexttile(layout);
    plot(t, rad2deg(nominal.attitude_error), 'Color', colors(1,:), 'LineWidth', lw);
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 0.8);
    ylabel('Attitude error [deg]');
    nexttile(layout);
    plot(t, rad2deg(nominal.rate_error), 'Color', colors(1,:), 'LineWidth', lw);
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 0.8);
    xlabel('Time [s]'); ylabel('Rate error [deg/s]');
    save_column(fig, fig_dir, 'nominal_tracking_errors', T);

    %% Nominal Control Input
    [fig, layout] = column_figure(2, 3.2);
    nexttile(layout);
    torque = plot(t, 1000 * nominal.tau_cmd, 'LineWidth', lw);
    set(torque, {'Color'}, num2cell(colors(1:3,:), 2));
    set(torque, {'LineStyle'}, component_styles);
    ylim padded;
    ylabel('Torque [mN m]');
    legend('\tau_x', '\tau_y', '\tau_z', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    nexttile(layout);
    gimbal_rate = plot(t, rad2deg(nominal.delta_dot), 'LineWidth', lw);
    set(gimbal_rate, {'Color'}, num2cell(colors, 2));
    set(gimbal_rate, {'LineStyle'}, series_styles);
    ylim padded;
    xlabel('Time [s]'); ylabel('Gimbal rate [deg/s]');
    legend('CMG 1', 'CMG 2', 'CMG 3', 'CMG 4', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    save_column(fig, fig_dir, 'nominal_control_input', T);

    %% Nominal Inertia Estimates
    [fig, layout] = column_figure(2, 3.2);
    nexttile(layout); hold on;
    for ii = 1:3
        yline(theta(ii), '--', 'Color', colors(ii,:), 'LineWidth', 0.8, 'HandleVisibility', 'off');
    end
    estimate = plot(t, nominal.theta_hat, 'LineWidth', lw);
    set(estimate, {'Color'}, num2cell(colors(1:3,:), 2));
    set(estimate, {'LineStyle'}, component_styles);
    ylim padded;
    ylabel('Inertia [kg m^2]');
    legend('J_1', 'J_2', 'J_3', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    nexttile(layout);
    estimate_error = plot(t, nominal.theta_tilde, 'LineWidth', lw);
    set(estimate_error, {'Color'}, num2cell(colors(1:3,:), 2));
    set(estimate_error, {'LineStyle'}, component_styles);
    yline(0, ':', 'Color', gray, 'LineWidth', 0.8);
    ylim padded;
    xlabel('Time [s]'); ylabel('Estimate error [kg m^2]');
    save_column(fig, fig_dir, 'nominal_inertia', T);

    %% Slew Gain Comparison
    [fig, layout] = column_figure(3, 4.2);
    nexttile(layout); hold on;
    for ii = 1:numel(gain_results)
        plot(gain_results(ii).t, rad2deg(gain_results(ii).attitude_error), ...
             'Color', colors(ii,:), 'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 0.8, 'HandleVisibility', 'off');
    ylabel('Attitude error [deg]');
    legend(case_name(gain_cases), 'Location', 'northoutside', 'NumColumns', 2, 'Box', 'off');
    nexttile(layout); hold on;
    for ii = 1:numel(gain_results)
        plot(gain_results(ii).t, rad2deg(gain_results(ii).rate_error), ...
             'Color', colors(ii,:), 'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 0.8);
    ylabel('Rate error [deg/s]');
    nexttile(layout); hold on;
    for ii = 1:numel(gain_results)
        plot(gain_results(ii).t, vecnorm(gain_results(ii).theta_tilde, 2, 2), ...
             'Color', colors(ii,:), 'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 0.8);
    xlabel('Time [s]'); ylabel('Inertia error [kg m^2]');
    save_column(fig, fig_dir, 'slew_gain_comparison', T);

    %% Initial Attitude Error, Tracking
    [fig, layout] = column_figure(2, 3.2);
    nexttile(layout);
    for ii = 1:numel(offset_results)
        semilogy(offset_results(ii).t, rad2deg(offset_results(ii).attitude_error), ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    ylim([1e-8, 1e2]);
    ylabel('Attitude error [deg]');
    legend(case_name(offset_cases), 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    nexttile(layout);
    for ii = 1:numel(offset_results)
        semilogy(offset_results(ii).t, rad2deg(offset_results(ii).rate_error), ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    ylim([1e-8, 1e2]);
    xlabel('Time [s]'); ylabel('Rate error [deg/s]');
    save_column(fig, fig_dir, 'offset_tracking_errors', T);

    %% Initial Attitude Error, Lyapunov Function and Estimates
    [fig, layout] = column_figure(2, 3.2);
    nexttile(layout);
    for ii = 1:numel(offset_results)
        semilogy(offset_results(ii).t, offset_results(ii).V, ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    ylim([1e-7, 1e-2]);
    yticks(10.^(-7:-2));
    ylabel('V');
    legend(case_name(offset_cases), 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    nexttile(layout); hold on;
    for ii = 1:3
        yline(theta(ii), '--', 'Color', colors(ii,:), 'LineWidth', 0.8, 'HandleVisibility', 'off');
    end
    estimate = plot(offset_results(1).t, offset_results(1).theta_hat, 'LineWidth', lw);
    set(estimate, {'Color'}, num2cell(colors(1:3,:), 2));
    set(estimate, {'LineStyle'}, component_styles);
    ylim padded;
    xlabel('Time [s]'); ylabel('Inertia [kg m^2]');
    legend('J_1', 'J_2', 'J_3', 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    save_column(fig, fig_dir, 'offset_lyapunov_estimates', T);

    %% Motion Comparison
    [fig, layout] = column_figure(2, 3.2);
    nexttile(layout); hold on;
    plot(oblique.t, rad2deg(oblique.desired_rate), 'Color', colors(1,:), 'LineWidth', lw);
    plot(periodic.t, rad2deg(periodic.desired_rate), 'Color', colors(3,:), 'LineWidth', lw);
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 0.8, 'HandleVisibility', 'off');
    ylabel('Desired rate [deg/s]');
    legend('Both slews', 'Periodic', 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    nexttile(layout);
    motion_results = [oblique, principal, periodic];
    for ii = 1:numel(motion_results)
        semilogy(motion_results(ii).t, vecnorm(motion_results(ii).theta_tilde, 2, 2), ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 0.8, 'HandleVisibility', 'off');
    xlabel('Time [s]'); ylabel('Inertia error [kg m^2]');
    legend('Oblique', 'Principal-axis', 'Periodic', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    save_column(fig, fig_dir, 'motion_comparison', T);

    %% CMG Feasibility
    [fig, layout] = column_figure(1, 2.4);
    cmg_results = [nominal, periodic, offset_results];
    cmg_names = [case_name(gain_cases(1)); case_name(motion_cases(3)); case_name(offset_cases)];
    nexttile(layout); hold on;
    for ii = 1:numel(cmg_results)
        plot(cmg_results(ii).t, cmg_results(ii).sigma_min, 'Color', colors(ii,:), ...
             'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    yline(cmg.a_min, '--', 'Color', gray, 'LineWidth', 0.8);
    ylim([0, 1]);
    xlabel('Time [s]'); ylabel('\sigma_{min}(A)');
    legend([cmg_names; {'Threshold'}], 'Location', 'northoutside', 'NumColumns', 3, 'Box', 'off');
    save_column(fig, fig_dir, 'cmg_feasibility', T);
end

function [fig, layout] = column_figure(rows, height)
    fig = figure('Visible', 'off', 'Units', 'inches', 'Position', [1, 1, 3.5, height], 'Color', 'white');
    layout = tiledlayout(fig, rows, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
end

function save_column(fig, fig_dir, name, T)
    set(findall(fig, 'Type', 'axes'), ...
        'XLim', [0, T], 'XTick', linspace(0, T, 6), 'Box', 'on', 'FontSize', 8, ...
        'XGrid', 'on', 'YGrid', 'on', 'XMinorGrid', 'off', 'YMinorGrid', 'off', ...
        'GridColor', [0.85, 0.85, 0.85], 'GridAlpha', 1);
    set(findall(fig, 'Type', 'legend'), 'FontSize', 8, 'ItemTokenSize', [15, 18]);
    set(findall(fig, '-property', 'FontName'), 'FontName', 'Times New Roman');
    save_file(fig, fullfile(fig_dir, name));
    close(fig);
end

function save_file(obj, name)
    try
        exportgraphics(obj, [name '.pdf'], 'ContentType', 'vector');
        exportgraphics(obj, [name '.png'], 'Resolution', 600);
    catch err
        warning('Could not save %s (close it if it is open in another app): %s', name, err.message);
    end
end
