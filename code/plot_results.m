function plot_results(results, case_name, gain_cases, offset_cases, motion_cases, theta, cmg, slew_time, T)
    nominal = results(gain_cases(1));
    gain_results = results(gain_cases);
    oblique = results(motion_cases(1));
    principal = results(motion_cases(2));
    periodic = results(motion_cases(3));
    offset_results = results(offset_cases);
    t = nominal.t;
    lw = 2.0;
    colors = [0.00, 0.45, 0.74;
              0.85, 0.33, 0.10;
              0.13, 0.60, 0.33;
              0.49, 0.18, 0.56];
    gray = [0.55, 0.55, 0.55];
    series_styles = {'-'; '--'; '-.'; ':'};
    component_styles = {'-'; '-'; '-'};

    fig = figure('Name', 'Spacecraft Simulation Results', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 980, 650], 'Color', 'white', ...
                 'DefaultAxesColorOrder', colors);
    tabs = uitabgroup(fig);
    summary_tab = uitab(tabs, 'Title', 'Nominal Slew');
    gain_tab = uitab(tabs, 'Title', 'Slew Gain Comparison');
    offset_tab = uitab(tabs, 'Title', 'Initial Attitude Error');
    motion_tab = uitab(tabs, 'Title', 'Motion Comparison');
    cmg_tab = uitab(tabs, 'Title', 'CMG Feasibility');

    %% Nominal Slew
    summary_layout = tiledlayout(summary_tab, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(summary_layout);
    plot(t, rad2deg(nominal.attitude_error), 'LineWidth', lw);
    xline(slew_time, ':', 'End of slew', 'Color', gray, 'LabelOrientation', 'horizontal', 'LineWidth', 1.0);
    ylabel('Angle [deg]');
    title('Attitude error');

    nexttile(summary_layout);
    plot(t, rad2deg(nominal.rate_error), 'LineWidth', lw);
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 1.0);
    ylabel('Rate [deg/s]');
    title('Angular velocity error');

    nexttile(summary_layout);
    torque = plot(t, 1000 * nominal.tau_cmd, 'LineWidth', lw);
    set(torque, {'LineStyle'}, component_styles);
    ylim padded;
    ylabel('Torque [mN m]');
    title('Commanded body torque');
    legend('\tau_x', '\tau_y', '\tau_z', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    nexttile(summary_layout);
    gimbal_rate = plot(t, rad2deg(nominal.delta_dot), 'LineWidth', lw);
    set(gimbal_rate, {'LineStyle'}, series_styles);
    ylim padded;
    ylabel('Gimbal rate [deg/s]');
    title('Gimbal rates (control input)');
    legend('CMG 1', 'CMG 2', 'CMG 3', 'CMG 4', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    nexttile(summary_layout); hold on;
    for ii = 1:3
        yline(theta(ii), '--', 'Color', colors(ii,:), ...
              'LineWidth', 1.1, 'HandleVisibility', 'off');
    end
    estimate = plot(t, nominal.theta_hat, 'LineWidth', lw);
    set(estimate, {'Color'}, num2cell(colors(1:3,:), 2));
    set(estimate, {'LineStyle'}, component_styles);
    ylim padded;
    xlabel('Time [s]'); ylabel('Inertia [kg m^2]');
    title('Inertia estimates (dashed = true values)');
    legend('J_1', 'J_2', 'J_3', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    nexttile(summary_layout);
    estimate_error = plot(t, nominal.theta_tilde, 'LineWidth', lw);
    set(estimate_error, {'LineStyle'}, component_styles);
    yline(0, ':', 'Color', gray, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylim padded;
    xlabel('Time [s]'); ylabel('True minus estimate [kg m^2]');
    title('Inertia estimation errors');
    legend('J_1', 'J_2', 'J_3', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    %% Slew Gain Comparison
    gain_layout = tiledlayout(gain_tab, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(gain_layout); hold on;
    for ii = 1:numel(gain_results)
        plot(gain_results(ii).t, rad2deg(gain_results(ii).attitude_error), ...
             'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylabel('Attitude error [deg]');
    legend(case_name(gain_cases), 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'NumColumns', numel(gain_cases), 'Box', 'off');

    nexttile(gain_layout); hold on;
    for ii = 1:numel(gain_results)
        plot(gain_results(ii).t, rad2deg(gain_results(ii).rate_error), ...
             'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylabel('Rate error [deg/s]');

    nexttile(gain_layout); hold on;
    for ii = 1:numel(gain_results)
        plot(gain_results(ii).t, vecnorm(gain_results(ii).theta_tilde, 2, 2), ...
             'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xlabel('Time [s]'); ylabel('Inertia error norm [kg m^2]');

    %% Initial Attitude Error
    offset_layout = tiledlayout(offset_tab, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(offset_layout);
    for ii = 1:numel(offset_results)
        semilogy(offset_results(ii).t, rad2deg(offset_results(ii).attitude_error), ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    ylim([1e-8, 1e2]);
    ylabel('Attitude error [deg]');
    legend(case_name(offset_cases), 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    nexttile(offset_layout);
    for ii = 1:numel(offset_results)
        semilogy(offset_results(ii).t, rad2deg(offset_results(ii).rate_error), ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    ylim([1e-8, 1e2]);
    ylabel('Rate error [deg/s]');
    legend(case_name(offset_cases), 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    nexttile(offset_layout);
    for ii = 1:numel(offset_results)
        semilogy(offset_results(ii).t, offset_results(ii).V, ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    xlabel('Time [s]'); ylabel('V');
    title('Lyapunov function');

    nexttile(offset_layout); hold on;
    for ii = 1:3
        yline(theta(ii), '--', 'Color', colors(ii,:), ...
              'LineWidth', 1.1, 'HandleVisibility', 'off');
    end
    estimate = plot(offset_results(1).t, offset_results(1).theta_hat, 'LineWidth', lw);
    set(estimate, {'Color'}, num2cell(colors(1:3,:), 2));
    set(estimate, {'LineStyle'}, component_styles);
    ylim padded;
    xlabel('Time [s]'); ylabel('Inertia [kg m^2]');
    title(sprintf('Inertia estimates, %s', lower(case_name{offset_cases(1)})));
    legend('J_1', 'J_2', 'J_3', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    %% Motion Comparison
    motion_layout = tiledlayout(motion_tab, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(motion_layout); hold on;
    plot(oblique.t, rad2deg(oblique.desired_rate), 'Color', colors(1,:), 'LineWidth', lw);
    plot(periodic.t, rad2deg(periodic.desired_rate), 'Color', colors(3,:), 'LineWidth', lw);
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylabel('Desired rate norm [deg/s]');
    legend('Both slews', 'Periodic', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    nexttile(motion_layout);
    motion_results = [oblique, principal, periodic];
    for ii = 1:numel(motion_results)
        semilogy(motion_results(ii).t, vecnorm(motion_results(ii).theta_tilde, 2, 2), ...
                 'Color', colors(ii,:), 'LineWidth', lw);
        hold on;
    end
    xline(slew_time, ':', 'Color', gray, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xlabel('Time [s]'); ylabel('Inertia error norm [kg m^2]');
    legend('Oblique slew', 'Principal-axis slew', 'Periodic', ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    %% CMG Feasibility
    cmg_layout = tiledlayout(cmg_tab, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    cmg_results = [nominal, periodic, offset_results];
    cmg_names = [case_name(gain_cases(1)); case_name(motion_cases(3)); case_name(offset_cases)];

    nexttile(cmg_layout); hold on;
    for ii = 1:numel(cmg_results)
        plot(cmg_results(ii).t, cmg_results(ii).sigma_min, 'Color', colors(ii,:), ...
             'LineStyle', series_styles{ii}, 'LineWidth', lw);
    end
    yline(cmg.a_min, '--', 'Color', gray, 'LineWidth', 1.1);
    ylim([0, 1]);
    xlabel('Time [s]'); ylabel('\sigma_{min}(A)');
    legend([cmg_names; {'Singularity threshold'}], ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');

    set(findall(fig, 'Type', 'axes'), ...
        'XLim', [0, T], 'XTick', linspace(0, T, 6), 'Box', 'on', ...
        'XGrid', 'on', 'YGrid', 'on', 'XMinorGrid', 'off', 'YMinorGrid', 'off', ...
        'GridColor', [0.85, 0.85, 0.85], 'GridAlpha', 1);
    set(findall(fig, '-property', 'FontName'), 'FontName', 'Times New Roman');

    %% Save Figures
    % Full IEEE text width is 7.16 in, so figures are exported at that size with 9 pt text.
    fig_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures', 'full');
    if ~exist(fig_dir, 'dir')
        mkdir(fig_dir);
    end
    tab_list = {summary_tab, gain_tab, offset_tab, motion_tab, cmg_tab};
    layouts = {summary_layout, gain_layout, offset_layout, motion_layout, cmg_layout};
    heights = [6.0, 5.0, 4.6, 3.8, 3.0];  % in

    axes_list = findall(fig, 'Type', 'axes');
    legend_list = findall(fig, 'Type', 'legend');
    axes_size = get(axes_list, 'FontSize');
    legend_size = get(legend_list, 'FontSize');
    window = get(fig, 'Position');
    set(axes_list, 'FontSize', 9);
    set(legend_list, 'FontSize', 9);
    set(fig, 'Units', 'inches');

    for ii = 1:numel(layouts)
        tabs.SelectedTab = tab_list{ii};
        fig.Position(3:4) = [7.16, heights(ii)];
        drawnow;
        name = fullfile(fig_dir, strrep(lower(tab_list{ii}.Title), ' ', '_'));
        try
            exportgraphics(layouts{ii}, [name '.pdf'], 'ContentType', 'vector');
            exportgraphics(layouts{ii}, [name '.png'], 'Resolution', 600);
        catch err
            warning('Could not save %s (close it if it is open in another app): %s', name, err.message);
        end
    end

    set(axes_list, {'FontSize'}, axes_size);
    set(legend_list, {'FontSize'}, legend_size);
    set(fig, 'Units', 'pixels', 'Position', window);
    tabs.SelectedTab = summary_tab;
end
