function plot_trace_like_c(sig, prev_sig, title_txt, out_dir, base_name, fs, Rs, show_prev)
% plot_trace_like_c — Time-domain I/Q trace plot matching C simulator style.
if nargin < 8
    show_prev = false;
end
if isempty(sig)
    return;
end

f = figure('Name', title_txt, 'Position', [100, 100, 1100, 600], 'Visible', 'off');
ax = axes(f);
hold(ax, 'on');

window_samples = min(length(sig), max(2, round((10 * fs) / Rs)));
plot_points = 2400;
src_pos = linspace(1, window_samples, plot_points);
t_us = (src_pos - 1) / fs * 1e6;

sig_win = sig(1:window_samples);
i_plot = interp1(1:window_samples, real(sig_win), src_pos, 'linear');
q_plot = interp1(1:window_samples, imag(sig_win), src_pos, 'linear');

if show_prev && ~isempty(prev_sig)
    prev_window = min(length(prev_sig), window_samples);
    prev_src = prev_sig(1:prev_window);
    prev_pos = linspace(1, prev_window, plot_points);
    i_prev = interp1(1:prev_window, real(prev_src), prev_pos, 'linear');
    q_prev = interp1(1:prev_window, imag(prev_src), prev_pos, 'linear');
    plot(ax, t_us, i_prev, '--', 'Color', [0.58, 0.64, 0.72], 'LineWidth', 1.0);
    plot(ax, t_us, q_prev, '--', 'Color', [0.80, 0.84, 0.88], 'LineWidth', 1.0);
end

plot(ax, t_us, i_plot, '-', 'Color', [0.23, 0.51, 0.96], 'LineWidth', 1.2);
plot(ax, t_us, q_plot, '-', 'Color', [0.94, 0.27, 0.27], 'LineWidth', 1.2);

title(ax, title_txt, 'FontSize', 14, 'FontWeight', 'normal');
xlabel(ax, 'Time (us)');
ylabel(ax, 'Amplitude (V)');
grid(ax, 'on');
set(ax, 'Color', 'w', 'GridColor', [0.90, 0.91, 0.92], 'GridAlpha', 1.0);

if show_prev && ~isempty(prev_sig)
    legend(ax, {'Previous I', 'Previous Q', 'In-Phase (I)', 'Quadrature (Q)'}, ...
           'Location', 'northeast');
else
    legend(ax, {'In-Phase (I)', 'Quadrature (Q)'}, 'Location', 'northeast');
end

hold(ax, 'off');
export_figure_png_svg(f, out_dir, base_name);
close(f);
end
