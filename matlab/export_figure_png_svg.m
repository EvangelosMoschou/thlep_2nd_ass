function export_figure_png_svg(fig_handle, out_dir, base_name)
% export_figure_png_svg — Save figure as PNG + SVG vector.
png_path = fullfile(out_dir, [base_name '.png']);
svg_path = fullfile(out_dir, [base_name '.svg']);

saveas(fig_handle, png_path);
try
    exportgraphics(fig_handle, svg_path, 'ContentType', 'vector');
catch
    print(fig_handle, svg_path, '-dsvg');
end

disp(['Saved figure to ' strrep(png_path, filesep, '/') ...
      ' and ' strrep(svg_path, filesep, '/')]);
end
