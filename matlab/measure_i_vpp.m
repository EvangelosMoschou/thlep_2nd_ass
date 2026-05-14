function vpp = measure_i_vpp(x)
% measure_i_vpp — Peak-to-peak voltage of I (real) component.
% Mirrors complex_real_vpp() in main.c.
if isempty(x)
    vpp = 0;
    return;
end
vpp = max(real(x)) - min(real(x));
end
