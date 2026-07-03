% Load initial parameters

init_vx = 0;
init_vy = 0;
init_r = 0;
Mode = 1;
Mode_ggv = Simulink.Variant('Mode == 0');
Mode_simulink = Simulink.Variant('Mode == 1');
Mode_timeseries = Simulink.Variant('Mode == 2');
Mode_curvature_vs_steer = Simulink.Variant("Mode == 3");
Mode_sideslip_vs_curvature = Simulink.Variant("Mode == 4");
Mode_sideslip_vs_curvature_cst_accln = Simulink.Variant("Mode == 5");
Mode_steerdiff_vs_ay = Simulink.Variant("Mode == 6");
% TireData = struct();
V_ref = 45;


