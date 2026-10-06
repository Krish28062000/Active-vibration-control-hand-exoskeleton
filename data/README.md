# Experimental data guide

All datasets are Excel workbooks evaluated for the thesis. Results, statistics and exclusion criteria are reported in the thesis (`docs/thesis/`).

| Folder | File | Experiment | Purpose |
|---|---|---|---|
| 01_sensor_validation | [static_gravity_validation.xlsx](01_sensor_validation/static_gravity_validation.xlsx) | Static gravity test | Sign, scale, bias and repeatability of both ADXL345 sensors in six orientations |
| 01_sensor_validation | [stationary_noise_floor.xlsx](01_sensor_validation/stationary_noise_floor.xlsx) | Stationary noise floor | Sensor noise with DATA_READY-synchronized sampling |
| 01_sensor_validation | [tool_hand_frequency_amplitude_9_runs.xlsx](01_sensor_validation/tool_hand_frequency_amplitude_9_runs.xlsx) | Tool/hand measurement | Dominant frequency and amplitude at tool and hand, 9 runs |
| 02_actuator_characterization | [visaton_ex45s_characterization.xlsx](02_actuator_characterization/visaton_ex45s_characterization.xlsx) | Actuator characterization | DAC command vs. measured acceleration of the exciter |
| 02_actuator_characterization | [visaton_ex45s_frequency_control_map.xlsx](02_actuator_characterization/visaton_ex45s_frequency_control_map.xlsx) | Frequency control map | Actuator response across frequency |
| 02_actuator_characterization | [visaton_ex45s_inclined_frequency_map.xlsx](02_actuator_characterization/visaton_ex45s_inclined_frequency_map.xlsx) | Inclined frequency map | Actuator response in inclined orientation |
| 02_actuator_characterization | [visaton_3axis_characterization.xlsx](02_actuator_characterization/visaton_3axis_characterization.xlsx) | Triaxial characterization | X/Y/Z response to a single exciter |
| 02_actuator_characterization | [m3_dataset.xlsx](02_actuator_characterization/m3_dataset.xlsx) | Measurement series M3 | Characterization dataset |
| 02_actuator_characterization | [m6_m8_broadband_dataset.xlsx](02_actuator_characterization/m6_m8_broadband_dataset.xlsx) | Measurement series M6–M8 | Broadband characterization |
| 03_amplitude_matching | [amplitude_matcher_v5_1r_runs.xlsx](03_amplitude_matching/amplitude_matcher_v5_1r_runs.xlsx) | Amplitude matcher V5.1R | Matching runs of the closed-loop matcher |
| 03_amplitude_matching | [amplitude_matching_validation.xlsx](03_amplitude_matching/amplitude_matching_validation.xlsx) | Amplitude matching validation | Repeatability of matching |
| 03_amplitude_matching | [orientation_vs_axis_control.xlsx](03_amplitude_matching/orientation_vs_axis_control.xlsx) | Orientation vs. axis control | Effect of orientation on controlled-axis response |
| 04_phase_cancellation | [multiaxis_phase_test.xlsx](04_phase_cancellation/multiaxis_phase_test.xlsx) | Phase sweep | Multi-axis response to phase cancellation |
| 05_fxlms | [fxlms_v1_4r_x_runs.xlsx](05_fxlms/fxlms_v1_4r_x_runs.xlsx) | FxLMS V1.4R, X-axis | Final frozen-controller validation and 60 s persistence (57.15 ± 5.26 %) |
| 05_fxlms | [fxlms_v1_4_xy_runs.xlsx](05_fxlms/fxlms_v1_4_xy_runs.xlsx) | FxLMS V1.4, X/Y | Earlier X- and Y-axis campaign |
| 05_fxlms | [fxlms_y_axis_analysis.xlsx](05_fxlms/fxlms_y_axis_analysis.xlsx) | FxLMS, Y-axis | Y-axis analysis (5 retained runs, 55.13 ± 3.74 %) |
| 05_fxlms | [fxlms_y_axis_second_by_second.xlsx](05_fxlms/fxlms_y_axis_second_by_second.xlsx) | FxLMS, Y-axis | Second-by-second time series |
| 05_fxlms | [abab_causal_validation.xlsx](05_fxlms/abab_causal_validation.xlsx) | OFF–ON–OFF–ON | Controller ON/OFF validation |
