# Final controller: FxLMS V1.4R

| File | Use |
|---|---|
| `fxlms_v1_4r_auto60/` | **Final validated controller.** Use this one. |
| `fxlms_v1_4r_auto60_cx2/` | Experimental variant with a relaxed cross-axis (vector) guard. Not the main result firmware. |

## What V1.4R does

1. **Baseline (tool ON):** measures the tool-induced vibration at the hand sensor with the actuator off (retries rejected blocks, up to 10 attempts for 5 good windows).
2. **Secondary-path identification (tool OFF):** automated in-situ probing of the actuator → hand-sensor path to estimate its complex gain Ŝ before adaptation starts.
3. **Adaptive control:** complex normalized FxLMS updates a single complex weight W at the tool frequency, using the live tool-side reference. A STARTUP / LEARN / FINE / HOLD state machine controls adaptation; the best-performing weight (Wbest) is retained.
4. **Frozen validation:** W is frozen and attenuation is measured on new data for 10 s.
5. **AUTO60:** if the selected axis reaches ≥ 20 % attenuation, the same frozen Wbest runs automatically for another 60 s to test persistence.
6. **Optional OFF–ON–OFF–ON test:** actuator switched off and on repeatedly to verify that the reduction is produced by the controller.

## Safety guards

DAC ceiling derived from baseline and Ŝ, saturation handling, emergency stop if the selected axis grows above 150 % of baseline, cross-axis vector guard (200 % for three consecutive blocks in the CX2 variant), absolute software limits.

## Version history (short)

- **V1.2:** secondary-path identification + core complex normalized FxLMS
- **V1.3:** supervisory state machine, Wbest retention, frozen-W validation, A-B-A-B test
- **V1.4:** Wbest qualification based on the manually selected axis only (single-axis characterization)
- **V1.4R AUTO60:** baseline retry logic + automatic 60 s persistence stage

The full description is in the header comment of each `.ino` file.
