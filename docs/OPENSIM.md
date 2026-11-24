# PrimaLab → OpenSim

The **Export** card has an *OpenSim (Gait 2392)* section. It writes one zip:

| File | Content |
| --- | --- |
| `<trial>.trc`, `<trial>_static.trc` | markers in mm, OpenSim ground (Y up). The lab is rotated so the static pelvis faces +X (or only Z-up → Y-up). Missing samples = `NaN`. Optional extra markers `RHJC LHJC RKJC LKJC RAJC LAJC` = PrimaLab joint centres. |
| `<trial>_grf.mot`, `<trial>_grf.xml` | ground reaction per foot at the analog rate: `ground_force_*` right foot → `calcn_r`, `1_ground_force_*` left foot → `calcn_l`, point and free torque in ground. Each plate sample goes to the foot over that plate (same rule as PrimaLab ID; both feet on one plate → nearer foot to the COP, counted in the notes). Several plates under one foot are combined into one force, one COP on the plate plane and one free moment (moment about the origin preserved). Optional 2nd-order Butterworth (dual pass) on force and moment before the COP is formed. |
| `PrimaLab_markers_gait2392.xml` | marker set with PrimaLab / Plug-in Gait names on the Gait 2392 bodies (starting positions; Scale moves them onto the static). Joint-centre markers are fixed. |
| `setup_scale.xml` | measurements: pelvis (RASI–LASI, RHJC–LHJC), thigh (HJC–KJC), shank (KJC–AJC), foot (HEE–TOE), torso (RSHO–LSHO, CLAV–STRN, C7–SACR when present); marker placement on the static. |
| `setup_ik.xml` | marker weights: pelvis 10, epicondyles / malleoli / foot 5, clusters 1, acromion 0.5; head markers unused (no neck in Gait 2392); subtalar and MTP held at 0 (PiG feet carry only HEE + TOE, so rotation about the heel–toe line is not observable). |
| `setup_id.xml` | coordinates low-pass at the ID cutoff of the IK & ID card (default 6 Hz), muscles excluded. |
| `gait2392_thelen2003muscle.osim` | OpenSim Gait 2392 (Delp et al. 1990; Thelen 2003), CC BY 3.0, opensim-org/opensim-models. |
| `run_opensim.py` | Scale → IK → ID in one go with the `opensim` Python package. |

OpenSim GUI: *Tools → Scale Model → Load* `setup_scale.xml` → Run; then *Inverse Kinematics* (`setup_ik.xml`) and *Inverse Dynamics* (`setup_id.xml`).
Python: `pip install opensim`, then `python run_opensim.py` in the unzipped folder.

## Check with OpenSim 4.6

`tests/opensim_roundtrip.py` (also in CI) exports the synthetic CMJ, runs OpenSim Scale → IK → ID and compares with PrimaLab
(6DOF Cardan Z-X-Y angles, PrimaLab inverse dynamics, both filtered at 6 Hz; the angle offset comes from the different
segment-axis definitions, e.g. the PrimaLab foot axis runs AJC → TOE):

| | r | RMSD (offset removed) |
| --- | --- | --- |
| hip / knee flexion | 0.999–1.000 | 1.9–2.2° |
| ankle | 0.992–0.993 | 1.9° |
| hip / knee / ankle sagittal moment | 0.95–0.98 | 3.5–17.6 N·m (peaks 62–346 N·m) |

On three lab trials (two badminton lunges, one table-tennis rally; Plug-in Gait lower body + medial markers, plates at 1800 Hz)
the same comparison gave, for the six sagittal joint angles, r = 0.93–0.998 with 0.5–5 ° RMSD after the offset, and for the six
sagittal joint moments r = 0.92–1.00 with 1.4–17 N·m RMSD (peaks 49–354 N·m). OpenSim IK marker RMS was 17–21 mm on average:
Gait 2392 keeps the knee planar and the hip at a model-defined position, so skin-marker residuals are expected to be larger
than for the PrimaLab 6DOF fit.
