# PrimaLab

Browser-based mocap viewer and analysis tool by Prima Motion. Opens C3D files locally (nothing is uploaded), shows markers in 3D, computes joint angles (Cardan Z-X-Y) for lower-body markers, provides initial anthropometry estimation (height, weight, sex) for knee/ankle widths and CMJ body mass, and runs a force-plate jump analysis (CMJ) ported from [Force-Analysis](https://github.com/RatTongiam/Force-Analysis). Thai / English UI.

[![tests](https://github.com/chawinty08-debug/Primalab---3D/actions/workflows/test.yml/badge.svg)](https://github.com/chawinty08-debug/Primalab---3D/actions/workflows/test.yml)

**Live app (GitHub Pages):** https://chawinty08-debug.github.io/Primalab---3D/ — the root `index.html` forwards to the built single-file app `dist/index.html` (Pages: branch `main`, folder `/ (root)`).

## Features
- **Local C3D parsing**: points, analog channels, events, force plates **FP_TYPE 1, 2, 3 (Kistler 8-channel, with FPCOPPOLY), 4**; processor formats Intel (84), DEC/VAX (85) and SGI/MIPS (86); integer or float storage (ANALOG:SCALE/OFFSET applied to both, as ezc3d/BTK). Plate frame = orthonormalised corners; ORIGIN sign convention as ezc3d; FP_ZERO baseline only when the plate is unloaded in that window; plates with implausible CORNERS are skipped.
- **Vertical axis**: files recorded Y-up or X-up are detected and rotated rigidly to Z-up at load. Evidence in order: force-plate normal, marker-name cues (head/trunk above pelvis above feet), POINT:Y_SCREEN, body extent. Conflicts are reported; the file card has a manual override.
- **Marker-name mapping**: model marker sets are a config table (`MODEL_DEFS`: lower 16, lower 18, upper body ISB) with an alias table of common alternatives (ASIS/L_ASIS/ASIS_L/Left ASIS, LFE/MFE, LMAL/MMAL, MT2/MH2, CAL/HEEL, ACR, IJ/PX, …). The mapping card reports what each model finds, suggests file markers for the missing names, and stores the mapping in the browser (JSON import/export) for every file opened afterwards. The segment geometry itself stays in code; the mapping only renames.
- **3D Viewer**: Canvas2D orbit camera, marker trails, skeleton links, segment shapes, and ground reaction force (GRF) vectors.
- **Anthropometry & Subject Estimation**: Enter height (cm), body mass (kg), and sex to estimate initial knee/ankle widths (used for virtual medial markers and joint centers) and body mass (for CMJ force analysis). Also displays reference segment lengths (Winter 2009) and segment masses (De Leva 1996).
- **Kinematics**: Lower-body Cardan Z-X-Y joint angles (Hip, Knee, Ankle, Pelvis).
- **Marker sets**: Conventional Lower 16 (TIB wand) or Lower 18 (real medial KNEM/ANKM, no TIB; robust shank fit with outlier-marker rejection, virtual TIB for the shank frame). Separate static C3D supported.
- **Plug-in Gait style mode** (Lower 18 only): X anterior / Y left / Z up segment axes, Euler Y-X-Z, flexion/dorsiflexion, adduction/varus and internal rotation positive, static foot offset subtracted. Not yet compared with Vicon Plug-in Gait output.
- **6DOF tracking sets**: *Anatomical* (default) tracks every segment only with points that move with it, chaining joint centres from the parent pose (thigh HJC + THI + KNE [+ KNEM]; shank KJC + ANK [+ ANKM] or TIB; foot AJC + HEE + TOE). *Mixed* (v0.2 behaviour, bit-identical) also tracks the shank with the femoral epicondyles and the foot with the malleoli. On the badminton trials, anatomical tracking brings 6DOF within 0.7–2 ° RMS of IK for knee/ankle rotation (mixed: 9–24 °).
- **Marker QC**: rigid-body check of marker pairs on the same bone against the static (bias = median Δ, p95 |Δ|, max), lateral/medial knee–ankle consistency, gaps and jumps; tolerances allow thigh/pelvis soft-tissue artefact (Leardini et al. 2005). Names suspect markers with the reason, lists flagged pairs (click → QC chart of Δ with limits, jump to the worst frame). Validated by fault injection (`node tests/qc_validation.js`).
- **Upper body (ISB, Wu et al. 2005)**: Plug-in Gait upper-body markers + medial epicondyles (C7 T10 CLAV STRN, head, SHO UPA ELB ELBM FRM WRA WRB, FIN optional). Shoulder joint centre after Rab et al. (2002); humerus tracked by UPA ELB ELBM; forearm built directly each frame from the elbow centre and both styloids (the forearm is not rigid); hand from the wrist centre and FIN. Angles: thorax vs lab, trunk (thorax vs pelvis), neck (head offset from the static, as Plug-in Gait), shoulder humerothoracic Y-X-Y (plane of elevation, elevation, axial rotation), elbow Z-X-Y (flexion, varus, pronation), wrist (flexion, ulnar deviation). Pronation-plausibility check flags WRA/WRB label swaps; per-side swap option. Sign conventions verified in `node tests/ub_validation.js`. Not yet cross-checked against other software.
- Lower body: the medial-marker set is preferred when both TIB and KNEM/ANKM exist, and in anatomical tracking a segment with ≥ 3 of its points still tracks (e.g. a pelvis marker lost for most of a trial).
- **Inverse Kinematics (global optimisation)**: Lu & O'Connor (1999) type multibody fit of pelvis → thigh → shank → foot (both legs) with coincident joint centres (no joint dislocation). Levenberg–Marquardt per frame, warm-started, with multi-start guard against local minima. Models: ball joints (hip 3 / knee 3 / ankle 3 DOF) or Lu & O'Connor (hip 3 / knee 1 hinge / ankle 2 universal). Geometry comes from the same static calibration as the joint angles; angles are reported in the selected convention (Cardan Z-X-Y or PiG-style Y-X-Z). Reports marker residuals per frame and per marker. Not a numerical replica of OpenSim IK (subject-specific static geometry instead of a scaled generic model).
- **Inverse Dynamics (Newton–Euler, bottom-up)**: foot → shank → thigh per leg from either kinematic source (6DOF or IK).
  - BSIP: De Leva (1996) mass, CoM and radii of gyration by sex; segment ends from the static calibration (thigh HJC–KJC, shank KJC–AJC, foot HEE–TOE).
  - Filtering: 2nd-order Butterworth dual pass with cutoff correction (Winter 2009) on segment pose; GRF and plate wrench filtered at the same cutoff by default (Kristianslund et al. 2012). Fz threshold.
  - Force plates: free moment (Tz) now read from the C3D; each plate is assigned to the foot standing on it. Both feet on one plate cannot be split and those frames get no result.
  - Output: internal net joint moment (resolved in the proximal segment (default) or distal), net intersegmental joint force, joint power $P = \mathbf{M}\cdot(\boldsymbol{\omega}_d-\boldsymbol{\omega}_p)$, per-foot GRF; optional normalisation to body mass. Moment components follow the angle sign rules (positive = acts to increase the positive angle). Kinetics chart tab and CSV export.
- **Total support moment** (Winter 1980): per leg, $M_s = M_{hip,ext} + M_{knee,ext} + M_{ankle,PF}$ (sagittal, extensor positive) with each joint's contribution plotted and its share (%) at the Ms peak; shown during stance (ground contact on that foot) only; chart (Kinetics → Support moment), ID note and CSV columns. Sign mapping verified on real data: Ms from the Cardan Z-X-Y and PiG-style conventions agree within ~1 % RMS (residual = their different segment axes).
- **Force-plate selection**: a Force plates card lists every plate with its peak Fz and loaded time; untick plates or press "Exclude unused plates" (Fz > 50 N for < 50 ms in total). Excluded plates are left out of the GRF chart, CMJ analysis, inverse dynamics and the 3D view. The GRF tab has a plate selector (defaults to the most loaded plate).
- **Musculoskeletal model overlay**: bones and muscle paths of OpenSim **Gait 2392** (pelvis, legs, torso, 92 muscles) and **Arm 26** (humerus, ulna, radius, hand, scapula, clavicle; mirrored for the left) are placed on the subject from the static and follow the tracked segments (6DOF or IK, plus the upper body). Placement per segment: similarity fit (uniform scale from segment lengths, rotation from landmarks with the joint centres weighted so bones meet at the joints); motion: bone(f) = segment(f)·segment(static)⁻¹·placement, independent of the axis convention. Bones are drawn with WebGL; muscles as lines of action coloured by length change vs the static (shorter blue / longer orange). The **MTU** chart tab plots muscle–tendon length per muscle (conditional and moving path points driven by the knee/hip flexion angles). Visual and kinematic only: no muscle forces, no wrapping surfaces (Gait 2392 uses via points), no non-uniform scaling. Geometry © OpenSim models, CC BY 3.0 (Delp et al. 1990; Holzbaur et al. 2005). Validated in `tests/msk_validation.js` (exact recovery of a scaled, moved model; muscle-length signs vs knee/hip/ankle angles on the synthetic CMJ).
- **Event detection**: foot contact from the force plates without inverse dynamics (foot side from the COP vs HEE/TOE/ANK), gait events from markers after Zeni et al. (2008) (heel strike = max heel–sacrum, toe off = min toe–sacrum along the progression direction from the pelvis velocity, so either walking direction works), C3D EVENT (Foot Strike/Foot Off), ID contact, or peaks/valleys of any series in the top chart (min distance, prominence) for movements that are not gait. Detected events are drawn on the charts.
- **Batch processing**: run many C3D files with the current settings (static, angle settings, anthropometry, ID, mapping, vertical axis) and one cycle rule; exports a per-cycle summary in long format (file, cycle, variable, mean, min, max, range, time of min/max %, start/end value, valid %) and the 0–100 % curves of every cycle.
- **Movement cycles + combined export**: cut cycles by frame (“Start/End = this frame”, or type frames; or automatically from any of the event sources above), shown as shaded C1, C2 … bands on the charts. Export kinematics (6 joints × 3 angles, selected 6DOF/IK source) and/or kinetics (moments, power, joint forces, GRF, support moment) either per time (every frame with a cycle column, or only frames inside cycles) or per cycle: each cycle linearly time-normalised to 0–100 % (101 points, gaps not bridged) with per-cycle columns plus mean and sample SD.
- **Chart CSV export**: the CSV button above the chart saves exactly what is plotted — panel A, plus panel B in compare mode — with columns like `A · R Knee angle · X flex(+)/ext (°)`. Panels at different sampling rates (e.g. force analysis at the analog rate) get their own time columns.
- **Chart read-out**: hover any time-series chart (touch: drag) to see time, frame and the value of every series at that point.
- **Force Analysis (CMJ)**: Automated quiet-standing window, phase segmentation (unweighting, braking, propulsive, flight, landing), metrics, and landing impact frequency check against plate natural frequency ($f_n$).
- **Bilingual**: Thai (TH) and English (EN) with seamless language switching.

## Files
| File | Role |
| --- | --- |
| `template.html` | page markup and CSS |
| `engine.js` | C3D reader, 3D viewer, charts, kinematics, force-plate engine |
| `app.js` | UI wiring |
| `i18n.js` | TH/EN phrase tables and switcher |
| `msk_data.js` | generated bone meshes, landmarks and muscle paths (`tools/make_msk.py` from an opensim-models checkout; needs numpy + fast-simplification) |
| `build.py` | concatenates the above |
| `tests/` | Node validation suites (`npm test` or `node tests/run_all.js`); `c3d_writer.js` writes synthetic C3D files for the reader tests; `fixtures/ezc3d_ref.json` holds ezc3d reference values; `tools/make_ezc3d_ref.py` regenerates them |
| `.github/workflows/test.yml` | CI: fetches the public reference files and runs every suite on each push |

## Build
```
python build.py
```

## Tests
```
npm test                      # or: node tests/run_all.js
```
| Suite | What it checks |
| --- | --- |
| `ref_validation.js` | C3D reader and force plates against **ezc3d** (independent C++ reader) on the public [ezc3d test files](https://github.com/pyomeca/ezc3d-testFiles) (Vicon, Qualisys, Cortex, BTS, Optotrak, Kistler type 3, type 1 …): markers, analog, plate force, COP and free moment agree to ~1e-6 (one global force sign per plate, since the app reports the reaction on the body). Needs `C3D_REF_DIR`; CI clones the files. |
| `processor_samples.js` | c3d.org Sample01 (same trial as Intel / DEC / MIPS, integer / float) must agree. Needs `C3D_SAMPLE_DIR`; CI downloads it when c3d.org is reachable. |
| `plate_types.js` | synthetic C3D with a known load for plate types 1–4 × processors 84/85/86 × Z-up/Y-up: force, COP, free moment and markers recovered; vertical-axis detection and override |
| `mapping_validation.js` | a file with lab-specific names gives identical joint angles after mapping |
| `events_validation.js` | Zeni events on synthetic walking (both directions, 100/200 Hz), plate contact events, peak detection, cycle summary |
| `ik_validation.js`, `id_validation.js` | IK angle recovery, ID against an independent momentum solution |
| `qc_validation.js`, `ub_validation.js` | marker QC fault injection, upper-body sign conventions |
Writes `mocap-viewer.html` (fragment) and `dist/index.html` (standalone page, committed so it can be uploaded as-is).

## Deploy
Upload `dist/index.html` to `public_html/primalab/index.html` (currently https://app.primamotion.tech/primalab/).

## Status
- Joint angles are not yet cross-checked against other software.
- Knee/ankle widths are rough empirical placeholders; direct caliper measurement is strongly recommended.
- Two-plate (L/R) force analysis is validated on simulated data only.
- IK and ID are validated on synthetic data only, not yet cross-checked against other software:
  - IK recovers known joint angles exactly from model-consistent markers; RMSE 0.2–0.5° with 0.5 mm marker noise (both marker sets, both conventions, both joint models).
  - ID matches an independent whole-leg momentum solution (prescribed hip wrench, 3D motion, plate F/COP/Tz) within 0.1 % of peak, with and without filtering. Moment signs checked with static postures.
- The C3D reader and plate maths agree with ezc3d on every public test file with markers; on the lab's own badminton/table-tennis files too (markers, analog, plate force, COP, free moment). Versions before this one built the plate frame from the raw (slightly non-orthogonal) digitised corners: plate force differed by up to ~1 % and COP by up to ~4 mm on those files.
- Zeni gait events are tested on synthetic walking only (≤ 50 ms vs the true contact with a slowly decelerating synthetic swing; Zeni et al. report ~16 ms mean error vs force plates in real gait).
- The bundled synthetic sample now has two plates (one per foot) so ID can be demonstrated. Its GRF is derived from hip height only, so its joint moments are not physiological.
