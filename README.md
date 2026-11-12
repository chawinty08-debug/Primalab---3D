# PrimaLab

Browser-based mocap viewer and analysis tool by Prima Motion. Opens C3D files locally (nothing is uploaded), shows markers in 3D, computes V3D-style joint angles (Cardan Z-X-Y) for lower-body markers, provides initial anthropometry estimation (height, weight, sex) for knee/ankle widths and CMJ body mass, runs joint-constrained inverse kinematics and Newton–Euler inverse dynamics (joint moments, forces, power), and runs a force-plate jump analysis (CMJ) ported from [Force-Analysis](https://github.com/RatTongiam/Force-Analysis). Thai / English UI.

## Features
- **Local C3D parsing**: Points, analog channels, events, force plates (Type 2/4).
- **3D Viewer**: Canvas2D orbit camera, marker trails, skeleton links, segment shapes, and ground reaction force (GRF) vectors.
- **Anthropometry & Subject Estimation**: Enter height (cm), body mass (kg), and sex to estimate initial knee/ankle widths (used for virtual medial markers and joint centers) and body mass (for CMJ force analysis). Also displays reference segment lengths (Winter 2009) and segment masses (De Leva 1996).
- **Kinematics**: Visual3D-style lower-body Cardan Z-X-Y joint angles (Hip, Knee, Ankle, Pelvis).
- **Marker sets**: Conventional Lower 16 (TIB wand) or Lower 18 (real medial KNEM/ANKM, no TIB; robust shank fit with outlier-marker rejection, virtual TIB for the shank frame). Separate static C3D supported.
- **Plug-in Gait style mode** (Lower 18 only): X anterior / Y left / Z up segment axes, Euler Y-X-Z, flexion/dorsiflexion, adduction/varus and internal rotation positive, static foot offset subtracted. Not yet compared with Vicon PiG or Visual3D.
- **6DOF tracking sets**: *Anatomical* (default) tracks every segment only with points that move with it, chaining joint centres from the parent pose (thigh HJC + THI + KNE [+ KNEM]; shank KJC + ANK [+ ANKM] or TIB; foot AJC + HEE + TOE). *Mixed* (v0.2 behaviour, bit-identical) also tracks the shank with the femoral epicondyles and the foot with the malleoli. On the badminton trials, anatomical tracking brings 6DOF within 0.7–2 ° RMS of IK for knee/ankle rotation (mixed: 9–24 °).
- **Marker QC**: rigid-body check of marker pairs on the same bone against the static (bias = median Δ, p95 |Δ|, max), lateral/medial knee–ankle consistency, gaps and jumps; tolerances allow thigh/pelvis soft-tissue artefact (Leardini et al. 2005). Names suspect markers with the reason, lists flagged pairs (click → QC chart of Δ with limits, jump to the worst frame). Validated by fault injection (`node tests/qc_validation.js`).
- **Inverse Kinematics (global optimisation)**: Lu & O'Connor (1999) type multibody fit of pelvis → thigh → shank → foot (both legs) with coincident joint centres (no joint dislocation). Levenberg–Marquardt per frame, warm-started, with multi-start guard against local minima. Models: ball joints (hip 3 / knee 3 / ankle 3 DOF) or Lu & O'Connor (hip 3 / knee 1 hinge / ankle 2 universal). Geometry comes from the same static calibration as the joint angles; angles are reported in the selected convention (V3D Z-X-Y or PiG-style Y-X-Z). Reports marker residuals per frame and per marker. Not a numerical replica of OpenSim IK (subject-specific static geometry instead of a scaled generic model).
- **Inverse Dynamics (Newton–Euler, bottom-up)**: foot → shank → thigh per leg from either kinematic source (6DOF or IK).
  - BSIP: De Leva (1996) mass, CoM and radii of gyration by sex; segment ends from the static calibration (thigh HJC–KJC, shank KJC–AJC, foot HEE–TOE).
  - Filtering: 2nd-order Butterworth dual pass with cutoff correction (Winter 2009) on segment pose; GRF and plate wrench filtered at the same cutoff by default (Kristianslund et al. 2012). Fz threshold.
  - Force plates: free moment (Tz) now read from the C3D; each plate is assigned to the foot standing on it. Both feet on one plate cannot be split and those frames get no result.
  - Output: internal net joint moment (resolved in the proximal segment, V3D default, or distal), net intersegmental joint force, joint power $P = \mathbf{M}\cdot(\boldsymbol{\omega}_d-\boldsymbol{\omega}_p)$, per-foot GRF; optional normalisation to body mass. Moment components follow the angle sign rules (positive = acts to increase the positive angle). Kinetics chart tab and CSV export.
- **Total support moment** (Winter 1980): per leg, $M_s = M_{hip,ext} + M_{knee,ext} + M_{ankle,PF}$ (sagittal, extensor positive) with each joint's contribution plotted and its share (%) at the Ms peak; shown during stance (ground contact on that foot) only; chart (Kinetics → Support moment), ID note and CSV columns. Sign mapping verified on real data: Ms from the V3D and PiG-style conventions agree within ~1 % RMS (residual = their different segment axes).
- **Force-plate selection**: a Force plates card lists every plate with its peak Fz and loaded time; untick plates or press "Exclude unused plates" (Fz > 50 N for < 50 ms in total). Excluded plates are left out of the GRF chart, CMJ analysis, inverse dynamics and the 3D view. The GRF tab has a plate selector (defaults to the most loaded plate).
- **Movement cycles + combined export**: cut cycles by frame (“Start/End = this frame”, or type frames; optional auto-suggestion from foot contact: stance or strike → strike), shown as shaded C1, C2 … bands on the charts. Export kinematics (6 joints × 3 angles, selected 6DOF/IK source) and/or kinetics (moments, power, joint forces, GRF, support moment) either per time (every frame with a cycle column, or only frames inside cycles) or per cycle: each cycle linearly time-normalised to 0–100 % (101 points, gaps not bridged) with per-cycle columns plus mean and sample SD.
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
| `build.py` | concatenates the above |
| `tests/` | Node validation scripts for IK, ID and marker QC (`node tests/ik_validation.js`, `node tests/id_validation.js`, `node tests/qc_validation.js`) |

## Build
```
python build.py
```
Writes `mocap-viewer.html` (fragment) and `dist/index.html` (standalone page, committed so it can be uploaded as-is).

## Deploy
Upload `dist/index.html` to `public_html/primalab/index.html` (currently https://app.primamotion.tech/primalab/).

## Status
- Joint angles are not yet compared against Visual3D output.
- Knee/ankle widths are rough empirical placeholders; direct caliper measurement is strongly recommended.
- Two-plate (L/R) force analysis is validated on simulated data only.
- IK and ID are validated on synthetic data only, not yet against OpenSim or Visual3D:
  - IK recovers known joint angles exactly from model-consistent markers; RMSE 0.2–0.5° with 0.5 mm marker noise (both marker sets, both conventions, both joint models).
  - ID matches an independent whole-leg momentum solution (prescribed hip wrench, 3D motion, plate F/COP/Tz) within 0.1 % of peak, with and without filtering. Moment signs checked with static postures.
- The bundled synthetic sample now has two plates (one per foot) so ID can be demonstrated. Its GRF is derived from hip height only, so its joint moments are not physiological.
