/* ---------- TH/EN switch: Thai is the source text; English via phrase tables applied to DOM text + canvas strings */
let LANG = 'th';
try { const s = localStorage.getItem('primalab.lang'); if (s === 'en' || s === 'th') LANG = s; } catch (e) { /* storage unavailable */ }
const I18N_EXACT = {
  'วน': 'Loop', 'ปิด': 'Off', 'ทั้งหมด': 'All', 'ไม่เลือก': 'None', 'ขนาด': 'Size', 'ไฟล์': 'File', 'ชื่อ': 'Name', 'ความถี่': 'Rate', 'เฟรม': 'Frames',
  'รวม': 'Total', 'ซ้าย': 'Left', 'ขวา': 'Right', 'เล่น': 'Play', 'ไม่แสดง': 'None', 'เมนู': 'Menu', '◧ เมนู': '◧ Menu', '◨ เมนู': '◨ Menu', 'ไม่พบ': 'Not found',
  'ความเร็ว': 'Speed', 'Fz รวม': 'Fz total', 'ความเร็ว COM': 'COM velocity', 'เลือก': 'Select',
  'ส่วนสูง (cm)': 'Height (cm)', 'น้ำหนัก (kg)': 'Weight (kg)', 'เพศ': 'Sex', 'ชาย': 'Male', 'หญิง': 'Female', 'ชาย (M)': 'Male (M)', 'หญิง (F)': 'Female (F)'
};
const I18N_FRAG = [
  // --- v0.8: gap filling, force-only files, CMJ sample
  ['Rigid: ใช้ marker ≥ 3 ตัวบน segment เดียวกัน (หรือที่ระยะห่างคงที่) พาตำแหน่งไป ถ้ามีข้อมูลก่อนและหลัง gap จะผสมสองฝั่งให้ต่อเนื่อง · Spline: เฉพาะ gap ภายในที่สั้นกว่าค่าที่ตั้ง · Pattern: เลียนการเคลื่อนที่ของ marker ข้างเคียง (เลื่อนตำแหน่งอย่างเดียว ไม่หมุน) ใช้กับ gap ยาวที่ rigid ทำไม่ได้ · ในหน้า 3D วงประเขียว = rigid, เหลือง = spline, ส้ม = pattern · การคำนวณทั้งหมด (มุมข้อต่อ, IK, ID, export) จะใช้ข้อมูลที่เติมแล้ว', 'Rigid: ≥ 3 markers on the same segment (or at constant distance) carry the missing one; with data on both sides the two estimates are blended so the path stays continuous · Spline: interior gaps shorter than the limit only · Pattern: copies the displacement of a neighbouring marker (translation only, no rotation), for long gaps where rigid is impossible · in 3D a dashed ring marks filled samples: green rigid, yellow spline, orange pattern · every computation (angles, IK, ID, export) uses the filled data'],
  ['CMJ สังเคราะห์จาก force plate 2 แผ่น ที่รู้ค่าคำตอบ (jump height, flight time, สัดส่วนซ้าย/ขวา)', 'Synthetic CMJ on two force plates with known answers (jump height, flight time, left/right share)'],
  ['ใช้ static เป็นตำแหน่งอ้างอิงเมื่อ marker หายเกือบทั้ง trial', 'Use the static as reference when a marker is missing for (almost) the whole trial'],
  [' (gap ที่ไม่มีข้อมูลอ้างอิงพอ)', ' (gaps without enough reference data)'], [' · คำนวณมุมข้อต่อใหม่เพื่อใช้ข้อมูลที่เติม', ' · recompute the joint angles to use the filled data'],
  ['อัตโนมัติ: rigid → spline → pattern', 'Auto: rigid → spline → pattern'], ['Rigid body (≥ 3 marker บน segment เดียวกัน)', 'Rigid body (≥ 3 markers on the same segment)'],
  ['Cubic spline (gap สั้น)', 'Cubic spline (short gaps)'], ['Pattern (ตาม marker ข้างเคียง)', 'Pattern (follows a neighbouring marker)'],
  ['เติม gap (gap filling)', 'Gap filling'], ['กลับเป็นข้อมูลเดิมแล้ว', 'Back to the original data'], ['กลับเป็นข้อมูลเดิม', 'Back to original'], ['เติม gap', 'Fill gaps'],
  ['Spline สูงสุด (s)', 'Spline max (s)'], ['ไม่มี gap ใน marker ที่มีชื่อ', 'no gaps in labelled markers'], ['เติมแล้ว ', 'Filled '], [' · เหลือ ', ' · left '],
  ['ค่าคำตอบของตัวอย่าง: ', 'Known answers of the sample: '], ['โหลดตัวอย่าง CMJ', 'Load CMJ sample'], ['ไฟล์นี้ไม่มีทั้ง marker (POINT:USED) และ analog', 'this file has neither markers (POINT:USED) nor analog data'],
  ['มวล ', 'mass '], ['ซ้าย ', 'left '], ['ขวา ', 'right '], ['เหลือ', 'left'], ['ขาด', 'missing'],
  // --- v0.7: CMJ workspace tab
  ['ไฟล์นี้ไม่มี force plate ที่อ่านได้ · เปิดไฟล์ C3D ที่มี force plate (CMJ, squat jump, drop jump …) แล้วกลับมาที่แถบนี้', 'This file has no readable force plate · open a C3D file with force plates (CMJ, squat jump, drop jump …) and come back to this tab'],
  ['เปิดแถบ CMJ analyzer (force plate)', 'Open the CMJ analyzer tab (force plates)'], ['ปิดแถบ CMJ', 'Close the CMJ tab'], ['พื้นที่ทำงาน', 'Workspace'],
  // --- v0.6: OpenSim export
  ['.trc (marker + static), GRF .mot + ExternalLoads ต่อเท้า, marker set บน Gait 2392, setup Scale / IK / ID, ไฟล์ model และ run_opensim.py · เปิดใน OpenSim GUI: Tools → Scale Model → Load setup_scale.xml แล้ว IK และ ID ต่อ · ใช้มุมข้อต่อส่วนล่างที่คำนวณแล้ว (สำหรับ joint centre, ทิศหน้า และการแบ่งแรงให้เท้า) และน้ำหนักตัวจากการ์ดผู้ทดสอบ', '.trc (trial + static), per-foot GRF .mot + ExternalLoads, marker set on Gait 2392, Scale / IK / ID setup files, the model and run_opensim.py · in the OpenSim GUI: Tools → Scale Model → Load setup_scale.xml, then IK and ID · uses the computed lower-body kinematics (joint centres, heading, foot assignment of the plates) and the body mass from the subject card'],
  ['ยังไม่ได้คำนวณมุมข้อต่อ: ไม่มี joint centre และแบ่งแรงให้เท้าจากช่วงสัมผัสแทน', 'joint angles not computed: no joint centres, plates assigned to feet per contact instead'],
  ['หมุนให้ pelvis หันไป +X ของ OpenSim', 'rotate so the pelvis faces OpenSim +X'], ['คงทิศของ lab (เปลี่ยนแค่ Z-up → Y-up)', 'keep the lab heading (Z-up → Y-up only)'],
  ['ใส่ joint centre (RHJC … LAJC) จาก PrimaLab ใน .trc', 'include PrimaLab joint centres (RHJC … LAJC) in the .trc'], ['ส่งออกชุด OpenSim (.zip)', 'Export OpenSim bundle (.zip)'],
  ['กรอง GRF (Hz)', 'GRF low-pass (Hz)'], ['ทิศหน้า', 'Heading'], [' · ไม่ได้ใส่น้ำหนักตัว ใช้ 70 kg', ' · no body mass entered, 70 kg used'], ['ส่งออกไม่ได้: ', 'export failed: '],
  [' frame มีสองเท้าบน plate เดียวกัน: ให้แรงทั้งหมดกับเท้าที่ใกล้ COP (แยกไม่ได้)', ' frames with both feet on one plate: all force given to the foot nearer the COP (cannot be split)'],
  [' frame ไม่มีเท้าอยู่เหนือ plate: ให้เท้าที่ใกล้ COP ที่สุด', ' frames with no foot over the plate: nearest foot to the COP'],
  [' ช่วงสัมผัสไม่มีเท้าอยู่ใกล้ COP ชัดเจน: ให้เท้าที่ใกล้ที่สุด', ' contacts with no foot clearly near the COP: nearest foot'], ['ไม่มี force plate: ไม่มีไฟล์ GRF และ setup ID', 'no force plate: no GRF file or ID setup'],
  // --- v0.5: musculoskeletal overlay
  ['วาง model กระดูกและเส้นทางกล้ามเนื้อของ OpenSim Gait 2392 (+ แขนจาก Arm 26) ลงบนผู้ทดสอบจาก static: แต่ละ segment ปรับขนาดแบบ uniform ตามความยาว segment และหมุนให้ตรง landmark (joint centre, epicondyle, malleolus, ASIS …) แล้วเคลื่อนตาม segment ที่ track ได้ · ใช้ผลจากการ์ดมุมข้อต่อ (6DOF หรือ IK) และส่วนบน · สีกล้ามเนื้อ: ', 'Places the OpenSim Gait 2392 bones and muscle paths (+ arms from Arm 26) on the subject from the static: each segment is scaled uniformly by its length and rotated onto the landmarks (joint centres, epicondyles, malleoli, ASIS …), then moves with its tracked segment · uses the joint-angle card (6DOF or IK) and the upper body · muscle colour: '],
  [' · แท็บ MTU ในกราฟแสดงความยาว muscle–tendon unit', ' · the MTU chart tab shows muscle–tendon unit length'],
  ['Geometry และ muscle path: OpenSim Gait 2392 (Delp et al. 1990) และ Arm 26 (Holzbaur et al. 2005), opensim-org/opensim-models, CC BY 3.0 · เป็น model เชิงภาพและ kinematics (line of action, ความยาว MTU) ไม่ใช่ simulation ของ OpenSim: ไม่มีแรงกล้ามเนื้อ ไม่มี wrapping surface (Gait 2392 ใช้ via point)', 'Geometry and muscle paths: OpenSim Gait 2392 (Delp et al. 1990) and Arm 26 (Holzbaur et al. 2005), opensim-org/opensim-models, CC BY 3.0 · a visual and kinematic model (lines of action, MTU length), not an OpenSim simulation: no muscle forces, no wrapping surfaces (Gait 2392 uses via points)'],
  ['เปิด “แสดงกระดูกและกล้ามเนื้อ” ในการ์ด Musculoskeletal model ก่อน', 'turn on “Show bones and muscles” in the Musculoskeletal model card first'],
  ['ต้องคำนวณมุมข้อต่อส่วนล่าง (6DOF หรือ IK) ก่อน', 'compute the lower-body joint angles (6DOF or IK) first'],
  ['ไม่มีข้อมูล musculoskeletal model (msk_data.js)', 'musculoskeletal model data missing (msk_data.js)'],
  ['กล้ามเนื้อนี้ไม่มีข้อมูล (segment ไม่ครบ)', 'no data for this muscle (segments missing)'],
  [' · ความยาว muscle–tendon unit (m) · static ', ' · muscle–tendon unit length (m) · static '],
  [' (ไม่มีส่วนบน: ลำตัววางตาม pelvis)', ' (no upper body: torso placed on the pelvis)'], [' · เบราว์เซอร์นี้ไม่รองรับ WebGL: แสดงเฉพาะกล้ามเนื้อ', ' · this browser has no WebGL: muscles only'],
  [': landmark ของ model กับของผู้ทดสอบไม่เข้ากัน (RMS ', ': model and subject landmarks disagree (RMS '], [': landmark ไม่พอ (', ': not enough landmarks ('],
  ['แสดงกระดูกและกล้ามเนื้อ (ตามการ track)', 'Show bones and muscles (following the tracking)'], ['กล้ามเนื้อ (line of action)', 'Muscles (line of action)'], ['ความทึบกระดูก', 'Bone opacity'],
  ['แสดงกล้ามเนื้อ', 'Muscles shown'], ['เฉพาะที่เลือก', 'selected only'], ['ขาขวา', 'right leg'], ['ขาซ้าย', 'left leg'], ['ทั้งหมด', 'all'],
  [' ชิ้นกระดูก · ', ' bone meshes · '], [' กล้ามเนื้อ · จาก ', ' muscles · from '], [' + ส่วนบน', ' + upper body'], ['สั้นลง', 'shorter'], ['เท่า static', 'as static'], ['ยาวขึ้น', 'longer'],
  ['เลือกกล้ามเนื้อ', 'choose muscle'], ['กระดูก', 'Bones'],
  // --- v0.4: vertical axis, plate types, marker mapping, events, batch
  ['ใช้การตั้งค่าจากการ์ดด้านบนกับทุกไฟล์: static, ค่าตั้งมุมข้อต่อ, anthropometry, ID, mapping และแกนตั้ง · ผลแต่ละ cycle normalize 0–100% (101 จุด)', 'Uses the settings of the cards above for every file: static, joint-angle settings, anthropometry, ID, mapping and vertical axis · each cycle normalised to 0–100% (101 points)'],
  ['เลือก marker ในไฟล์ให้ตรงกับชื่อที่ model ใช้ (เดาให้จากชื่อที่พบบ่อย) · mapping ถูกจำไว้ในเบราว์เซอร์นี้และใช้กับทุกไฟล์ที่เปิดต่อจากนี้ · ตำแหน่ง/ความหมายของ marker ต้องตรงกับนิยามของ model จริง (เช่น KNE = lateral femoral epicondyle)', 'Pick the file marker that matches each model name (suggested from common names) · the mapping is remembered in this browser and applied to every file opened afterwards · the anatomical landmark must match the model definition (e.g. KNE = lateral femoral epicondyle)'],
  ['เลือก marker ในไฟล์ให้ตรงกับชื่อที่ model ใช้ (เดาให้จากชื่อที่พบบ่อย) · ตำแหน่งทางกายวิภาคต้องตรงกับนิยามของ model', 'Pick the file marker that matches each model name (suggested from common names) · the anatomical landmark must match the model definition'],
  ['ไฟล์นี้มี marker ครบตามชื่อของ model ที่เลือก', 'This file has every marker name the selected model needs'],
  ['ไม่มีตัวแปรที่คำนวณได้ (marker set ไม่ตรง model? ดูการ์ด mapping)', 'nothing could be computed (marker set does not match a model? see the mapping card)'],
  ['ไม่พบ cycle (ลองเลือก “ทั้ง trial = 1 cycle” หรือวิธีอื่น)', 'no cycle found (try “whole trial = 1 cycle” or another method)'],
  ['แถว (long format: 1 แถว = ไฟล์ × cycle × ตัวแปร)', 'rows (long format: 1 row = file × cycle × variable)'],
  ['กราฟบนยังไม่มีเส้นข้อมูล · เลือกแท็บ/ข้อต่อที่ต้องการก่อน', 'the top chart has no series yet · choose a tab/joint first'],
  ['ต้องมี marker SACR หรือ LPSI/RPSI (และ ASI) สำหรับหาทิศการเดิน', 'needs SACR or LPSI/RPSI (and ASI) markers for the walking direction'],
  ['ต้องมี marker HEE และ TOE อย่างน้อย 1 ข้าง', 'needs HEE and TOE markers on at least one side'],
  ['สร้าง cycle จาก contact ต้องคำนวณ Inverse Dynamics ก่อน', 'cycles from ID contact need Inverse Dynamics first'],
  ['เลือก Kinematics หรือ Kinetics อย่างน้อย 1 อย่าง', 'select Kinematics or Kinetics'],
  ['FORCE_PLATFORM:CORNERS ผิดปกติ (ค่าไม่สมเหตุสมผล) ข้าม plate นี้', 'FORCE_PLATFORM:CORNERS invalid (implausible values), plate skipped'],
  [' · ถ้ารูปคนนอน/กลับหัว ให้เลือกแกนตั้งเอง', ' · if the body looks lying down or upside down, choose the vertical axis manually'],
  [' · ไม่ลบ baseline FP_ZERO (ช่วงนั้นมีแรง ', ' · FP_ZERO baseline not subtracted (load in that window '],
  ['ไม่รู้จัก (รองรับ 84 Intel, 85 DEC, 86 MIPS)', 'unknown (84 Intel, 85 DEC, 86 MIPS supported)'], ['(รองรับ 1, 2, 3, 4)', '(1, 2, 3, 4 supported)'],
  ['ชื่อ marker → model (mapping)', 'Marker names → model (mapping)'], ['ชื่อใน model', 'Model name'], ['marker ในไฟล์', 'File marker'],
  ['ใช้ mapping อยู่ ', 'Mapping in use: '], ['ล้าง mapping', 'Clear mapping'], ['ใช้ mapping', 'Apply mapping'], ['นำเข้า mapping ไม่ได้: ', 'Could not import mapping: '], ['ส่งออก mapping ', 'Exported mapping: '],
  ['นำเข้า JSON', 'Import JSON'], ['ส่งออก JSON', 'Export JSON'], ['รูปแบบไฟล์ไม่ถูกต้อง', 'invalid file format'],
  ['หลักฐานแกนตั้งขัดกัน: ', 'Conflicting vertical-axis evidence: '], ['แกนตั้งของไฟล์เป็น ', 'File vertical axis is '], [') หมุนเป็น Z-up ตอนโหลด', ') rotated to Z-up at load'],
  ['แกนตั้ง (vertical)', 'Vertical axis'], [' → หมุนเป็น +Z', ' → rotated to +Z'], ['กำหนดเอง (ตรวจพบ ', 'manual (detected '], ['แกนตั้ง', 'Vertical axis'],
  ['ไม่มีหลักฐาน ใช้ค่าเริ่มต้น', 'no evidence, default'], ['ช่วงกว้างของ marker', 'marker extent'], ['อัตโนมัติ', 'Auto'],
  ['Force plate (ไม่ต้องคำนวณ ID)', 'Force plate (no ID needed)'], ['Gait จาก marker (Zeni 2008)', 'Gait from markers (Zeni 2008)'], ['Event ในไฟล์ C3D', 'C3D file events'],
  ['Contact จาก Inverse Dynamics', 'Contact from Inverse Dynamics'], ['Contact จาก ID', 'Contact from ID'], ['สัญญาณในกราฟบน (ทุกการเคลื่อนไหว)', 'Signal in the top chart (any movement)'],
  ['ระหว่าง peak → peak', 'peak → peak'], ['ระหว่าง valley → valley', 'valley → valley'], ['ห่างกันอย่างน้อย (s)', 'Min. distance (s)'], ['prominence (% ของช่วง)', 'Prominence (% of range)'], ['เส้นในกราฟ', 'Chart series'],
  ['ทุก contact (ไม่แยกเท้า)', 'every contact (either foot)'], ['ทุก contact', 'every contact'], ['ทั้ง trial = 1 cycle', 'whole trial = 1 cycle'], ['สร้างอัตโนมัติ', 'Automatic'],
  ['ไม่พบ event ที่ใช้สร้าง cycle ได้', 'no events usable for cycles'], ['ไฟล์นี้ไม่มี event', 'this file has no events'], ['ไม่มี force plate ที่อ่านได้', 'no readable force plate'],
  ['Batch · หลายไฟล์', 'Batch · many files'], ['เลือกไฟล์ C3D แล้วเริ่ม', 'Choose C3D files and run'], ['ส่งออกสรุปต่อ cycle', 'Export per-cycle summary'], ['ส่งออก curve 0–100%', 'Export 0–100% curves'],
  ['Kinematics (ขา + ส่วนบนถ้ามี marker)', 'Kinematics (legs + upper body when markers exist)'], ['Kinetics (ต้องมีน้ำหนักตัวและ force plate)', 'Kinetics (needs body mass and force plates)'],
  ['Kinematics จาก', 'Kinematics from'], ['IK (global optimisation, ช้ากว่า)', 'IK (global optimisation, slower)'],
  ['กำลังประมวลผล ', 'Processing '], ['ไฟล์สำเร็จ · ', 'files done · '], [' cycle · กดส่งออกด้านล่าง', ' cycles · export below'], ['อ่าน/คำนวณไม่ได้: ', 'could not read/compute: '], ['อ่านไฟล์ใหม่ไม่ได้: ', 'could not re-read file: '],
  ['ขวา · heel strike → heel strike', 'Right · heel strike → heel strike'], ['ขวา · heel strike → toe off', 'Right · heel strike → toe off'], ['ซ้าย · heel strike → heel strike', 'Left · heel strike → heel strike'], ['ซ้าย · heel strike → toe off', 'Left · heel strike → toe off'],
  ['เท้าขวา · stance', 'Right foot · stance'], ['เท้าขวา · stride', 'Right foot · stride'], ['เท้าซ้าย · stance', 'Left foot · stance'], ['เท้าซ้าย · stride', 'Left foot · stride'],
  ['ขวา · stride', 'Right · stride'], ['ขวา · stance', 'Right · stance'], ['ซ้าย · stride', 'Left · stride'], ['ซ้าย · stance', 'Left · stance'],
  [' · ขาด ', ' · missing '], [' ตัวแปร', ' variables'], ['ส่งออกแล้ว · ', 'Exported · '],
  ['หน่วยเมตร', 'units: metres'], ['ข้อมูลสังเคราะห์', 'Synthetic data'], ['โหลดตัวอย่าง', 'Load sample'], ['เปิดไฟล์ C3D', 'Open C3D file'],
  ['มุมมอง 3D ของ marker', '3D view of markers'], ['มุมมอง 3D', '3D view'], ['มุมมองกล้อง', 'Camera views'],
  ['ซ่อน/แสดงเมนูด้านข้าง เพื่อขยายหน้า 3D และกราฟ', 'Show/hide the side menu to enlarge the 3D view and charts'],
  ['มุมเฉียง', 'Isometric view'], ['มองจากด้านบน', 'View from above'], ['มองข้าง แกน X ไปทางขวา', 'Side view, X axis to the right'], ['มองข้าง แกน Y ไปทางขวา', 'Side view, Y axis to the right'],
  ['จัดให้พอดีกับข้อมูล', 'Fit to data'], ['ลาก = หมุน', 'Drag = rotate'], ['Shift/คลิกขวา+ลาก = เลื่อน', 'Shift/right-drag = pan'], ['ล้อ = ซูม', 'Wheel = zoom'], ['คลิก marker = เลือก', 'Click marker = select'],
  ['วางไฟล์ .c3d ที่นี่', 'Drop a .c3d file here'], ['ตัวเลือกการแสดงผล', 'Display options'], ['แกน Lab', 'Lab axes'], ['รูปร่าง segment', 'Segment shapes'], ['แกน segment / JC', 'Segment axes / JC'],
  ['ชื่อ marker', 'Marker names'], ['เฉพาะที่เลือก', 'Selected only'], ['ขนาด marker', 'Marker size'], ['แก้ไข link', 'Edit links'], ['รีเซ็ต link', 'Reset links'],
  ['คลิก marker สองตัวเพื่อเพิ่มหรือลบ link', 'Click two markers to add or remove a link'], ['กลับไปใช้ link อัตโนมัติจากชื่อ marker', 'Revert to automatic links from marker names'],
  ['โหมดแก้ไข link: คลิก marker สองตัวเพื่อเพิ่มหรือลบ link (Esc = ออก)', 'Link edit mode: click two markers to add or remove a link (Esc = exit)'],
  ['ควบคุมเวลา', 'Playback controls'], ['เฟรมก่อนหน้า', 'Previous frame'], ['เฟรมถัดไป', 'Next frame'], ['เล่น/หยุด', 'Play/Pause'], ['หยุดชั่วคราว', 'Pause'], ['ตำแหน่งเฟรม', 'Frame position'],
  ['ผู้ทดสอบ & สัดส่วน (Anthropometry)', 'Subject & Anthropometry'], ['ผู้ทดสอบ & สัดส่วน', 'Subject & Anthropometry'],
  ['ประมาณการและนำไปใช้', 'Estimate & Apply'], ['ส่วนสูง (cm)', 'Height (cm)'], ['น้ำหนัก (kg)', 'Weight (kg)'],
  ['* ค่า width เป็นเพียงค่าประมาณการเบื้องต้น แนะนำให้ใช้คาลิปเปอร์วัดจริง', '* Knee/ankle widths are rough initial estimates. Direct measurement is strongly recommended.'],
  ['คำนวณมุมข้อต่อ', 'Compute joint angles'], ['มุมข้อต่อ', 'Joint angles'], ['ส่งออก CSV', 'Export CSV'],
  ['Plug-in Gait style · Y-X-Z (ชุด 18 marker)', 'Plug-in Gait style · Y-X-Z (18-marker set)'], ['โหลดไฟล์ static แยก', 'Load separate static file'], ['ใช้ trial นี้เป็น static', 'Use this trial as static'], ['Static: ใช้ frame ต้นของ trial ที่เปิดอยู่', 'Static: uses the first frames of the open trial'],
  ['ลบค่าเฉลี่ยช่วง static (zero offset)', 'Subtract static-period mean (zero offset)'], ['จำนวน frame', 'No. of frames'], ['Static: frame เริ่ม', 'Static: start frame'],
  ['ค่า width เป็น placeholder ต้องวัดจริง', 'width values are placeholders — measure real ones'], ['ยังไม่ได้ตรวจเทียบกับซอฟต์แวร์อื่น', 'not yet cross-checked against other software'],
  ['ค้นหาชื่อ', 'Search name'], ['ค้นหา marker', 'Search markers'], ['คีย์ลัด', 'Shortcuts'], ['ยกเลิกโหมด link', 'cancel link mode'], ['±1 เฟรม', '±1 frame'], ['±10', '±10'],
  ['หน่วยในไฟล์', 'File units'], [' → แสดงเป็น m', ' → shown as m'], ['ข้อมูลขาด', 'Missing data'], ['รายการ', 'items'], ['แสดง ', 'Show '], ['สัดส่วนเฟรมที่มีข้อมูล', 'Fraction of frames with data'],
  ['คลิก marker ในมุมมอง 3D หรือในรายการ เพื่อดูกราฟ', 'Click a marker in the 3D view or the list to plot it'], ['ตำแหน่งใน lab frame (m)', 'position in lab frame (m)'],
  ['กด “วิเคราะห์ FP” ก่อน', 'Press “Analyze FP” first'], ['กด “คำนวณมุมข้อต่อ” ก่อน', 'Press “Compute joint angles” first'],
  ['Fz หลังกรอง ลบ offset flight (N)', 'Fz filtered, flight offset removed (N)'], [' · ลบ offset static', ' · static offset removed'], ['GRF ใน lab frame (N)', 'GRF in lab frame (N)'], [' · สังเคราะห์', ' · synthetic'],
  ['ไม่มีข้อมูล force plate', 'No force plate data'], ['กราฟตามเวลา B คลิกหรือลากเพื่อเลื่อนเฟรม', 'Time chart B — click or drag to scrub frames'], ['กราฟตามเวลา คลิกหรือลากเพื่อเลื่อนเฟรม', 'Time chart — click or drag to scrub frames'],
  ['เลือกข้อต่อ กราฟ B', 'Choose joint (chart B)'], ['เลือก marker กราฟ B', 'Choose marker (chart B)'], ['เลือกข้อต่อ', 'Choose joint'], ['เทียบ 2 กราฟ', 'Compare 2 charts'],
  ['force plate ใน C3D', 'force plate in C3D'], ['เลือกช่วงเวลา', 'Select time window'], ['ใส่มวลกาย', 'Enter body mass'], ['มวลกาย (kg)', 'Body mass (kg)'], ['Quiet เริ่ม (s)', 'Quiet start (s)'], ['Quiet จบ (s)', 'Quiet end (s)'],
  ['fn ของ plate (Hz)', 'Plate fn (Hz)'], ['Plate ซ้าย / ขวา', 'Plate left / right'], ['วิเคราะห์ FP', 'Analyze FP'], ['เวลา phase (s)', 'Phase times (s)'], ['คำนวณใหม่', 'Recalculate'], ['กลับเป็น auto', 'Back to auto'],
  ['กราฟแรงช่วง CMJ แยกตาม phase คลิกหรือลากเพื่อเลื่อนเฟรม', 'CMJ force chart by phase — click or drag to scrub frames'], ['กราฟความเร็วจุดศูนย์กลางมวลช่วง CMJ', 'CMJ centre-of-mass velocity chart'],
  ['พบ force plate ที่ใช้ได้ ', 'Usable force plates found: '], [' แผ่นใน C3D', ' in C3D'], [' · แผ่นเดียว: แสดงเฉพาะค่ารวม (ไม่มี L/R)', ' · single plate: totals only (no L/R)'], [' · เลือกแผ่นซ้าย/ขวาได้', ' · left/right plates selectable'],
  [' · พอร์ตจาก Force-Analysis', ' · ported from Force-Analysis'], [' จาก ', ' from '], ['ข้ามการวิเคราะห์: ', 'Analysis skipped: '], ['f ที่ใช้ (Hz)', 'f used (Hz)'],
  ['Fz ช่วง CMJ (N)', 'Fz during CMJ (N)'], ['ความเร็วจุดศูนย์กลางมวล (m/s)', 'Centre-of-mass velocity (m/s)'], ['บันทึกไฟล์ไม่ได้: ', 'Could not save file: '], ['เปิดไฟล์ไม่ได้: ', 'Could not open file: '],
  ['(marker ขยับสูงสุด ', '(max marker movement '], ['ตัวอย่างสังเคราะห์', 'Synthetic sample'], ['แรงสังเคราะห์จากความสูงสะโพก (ไม่ใช่ข้อมูลวัดจริง)', 'Force synthesised from hip height (not measured data)'],
  ['(หลาย prefix)', '(multiple prefixes)'], ['ไฟล์เล็กเกินไปสำหรับ C3D', 'File too small to be a C3D'], ['ไม่ใช่ไฟล์ C3D (byte ที่ 2 ของ header ต้องเป็น 80)', 'Not a C3D file (header byte 2 must be 80)'],
  ['รองรับเฉพาะ C3D แบบ Intel/PC (processor type 84) ไฟล์นี้เป็น type ', 'Only Intel/PC C3D (processor type 84) is supported; this file is type '], ['ไม่พบ POINT:USED ในไฟล์ (ไม่มีข้อมูล marker)', 'POINT:USED not found (no marker data)'],
  ['ไฟล์สั้นกว่าจำนวนเฟรมที่ header ระบุ', 'File is shorter than the frame count in the header'], [' จาก ', ' of '], ['อ่านจำนวนเฟรมไม่ได้', 'Could not read the frame count'],
  ['ไม่พบ POINT:UNITS เดาหน่วยเป็น ', 'POINT:UNITS missing; guessed units: '], ['มิลลิเมตร', 'millimetres'], ['เมตร', 'metres'], ['พารามิเตอร์ force plate ไม่ครบ', 'Force plate parameters incomplete'],
  ['ยังไม่รองรับ FP_TYPE ', 'Unsupported FP_TYPE '], ['(รองรับ 2 และ 4)', '(supported: 2 and 4)'], ['index ของ analog channel ไม่ถูกต้อง', 'Invalid analog channel index'], ['FP_TYPE 4 แต่ไม่มี CAL_MATRIX', 'FP_TYPE 4 but no CAL_MATRIX'],
  [' · กลับเครื่องหมายแรงให้ชี้ขึ้น', ' · force sign flipped to point up'], ['(เดา)', '(guessed)'],
  ['ไม่พบ marker: ', 'Markers not found: '], ['marker ', 'marker '], [' ไม่มีข้อมูลในช่วง static', ' has no data in the static range'],
  ['marker ขยับในช่วง static สูงสุด ', 'Markers moved up to '], [' (>5 mm) ช่วง static อาจไม่นิ่ง', ' (>5 mm) during static; static may not be still'],
  ['มุมแกนกลาง (ab/adduction) สูงสุด ', 'Middle-axis (ab/adduction) angle reaches '], ['° ใกล้ gimbal lock ค่ามุมสามแกนอาจไม่น่าเชื่อถือ', '° near gimbal lock; the three-axis angles may be unreliable'],
  ['ไฟล์สั้นเกินไปสำหรับหา quiet window', 'File too short to find a quiet window'], ['แทบไม่มีช่วงที่มีโหลดบนแผ่น (> 100 N)', 'Almost no loaded period on the plate (> 100 N)'],
  ['ไม่พบช่วง ', 'No '], [' s ที่แรงรวมนิ่งใกล้ระดับโหลดปกติ (', ' s window found with steady total force near normal load ('], ['ต่ำกว่า', 'is below'],
  ['ช่วงยืนนิ่งที่ดีที่สุดยังมี CV ', 'Best quiet-standing window still has CV '], ['ช่วง landing สั้นเกินไปสำหรับ FFT', 'Landing window too short for FFT'], ['flight สั้นเกินไป ใช้ noise floor ไม่ได้ (ข้าม f_snr)', 'Flight too short to use as noise floor (f_snr skipped)'],
  ['f_rise เป็นค่าประมาณหยาบ', 'f_rise is only a rough estimate'], ['หา onset/10%/90% ไม่ได้ (สัญญาณเริ่มสูงอยู่แล้ว หรือไม่มี impact ชัด)', 'Could not find onset/10%/90% (signal already high at start, or no clear impact)'],
  ['spectrum ไม่ตกต่ำกว่า noise floor ก่อน Nyquist: f_snr = Nyquist (อาจมี aliasing/noise สูง)', 'Spectrum never falls below the noise floor before Nyquist: f_snr = Nyquist (possible aliasing/high noise)'],
  ['ไม่มี force plate ที่ใช้ได้ใน C3D นี้ (ต้องเป็น FP_TYPE 2/4 และมี analog)', 'No usable force plate in this C3D (needs FP_TYPE 2/4 and analog data)'], ['ไม่พบ analog rate', 'Analog rate not found'],
  ['เลือก plate ซ้าย/ขวาไม่ถูกต้อง (ต้องเป็นคนละแผ่น)', 'Invalid left/right plate selection (must be different plates)'], ['น้ำหนักที่ระบุ ', 'specified weight '],
  ['ไม่พบ window อัตโนมัติ (ใช้ 0.5 s แรก)', 'no automatic window found (using the first 0.5 s)'], ['หา quiet-standing window อัตโนมัติไม่ผ่านเกณฑ์: ', 'Automatic quiet-standing window failed the criteria: '],
  ['BW ที่ใช้ = ', 'BW used = '], [' (< 200 N) ช่วงที่เลือกน่าจะไม่มีคนยืนบนแผ่น', ' (< 200 N); the selected window probably has nobody standing on the plate'], ['ช่วง quiet window สั้นกว่า 0.25 s', 'Quiet window is shorter than 0.25 s'],
  ['หยุดการคำนวณเพื่อไม่ให้ได้ค่าที่ผิดปกติ: ', 'Calculation stopped to avoid abnormal values: '], [' · เลือกช่วงยืนนิ่งเอง หรือใส่น้ำหนักตัว', ' · choose the quiet-standing window manually or enter body weight'],
  ['ช่วงที่เลือกยังไม่นิ่งมาก', 'Selected window is not very steady'], ['ลำดับเวลา phase ไม่เรียงจาก start < braking < split < take-off < landing', 'Phase times are not in order start < braking < split < take-off < landing'],
  ['ไม่พบ flight phase ที่ชัดเจน (ช่วงที่แรง < 25 N ยาว > 0.1 s หลัง 0.5 s แรก)', 'No clear flight phase found (force < 25 N for > 0.1 s after the first 0.5 s)'], [' · ข้อมูลนี้อาจไม่ใช่การกระโดด CMJ ค่า phase และ metric จึงอาจไม่มีความหมาย', ' · this may not be a CMJ, so phases and metrics may be meaningless'],
  ['จาก ', 'from '],
  /* inverse kinematics / inverse dynamics */
  ['IK ใช้ static calibration ชุดเดียวกับมุมข้อต่อ (กด “คำนวณมุมข้อต่อ” ก่อน)', 'IK uses the same static calibration as the joint angles (press “Compute joint angles” first)'],
  ['กด “คำนวณมุมข้อต่อ” ก่อน (IK ใช้ static calibration ชุดเดียวกัน)', 'Press “Compute joint angles” first (IK uses the same static calibration)'],
  ['รัน IK', 'Run IK'], ['Kinematics ที่ใช้', 'Kinematics source'], ['หารด้วยมวลกาย (Nm/kg, W/kg, N/kg)', 'Normalise to body mass (Nm/kg, W/kg, N/kg)'],
  ['คำนวณ Inverse Dynamics', 'Compute Inverse Dynamics'], ['กด “คำนวณ Inverse Dynamics” ก่อน', 'Press “Compute Inverse Dynamics” first'],
  ['เลือกตัวแปร kinetics กราฟ B', 'Choose kinetic variable (chart B)'], ['เลือกตัวแปร kinetics', 'Choose kinetic variable'],
  ['ใช้น้ำหนักและเพศจากช่อง Anthropometry', 'uses body mass and sex from the Anthropometry card'], ['ต้องมี force plate แยกเท้าซ้าย/ขวา', 'needs separate left/right force plates'],
  ['+ = ทิศที่เพิ่มมุม', '+ = direction that increases the angle'], ['แกน segment', 'segment axes'], [' · หลังกรอง', ' · filtered'],
  ['GRF เท้าขวา', 'GRF right foot'], ['GRF เท้าซ้าย', 'GRF left foot'],
  ['marker residual เฉลี่ย ', 'mean marker residual '], [' mm (สูงสุด ', ' mm (max '], [' · มากสุด: ', ' · largest: '], ['ยังไม่ได้ตรวจเทียบกับซอฟต์แวร์อื่น', 'not yet cross-checked against other software'],
  ['validate แล้วกับข้อมูลสังเคราะห์เท่านั้น ยังไม่ได้ตรวจเทียบกับซอฟต์แวร์อื่น', 'validated on synthetic data only, not yet cross-checked against other software'],
  [' · มวล ', ' · mass '], ['หญิง', 'female'], ['ชาย', 'male'],
  ['marker ที่ห่างจากโมเดลเฉลี่ย > 10 mm: ', 'Markers with mean model distance > 10 mm: '], [' · ตรวจ label หรือ soft-tissue artefact', ' · check labels or soft-tissue artefact'],
  ['pelvis มี marker < 3 ตัว ', 'Pelvis has < 3 markers in '], [' frame (ไม่มีผล IK)', ' frames (no IK result)'], [' frame มีบาง segment marker ไม่พอ (segment นั้นไม่แสดงผล)', ' frames have a segment with too few markers (that segment is not shown)'],
  ['ไม่มี frame ที่ pelvis มี marker ≥ 3 ตัว', 'No frame has ≥ 3 pelvis markers'],
  ['ระบุน้ำหนัก (มวลกาย) 20–300 kg ในช่อง Anthropometry', 'Enter body mass (20–300 kg) in the Anthropometry card'],
  [' frame มีเท้าทั้งสองข้างอยู่บน plate เดียวกัน แยกแรงไม่ได้ (ไม่มีผล ID) · ต้องใช้ plate แยกซ้าย/ขวา', ' frames have both feet on one plate; the force cannot be split (no ID result) · separate left/right plates are needed'],
  [' plate-frame มีแรงแต่หาเท้าที่เหยียบไม่ได้ (ไม่นำมาคิด)', ' plate-frames carry force but no foot could be matched (ignored)'],
  [' · analog ไม่ใช่ตัวเลข (NaN) ', ' · non-numeric analog (NaN) in '], [' frame แทนด้วย 0', ' frames, read as 0'],
  [' · เฉพาะช่วง stance', ' · stance only'],   [' · ขาขวา (', ' · right leg ('], [' · ขาซ้าย (', ' · left leg ('],
  ['ส่งออกข้อมูลในกราฟที่แสดงอยู่ (กราฟ A และ B เมื่อเปิดโหมดเทียบ)', 'Export the data shown in the chart (A, plus B in compare mode)'], ['กราฟยังไม่มีข้อมูลให้ส่งออก', 'The chart has no data to export yet'],
  /* cycles + combined export */
  ['Cycle การเคลื่อนไหว', 'Movement cycles'], ['เฟรมเริ่ม', 'Start frame'], ['เฟรมจบ', 'End frame'], ['เริ่ม = เฟรมนี้', 'Start = this frame'], ['จบ = เฟรมนี้', 'End = this frame'], ['เพิ่ม cycle', 'Add cycle'],
  ['สร้างอัตโนมัติจาก contact (ต้องคำนวณ ID)', 'Auto from foot contact (needs ID)'], ['เท้าขวา · stance (แตะ → ยก)', 'Right foot · stance (strike → off)'], ['เท้าขวา · แตะ → แตะครั้งถัดไป', 'Right foot · strike → next strike'],
  ['เท้าซ้าย · stance (แตะ → ยก)', 'Left foot · stance (strike → off)'], ['เท้าซ้าย · แตะ → แตะครั้งถัดไป', 'Left foot · strike → next strike'], ['สร้าง (แทนที่ของเดิม)', 'Create (replaces list)'], ['ล้างทั้งหมด', 'Clear all'],
  ['ไปที่จุดเริ่ม cycle', 'Go to cycle start'], ['ลบ cycle นี้', 'Delete this cycle'],
  [' cycle · ส่งออกแบบต่อ cycle จะ normalize แต่ละ cycle เป็น 0–100% (101 จุด) พร้อม mean ± SD', ' cycle(s) · per-cycle export normalises each cycle to 0–100 % (101 points) with mean ± SD'],
  ['ยังไม่มี cycle · เลื่อนไปเฟรมเริ่ม กด “เริ่ม = เฟรมนี้” แล้วไปเฟรมจบ กด “จบ = เฟรมนี้” จากนั้นกด “เพิ่ม cycle”', 'No cycles yet · go to the start frame, press “Start = this frame”, go to the end frame, press “End = this frame”, then “Add cycle”'],
  ['cycle ต้องยาวอย่างน้อย 3 เฟรม', 'A cycle needs at least 3 frames'], ['ใส่เฟรมเริ่มและเฟรมจบก่อน', 'Enter the start and end frames first'],
  ['สร้าง cycle จาก contact ต้องคำนวณ Inverse Dynamics ก่อน', 'Creating cycles from contact needs inverse dynamics first'], ['ไม่พบ contact ที่ใช้สร้าง cycle ได้', 'No foot contact found to build cycles from'],
  ['ส่งออก Kinematics + Kinetics', 'Export kinematics + kinetics'], ['Kinematics · มุมข้อต่อ 6 ข้อ × 3 แกน (ตามแหล่งที่เลือก 6DOF/IK)', 'Kinematics · 6 joint angles × 3 axes (selected source 6DOF/IK)'],
  ['รูปแบบ', 'Layout'], ['ต่อเวลา (ทุกเฟรม พร้อมคอลัมน์ cycle)', 'Per time (every frame, with a cycle column)'], ['ต่อ cycle (normalize 0–100%, mean ± SD)', 'Per cycle (normalised 0–100 %, mean ± SD)'], ['เฉพาะเฟรมที่อยู่ใน cycle', 'Only frames inside a cycle'],
  ['ยังไม่มีมุมข้อต่อ · กด “คำนวณมุมข้อต่อ” ก่อน หรือเอาเครื่องหมายถูกที่ Kinematics ออก', 'No joint angles yet · press “Compute joint angles” first, or untick Kinematics'],
  ['ยังไม่มีผล Inverse Dynamics · กด “คำนวณ Inverse Dynamics” ก่อน หรือเอาเครื่องหมายถูกที่ Kinetics ออก', 'No inverse-dynamics result yet · press “Compute Inverse Dynamics” first, or untick Kinetics'],
  ['ยังไม่มี cycle · ใส่เฟรมเริ่ม/จบ แล้วกด “เพิ่ม cycle” ในการ์ด Cycle การเคลื่อนไหว', 'No cycles yet · enter start/end frames and press “Add cycle” in the Movement cycles card'],
  ['เพิ่ม cycle จากเฟรมที่กรอกไว้ให้แล้ว · ', 'Added the cycle from the typed frames · '], ['ส่งออกแล้ว · ', 'Exported · '], [' แถว × ', ' rows × '], [' ตัวแปร (0–100%, mean ± SD)', ' variables (0–100 %, mean ± SD)'], [' ตัวแปร', ' variables'],
  ['เลือกข้อมูลที่จะส่งออกอย่างน้อย 1 ชุด', 'Choose at least one data set to export'], ['ยังไม่มี cycle · ตัดเฟรมในการ์ด “Cycle การเคลื่อนไหว” ก่อน', 'No cycles yet · cut frames in the “Movement cycles” card first'],
  /* upper body */
  ['ร่างกายส่วนบน (ISB)', 'Upper body (ISB)'], ['สลับ RWRA/RWRB ใน trial นี้', 'Swap RWRA/RWRB in this trial'], ['สลับ LWRA/LWRB ใน trial นี้', 'Swap LWRA/LWRB in this trial'],
  ['ลบค่าเฉลี่ยช่วง static (เฉพาะเมื่อ static อยู่ใน trial)', 'Subtract the static mean (only when the static is in this trial)'], ['คำนวณมุมส่วนบน', 'Compute upper-body angles'],
  ['Thorax, trunk, neck, shoulder (Y-X-Y), elbow, wrist ตาม ISB (Wu et al. 2005) · shoulder joint centre: Rab et al. (2002) · ใช้ไฟล์ static และช่วงเฟรมเดียวกับการ์ดมุมข้อต่อ', 'Thorax, trunk, neck, shoulder (Y-X-Y), elbow, wrist per ISB (Wu et al. 2005) · shoulder joint centre: Rab et al. (2002) · uses the static file and frames of the joint-angle card'],
  ['ไฟล์นี้ไม่มี marker ส่วนบนครบ (ต้องมี C7 T10 CLAV STRN และแขน SHO UPA ELB ELBM FRM WRA WRB)', 'This file lacks upper-body markers (needs C7 T10 CLAV STRN and an arm: SHO UPA ELB ELBM FRM WRA WRB)'],
  ['กด “คำนวณมุมส่วนบน” ก่อน', 'Press “Compute upper-body angles” first'], ['ข้อนี้ไม่มีข้อมูลในไฟล์ (marker ไม่ครบ)', 'No data for this joint in the file (markers missing)'], ['Kinetics มีเฉพาะขา (สะโพก เข่า ข้อเท้า)', 'Kinetics are available for the legs only (hip, knee, ankle)'],
  ['ต้องมี marker thorax (C7 T10 CLAV STRN) และแขนอย่างน้อย 1 ข้าง (SHO UPA ELB ELBM FRM WRA WRB; FIN ถ้าต้องการมุมข้อมือ)', 'Needs thorax markers (C7 T10 CLAV STRN) and at least one arm (SHO UPA ELB ELBM FRM WRA WRB; FIN for the wrist angle)'],
  ['ไฟล์ static ไม่มี marker ส่วนบนครบ', 'The static file lacks the upper-body markers'], ['มีแขนข้างเดียว: ใช้ระยะ acromion–acromion ประมาณ 0.35 m สำหรับ shoulder joint centre', 'One arm only: inter-acromion distance assumed 0.35 m for the shoulder joint centre'],
  [' (เช่นมือที่จับไม้) · ไม่คำนวณมุมข้อมือข้างนี้', ' (e.g. the racket hand) · no wrist angle on this side'], [': ไม่มี marker ', ': no marker '],
  [' สลับ label ใน trial นี้ (ตามที่เลือก)', ' labels swapped in this trial (as selected)'], ['ลบ offset static ได้เฉพาะเมื่อ static อยู่ใน trial เดียวกัน', 'The static offset can only be removed when the static is in the same trial'],
  [' (เกิน full supination) · ', ' (beyond full supination) · '], [' อาจสลับ label เทียบกับ static ตรวจในภาพ 3D ว่า WRA อยู่ฝั่งนิ้วโป้ง แล้วติ๊ก “สลับ WRA/WRB” ถ้าผิด', ' may be swapped relative to the static · check in the 3D view that WRA is on the thumb side, and tick “Swap WRA/WRB” if not'],
  [': elevation < 15° ใน ', ': elevation < 15° in '], ['% ของเฟรม · plane of elevation และ axial rotation ไม่มีนิยามชัดที่ elevation ต่ำ (gimbal lock ของ Y-X-Y)', '% of frames · plane of elevation and axial rotation are ill-defined at low elevation (Y-X-Y gimbal lock)'],
  [': มุมแกนกลางสูงสุด ', ': middle-axis angle up to '], ['° ใกล้ gimbal lock', '° near gimbal lock'], [': fit RMS เฉลี่ย ', ': mean fit RMS '], [' mm (> 10 mm) · ตรวจ marker ด้วย QC', ' mm (> 10 mm) · check markers with QC'],
  ['ไม่มี marker ศีรษะครบ: ไม่คำนวณ neck', 'Head markers incomplete: no neck angle'], ['ไม่มี marker pelvis ครบ: ไม่คำนวณ trunk (thorax เทียบ pelvis)', 'Pelvis markers incomplete: no trunk angle (thorax vs pelvis)'],
  ['marker ศีรษะเอียงจาก thorax ใน static (flex ', 'Head markers are tilted from the thorax in the static (flex '], ['°) · ใช้ head offset: มุม neck เทียบกับท่า static', '°) · head offset applied: neck angles are relative to the static posture'],
  /* marker QC + tracking */
  ['ตรวจคุณภาพ marker (QC)', 'Marker quality control (QC)'], ['ตรวจ marker', 'Check markers'], ['แสดงทุกคู่', 'Show all pairs'], ['คู่ marker', 'Marker pair'],
  ['เทียบระยะระหว่าง marker บนกระดูกชิ้นเดียวกันกับ static (ใช้ไฟล์ static และช่วงเฟรมเดียวกับการ์ดมุมข้อต่อ) · หา marker ที่ label ผิด, gap-fill ผิด segment, หลุด/ติดใหม่ หรือ soft-tissue artefact สูง', 'Compares distances between markers on the same bone with the static (same static file and frames as the joint-angle card) · finds mislabelled markers, wrong-segment gap-fills, markers that fell off / were re-attached, or high soft-tissue artefact'],
  ['กด “ตรวจ marker” แล้วคลิกคู่ในตาราง', 'Press “Check markers”, then click a pair in the table'],
  ['ตำแหน่งกระโดดระหว่างเฟรม (label สลับหรือ marker หาย)', 'position jumps between frames (label swap or dropout)'],
  ['ระยะเข่า–ข้อเท้าด้าน medial กับ lateral เปลี่ยนไม่สอดคล้องกัน (gap-fill ผิด segment หรือ label ผิด)', 'medial and lateral knee–ankle spans change inconsistently (wrong-segment gap-fill or mislabel)'],
  ['ระยะกับ marker อื่นบนกระดูกเดียวกันไม่คงที่ (หลุด/ติดใหม่ หรือ soft-tissue artefact สูง)', 'distance to other markers on the same bone is not constant (fell off / re-attached, or high soft-tissue artefact)'],
  [' · marker ที่น่าสงสัย: ', ' · suspect markers: '],
  ['ไม่พบ marker ที่น่าสงสัย', 'No suspect markers'], ['คลิกแถวเพื่อดูกราฟ Δ และไปเฟรมที่ห่างมากสุด', 'Click a row to plot Δ and go to the frame of the largest deviation'],
  ['mm · bias = median Δ (ตำแหน่งเปลี่ยนจาก static) · p95 = p95 |Δ| (แกว่งระหว่างเคลื่อนไหว) · ชี้แถวเพื่อดูรายละเอียด', 'mm · bias = median Δ (shift from static) · p95 = p95 |Δ| (variation during movement) · hover a row for details'], [' · ไม่พบ marker ที่น่าสงสัย', ' · no suspect markers'], ['QC: มีข้อควรระวัง', 'QC: caution'], ['QC: มีปัญหา', 'QC: problems'], ['QC: ผ่าน', 'QC: pass'],
  [' ต่างกัน p95 ', ' differ by p95 '], [' ครั้ง (แรกที่ f', ' times (first at f'], [' กระโดด ', ' jumps '], [' หาย ', ' missing '],
  [' · คลิกแถวเพื่อดูกราฟ Δ และไปเฟรมที่ห่างมากสุด', ' · click a row to plot Δ and go to the frame of the largest deviation'],
  [' · ไม่มีคู่ที่เกินเกณฑ์ (ติ๊ก “แสดงทุกคู่” เพื่อดูทั้งหมด)', ' · no pair exceeds the limits (tick “Show all pairs” to list them all)'],
  [' · Δ ระยะจาก static (mm) · static ', ' · Δ distance from static (mm) · static '],
  ['Anatomical · ต่อ joint centre จาก segment แม่ (แนะนำ)', 'Anatomical · joint centres chained from the parent segment (recommended)'],
  ['Mixed · KNE/KNEM ช่วย track shank, ANK/ANKM ช่วย track foot (v0.2)', 'Mixed · KNE/KNEM also track the shank, ANK/ANKM the foot (v0.2)'],
  ['ยกเลิกการบันทึกไฟล์', 'File save cancelled'], ['มีหน้าต่างยืนยันการบันทึกเปิดอยู่ ลองใหม่อีกครั้ง', 'A save confirmation is already open · try again'], ['บันทึกไฟล์ไม่ได้: ', 'Could not save the file: '],
  ['หน้านี้เปิดใน Claude ซึ่งบล็อกการดาวน์โหลดไฟล์ · เปิด index.html ใน Chrome/Edge หรือจากเว็บของ lab แล้วกดส่งออกอีกครั้ง', 'This page is open inside Claude, which blocks file downloads · open index.html in Chrome/Edge or from the lab web server and export again'],
  /* force plate selection */
  ['ตัดแผ่นที่ไม่ได้ใช้', 'Exclude unused plates'], ['ใช้ทุกแผ่น', 'Use all plates'], ['เวลามีแรง ', 'loaded '], ['ไม่มีแรง (peak ', 'no load (peak '], ['อ่านไม่ได้', 'unreadable'],
  ['แผ่นที่ใช้: ', 'Plates in use: '], [' · ตัดออก: ', ' · excluded: '], [' · มีผลกับกราฟ GRF, การวิเคราะห์ CMJ, Inverse Dynamics และภาพ 3D', ' · applies to the GRF chart, CMJ analysis, inverse dynamics and the 3D view'],
  ['เลือก force plate กราฟ B', 'Choose force plate (chart B)'], ['เลือก force plate', 'Choose force plate'],
  ['บาง plate ไม่มี free moment (Tz) ใช้ค่า 0', 'Some plates have no free moment (Tz); 0 used'],
  [' อาจทำให้ joint moment มี artefact ช่วง impact (Kristianslund et al. 2012)', ' may cause joint-moment artefacts around impact (Kristianslund et al. 2012)']
];
const _frags = I18N_FRAG.slice().sort((a, b) => b[0].length - a[0].length);
function T(s) {
  if (LANG !== 'en' || typeof s !== 'string' || !/[฀-๿]/.test(s)) return s;
  const ex = I18N_EXACT[s.trim()]; if (ex) return s.replace(s.trim(), ex);
  const hold = []; let o = s;
  for (const [th, en] of _frags) { if (o.indexOf(th) < 0) continue; o = o.split(th).join('' + hold.length + ''); hold.push(en); }
  return o.replace(/(\d+)/g, (m, i) => hold[+i]);
}
const _txt = new WeakMap(), _att = new WeakMap(), ATTRS = ['title', 'aria-label', 'placeholder'];
function trNode(n) {
  if (n.nodeType === 3) {
    const v = n.nodeValue, rec = _txt.get(n);
    if (rec && v === rec.en) { if (LANG === 'th') { n.nodeValue = rec.th; _txt.delete(n); } return; }
    if (LANG === 'en' && /[฀-๿]/.test(v)) { const en = T(v); if (en !== v) { _txt.set(n, { th: v, en }); n.nodeValue = en; } }
    else if (rec) _txt.delete(n);
  } else if (n.nodeType === 1) {
    for (const a of ATTRS) {
      if (!n.hasAttribute(a)) continue;
      const v = n.getAttribute(a); let m = _att.get(n); if (!m) { m = {}; _att.set(n, m); }
      const rec = m[a];
      if (rec && v === rec.en) { if (LANG === 'th') { n.setAttribute(a, rec.th); delete m[a]; } continue; }
      if (LANG === 'en' && /[฀-๿]/.test(v)) { const en = T(v); if (en !== v) { m[a] = { th: v, en }; n.setAttribute(a, en); } } else if (rec) delete m[a];
    }
  }
}
function trTree(root) {
  trNode(root);
  const w = document.createTreeWalker(root, 1 | 4); let n; while ((n = w.nextNode())) trNode(n);
}
const _mo = new MutationObserver(ms => {
  for (const m of ms) {
    if (m.type === 'characterData') trNode(m.target);
    else if (m.type === 'attributes') trNode(m.target);
    else for (const a of m.addedNodes) trTree(a);
  }
});
_mo.observe(document.body, { subtree: true, childList: true, characterData: true, attributes: true, attributeFilter: ATTRS });
function setLang(l) {
  LANG = l; try { localStorage.setItem('primalab.lang', l); } catch (e) { /* ignore */ }
  document.documentElement.lang = l; trTree(document.body);
  const b = document.getElementById('bLang'); if (b) { b.textContent = l === 'en' ? 'ไทย' : 'EN'; b.title = l === 'en' ? 'เปลี่ยนเป็นภาษาไทย' : 'Switch to English'; b.setAttribute('aria-label', b.title); }
  if (typeof langChanged === 'function') langChanged();
}
