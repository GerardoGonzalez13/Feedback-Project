# Technical Report Recovery

## Recovered values
### baseline
- Run: `data/runs/baseline_1775260512.csv`
- Handoff metric source: first baseline_phase == neutral_observation

### PD
- Run: `data/runs/controlled_1774837914.csv`
- Handoff metric source: first sustained corrective cmd_roll after reaching target bank

### 2-state LQR
- Run: `data/runs/controlled_1774849067.csv`
- Handoff metric source: first sustained corrective cmd_roll after reaching target bank

### 2-state LQI
- Run: `data/runs/controlled_1774927980.csv`
- Handoff metric source: first sustained corrective cmd_roll after reaching target bank

### 4-state LQR
- Run: `data/runs/controlled_1774975570.csv`
- Handoff metric source: first logged controller_handoff_elapsed_s

## Files used
### baseline
- `data/runs/baseline_1775260512.csv`

### PD
- `data/runs/controlled_1774837914.csv`
- `git:aedd842:external/controller_roll.py`

### 2-state LQR
- `data/runs/controlled_1774849067.csv`
- `data/runs/run_1774842310.csv`
- `external/estimate_ab_id.py`

### 2-state LQI
- `data/runs/controlled_1774927980.csv`
- `data/runs/run_1774842310.csv`
- `external/estimate_ab_id.py`

### 4-state LQR
- `data/runs/controlled_1774975570.csv`
- `data/runs/run_1774842591_lateral4_summary.json`
- `data/runs/run_1774933421_lateral_mimo_summary.json`
- `external/run_controlled.py`

## How each value was obtained
### baseline
- Run identification: Exact run file provided in the recovery request.
- Performance metrics: first baseline_phase == neutral_observation

### PD
- Run identification: Exact run file provided in the recovery request.
- Performance metrics: first sustained corrective cmd_roll after reaching target bank

### 2-state LQR
- Run identification: Exact run file provided in the recovery request.
- Performance metrics: first sustained corrective cmd_roll after reaching target bank

### 2-state LQI
- Run identification: Exact run file provided in the recovery request.
- Performance metrics: first sustained corrective cmd_roll after reaching target bank

### 4-state LQR
- Run identification: Exact run file provided in the recovery request.
- Performance metrics: first logged controller_handoff_elapsed_s

## Ambiguities or assumptions
### baseline
- None beyond file-format-based handoff detection.

### PD
- None beyond file-format-based handoff detection.

### 2-state LQR
- Selected the latest open-loop pulse-identification run immediately preceding the 2-state LQR run. The earlier run_1774645989.csv is also preserved as an ambiguity candidate.

### 2-state LQI
- Selected the latest open-loop pulse-identification run immediately preceding the 2-state LQR run. The earlier run_1774645989.csv is also preserved as an ambiguity candidate.

### 4-state LQR
- The exact candidate name was not logged in the March 31 run CSV. The values above are the closest match among the candidates currently present in external/run_controlled.py.

## Still missing
### baseline
- state-space model for baseline [UNRECOVERABLE FROM CURRENT FILES]
- closed-loop matrices for baseline [UNRECOVERABLE FROM CURRENT FILES]
- eigenvalues for baseline [UNRECOVERABLE FROM CURRENT FILES]

### PD
- PD closed-loop matrix [UNRECOVERABLE FROM CURRENT FILES]
- PD weighting matrices [UNRECOVERABLE FROM CURRENT FILES]
- PD eigenvalues [UNRECOVERABLE FROM CURRENT FILES]
- PD controllability rank [UNRECOVERABLE FROM CURRENT FILES]

### 2-state LQR
- 2-state LQR Q matrix [UNRECOVERABLE FROM CURRENT FILES]
- 2-state LQR R matrix [UNRECOVERABLE FROM CURRENT FILES]

### 2-state LQI
- 2-state LQI Q matrix [UNRECOVERABLE FROM CURRENT FILES]
- 2-state LQI R matrix [UNRECOVERABLE FROM CURRENT FILES]

### 4-state LQR

## Suggested placeholders for unrecoverable values
`[UNRECOVERABLE FROM CURRENT FILES]`

## Paste-ready table
| Controller | Run | Handoff phi (deg) | Handoff p (deg/s) | Peak |phi| (deg) | RMS phi (deg) | t to |phi|<10 (s) | t to |phi|<5 (s) | Settling (s) | Alt loss (ft) | Max |p| (deg/s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| baseline | data/runs/baseline_1775260512.csv | 58.911 | 1.408 | 82.163 | 71.244 | NOT_REACHED | NOT_REACHED | NOT_REACHED | 2729.376 | 4.089 |
| PD | data/runs/controlled_1774837914.csv | 57.493 | 2.095 | 71.227 | 40.933 | 16.636 | 21.428 | 21.428 | 0.000 | 12.448 |
| 2-state LQR | data/runs/controlled_1774849067.csv | 57.219 | 1.614 | 61.772 | 58.502 | NOT_REACHED | NOT_REACHED | NOT_REACHED | 503.164 | 2.209 |
| 2-state LQI | data/runs/controlled_1774927980.csv | 57.189 | 1.715 | 62.772 | 30.841 | 9.587 | 10.463 | 10.463 | 0.000 | 14.949 |
| 4-state LQR | data/runs/controlled_1774975570.csv | 58.998 | 1.432 | 59.456 | 17.445 | 4.616 | 6.497 | 6.497 | 0.000 | 24.723 |