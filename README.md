# Precision Farming Area Calculator (FastAPI)

This small service exposes the precision farming area calculator via FastAPI.

Quick start

1. Create a virtual environment and install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate   # or `.venv\Scripts\activate` on Windows
pip install -r requirements.txt
```

2. Run the API server:

```bash
uvicorn app:app --reload
```

3. Endpoints:

- `POST /calculate` — JSON body with `records` and `config` to compute area.
- `GET /manual-test` — Runs the bundled planar test and returns computed area.

Input schema for `POST /calculate` (example):

```json
{
  "records": [
    {"x": 0, "y": 0, "speed_kmh": 5, "engine_on": true, "pto_on": true},
    {"x": 10, "y": 0, "speed_kmh": 5, "engine_on": true, "pto_on": true}
  ],
  "config": {"implement_width_m": 4.0, "min_speed_kmh": 2.0, "grid_cell_size_m": 1.0}
}
```

Notes
- Records may provide either `x`/`y` (planar meters) or `latitude`/`longitude` (degrees). If `x`/`y` are provided they are used directly.
- The service uses the grid-based approach implemented in `area_calculator.py`.
