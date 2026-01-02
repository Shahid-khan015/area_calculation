"""
FastAPI wrapper for precision farming area calculation.

Endpoints:
- POST /calculate  → Final, billable area (geometry union)
- GET  /manual-test → Internal validation test
- POST /visualize → Debug / UI visualization (GeoJSON)
"""

from typing import List, Optional
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

from area_calculator import (
    SensorRecord,
    PlanarPoint,
    CalculationConfig,
    calculate_worked_area,
)

app = FastAPI(title="Precision Farming Area Calculator API")

# ---------------------------------------------------------------------------
# CORS
# ---------------------------------------------------------------------------

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------------------------------------------------------------------------
# REQUEST MODELS
# ---------------------------------------------------------------------------

class RecordModel(BaseModel):
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    x: Optional[float] = None
    y: Optional[float] = None
    timestamp: Optional[str] = None
    speed_kmh: float
    engine_on: bool
    pto_on: bool


class ConfigModel(BaseModel):
    implement_width_m: float = Field(..., gt=0)
    min_speed_kmh: float = Field(..., ge=0)


class CalculateRequest(BaseModel):
    records: List[RecordModel]
    config: ConfigModel


# ---------------------------------------------------------------------------
# FINAL AREA CALCULATION (BILLABLE)
# ---------------------------------------------------------------------------

@app.post("/calculate")
def calculate(req: CalculateRequest):
    """
    Final area calculation using geometry union.
    Overlaps are physically removed.
    """

    all_planar = all(r.x is not None and r.y is not None for r in req.records)
    all_gps = all(r.latitude is not None and r.longitude is not None for r in req.records)

    config = CalculationConfig(
        implement_width_m=req.config.implement_width_m,
        min_speed_kmh=req.config.min_speed_kmh,
    )

    if all_planar:
        planar_points = [
            PlanarPoint(
                x=r.x,
                y=r.y,
                speed_kmh=r.speed_kmh,
                engine_on=r.engine_on,
                pto_on=r.pto_on,
            )
            for r in req.records
        ]

        # Convert planar points into fake SensorRecords is NOT needed
        # Geometry logic works directly on planar data
        from area_calculator import build_work_polygons
        from shapely.ops import unary_union

        strips = build_work_polygons(planar_points, config.implement_width_m)
        merged = unary_union(strips) if strips else None

        area_m2 = merged.area if merged else 0.0

    elif all_gps:
        sensor_records = [
            SensorRecord(
                latitude=r.latitude,
                longitude=r.longitude,
                timestamp=r.timestamp or "",
                speed_kmh=r.speed_kmh,
                engine_on=r.engine_on,
                pto_on=r.pto_on,
            )
            for r in req.records
        ]

        result = calculate_worked_area(sensor_records, config)
        area_m2 = result.total_area_m2

    else:
        return {"error": "Provide either all GPS points or all planar points."}

    return {
        "total_area_m2": round(area_m2, 2),
        "total_area_ha": round(area_m2 / 10000.0, 6),
        "calculation_method": "geometry_union",
    }


# ---------------------------------------------------------------------------
# MANUAL VALIDATION
# ---------------------------------------------------------------------------

@app.get("/manual-test")
def manual_test():
    """
    Internal sanity test.
    """

    points = [
        PlanarPoint(0, 0, 5, True, True),
        PlanarPoint(20, 0, 5, True, True),
        PlanarPoint(0, 3, 5, True, True),
        PlanarPoint(20, 3, 5, True, True),
    ]

    from area_calculator import build_work_polygons
    from shapely.ops import unary_union

    strips = build_work_polygons(points, implement_width_m=4.0)
    merged = unary_union(strips)

    return {
        "total_area_m2": merged.area,
        "total_area_ha": merged.area / 10000,
        "status": "ok",
    }


# ---------------------------------------------------------------------------
# VISUALIZATION (DEBUG / UI ONLY)
# ---------------------------------------------------------------------------

@app.post("/visualize")
def visualize(req: CalculateRequest):
    """
    Return GeoJSON of final merged geometry.
    Not used for billing.
    """

    all_planar = all(r.x is not None and r.y is not None for r in req.records)

    if not all_planar:
        return {"error": "Visualization requires planar x/y input"}

    planar_points = [
        PlanarPoint(r.x, r.y, r.speed_kmh, r.engine_on, r.pto_on)
        for r in req.records
    ]

    from area_calculator import build_work_polygons
    from shapely.ops import unary_union
    from shapely.geometry import mapping

    strips = build_work_polygons(planar_points, req.config.implement_width_m)
    merged = unary_union(strips) if strips else None

    if not merged:
        return {"type": "FeatureCollection", "features": []}

    return {
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature",
            "properties": {},
            "geometry": mapping(merged)
        }]
    }
