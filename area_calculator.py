"""
Precision Farming Area Calculation Module (Corrected)

Accurate, overlap-safe area calculation using geometric union.
Grid logic is removed from final billing calculation.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class SensorRecord:
    latitude: float
    longitude: float
    timestamp: str
    speed_kmh: float
    engine_on: bool
    pto_on: bool


@dataclass
class PlanarPoint:
    x: float
    y: float
    speed_kmh: float
    engine_on: bool
    pto_on: bool


@dataclass
class CalculationConfig:
    implement_width_m: float
    min_speed_kmh: float


@dataclass
class CalculationResult:
    total_area_m2: float
    total_area_ha: float
    geometry: Polygon


# ============================================================================
# COORDINATE CONVERSION
# ============================================================================

def gps_to_planar(lat: float, lon: float,
                  origin_lat: float, origin_lon: float) -> Tuple[float, float]:
    R = 6371000.0  # Earth radius (m)

    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    o_lat = math.radians(origin_lat)
    o_lon = math.radians(origin_lon)

    dlat = lat_r - o_lat
    dlon = lon_r - o_lon
    avg_lat = (lat_r + o_lat) / 2.0

    x = dlon * R * math.cos(avg_lat)
    y = dlat * R
    return x, y


# ============================================================================
# FILTERING
# ============================================================================

def filter_and_project(records: List[SensorRecord],
                       min_speed: float) -> List[PlanarPoint]:

    valid: List[PlanarPoint] = []
    origin_lat = origin_lon = None

    for r in records:
        if not (r.engine_on and r.pto_on and r.speed_kmh >= min_speed):
            continue

        if origin_lat is None:
            origin_lat = r.latitude
            origin_lon = r.longitude

        x, y = gps_to_planar(r.latitude, r.longitude, origin_lat, origin_lon)
        valid.append(PlanarPoint(x, y, r.speed_kmh, r.engine_on, r.pto_on))

    return valid


# ============================================================================
# CORE GEOMETRIC LOGIC (FIXED)
# ============================================================================

def build_work_polygons(points: List[PlanarPoint],
                        implement_width_m: float) -> List[Polygon]:

    polygons = []

    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]

        distance = math.hypot(p2.x - p1.x, p2.y - p1.y)

        # Ignore jitter
        if distance < 0.3:
            continue

        # Ignore GPS gaps / teleportation
        if distance > 20.0:
            continue

        line = LineString([(p1.x, p1.y), (p2.x, p2.y)])
        strip = line.buffer(implement_width_m / 2.0, resolution=16)

        polygons.append(strip)

    return polygons


# ============================================================================
# FINAL AREA CALCULATION
# ============================================================================

def calculate_worked_area(records: List[SensorRecord],
                          config: CalculationConfig,
                          field_boundary: Optional[Polygon] = None
                          ) -> CalculationResult:

    valid_points = filter_and_project(records, config.min_speed_kmh)

    if len(valid_points) < 2:
        return CalculationResult(0.0, 0.0, Polygon())

    strips = build_work_polygons(valid_points, config.implement_width_m)

    if not strips:
        return CalculationResult(0.0, 0.0, Polygon())

    # 🔥 THIS IS THE CRITICAL FIX
    merged_area = unary_union(strips)

    # Optional clipping to field boundary
    if field_boundary:
        merged_area = merged_area.intersection(field_boundary)

    total_area_m2 = merged_area.area
    total_area_ha = total_area_m2 / 10000.0

    return CalculationResult(
        total_area_m2=total_area_m2,
        total_area_ha=total_area_ha,
        geometry=merged_area
    )


# ============================================================================
# MANUAL TEST
# ============================================================================

def run_manual_test():
    points = [
        PlanarPoint(0, 0, 5, True, True),
        PlanarPoint(20, 0, 5, True, True),
        PlanarPoint(0, 3, 5, True, True),
        PlanarPoint(20, 3, 5, True, True),
    ]

    strips = build_work_polygons(points, implement_width_m=4.0)
    merged = unary_union(strips)

    print("Total area (m²):", merged.area)
    print("Total area (ha):", merged.area / 10000)


if __name__ == "__main__":
    run_manual_test()
