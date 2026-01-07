
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union


# =============================================================================
# DATA STRUCTURES
# =============================================================================

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
    quality_score: float  # 0.0 to 1.0
    estimated_error_m2: float  # ±error in m²
    working_trajectory_points: int  # Count of working-speed points
    repositioning_points_filtered: int  # Count of high-speed (repositioning) points filtered out


# =============================================================================
# COORDINATE CONVERSION
# =============================================================================

def gps_to_planar(lat: float, lon: float,
                  origin_lat: float, origin_lon: float) -> Tuple[float, float]:
    R = 6371000.0

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


# =============================================================================
# FILTERING & SMOOTHING
# =============================================================================

def smooth_planar_points(points: List[PlanarPoint], window: int = 3) -> List[PlanarPoint]:
    if len(points) < window:
        return points

    smoothed = []
    for i in range(len(points)):
        xs, ys = [], []
        for j in range(max(0, i - window), min(len(points), i + window + 1)):
            xs.append(points[j].x)
            ys.append(points[j].y)

        smoothed.append(
            PlanarPoint(
                x=sum(xs) / len(xs),
                y=sum(ys) / len(ys),
                speed_kmh=points[i].speed_kmh,
                engine_on=points[i].engine_on,
                pto_on=points[i].pto_on,
            )
        )
    return smoothed


def separate_trajectories_by_speed(points: List[PlanarPoint],
                                    repositioning_speed_kmh: float = 15.0
                                    ) -> Tuple[List[PlanarPoint], List[PlanarPoint]]:
    """
    OPTIONAL: Separate working trajectory from repositioning trajectory.
    
    NOTE: All points here already have PTO ON, so they are "working" by definition.
    This function is kept for backward compatibility but has limited utility
    since PTO-OFF points are already filtered in filter_and_project().
    
    Use this only if you want to further subdivide by speed (e.g., identify
    periods of slow careful work vs faster field traversal).
    
    Returns:
        (working_points, repositioning_points) — two separate trajectory lists
    """
    working = []
    repositioning = []
    
    for point in points:
        if point.speed_kmh >= repositioning_speed_kmh:
            repositioning.append(point)
        else:
            working.append(point)
    
    return working, repositioning


def calculate_quality_metrics(valid_points: List[PlanarPoint],
                              working_points: List[PlanarPoint],
                              repositioning_count: int,
                              area_m2: float
                              ) -> Tuple[float, float]:
    """
    Calculate GPS quality score and estimated error bounds.
    
    Quality factors:
    1. Data density: points per 100m of travel
    2. Working trajectory ratio: % of points at working speed
    3. Speed consistency: std dev of working speeds
    4. GPS fix confidence: lower repositioning ratio = better confidence in working area
    
    Returns:
        (quality_score: 0.0-1.0, estimated_error_m2: ±meters²)
    """
    if not working_points or not valid_points:
        return 0.0, area_m2 * 0.5  # Low confidence
    
    # Factor 1: Data density (normalized)
    total_distance = 0.0
    for i in range(len(working_points) - 1):
        dx = working_points[i + 1].x - working_points[i].x
        dy = working_points[i + 1].y - working_points[i].y
        total_distance += math.hypot(dx, dy)
    
    points_per_100m = (len(working_points) / max(total_distance, 100.0)) * 100.0
    density_score = min(points_per_100m / 20.0, 1.0)  # 20+ points per 100m = perfect
    
    # Factor 2: Working trajectory ratio
    working_ratio = len(working_points) / max(len(valid_points), 1)
    working_ratio_score = min(working_ratio, 1.0)
    
    # Factor 3: Speed consistency in working trajectory
    if len(working_points) > 1:
        working_speeds = [p.speed_kmh for p in working_points]
        mean_speed = sum(working_speeds) / len(working_speeds)
        variance = sum((s - mean_speed) ** 2 for s in working_speeds) / len(working_speeds)
        std_dev = math.sqrt(variance)
        speed_consistency = max(1.0 - (std_dev / max(mean_speed, 0.1)), 0.0)
    else:
        speed_consistency = 0.5
    
    # Factor 4: Repositioning filtering confidence
    repositioning_ratio = repositioning_count / max(len(valid_points), 1)
    repositioning_score = 1.0 - min(repositioning_ratio, 1.0)
    
    # Combined quality score (weighted average)
    quality_score = (
        density_score * 0.25 +
        working_ratio_score * 0.30 +
        speed_consistency * 0.25 +
        repositioning_score * 0.20
    )
    
    # Estimated error based on quality
    # Assume standard GPS error ±5m for position, scales with data quality
    gps_position_error_m = 5.0
    implement_width_error_m = 0.5
    
    base_error = (gps_position_error_m + implement_width_error_m) * (1.0 - quality_score)
    estimated_error_m2 = base_error * math.sqrt(area_m2 / 100.0)
    
    return quality_score, estimated_error_m2



def filter_and_project(records: List[SensorRecord],
                       min_speed: float) -> List[PlanarPoint]:
    """
    Filter records by work criteria (engine ON, PTO ON, adequate speed).
    Project to planar coordinates.
    Apply heading jitter detection to remove GPS noise BEFORE smoothing.
    
    Returns clean, jitter-filtered planar points (NOT yet smoothed).
    """
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

    # Apply heading jitter detection to remove GPS noise
    # Only check interior points (not endpoints)
    if len(valid) < 3:
        return valid
    
    jitter_filtered = [valid[0]]  # Keep first point
    
    for i in range(1, len(valid) - 1):
        # Check if point i is heading jitter
        if not is_heading_jitter(valid[i - 1], valid[i], valid[i + 1]):
            jitter_filtered.append(valid[i])
    
    jitter_filtered.append(valid[-1])  # Keep last point
    
    # NOW smooth the clean points (much fewer of them, so smoothing won't collapse them)
    return smooth_planar_points(jitter_filtered, window=3)



# =============================================================================
# GEOMETRIC CORE (HARDENED)
# =============================================================================

def heading(p1: PlanarPoint, p2: PlanarPoint) -> float:
    return math.atan2(p2.y - p1.y, p2.x - p1.x)


def is_heading_jitter(p_prev: PlanarPoint,
                      p_curr: PlanarPoint,
                      p_next: PlanarPoint,
                      max_angle_rad: float = math.radians(45)) -> bool:
    h1 = heading(p_prev, p_curr)
    h2 = heading(p_curr, p_next)

    diff = abs(h2 - h1)
    diff = min(diff, 2 * math.pi - diff)

    return diff > max_angle_rad


def build_work_polygons(points, implement_width_m):
    polygons = []

    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]

        distance = math.hypot(p2.x - p1.x, p2.y - p1.y)

        if distance < max(0.5, implement_width_m * 0.25):
            continue

        if distance > 25.0:
            continue

        line = LineString([(p1.x, p1.y), (p2.x, p2.y)])

        strip = line.buffer(
            implement_width_m / 2.0,
            resolution=32,
            cap_style=1,
            join_style=1
        )

        polygons.append(strip)

    return polygons




# =============================================================================
# FINAL AREA CALCULATION
# =============================================================================

def calculate_worked_area(records: List[SensorRecord],
                          config: CalculationConfig,
                          field_boundary: Optional[Polygon] = None,
                          repositioning_speed_kmh: float = 15.0
                          ) -> CalculationResult:
    """
    Calculate worked area with diagnostics.
    
    Pipeline:
    1. Filter by engine ON, PTO ON, speed >= min_speed_kmh
    2. Project to planar coordinates
    3. Detect and remove heading jitter (GPS noise)
    4. Smooth remaining clean points
    5. Build work strips
    6. Union strips (remove overlaps)
    7. Optionally clip to field boundary
    8. Calculate quality metrics
    """

    valid_points = filter_and_project(records, config.min_speed_kmh)
    
    print(f"DEBUG: After filter + jitter removal + smoothing: {len(valid_points)} points")

    if len(valid_points) < 2:
        print("ERROR: Insufficient valid points (need ≥ 2)")
        return CalculationResult(
            total_area_m2=0.0,
            total_area_ha=0.0,
            geometry=Polygon(),
            quality_score=0.0,
            estimated_error_m2=0.0,
            working_trajectory_points=len(valid_points),
            repositioning_points_filtered=0
        )

    # Build work strips from ALL valid points (PTO already filtered)
    strips = build_work_polygons(valid_points, config.implement_width_m)
    
    print(f"DEBUG: Generated {len(strips)} work strips")

    if not strips:
        print("WARNING: No valid work strips generated. Check distance thresholds and heading jitter.")
        return CalculationResult(
            total_area_m2=0.0,
            total_area_ha=0.0,
            geometry=Polygon(),
            quality_score=0.0,
            estimated_error_m2=0.0,
            working_trajectory_points=len(valid_points),
            repositioning_points_filtered=0
        )

    merged = unary_union(strips)
    
    print(f"DEBUG: Merged geometry area: {merged.area:.2f} m²")

    if field_boundary:
        merged = merged.intersection(field_boundary)
        print(f"DEBUG: After field boundary clip: {merged.area:.2f} m²")

    total_area_m2 = merged.area
    total_area_ha = total_area_m2 / 10000.0

    # Calculate quality metrics based on data density and consistency
    quality_score, estimated_error_m2 = calculate_quality_metrics(
        valid_points, valid_points, 0, total_area_m2
    )

    print(f"DEBUG: Quality score: {quality_score:.4f}, Error bounds: ±{estimated_error_m2:.2f} m²")

    return CalculationResult(
        total_area_m2=total_area_m2,
        total_area_ha=total_area_ha,
        geometry=merged,
        quality_score=quality_score,
        estimated_error_m2=estimated_error_m2,
        working_trajectory_points=len(valid_points),
        repositioning_points_filtered=0
    )



# =============================================================================
# MANUAL TEST
# =============================================================================

def run_manual_test():
    points = [
        PlanarPoint(0, 0, 5, True, True),
        PlanarPoint(20, 0, 5, True, True),
        PlanarPoint(0, 3, 5, True, True),
        PlanarPoint(20, 3, 5, True, True),
    ]

    strips = build_work_polygons(points, implement_width_m=4.0)
    merged = unary_union(strips)

    print("Total area (m²):", round(merged.area, 2))
    print("Total area (ha):", round(merged.area / 10000, 6))


if __name__ == "__main__":
    run_manual_test()
