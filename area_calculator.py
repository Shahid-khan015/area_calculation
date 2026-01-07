
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
    """
    Smooth planar points using exponential weighted moving average (Kalman-like).
    
    More responsive than simple moving average while still filtering noise.
    Alpha=0.3 gives good balance between responsiveness and smoothing.
    """
    if len(points) < 2:
        return points

    alpha = 0.3  # Smoothing factor (0 = max smoothing, 1 = no smoothing)
    smoothed = [points[0]]  # Keep first point as-is

    for i in range(1, len(points)):
        prev_smooth = smoothed[-1]
        curr = points[i]
        
        # Exponential weighted moving average
        new_x = alpha * curr.x + (1 - alpha) * prev_smooth.x
        new_y = alpha * curr.y + (1 - alpha) * prev_smooth.y
        
        smoothed.append(
            PlanarPoint(
                x=new_x,
                y=new_y,
                speed_kmh=curr.speed_kmh,  # Keep original speed
                engine_on=curr.engine_on,
                pto_on=curr.pto_on,
            )
        )
    
    return smoothed


def deduplicate_points(points: List[PlanarPoint], min_distance_m: float = 0.1) -> List[PlanarPoint]:
    """
    Remove consecutive points that are closer than min_distance_m.
    These are typically duplicate GPS readings or noise spikes.
    
    Args:
        points: Input points
        min_distance_m: Minimum distance to keep a point (default 0.1m)
    
    Returns:
        Deduplicated points
    """
    if len(points) < 2:
        return points
    
    deduplicated = [points[0]]
    
    for i in range(1, len(points)):
        curr = points[i]
        last = deduplicated[-1]
        
        distance = math.hypot(curr.x - last.x, curr.y - last.y)
        
        if distance >= min_distance_m:
            deduplicated.append(curr)
    
    return deduplicated


def validate_segment_velocity(p1: PlanarPoint, p2: PlanarPoint, 
                              max_accel_kmh_per_point: float = 5.0) -> bool:
    """
    Check if segment velocity change is realistic.
    Rejects segments with unrealistic acceleration (likely GPS jumps).
    
    Args:
        p1, p2: Consecutive points
        max_accel_kmh_per_point: Max realistic speed change between points
    
    Returns:
        True if segment is valid, False if it's an acceleration anomaly
    """
    speed_diff = abs(p2.speed_kmh - p1.speed_kmh)
    
    # If speed jumps too much, it's likely a GPS error, not real acceleration
    if speed_diff > max_accel_kmh_per_point:
        return False
    
    return True


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
    
    Improved metrics:
    1. Data density: points per 100m (min 10 for good coverage)
    2. Trajectory consistency: low std dev of speeds = reliable work
    3. Segment smoothness: low heading changes = clean path
    4. Coverage completeness: ratio of working to total points
    
    Returns:
        (quality_score: 0.0-1.0, estimated_error_m2: ±meters²)
    """
    if not working_points or not valid_points:
        return 0.0, area_m2 * 0.5
    
    # Factor 1: Data density
    total_distance = 0.0
    for i in range(len(working_points) - 1):
        dx = working_points[i + 1].x - working_points[i].x
        dy = working_points[i + 1].y - working_points[i].y
        total_distance += math.hypot(dx, dy)
    
    total_distance = max(total_distance, 100.0)  # Normalize to 100m
    points_per_100m = (len(working_points) / total_distance) * 100.0
    density_score = min(points_per_100m / 10.0, 1.0)  # 10+ pts/100m = perfect
    
    # Factor 2: Speed consistency (low variance = reliable work)
    if len(working_points) > 1:
        working_speeds = [p.speed_kmh for p in working_points]
        mean_speed = sum(working_speeds) / len(working_speeds)
        
        if mean_speed > 0.1:
            variance = sum((s - mean_speed) ** 2 for s in working_speeds) / len(working_speeds)
            std_dev = math.sqrt(variance)
            coeff_variation = std_dev / mean_speed  # Normalized std dev
            speed_consistency = max(1.0 - coeff_variation, 0.0)
        else:
            speed_consistency = 0.5
    else:
        speed_consistency = 0.5
    
    # Factor 3: Coverage completeness
    working_ratio = len(working_points) / max(len(valid_points), 1)
    coverage_score = working_ratio
    
    # Factor 4: Path smoothness (heading stability = well-planned field work)
    heading_changes = []
    for i in range(1, len(working_points) - 1):
        h1 = heading(working_points[i - 1], working_points[i])
        h2 = heading(working_points[i], working_points[i + 1])
        diff = abs(h2 - h1)
        diff = min(diff, 2 * math.pi - diff)
        heading_changes.append(diff)
    
    if heading_changes:
        mean_heading_change = sum(heading_changes) / len(heading_changes)
        smoothness = max(1.0 - (mean_heading_change / math.pi), 0.0)  # 0=random, 1=straight
    else:
        smoothness = 0.8
    
    # Combined quality score (weighted average)
    quality_score = (
        density_score * 0.25 +
        speed_consistency * 0.25 +
        coverage_score * 0.25 +
        smoothness * 0.25
    )
    
    # Estimated error based on quality and area size
    # Standard GPS horizontal accuracy: ±5m
    # Implement width measurement error: ±0.5m
    # Both scale inversely with quality
    base_error_m = 5.0 * (1.0 - quality_score)
    
    # Error grows with square root of area (coverage uncertainty)
    estimated_error_m2 = base_error_m * math.sqrt(area_m2 / 100.0)
    
    return quality_score, estimated_error_m2



def filter_and_project(records: List[SensorRecord],
                       min_speed: float) -> List[PlanarPoint]:
    """
    Filter records by work criteria (engine ON, PTO ON, adequate speed).
    Project to planar coordinates.
    Deduplicate very close points.
    Detect and remove heading jitter (GPS noise).
    Smooth remaining clean points with Kalman-like filter.
    
    Returns clean, validated planar points.
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

    print(f"DEBUG: Raw valid points: {len(valid)}")
    
    # Step 1: Remove duplicate/very close points (noise spikes)
    valid = deduplicate_points(valid, min_distance_m=0.1)
    print(f"DEBUG: After deduplication: {len(valid)} points")

    # Step 2: Apply heading jitter detection to remove GPS noise
    if len(valid) < 3:
        return valid
    
    jitter_filtered = [valid[0]]  # Keep first point
    
    for i in range(1, len(valid) - 1):
        # Only keep point i if it's NOT heading jitter
        if not is_heading_jitter(valid[i - 1], valid[i], valid[i + 1]):
            jitter_filtered.append(valid[i])
    
    jitter_filtered.append(valid[-1])  # Keep last point
    print(f"DEBUG: After jitter filtering: {len(jitter_filtered)} points")
    
    # Step 3: Smooth clean points with Kalman-like filter
    smoothed = smooth_planar_points(jitter_filtered)
    print(f"DEBUG: After smoothing: {len(smoothed)} points")
    
    return smoothed




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


def build_work_polygons(points: List[PlanarPoint], implement_width_m: float) -> List[Polygon]:
    """
    Build work strip polygons from trajectory points.
    
    Adaptive thresholds:
    - min_distance: Depends on implement width and GPS noise levels
    - max_distance: Rejects unrealistic GPS jumps
    - velocity validation: Rejects segments with impossible acceleration
    
    High-resolution buffer for smooth geometry.
    """
    polygons: List[Polygon] = []

    if len(points) < 2:
        return polygons

    # Adaptive distance thresholds
    # Minimum: at least 1/4 implement width (to capture narrow strips)
    # Maximum: realistic field pass distance (usually < 100m)
    min_distance = max(0.5, implement_width_m * 0.25)
    max_distance = 100.0  # Unrealistic GPS jump threshold
    
    skipped_count = 0
    accepted_count = 0
    
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]

        distance = math.hypot(p2.x - p1.x, p2.y - p1.y)

        # 1️⃣ Reject if too short (GPS noise)
        if distance < min_distance:
            skipped_count += 1
            continue

        # 2️⃣ Reject if too long (GPS jump/teleportation)
        if distance > max_distance:
            skipped_count += 1
            continue

        # 3️⃣ Reject if velocity doesn't make sense
        if not validate_segment_velocity(p1, p2):
            skipped_count += 1
            continue

        # 4️⃣ Reject if heading jitter (last check for outliers)
        if i > 0 and i < len(points) - 2:
            if is_heading_jitter(points[i - 1], p1, p2, max_angle_rad=math.radians(60)):
                skipped_count += 1
                continue

        # 5️⃣ Valid strip - create with high-resolution buffer
        line = LineString([(p1.x, p1.y), (p2.x, p2.y)])

        # High resolution (64 points per quadrant) for smooth boundaries
        # Bevel join for sharp corners, flat cap for clean line ends
        strip = line.buffer(
            implement_width_m / 2.0,
            resolution=64,
            cap_style=1,  # flat cap
            join_style=2  # bevel join
        )

        polygons.append(strip)
        accepted_count += 1

    print(f"DEBUG: Segments: {accepted_count} accepted, {skipped_count} rejected")
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
