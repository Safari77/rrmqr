use crate::CapStone;
use crate::geometry::Perspective;
use crate::identify::Point;
use crate::prepare::{ImageBuffer, PreparedImage};

use image::{GrayImage, Luma};
use imageproc::contours::{BorderType, find_contours_with_threshold};
use imageproc::geometry::convex_hull;
use imageproc::rect::Rect;

/// Internal struct to track candidates with their estimated size
#[derive(Debug, Clone, Copy)]
struct Candidate {
    point: Point,
    estimated_width: usize,
}

/// Main entry point for rMQR Finder Detection.
/// Returns accurate CapStones based on contour topology with arbitrary rotation support.
pub fn find_rmqr_capstones<S>(img: &PreparedImage<S>) -> Vec<CapStone>
where
    S: ImageBuffer,
{
    crate::debug_log!("[rMQR Finder] Starting scan...");
    let mut candidates = Vec::new();
    let w = img.width();
    let h = img.height();

    // 1. Line Scan (Horizontal & Vertical)
    let step = 3;

    // Horizontal Scan
    for y in (0..h).step_by(step) {
        let mut scanner = RmqrScanner::new(img.get_pixel_byte(0, y));
        for x in 1..w {
            if let Some((center_x, width)) = scanner.advance(img.get_pixel_byte(x, y), x) {
                candidates.push(Candidate {
                    point: Point { x: center_x as i32, y: y as i32 },
                    estimated_width: width,
                });
            }
        }
    }

    // Vertical Scan
    for x in (0..w).step_by(step) {
        let mut scanner = RmqrScanner::new(img.get_pixel_byte(x, 0));
        for y in 1..h {
            if let Some((center_y, width)) = scanner.advance(img.get_pixel_byte(x, y), y) {
                candidates.push(Candidate {
                    point: Point { x: x as i32, y: center_y as i32 },
                    estimated_width: width,
                });
            }
        }
    }

    crate::debug_log!("[rMQR Finder] Scanner found {} raw hits", candidates.len());

    // 2. Cluster Candidates
    // We group them to avoid running heavy contours on every scanline hit
    let clusters = cluster_candidates(&candidates, 20); // 20px radius tolerance

    crate::debug_log!("[rMQR Finder] Clustered into {} potential locations", clusters.len());

    // 3. Verify via Contours
    let mut verified_capstones = Vec::new();

    for (idx, cand) in clusters.iter().enumerate() {
        // Dynamic ROI calculation:
        // We need enough space to see the whole finder + quiet zone.
        // cand.estimated_width is approx 7 modules.
        // Let's take radius = width (so diameter = 2*width) to be safe.
        let roi_radius = (cand.estimated_width as i32).max(20);

        crate::debug_log!(
            "[rMQR Finder] Checking cluster #{} at ({},{}) size~={} roi_rad={}",
            idx,
            cand.point.x,
            cand.point.y,
            cand.estimated_width,
            roi_radius
        );

        let (roi_img, offset_x, offset_y) = crop_roi(img, cand.point, roi_radius);
        if let Some(cap) = process_roi_contours(&roi_img, offset_x, offset_y, idx) {
            crate::debug_log!("[rMQR Finder] -> CONFIRMED Capstone #{}", idx);
            verified_capstones.push(cap);
        } else {
            crate::debug_log!("[rMQR Finder] -> REJECTED cluster #{}", idx);
        }
    }

    verified_capstones
}

// --- CONTOUR PROCESSING ---

fn process_roi_contours(
    roi: &GrayImage,
    off_x: i32,
    off_y: i32,
    debug_id: usize,
) -> Option<CapStone> {
    let mut black_count: usize = 0;
    let total_pixels = (roi.width() * roi.height()) as usize;
    for p in roi.pixels() {
        if p[0] >= 1 {
            black_count += 1;
        }
    }
    let black_ratio = black_count as f32 / total_pixels as f32;
    if black_ratio < 0.2 || black_ratio > 0.8 {
        crate::debug_log!("[rMQR Finder #{}] Reject ROI: black_ratio={:.2}", debug_id, black_ratio);
        return None;
    }
    // 1. Find Contours in ROI
    // Note: crop_roi ensures we have inverted colors (Non-Zero = Shape) for Suzuki85
    let contours = find_contours_with_threshold(roi, 1);

    crate::debug_log!("[rMQR Finder #{}] Found {} contours in ROI", debug_id, contours.len());

    // 2. Search for 1:1:3:1:1 Topology
    // Black(Outer) -> White(Hole) -> Black(Inner)
    // In contours: BorderType::Outer -> BorderType::Hole -> BorderType::Outer (Solid)

    for (_, inner) in contours.iter().enumerate() {
        // Check Inner Black Box (3x3)
        if inner.border_type == BorderType::Hole {
            continue;
        }

        // Check Parent (White Gap)
        if let Some(mid_idx) = inner.parent {
            let mid = &contours[mid_idx];
            if mid.border_type == BorderType::Outer {
                continue; // Should be hole
            }

            // Check Grandparent (Outer Black Ring 7x7)
            if let Some(outer_idx) = mid.parent {
                let outer = &contours[outer_idx];
                if outer.border_type == BorderType::Hole {
                    continue; // Should be solid
                }

                // MATCH FOUND: Outer(Black) -> Middle(White) -> Inner(Black)
                let outer_poly = &outer.points;
                let inner_poly = &inner.points;

                if validate_geometry(outer_poly, inner_poly, debug_id) {
                    return Some(create_capstone_from_contour(outer_poly, off_x, off_y));
                }
            }
        }
    }
    None
}

fn validate_geometry(
    outer: &[imageproc::point::Point<i32>],
    inner: &[imageproc::point::Point<i32>],
    debug_id: usize,
) -> bool {
    let out_rect = get_bbox(outer);
    let in_rect = get_bbox(inner);

    if out_rect.width() == 0 || out_rect.height() == 0 || in_rect.width() == 0 {
        return false;
    }

    let w_out = out_rect.width() as f32;
    let h_out = out_rect.height() as f32;

    // 1. Aspect Ratio (roughly 1.0)
    let ar = w_out / h_out;
    if ar < 0.75 || ar > 1.25 {
        crate::debug_log!(
            "[rMQR Finder #{}] Reject: Aspect Ratio {:.2} (Target 1.0)",
            debug_id,
            ar
        );
        return false;
    }

    // 2. Size Ratio (Outer 7 modules / Inner 3 modules ~= 2.33)
    let size_ratio = w_out / in_rect.width() as f32;
    if size_ratio < 1.75 || size_ratio > 2.92 {
        crate::debug_log!(
            "[rMQR Finder #{}] Reject: Module Ratio {:.2} (Target 2.33)",
            debug_id,
            size_ratio
        );
        return false;
    }

    // 3. Centroid Alignment
    let c_out = get_centroid(outer);
    let c_in = get_centroid(inner);
    let dist = ((c_out.0 - c_in.0).powi(2) + (c_out.1 - c_in.1).powi(2)).sqrt();

    // Centers should be close (within 1.5 module)
    let mod_size = w_out / 7.0;
    if dist > mod_size * 2.0 {
        crate::debug_log!("[rMQR Finder #{}] Reject: Center Misalignment {:.2}px", debug_id, dist);
        return false;
    }

    true
}

/// Creates a CapStone from the outer contour using Min-Area-Rect to support rotation.
fn create_capstone_from_contour(
    points: &[imageproc::point::Point<i32>],
    off_x: i32,
    off_y: i32,
) -> CapStone {
    // 1. Calculate the Minimum Area Rectangle (Arbitrary Rotation)
    let rotated_corners_local = min_area_rect(points);

    // 2. Convert to global coordinates
    let mut corners = [Point::default(); 4];
    for (i, p) in rotated_corners_local.iter().enumerate() {
        corners[i] = Point { x: p.x + off_x, y: p.y + off_y };
    }

    // 3. Order Corners: TL -> TR -> BR -> BL
    // We sort by Y first, then X to identify Top/Bottom, Left/Right
    // A simple heuristic for 45 deg rotation is to sort by angle from centroid.
    let center_x: f64 = corners.iter().map(|p| p.x as f64).sum::<f64>() / 4.0;
    let center_y: f64 = corners.iter().map(|p| p.y as f64).sum::<f64>() / 4.0;

    corners.sort_by(|a, b| {
        // Sort by angle around center
        let ang_a = (a.y as f64 - center_y).atan2(a.x as f64 - center_x);
        let ang_b = (b.y as f64 - center_y).atan2(b.x as f64 - center_x);
        ang_a.partial_cmp(&ang_b).unwrap()
    });

    // 4. Create Perspective Mapping (Homography)
    // We Map (0,0)->TL ... (7,7)->BR
    let c = Perspective::create(&corners, 7.0, 7.0).unwrap_or_default();

    let center = Point { x: center_x.round() as i32, y: center_y.round() as i32 };

    CapStone { corners, center, c }
}

// --- SCANNER IMPLEMENTATION ---

struct RmqrScanner {
    // Ring buffer for run lengths: [White, Black, White, Black, White]
    lookbehind_buf: [usize; 5],
    last_color: u8,
    run_length: usize,
    color_changes: usize,
    current_position: usize,
}

impl RmqrScanner {
    fn new(initial_byte: u8) -> Self {
        RmqrScanner {
            lookbehind_buf: [0; 5],
            last_color: initial_byte,
            run_length: 1,
            color_changes: 0,
            current_position: 0,
        }
    }

    /// Advance the scanner state. Returns Some((center_pos, total_width)) if 1:1:3:1:1 pattern found.
    #[inline(always)]
    fn advance(&mut self, pixel_byte: u8, pos: usize) -> Option<(usize, usize)> {
        self.current_position = pos;

        // No color change - just extend current run
        if self.last_color == pixel_byte {
            self.run_length += 1;
            return None;
        }

        // Color changed: Push to buffer
        self.last_color = pixel_byte;
        self.rotate_buffer();
        self.lookbehind_buf[4] = self.run_length;
        self.run_length = 1;
        self.color_changes += 1;

        if self.color_changes >= 5 {
            return self.test_pattern();
        }
        None
    }

    fn rotate_buffer(&mut self) {
        let tmp = self.lookbehind_buf[0];
        self.lookbehind_buf[0] = self.lookbehind_buf[1];
        self.lookbehind_buf[1] = self.lookbehind_buf[2];
        self.lookbehind_buf[2] = self.lookbehind_buf[3];
        self.lookbehind_buf[3] = self.lookbehind_buf[4];
        self.lookbehind_buf[4] = tmp;
    }

    fn test_pattern(&self) -> Option<(usize, usize)> {
        // We look for 1:1:3:1:1
        let b1 = self.lookbehind_buf[0];
        let w1 = self.lookbehind_buf[1];
        let b_center = self.lookbehind_buf[2];
        let w2 = self.lookbehind_buf[3];
        let b2 = self.lookbehind_buf[4];

        // Total width in pixels
        let total_width = b1 + w1 + b_center + w2 + b2;
        // Expected unit size (total / 7 modules)
        let unit_size = total_width as f32 / 7.0;

        // Define tolerance (e.g., 50% deviation allowed)
        let tolerance = unit_size * 0.75;

        let check_run = |val: usize, expected_mod: f32| -> bool {
            let diff = (val as f32 - (expected_mod * unit_size)).abs();
            diff < tolerance * expected_mod
        };

        // We only care if the *buffer* represents B-W-B-W-B.
        if self.last_color != 0 {
            return None; // We just finished a White run, so the sequence ends in White.
        }

        let is_candidate = check_run(b1, 1.0)
            && check_run(w1, 1.0)
            && check_run(b_center, 3.0)
            && check_run(w2, 1.0)
            && check_run(b2, 1.0);

        if is_candidate {
            // Center of b_center (the 3x module)
            let center_idx = self.current_position - b2 - w2 - (b_center / 2);
            return Some((center_idx, total_width));
        }

        None
    }
}

// --- UTILS ---

fn crop_roi<S: ImageBuffer>(
    img: &PreparedImage<S>,
    center: Point,
    radius: i32,
) -> (GrayImage, i32, i32) {
    let x_start = (center.x - radius).max(0);
    let y_start = (center.y - radius).max(0);
    // Ensure we don't go out of bounds
    let width = ((radius * 2).min(img.width() as i32 - x_start)) as u32;
    let height = ((radius * 2).min(img.height() as i32 - y_start)) as u32;

    let mut gray = GrayImage::new(width, height);

    for y in 0..height {
        for x in 0..width {
            let src_x = (x_start as u32 + x) as usize;
            let src_y = (y_start as u32 + y) as usize;

            // INVERTED LOGIC for Suzuki85:
            // PreparedImage: 1=Black, 0=White.
            // Contour Finder: Non-Zero=Shape.
            // We want to find the Black Finder Pattern. So Black(1) -> 255.
            let val = if img.get_pixel_byte(src_x, src_y) == 1 { 255 } else { 0 };
            gray.put_pixel(x, y, Luma([val]));
        }
    }
    (gray, x_start, y_start)
}

fn cluster_candidates(candidates: &[Candidate], radius: i32) -> Vec<Candidate> {
    let mut clusters = Vec::new();
    let mut visited = vec![false; candidates.len()];

    for i in 0..candidates.len() {
        if visited[i] {
            continue;
        }
        let c = candidates[i];
        let mut sum_x = c.point.x as i64;
        let mut sum_y = c.point.y as i64;
        let mut max_width = c.estimated_width;
        let mut count = 1;
        visited[i] = true;

        for j in (i + 1)..candidates.len() {
            if visited[j] {
                continue;
            }
            let c2 = candidates[j];
            if (c.point.x - c2.point.x).abs() < radius && (c.point.y - c2.point.y).abs() < radius {
                sum_x += c2.point.x as i64;
                sum_y += c2.point.y as i64;
                if c2.estimated_width > max_width {
                    max_width = c2.estimated_width;
                }
                count += 1;
                visited[j] = true;
            }
        }
        clusters.push(Candidate {
            point: Point { x: (sum_x / count) as i32, y: (sum_y / count) as i32 },
            estimated_width: max_width,
        });
    }
    clusters
}

fn get_bbox(points: &[imageproc::point::Point<i32>]) -> Rect {
    if points.is_empty() {
        return Rect::at(0, 0).of_size(1, 1);
    }
    let min_x = points.iter().map(|p| p.x).min().unwrap_or(0);
    let max_x = points.iter().map(|p| p.x).max().unwrap_or(0);
    let min_y = points.iter().map(|p| p.y).min().unwrap_or(0);
    let max_y = points.iter().map(|p| p.y).max().unwrap_or(0);

    let w = (max_x - min_x).max(1) as u32;
    let h = (max_y - min_y).max(1) as u32;

    Rect::at(min_x, min_y).of_size(w, h)
}

fn get_centroid(points: &[imageproc::point::Point<i32>]) -> (f32, f32) {
    let mut sx = 0.0;
    let mut sy = 0.0;
    for p in points {
        sx += p.x as f32;
        sy += p.y as f32;
    }
    let len = points.len() as f32;
    if len == 0.0 {
        return (0.0, 0.0);
    }
    (sx / len, sy / len)
}

// --- GEOMETRY HELPERS (Rotating Calipers) ---

/// Finds the Minimum Area Rectangle for a set of points (Arbitrary Rotation)
/// Returns the 4 corners of the rectangle.
fn min_area_rect(points: &[imageproc::point::Point<i32>]) -> [imageproc::point::Point<i32>; 4] {
    // 1. Compute Convex Hull
    let hull = convex_hull(points);
    if hull.len() < 3 {
        // Degenerate case, return bbox
        let r = get_bbox(points);
        return [
            imageproc::point::Point::new(r.left(), r.top()),
            imageproc::point::Point::new(r.right(), r.top()),
            imageproc::point::Point::new(r.right(), r.bottom()),
            imageproc::point::Point::new(r.left(), r.bottom()),
        ];
    }

    // 2. Rotating Calipers
    // To keep it simple and dependency-free, we will iterate over every edge of the hull
    // and use it as a candidate axis for the rectangle base.
    // (This is O(N^2) relative to hull size, which is small, so it's fast enough).

    let mut min_area = f64::MAX;
    let mut best_corners = [imageproc::point::Point::new(0, 0); 4];

    for i in 0..hull.len() {
        let p1 = hull[i];
        let p2 = hull[(i + 1) % hull.len()];

        // Edge vector
        let dx = (p2.x - p1.x) as f64;
        let dy = (p2.y - p1.y) as f64;
        let edge_len = (dx * dx + dy * dy).sqrt();
        if edge_len == 0.0 {
            continue;
        }

        // Normalized axes
        let u = (dx / edge_len, dy / edge_len); // Parallel
        let v = (-u.1, u.0); // Perpendicular

        // Project all points onto axes
        let mut min_u = f64::MAX;
        let mut max_u = f64::MIN;
        let mut min_v = f64::MAX;
        let mut max_v = f64::MIN;

        for p in &hull {
            let px = p.x as f64;
            let py = p.y as f64;
            let proj_u = px * u.0 + py * u.1;
            let proj_v = px * v.0 + py * v.1;

            if proj_u < min_u {
                min_u = proj_u;
            }
            if proj_u > max_u {
                max_u = proj_u;
            }
            if proj_v < min_v {
                min_v = proj_v;
            }
            if proj_v > max_v {
                max_v = proj_v;
            }
        }

        let area = (max_u - min_u) * (max_v - min_v);

        if area < min_area {
            min_area = area;
            // point = proj_u * U + proj_v * V
            let to_point = |pu: f64, pv: f64| -> imageproc::point::Point<i32> {
                let x = pu * u.0 + pv * v.0;
                let y = pu * u.1 + pv * v.1;
                imageproc::point::Point::new(x.round() as i32, y.round() as i32)
            };

            best_corners = [
                to_point(min_u, min_v),
                to_point(max_u, min_v),
                to_point(max_u, max_v),
                to_point(min_u, max_v),
            ];
        }
    }

    best_corners
}
