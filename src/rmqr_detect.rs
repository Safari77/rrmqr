//! rMQR (Rectangular Micro QR Code) finder pattern detection
use crate::geometry::Perspective;
use crate::identify::Point;
use crate::prepare::{ImageBuffer, PreparedImage};
use crate::rmqr_version_db::{RMQR_VERSION_DATA, correct_rmqr_format_pattern, extract_format_data};

// ============================================================================
// CONFIGURABLE PARAMETERS
// ============================================================================

/// An rMQR finder pattern detected in an image
#[derive(Debug, Clone)]
pub struct RmqrFinderPattern {
    /// The 4 corners of the finder pattern
    pub corners: [Point; 4],
    /// The center point of the finder pattern
    pub center: Point,
    /// The local perspective of the pattern (maps 0.0..7.0 to the 7x7 finder extent)
    pub c: Perspective,
    /// Whether this is the main finder (true) or sub-finder (false)
    pub is_main_finder: bool,
    /// Estimated module size
    pub module_size: f64,
}

/// An rMQR finder sub-pattern (bottom-right corner)
#[derive(Debug, Clone)]
pub struct RmqrSubPattern {
    /// The 4 corners of the sub-pattern
    pub corners: [Point; 4],
    /// The center point
    pub center: Point,
    /// Estimated module size
    pub module_size: f64,
}

/// A detected rMQR code region
#[derive(Debug, Clone)]
pub struct RmqrRegion {
    /// Main finder pattern (top-left)
    pub main_finder: RmqrFinderPattern,
    /// Sub finder pattern (bottom-right)
    pub sub_finder: Option<RmqrSubPattern>,
    /// Estimated corners of the entire rMQR code [TL, TR, BR, BL]
    pub corners: [Point; 4],
    /// Estimated width in modules
    pub width: usize,
    /// Estimated height in modules
    pub height: usize,
}

pub fn find_rmqr_patterns<S>(
    _img: &mut PreparedImage<S>,
    capstones: &[crate::CapStone],
) -> Vec<RmqrFinderPattern>
where
    S: ImageBuffer,
{
    let mut results = Vec::new();

    crate::debug_log!(
        "[rMQR] Using {} capstones as potential main finder patterns",
        capstones.len()
    );

    for cap in capstones {
        // Recalculate perspective to ensure it maps 0..7, 0..7 to the corners.
        let c = Perspective::create(&cap.corners, 7.0, 7.0).unwrap_or_else(|| cap.c.clone());

        // Recalculate center based on this 7x7 mapping
        let p_center = c.map(3.5, 3.5);
        let center = Point { x: p_center.x, y: p_center.y };

        let dx_w = (cap.corners[1].x - cap.corners[0].x) as f64;
        let dy_w = (cap.corners[1].y - cap.corners[0].y) as f64;
        let width_px = dx_w.hypot(dy_w);

        let dx_h = (cap.corners[3].x - cap.corners[0].x) as f64;
        let dy_h = (cap.corners[3].y - cap.corners[0].y) as f64;
        let height_px = dx_h.hypot(dy_h);

        let mod_size = ((width_px + height_px) / 2.0) / 7.0;

        let main_finder = RmqrFinderPattern {
            corners: cap.corners,
            center,
            c,
            is_main_finder: true,
            module_size: mod_size,
        };
        results.push(main_finder);
    }

    results
}

pub fn match_rmqr_patterns<S>(
    img: &PreparedImage<S>,
    patterns: &[RmqrFinderPattern],
) -> Vec<RmqrRegion>
where
    S: ImageBuffer,
{
    let mut regions = Vec::new();

    for main in patterns {
        if !main.is_main_finder {
            continue;
        }

        crate::debug_log!(
            "[rMQR Detect] Processing finder at ({},{}) with corners: TL({},{}), TR({},{}), BR({},{}), BL({},{})",
            main.center.x,
            main.center.y,
            main.corners[0].x,
            main.corners[0].y,
            main.corners[1].x,
            main.corners[1].y,
            main.corners[2].x,
            main.corners[2].y,
            main.corners[3].x,
            main.corners[3].y
        );

        // 1. Probe Format Info in all 4 rotations
        let candidates = probe_format_info_multidir(img, main);

        if candidates.is_empty() {
            crate::debug_log!(
                "[rMQR] No valid format info found around capstone at ({},{})",
                main.center.x,
                main.center.y
            );
        }

        for (width, height, rotation_idx) in candidates {
            crate::debug_log!(
                "[rMQR] Main Finder implies Version: {}x{} (Rotation: {})",
                width,
                height,
                rotation_idx
            );

            // 2. Compute the correctly-oriented perspective for this rotation
            // The finder corners are [TL, TR, BR, BL] in the ORIGINAL image orientation.
            // After rotation, we need to map these to the rMQR coordinate system.
            //
            // rMQR coordinate system after rotation:
            // - (0, 0) is top-left of the code
            // - X increases to the right (width direction)
            // - Y increases downward (height direction)

            let rotated_corners = get_rotated_corners(&main.corners, rotation_idx);

            crate::debug_log!(
                "[rMQR Detect] Rotated corners (rot={}): TL({},{}), TR({},{}), BR({},{}), BL({},{})",
                rotation_idx,
                rotated_corners[0].x,
                rotated_corners[0].y,
                rotated_corners[1].x,
                rotated_corners[1].y,
                rotated_corners[2].x,
                rotated_corners[2].y,
                rotated_corners[3].x,
                rotated_corners[3].y
            );

            let c_rot = match Perspective::create(&rotated_corners, 7.0, 7.0) {
                Some(p) => p,
                None => {
                    crate::debug_log!(
                        "[rMQR Detect] Failed to create perspective for rotation {}",
                        rotation_idx
                    );
                    continue;
                }
            };

            // 3. Compute grid corners by projecting from the rotated finder
            // The finder occupies (0,0) to (7,7) in module space
            // The full grid extends from (0,0) to (width, height)
            //
            // IMPORTANT: We use the finder's perspective to extrapolate the full grid.
            // The unit vectors from the finder tell us how modules are spaced.

            // Calculate unit vectors from the rotated finder
            let p_00 = c_rot.map(0.0, 0.0);
            let p_70 = c_rot.map(7.0, 0.0);
            let p_07 = c_rot.map(0.0, 7.0);

            // Unit vector in X direction (per module)
            let ux = (p_70.x - p_00.x) as f64 / 7.0;
            let uy = (p_70.y - p_00.y) as f64 / 7.0;

            // Unit vector in Y direction (per module)
            let vx = (p_07.x - p_00.x) as f64 / 7.0;
            let vy = (p_07.y - p_00.y) as f64 / 7.0;

            crate::debug_log!(
                "[rMQR Detect] Unit vectors: u=({:.3}, {:.3}), v=({:.3}, {:.3})",
                ux,
                uy,
                vx,
                vy
            );

            // Extrapolate to full grid corners using unit vectors
            let origin_x = p_00.x as f64;
            let origin_y = p_00.y as f64;

            let p_tl = Point { x: origin_x.round() as i32, y: origin_y.round() as i32 };
            let p_tr = Point {
                x: (origin_x + (width as f64) * ux).round() as i32,
                y: (origin_y + (width as f64) * uy).round() as i32,
            };
            let p_br = Point {
                x: (origin_x + (width as f64) * ux + (height as f64) * vx).round() as i32,
                y: (origin_y + (width as f64) * uy + (height as f64) * vy).round() as i32,
            };
            let p_bl = Point {
                x: (origin_x + (height as f64) * vx).round() as i32,
                y: (origin_y + (height as f64) * vy).round() as i32,
            };

            let corners = [p_tl, p_tr, p_br, p_bl];

            // Validate corners are reasonable (all within image bounds or slightly outside)
            let img_w = img.width() as i32;
            let img_h = img.height() as i32;
            let margin = (main.module_size * 10.0) as i32;

            let corners_valid = corners.iter().all(|c| {
                c.x >= -margin && c.x <= img_w + margin && c.y >= -margin && c.y <= img_h + margin
            });

            if !corners_valid {
                crate::debug_log!(
                    "[rMQR] Rotation {} produces invalid corners, skipping",
                    rotation_idx
                );
                continue;
            }

            crate::debug_log!(
                "[rMQR] Corners for rotation {}: TL({},{}), TR({},{}), BR({},{}), BL({},{})",
                rotation_idx,
                corners[0].x,
                corners[0].y,
                corners[1].x,
                corners[1].y,
                corners[2].x,
                corners[2].y,
                corners[3].x,
                corners[3].y
            );

            // 4. Predict Sub-Finder Location using the unit vectors
            // Sub-finder is near bottom-right, centered around (width - 2.5, height - 2.5)
            let sub_x = origin_x + (width as f64 - 2.5) * ux + (height as f64 - 2.5) * vx;
            let sub_y = origin_y + (width as f64 - 2.5) * uy + (height as f64 - 2.5) * vy;

            crate::debug_log!("[rMQR] Sub-finder Estimation: ({:.2}, {:.2})", sub_x, sub_y);
            let virtual_sub = create_virtual_sub_pattern(sub_x, sub_y, main.module_size);

            let region = RmqrRegion {
                main_finder: main.clone(),
                sub_finder: Some(virtual_sub),
                corners,
                width,
                height,
            };
            regions.push(region);
        }
    }

    deduplicate_regions(regions)
}

/// Probe format info in all 4 rotations, return (width, height, rotation_index) for valid ones
fn probe_format_info_multidir<S>(
    img: &PreparedImage<S>,
    finder: &RmqrFinderPattern,
) -> Vec<(usize, usize, usize)>
where
    S: ImageBuffer,
{
    let mut candidates = Vec::new();

    for rotation in 0..4 {
        let rot_corners = get_rotated_corners(&finder.corners, rotation);

        // Create perspective for the rotated 7x7 finder
        let c_rot_opt = Perspective::create(&rot_corners, 7.0, 7.0);

        // Calculate unit vectors for extrapolating beyond the finder
        let p_tl = rot_corners[0];
        let p_tr = rot_corners[1];
        let p_bl = rot_corners[3];

        // Unit vectors per module (finder is 7x7)
        let ux = (p_tr.x - p_tl.x) as f64 / 7.0;
        let uy = (p_tr.y - p_tl.y) as f64 / 7.0;
        let vx = (p_bl.x - p_tl.x) as f64 / 7.0;
        let vy = (p_bl.y - p_tl.y) as f64 / 7.0;

        let origin_x = p_tl.x as f64;
        let origin_y = p_tl.y as f64;

        let mut read_bits = 0u32;
        let mut debug_bits = String::new();
        let mut first_probe_pos = (0.0, 0.0);
        let mut last_probe_pos = (0.0, 0.0);

        // Format info bit positions (relative to finder, module centers)
        // These positions are OUTSIDE the 7x7 finder area, so we need to extrapolate
        let probe_sequence = [
            (11, 3),
            (11, 2),
            (11, 1),
            (10, 5),
            (10, 4),
            (10, 3),
            (10, 2),
            (10, 1),
            (9, 5),
            (9, 4),
            (9, 3),
            (9, 2),
            (9, 1),
            (8, 5),
            (8, 4),
            (8, 3),
            (8, 2),
            (8, 1),
        ];

        for (i, &(col, row)) in probe_sequence.iter().enumerate() {
            // Module center coordinates
            let tx = col as f64 + 0.5;
            let ty = row as f64 + 0.5;

            // For positions outside the 7x7 finder, we MUST use extrapolation
            // The perspective transform may not be accurate for these positions
            // because it was fitted to only the 7x7 finder corners.
            //
            // Use linear extrapolation with unit vectors for better accuracy
            let (px, py) = if col <= 7 && row <= 7 {
                // Inside finder - can use perspective if available
                if let Some(ref c) = c_rot_opt {
                    let p = c.map(tx, ty);
                    (p.x as f64, p.y as f64)
                } else {
                    (origin_x + tx * ux + ty * vx, origin_y + tx * uy + ty * vy)
                }
            } else {
                // Outside finder - must use linear extrapolation
                (origin_x + tx * ux + ty * vx, origin_y + tx * uy + ty * vy)
            };

            if i == 0 {
                first_probe_pos = (px, py);
            }
            if i == 17 {
                last_probe_pos = (px, py);
            }

            let val = get_pixel_safe(img, px, py);
            read_bits = (read_bits << 1) | val;
            debug_bits.push(if val == 1 { '1' } else { '0' });
        }

        crate::debug_log!(
            "[rMQR] Rot {}: First(11,3)@({:.1},{:.1}) Last(8,1)@({:.1},{:.1}) Bits:{:05X} u=({:.2},{:.2}) v=({:.2},{:.2})",
            rotation,
            first_probe_pos.0,
            first_probe_pos.1,
            last_probe_pos.0,
            last_probe_pos.1,
            read_bits,
            ux,
            uy,
            vx,
            vy
        );

        if let Some(corrected) = correct_rmqr_format_pattern(read_bits, false) {
            let data = extract_format_data(corrected, false);
            let version_idx = (data & 0x1F) as usize;
            if version_idx < RMQR_VERSION_DATA.len() {
                let info = &RMQR_VERSION_DATA[version_idx];
                crate::debug_log!(
                    "[rMQR] Rot {} matched version {} ({}x{}), raw={:05X} corrected={:05X}",
                    rotation,
                    version_idx,
                    info.width,
                    info.height,
                    read_bits,
                    corrected
                );
                candidates.push((info.width, info.height, rotation));
            }
        }
    }

    candidates
}

/// Rotate the corner array to match different orientations
/// corners input: [TL, TR, BR, BL] in original image
///
/// The rotation reorders corners so that after rotation:
/// - corners[0] is the new top-left
/// - corners[1] is the new top-right
/// - etc.
fn get_rotated_corners(corners: &[Point; 4], rotation: usize) -> [Point; 4] {
    match rotation {
        0 => [corners[0], corners[1], corners[2], corners[3]],
        1 => [corners[3], corners[0], corners[1], corners[2]], // 90° CW
        2 => [corners[2], corners[3], corners[0], corners[1]], // 180°
        3 => [corners[1], corners[2], corners[3], corners[0]], // 270° CW (90° CCW)
        _ => *corners,
    }
}

fn get_pixel_safe<S>(img: &PreparedImage<S>, x: f64, y: f64) -> u32
where
    S: ImageBuffer,
{
    if x < 0.0 || y < 0.0 {
        return 0;
    }
    let ix = x as usize;
    let iy = y as usize;
    if ix >= img.width() || iy >= img.height() {
        return 0;
    }
    if img.get_pixel_byte(ix, iy) != 0 { 1 } else { 0 }
}

fn create_virtual_sub_pattern(x: f64, y: f64, ms: f64) -> RmqrSubPattern {
    let half_sz = (ms * 2.5).round() as i32;
    let cx = x.round() as i32;
    let cy = y.round() as i32;

    let corners = [
        Point { x: cx - half_sz, y: cy - half_sz },
        Point { x: cx + half_sz, y: cy - half_sz },
        Point { x: cx + half_sz, y: cy + half_sz },
        Point { x: cx - half_sz, y: cy + half_sz },
    ];

    RmqrSubPattern { corners, center: Point { x: cx, y: cy }, module_size: ms }
}

fn deduplicate_regions(regions: Vec<RmqrRegion>) -> Vec<RmqrRegion> {
    let mut unique = Vec::new();
    for r in regions {
        let is_dup = unique.iter().any(|u: &RmqrRegion| {
            let dist = (r.main_finder.center.x - u.main_finder.center.x).abs()
                + (r.main_finder.center.y - u.main_finder.center.y).abs();
            dist < 10 && r.width == u.width && r.height == u.height
        });
        if !is_dup {
            unique.push(r);
        }
    }
    unique
}

pub fn log_rmqr_detection(patterns: &[RmqrFinderPattern], regions: &[RmqrRegion]) {
    if !crate::debug::is_debug_enabled() {
        return;
    }
    crate::debug_log!(
        "[rMQR] Detection summary: {} patterns, {} regions",
        patterns.len(),
        regions.len()
    );
}
