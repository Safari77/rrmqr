//! rMQR (Rectangular Micro QR Code) grid handling
//!
//! This module implements pattern-following grid construction.
//! Key principles:
//! - Geometry First: All decoding failures start as geometry failures
//! - Pattern Following: Timing patterns are ground truth, grid derived from them
//! - Piecewise Affine: Wide symbols divided into bands, each with own affine transform
//! - Grayscale Sampling: Binarize only after module sampling

use crate::geometry::Perspective;
use crate::identify::Point;
use crate::prepare::{ImageBuffer, PreparedImage};
use crate::rmqr_decode::RectBitGrid;
use crate::rmqr_detect::RmqrRegion;
use crate::rmqr_version_db::get_alignment_positions;

// ============================================================================
// CONFIGURABLE PARAMETERS
// ============================================================================

/// Minimum module size in pixels below which we use enhanced sampling
const MIN_MODULE_SIZE_FOR_SIMPLE_SAMPLE: f64 = 15.0;

/// Number of pixels to sample in disk/cross pattern for module decision
const SAMPLE_DISK_RADIUS: f64 = 0.3;

/// Band width in modules for piecewise affine grid
const BAND_WIDTH_MODULES: usize = 3;

/// Sub-finder search tolerance multiplier for module size
const SUB_FINDER_SEARCH_RANGE_MODULES: f64 = 8.0;

/// Sub-finder acceptance threshold
const SUB_FINDER_ACCEPTANCE_THRESHOLD: f64 = 0.7;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/// A located rMQR grid in an image
#[derive(Debug, Clone)]
pub struct RmqrGridLocation {
    pub width: usize,
    pub height: usize,
    pub perspective: Perspective,
    pub corners: [Point; 4],
    /// Timing column centers extracted from the image (x coordinates in image space)
    pub timing_centers: Vec<f64>,
    /// Module size estimate
    pub module_size: f64,
    /// Band affine transforms for piecewise sampling
    pub bands: Vec<BandTransform>,
}

/// Affine transform for a vertical band of the symbol
#[derive(Debug, Clone)]
pub struct BandTransform {
    /// Starting column in symbol space (inclusive)
    pub col_start: usize,
    /// Ending column in symbol space (exclusive)
    pub col_end: usize,
    /// Origin in image space (top-left of band)
    pub origin_x: f64,
    pub origin_y: f64,
    /// Unit vectors for column and row directions
    pub ux: f64,
    pub uy: f64,
    pub vx: f64,
    pub vy: f64,
}

impl BandTransform {
    /// Map symbol coordinates (col, row) to image coordinates (x, y)
    #[inline]
    pub fn map(&self, col: f64, row: f64) -> (f64, f64) {
        let local_col = col - self.col_start as f64;
        let x = self.origin_x + local_col * self.ux + row * self.vx;
        let y = self.origin_y + local_col * self.uy + row * self.vy;
        (x, y)
    }
}

// ============================================================================
// GRID LOCATION CREATION
// ============================================================================

impl RmqrGridLocation {
    /// Create grid using PATTERN-FOLLOWING approach
    ///
    /// Key principle: Do NOT use projection to infer module positions.
    /// Instead, find actual pattern centers in the image and derive grid from them.
    pub fn from_region<S>(img: &PreparedImage<S>, region: &RmqrRegion) -> Option<Self>
    where
        S: ImageBuffer,
    {
        crate::debug_log!(
            "[rMQR Grid] Creating grid {}x{} using PATTERN-FOLLOWING",
            region.width,
            region.height
        );

        // STEP 1: Get the main finder center (this is reliable from capstone detection)
        // The finder is 7x7 modules, its center is at module (3.5, 3.5)
        let finder_center = region.main_finder.center;
        crate::debug_log!(
            "[rMQR Grid] Main finder center: ({}, {})",
            finder_center.x,
            finder_center.y
        );

        // STEP 2: Estimate module size from finder (7x7 modules)
        let finder_corners = &region.main_finder.corners;
        let dx = (finder_corners[2].x - finder_corners[0].x) as f64;
        let dy = (finder_corners[2].y - finder_corners[0].y) as f64;
        let finder_diagonal = dx.hypot(dy);
        // Diagonal of 7x7 = 7*sqrt(2) modules
        let module_size_est = finder_diagonal / (7.0 * std::f64::consts::SQRT_2);

        crate::debug_log!("[rMQR Grid] Module size estimate from finder: {:.2}px", module_size_est);

        // DEBUG: Log the region corners for perspective analysis
        crate::debug_log!(
            "[rMQR Grid] Region corners: TL({},{}), TR({},{}), BR({},{}), BL({},{})",
            region.corners[0].x,
            region.corners[0].y,
            region.corners[1].x,
            region.corners[1].y,
            region.corners[2].x,
            region.corners[2].y,
            region.corners[3].x,
            region.corners[3].y
        );

        // STEP 3: Search for actual sub-finder center to refine the geometry.
        // The detector provides a good hull, but the Sub-Finder position is the
        // precise "anchor" for the end of the QR code.
        let sub_est = region.sub_finder.as_ref().map(|s| s.center).unwrap_or(Point { x: 0, y: 0 });

        crate::debug_log!(
            "[rMQR Grid] Estimated sub-finder position: ({}, {})",
            sub_est.x,
            sub_est.y
        );

        let actual_sub_center =
            search_for_sub_finder_center(img, sub_est.x as f64, sub_est.y as f64, module_size_est);

        crate::debug_log!(
            "[rMQR Grid] Actual sub-finder center found: ({:.1}, {:.1})",
            actual_sub_center.0,
            actual_sub_center.1
        );

        // Calculate the vector offset between estimated and actual sub-finder
        let shift_x = actual_sub_center.0 - sub_est.x as f64;
        let shift_y = actual_sub_center.1 - sub_est.y as f64;

        crate::debug_log!("[rMQR Grid] Sub-finder shift: ({:.1}, {:.1})", shift_x, shift_y);

        // Apply this shift to the Right-Side corners (TR and BR).
        // This preserves the detector's perspective "shape" (trapezoid height difference)
        // while correcting the length/width to match the actual pattern.
        let mut refined_corners = region.corners;

        // TR is corner 1
        refined_corners[1].x = (refined_corners[1].x as f64 + shift_x).round() as i32;
        refined_corners[1].y = (refined_corners[1].y as f64 + shift_y).round() as i32;

        // BR is corner 2
        refined_corners[2].x = (refined_corners[2].x as f64 + shift_x).round() as i32;
        refined_corners[2].y = (refined_corners[2].y as f64 + shift_y).round() as i32;

        if shift_x.abs() > 2.0 || shift_y.abs() > 2.0 {
            crate::debug_log!(
                "[rMQR Grid] Refined corners by shift ({:.1}, {:.1}) based on Sub-Finder",
                shift_x,
                shift_y
            );
            crate::debug_log!(
                "[rMQR Grid] Refined corners: TL({},{}), TR({},{}), BR({},{}), BL({},{})",
                refined_corners[0].x,
                refined_corners[0].y,
                refined_corners[1].x,
                refined_corners[1].y,
                refined_corners[2].x,
                refined_corners[2].y,
                refined_corners[3].x,
                refined_corners[3].y
            );
        }

        // STEP 4: Create Perspective from REFINED corners.
        let perspective = match Perspective::create(
            &refined_corners,
            region.width as f64,
            region.height as f64,
        ) {
            Some(p) => p,
            None => {
                crate::debug_log!("[rMQR Grid] Failed to create perspective from refined corners");
                return None;
            }
        };

        // DEBUG: Verify perspective by mapping some known points
        let p00 = perspective.map(0.0, 0.0);
        let p_w0 = perspective.map(region.width as f64, 0.0);
        let p_wh = perspective.map(region.width as f64, region.height as f64);
        let p0h = perspective.map(0.0, region.height as f64);
        crate::debug_log!(
            "[rMQR Grid] Perspective verification: (0,0)->({:.1},{:.1}), (w,0)->({:.1},{:.1}), (w,h)->({:.1},{:.1}), (0,h)->({:.1},{:.1})",
            p00.x,
            p00.y,
            p_w0.x,
            p_w0.y,
            p_wh.x,
            p_wh.y,
            p0h.x,
            p0h.y
        );

        // Calculate average module size for thresholding
        let pixel_dx = actual_sub_center.0 - finder_center.x as f64;
        let pixel_dy = actual_sub_center.1 - finder_center.y as f64;
        let sub_module_x = region.width as f64 - 2.5;
        let sub_module_y = region.height as f64 - 2.5;
        let module_dx = sub_module_x - 3.5;
        let module_dy = sub_module_y - 3.5;

        let pixel_dist = pixel_dx.hypot(pixel_dy);
        let module_dist = module_dx.hypot(module_dy);
        let module_size =
            if module_dist > 0.0 { pixel_dist / module_dist } else { module_size_est };

        crate::debug_log!("[rMQR Grid] Measured module size (avg): {:.2}px", module_size);

        // STEP 5: Create bands derived from the PERSPECTIVE.
        // This handles perspective distortion correctly by sampling the perspective transform
        // locally, while the corner refinement ensures the bands span the correct total width.
        let bands = create_simple_bands_from_perspective(
            region.width,
            region.height,
            &perspective,
            module_size,
        );

        crate::debug_log!("[rMQR Grid] Created {} bands from perspective", bands.len());

        Some(RmqrGridLocation {
            width: region.width,
            height: region.height,
            perspective,
            corners: refined_corners,
            timing_centers: Vec::new(),
            module_size,
            bands,
        })
    }

    pub fn into_grid_image<S>(self, img: &PreparedImage<S>) -> RmqrRefGridImage<'_, S> {
        RmqrRefGridImage { grid: self, img }
    }
}

// ============================================================================
// PATTERN-FOLLOWING HELPER FUNCTIONS
// ============================================================================

/// Search for the actual sub-finder center using Optimized Sparse Sampling.
fn search_for_sub_finder_center<S>(
    img: &PreparedImage<S>,
    est_x: f64,
    est_y: f64,
    module_size: f64,
) -> (f64, f64)
where
    S: ImageBuffer,
{
    // Stepping by 1/4 module size guarantees we land within the "sweet spot" (center 50%)
    // of the modules at least once.
    let search_step = (module_size / 4.0).max(1.0);

    // Search Area - increased for perspective-distorted images
    let search_range = module_size * SUB_FINDER_SEARCH_RANGE_MODULES;
    let start_x = (est_x - search_range).max(0.0);
    let start_y = (est_y - search_range).max(0.0);
    let end_x = (est_x + search_range).min(img.width() as f64 - 1.0);
    let end_y = (est_y + search_range).min(img.height() as f64 - 1.0);

    crate::debug_log!(
        "[rMQR Grid] Sub-finder search: range=({:.0},{:.0}) to ({:.0},{:.0}), step={:.1}",
        start_x,
        start_y,
        end_x,
        end_y,
        search_step
    );

    let mut best_x = est_x;
    let mut best_y = est_y;
    let mut best_score = 0.0;
    let mut candidates_found = 0;

    // Search Loop
    let mut y = start_y;
    while y <= end_y {
        let mut x = start_x;
        while x <= end_x {
            // Fast-Fail Check
            if is_pixel_black(img, x, y) {
                let score = score_sub_finder_fast(img, x, y, module_size);

                if score > 0.5 {
                    candidates_found += 1;
                }

                if score > best_score {
                    best_score = score;
                    best_x = x;
                    best_y = y;
                }
            }
            x += search_step;
        }
        y += search_step;
    }

    crate::debug_log!(
        "[rMQR Grid] Sub-finder search found {} candidates, best score: {:.3} at ({:.1}, {:.1})",
        candidates_found,
        best_score,
        best_x,
        best_y
    );

    // Refinement - lowered threshold for perspective-distorted images
    if best_score > SUB_FINDER_ACCEPTANCE_THRESHOLD {
        let (cx, cy) = find_black_centroid(img, best_x, best_y, module_size);
        crate::debug_log!(
            "[rMQR Fast] Sub-finder Match: ({:.1}, {:.1}) Score: {:.4} -> Refined: ({:.1}, {:.1})",
            best_x,
            best_y,
            best_score,
            cx,
            cy
        );
        return (cx, cy);
    } else {
        crate::debug_log!(
            "[rMQR Fast] Sub-finder Match: best_score {} too low (threshold {})",
            best_score,
            SUB_FINDER_ACCEPTANCE_THRESHOLD
        );
    }

    (est_x, est_y)
}

/// Focuses on the "Center 50%" of each module to ignore boundary aliasing.
fn score_sub_finder_fast<S>(img: &PreparedImage<S>, cx: f64, cy: f64, ms: f64) -> f64
where
    S: ImageBuffer,
{
    let mut center_black = 0;
    let mut center_total = 0;
    let mut inner_black = 0;
    let mut inner_total = 0;
    let mut outer_black = 0;
    let mut outer_total = 0;
    let mut quiet_black = 0;
    let mut quiet_total = 0;

    // Look at the inner 50% of the module to avoid edge noise.
    // We step by ms/3, giving us a 3x3 grid (9 points) per module.
    let sample_margin = ms * 0.25;
    let scan_step = (ms / 3.0).max(2.0);

    // Iterate over relevant modules (7x7 grid: Pattern + Quiet Zones)
    // Sub-finder is 5x5: BBBBB, BWWWB, BWBWB, BWWWB, BBBBB
    // Quiet zone extends beyond
    for row in -2i32..=4i32 {
        for col in -2i32..=4i32 {
            // Filter: Only check relevant L-shape areas
            let in_finder = (-2..=2).contains(&row) && (-2..=2).contains(&col);
            let in_quiet_r = (col == 3 || col == 4) && (-2..=2).contains(&row);
            let in_quiet_b = (row == 3 || row == 4) && (-2..=2).contains(&col);

            if !in_finder && !in_quiet_r && !in_quiet_b {
                continue;
            }

            // Module Center
            let mod_cx = cx + (col as f64 * ms);
            let mod_cy = cy + (row as f64 * ms);

            // Sampling Box (Inner 50%)
            let start_px = (mod_cx - ms * 0.5 + sample_margin).floor();
            let end_px = (mod_cx + ms * 0.5 - sample_margin).ceil();
            let start_py = (mod_cy - ms * 0.5 + sample_margin).floor();
            let end_py = (mod_cy + ms * 0.5 - sample_margin).ceil();

            // Sparse Scan Loop
            let mut py = start_py;
            while py < end_py {
                let mut px = start_px;
                while px < end_px {
                    // Safe Check
                    if px >= 0.0 && py >= 0.0 && px < img.width() as f64 && py < img.height() as f64
                    {
                        let is_black = img.get_pixel_byte(px as usize, py as usize) != 0;

                        if in_finder {
                            let dist = row.abs().max(col.abs());
                            if dist == 0 {
                                center_total += 1;
                                if is_black {
                                    center_black += 1;
                                }
                            } else if dist == 1 {
                                inner_total += 1;
                                if is_black {
                                    inner_black += 1;
                                }
                            } else {
                                // dist == 2
                                outer_total += 1;
                                if is_black {
                                    outer_black += 1;
                                }
                            }
                        } else {
                            quiet_total += 1;
                            if is_black {
                                quiet_black += 1;
                            }
                        }
                    }
                    px += scan_step;
                }
                py += scan_step;
            }
        }
    }

    if center_total == 0 || inner_total == 0 || outer_total == 0 || quiet_total == 0 {
        return 0.0;
    }

    // Ratios
    let r_center = center_black as f64 / center_total as f64;
    let r_inner = inner_black as f64 / inner_total as f64;
    let r_outer = outer_black as f64 / outer_total as f64;
    let r_quiet = quiet_black as f64 / quiet_total as f64;

    // IMPROVED: More tolerant thresholds for perspective-distorted images
    if r_center < 0.4 {
        return 0.0;
    } // Center must be mostly black
    if r_inner > 0.5 {
        return 0.0;
    } // Inner must be mostly white
    if r_outer < 0.4 {
        return 0.0;
    } // Outer must be mostly black
    if r_quiet > 0.5 {
        return 0.0;
    } // Quiet must be mostly white

    // Final Normalized Score
    (r_center + (1.0 - r_inner) + r_outer + (1.0 - r_quiet)) / 4.0
}

fn find_black_centroid<S>(img: &PreparedImage<S>, x: f64, y: f64, module_size: f64) -> (f64, f64)
where
    S: ImageBuffer,
{
    let r = module_size * 0.6;
    let sx_min = (x - r).floor() as usize;
    let sx_max = (x + r).ceil() as usize;
    let sy_min = (y - r).floor() as usize;
    let sy_max = (y + r).ceil() as usize;

    let mut tx = 0.0;
    let mut ty = 0.0;
    let mut tm = 0.0;

    for py in sy_min..=sy_max {
        for px in sx_min..=sx_max {
            if px < img.width() && py < img.height() && img.get_pixel_byte(px, py) > 0 {
                tx += px as f64;
                ty += py as f64;
                tm += 1.0;
            }
        }
    }
    if tm > 0.0 { (tx / tm + 0.5, ty / tm + 0.5) } else { (x, y) }
}

#[inline(always)]
fn is_pixel_black<S>(img: &PreparedImage<S>, x: f64, y: f64) -> bool
where
    S: ImageBuffer,
{
    if x < 0.0 || y < 0.0 {
        return false;
    }
    let ix = x as usize;
    let iy = y as usize;
    if ix >= img.width() || iy >= img.height() {
        return false;
    }
    img.get_pixel_byte(ix, iy) != 0
}

// ============================================================================
// BAND CREATION
// ============================================================================

fn create_band_from_perspective(
    col_start: usize,
    col_end: usize,
    height: usize,
    perspective: &Perspective,
    _module_size: f64,
) -> BandTransform {
    // IMPORTANT: We need to compute transforms such that when we later call
    // map(col + 0.5, row + 0.5), we get the CENTER of module (col, row).
    //
    // The perspective maps integer coordinates to grid corners:
    // perspective.map(0, 0) = top-left corner of module (0,0)
    // perspective.map(1, 0) = top-left corner of module (1,0) = top-right corner of module (0,0)
    //
    // So perspective.map(col + 0.5, row + 0.5) gives the CENTER of module (col, row).
    //
    // For BandTransform, we want:
    // band.map(col + 0.5, row + 0.5) = perspective.map(col + 0.5, row + 0.5)
    //
    // With band.map implementation:
    // x = origin_x + (col + 0.5 - col_start) * ux + (row + 0.5) * vx
    //
    // For this to equal perspective.map(col + 0.5, row + 0.5), we set:
    // origin = perspective.map(col_start, 0)  (the corner)
    // Then band.map(col_start + 0.5, 0.5) = origin + 0.5*ux + 0.5*vx
    //                                      = perspective.map(col_start, 0) + 0.5*ux + 0.5*vx
    //
    // This should equal perspective.map(col_start + 0.5, 0.5).
    // The affine approximation assumes:
    // perspective.map(col_start + 0.5, 0.5) ≈ perspective.map(col_start, 0) + 0.5*ux + 0.5*vx
    //
    // This is accurate for affine transforms, but perspective can have error.
    // For better accuracy, we compute unit vectors from module centers directly.

    // Get reference points using MODULE CENTERS for better accuracy
    let p_center_start = perspective.map(col_start as f64 + 0.5, 0.5);
    let p_center_end = perspective.map(col_end as f64 - 0.5, 0.5);
    let p_center_bottom = perspective.map(col_start as f64 + 0.5, height as f64 - 0.5);

    let cols = (col_end - col_start) as f64;
    let rows = height as f64;

    // Unit vector in column direction (from center of first module to center of last module)
    // We have (cols - 1) intervals between first and last module centers
    let col_intervals = (cols - 1.0).max(1.0);
    let ux = (p_center_end.x as f64 - p_center_start.x as f64) / col_intervals;
    let uy = (p_center_end.y as f64 - p_center_start.y as f64) / col_intervals;

    // Unit vector in row direction (from center of first row to center of last row)
    let row_intervals = (rows - 1.0).max(1.0);
    let vx = (p_center_bottom.x as f64 - p_center_start.x as f64) / row_intervals;
    let vy = (p_center_bottom.y as f64 - p_center_start.y as f64) / row_intervals;

    // Origin: when we call map(col_start + 0.5, 0.5), we want to get p_center_start
    // map(col + 0.5, row + 0.5) = origin + (col + 0.5 - col_start) * u + (row + 0.5) * v
    // For col = col_start, row = 0:
    // p_center_start = origin + 0.5 * u + 0.5 * v
    // origin = p_center_start - 0.5 * u - 0.5 * v
    let origin_x = p_center_start.x as f64 - 0.5 * ux - 0.5 * vx;
    let origin_y = p_center_start.y as f64 - 0.5 * uy - 0.5 * vy;

    BandTransform { col_start, col_end, origin_x, origin_y, ux, uy, vx, vy }
}

/// Create simple bands from perspective and dimensions (no region needed)
fn create_simple_bands_from_perspective(
    width: usize,
    height: usize,
    perspective: &Perspective,
    module_size: f64,
) -> Vec<BandTransform> {
    let mut bands = Vec::new();
    let mut col = 0usize;

    // Get alignment positions for this width (vertical timing columns)
    let align_cols = get_alignment_positions(width);

    while col < width {
        // Find next band boundary
        let mut band_end = col + BAND_WIDTH_MODULES;

        // Align band boundary to alignment pattern if nearby
        for &ac in align_cols {
            if ac > col && ac < band_end + 3 && ac > band_end.saturating_sub(3) {
                band_end = ac;
                break;
            }
        }

        band_end = band_end.min(width);

        let band = create_band_from_perspective(col, band_end, height, perspective, module_size);

        crate::debug_log!(
            "[rMQR Grid] Perspective Band {}: cols [{}, {}), origin ({:.1}, {:.1}), u=({:.3}, {:.3}), v=({:.3}, {:.3})",
            bands.len(),
            band.col_start,
            band.col_end,
            band.origin_x,
            band.origin_y,
            band.ux,
            band.uy,
            band.vx,
            band.vy
        );

        bands.push(band);

        col = band_end;
    }

    bands
}

// ============================================================================
// GRID IMAGE (RectBitGrid IMPLEMENTATION)
// ============================================================================

pub struct RmqrRefGridImage<'a, S> {
    pub grid: RmqrGridLocation,
    pub img: &'a PreparedImage<S>,
}

impl<S> RmqrRefGridImage<'_, S>
where
    S: ImageBuffer,
{
    /// Get image pixel coordinates for a grid cell (for debug output)
    pub fn get_pixel_coords(&self, row: usize, col: usize) -> (f64, f64) {
        // Find the band for this column
        let band = self.grid.bands.iter().find(|b| col >= b.col_start && col < b.col_end);

        if let Some(b) = band {
            // Use piecewise affine transform
            b.map(col as f64 + 0.5, row as f64 + 0.5)
        } else {
            // Fallback to global perspective
            let p = self.grid.perspective.map(col as f64 + 0.5, row as f64 + 0.5);
            (p.x as f64, p.y as f64)
        }
    }
}

impl<S> RectBitGrid for RmqrRefGridImage<'_, S>
where
    S: ImageBuffer,
{
    fn width(&self) -> usize {
        self.grid.width
    }
    fn height(&self) -> usize {
        self.grid.height
    }

    fn bit(&self, row: usize, col: usize) -> bool {
        let (px, py) = self.get_pixel_coords(row, col);

        // Check bounds
        if px < 0.0 || py < 0.0 {
            return false;
        }
        let x = px as usize;
        let y = py as usize;
        if x >= self.img.width() || y >= self.img.height() {
            return false;
        }

        // Use enhanced sampling for small modules
        if self.grid.module_size < MIN_MODULE_SIZE_FOR_SIMPLE_SAMPLE {
            self.sample_module_enhanced(px, py)
        } else {
            // Simple single-pixel sample for large modules
            self.img.get_pixel_byte(x, y) != 0
        }
    }

    fn get_pixel_coords(&self, row: usize, col: usize) -> Option<(f64, f64)> {
        Some(RmqrRefGridImage::get_pixel_coords(self, row, col))
    }
}

impl<S> RmqrRefGridImage<'_, S>
where
    S: ImageBuffer,
{
    /// Enhanced module sampling: sample a small disk/cross and use majority vote
    fn sample_module_enhanced(&self, center_x: f64, center_y: f64) -> bool {
        let radius = (self.grid.module_size * SAMPLE_DISK_RADIUS).max(0.5);
        let mut samples = Vec::new();

        // Sample center
        let cx = center_x as usize;
        let cy = center_y as usize;
        if cx < self.img.width() && cy < self.img.height() {
            samples.push(self.img.get_pixel_byte(cx, cy));
        }

        // Sample cross pattern around center
        let offsets = [(radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius)];

        for (dx, dy) in offsets {
            let sx = (center_x + dx) as usize;
            let sy = (center_y + dy) as usize;
            if sx < self.img.width() && sy < self.img.height() {
                samples.push(self.img.get_pixel_byte(sx, sy));
            }
        }

        if samples.is_empty() {
            return false;
        }

        // Use majority vote (count how many pixels are black)
        // In prepared image: 0 = white, non-0 = black
        let black_count = samples.iter().filter(|&&v| v != 0).count();

        // Return true if majority are black
        black_count > samples.len() / 2
    }
}
