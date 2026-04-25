use std::{cmp, mem};

use crate::{
    BitGrid, CapStone, Point, geometry,
    identify::match_capstones::CapStoneGroup,
    prepare::PreparedImage,
    prepare::{AreaFiller, ColoredRegion, ImageBuffer, PixelColor, Row},
    version_db::VERSION_DATA_BASE,
};

/// Location of a skewed square in an image
///
/// A skewed square formed by 3 [CapStones](struct.CapStone.html) and possibly
/// an alignment pattern located in the 4th corner. This can be converted into a
/// normal [Grid](trait.Grid.html) that can be decoded.
#[derive(Debug, Clone)]
pub struct SkewedGridLocation {
    pub grid_size: usize,
    pub c: geometry::Perspective,
}

impl SkewedGridLocation {
    /// Create a SkewedGridLocation from corners
    ///
    /// Given 3 corners of a grid, this tries to match the expected features of
    /// a QR code such that the SkewedGridLocation maps onto those features.
    ///
    /// For all grids this includes searching for timing patterns between
    /// capstones to determine the grid size.
    ///
    /// For bigger grids this includes searching for an alignment pattern in the
    /// 4 corner.
    ///
    /// If no sufficient match could be produces, return `None` instead.
    pub fn from_group<S>(img: &mut PreparedImage<S>, mut group: CapStoneGroup) -> Option<Self>
    where
        S: ImageBuffer,
    {
        crate::debug_log!("=== Starting SkewedGridLocation::from_group ===");
        /* Construct the hypotenuse line from A to C. B should be to
         * the left of this line.
         */
        let h0 = group.0.center;
        let mut hd = Point {
            x: group.2.center.x - group.0.center.x,
            y: group.2.center.y - group.0.center.y,
        };
        crate::debug_log!("Initial capstone positions:");
        crate::debug_log!("  A (group.0): center=({}, {})", group.0.center.x, group.0.center.y);
        crate::debug_log!("  B (group.1): center=({}, {})", group.1.center.x, group.1.center.y);
        crate::debug_log!("  C (group.2): center=({}, {})", group.2.center.x, group.2.center.y);
        crate::debug_log!("Hypotenuse: h0=({}, {}), hd=({}, {})", h0.x, h0.y, hd.x, hd.y);
        /* Make sure A-B-C is clockwise */
        let cross_product = (group.1.center.x - h0.x) * -hd.y + (group.1.center.y - h0.y) * hd.x;
        crate::debug_log!("Cross product for clockwise check: {}", cross_product);
        let was_swapped = cross_product > 0;
        if was_swapped {
            crate::debug_log!("SWAPPING A and C to ensure clockwise order (image may be mirrored)");
            mem::swap(&mut group.0, &mut group.2);
            hd.x = -hd.x;
            hd.y = -hd.y;
        }
        crate::debug::log_clockwise_check(was_swapped, &h0, &hd);
        /* Rotate each capstone so that corner 0 is top-left with respect
         * to the grid.
         */
        crate::debug_log!("Rotating capstones to align corners...");
        rotate_capstone(&mut group.0, &h0, &hd, "A");
        rotate_capstone(&mut group.1, &h0, &hd, "B");
        rotate_capstone(&mut group.2, &h0, &hd, "C");
        /* Check the timing pattern. This doesn't require a perspective
         * transform.
         */
        let grid_size = measure_timing_pattern(img, &group);
        crate::debug_log!("Measured grid size from timing pattern: {}", grid_size);

        /* Calculate geometric estimate for the 4th corner (alignment region)
         * This assumes a parallelogram formed by the three finder patterns.
         * For axis-aligned QR codes, this is exact. For skewed codes, line_intersect
         * may give a better result.
         */
        let expected_x = group.0.corners[0].x + group.2.corners[0].x - group.1.corners[0].x;
        let expected_y = group.0.corners[0].y + group.2.corners[0].y - group.1.corners[0].y;

        crate::debug_log!("Geometric estimate for 4th corner: ({}, {})", expected_x, expected_y);

        /* Make an estimate based for the alignment pattern based on extending
         * lines from capstones A and C.
         */
        let align = match geometry::line_intersect(
            &group.0.corners[0],
            &group.0.corners[1],
            &group.2.corners[0],
            &group.2.corners[3],
        ) {
            Some(intersection) => {
                crate::debug_log!(
                    "Line intersection result: ({}, {})",
                    intersection.x,
                    intersection.y
                );

                // Validate the intersection result:
                // 1. Must not be negative
                // 2. Must be within reasonable bounds of the image
                // 3. Must be reasonably close to geometric estimate (for axis-aligned QR codes,
                //    line_intersect can produce garbage because lines are parallel)
                let margin = img.width() as i32 / 4;
                let in_bounds = intersection.x >= 0
                    && intersection.y >= 0
                    && intersection.x <= img.width() as i32 + margin
                    && intersection.y <= img.height() as i32 + margin;

                // Check if reasonably close to geometric estimate
                // Use 30% of image dimension as threshold
                let max_diff = img.width() as i32 * 3 / 10;
                let close_to_estimate = (intersection.x - expected_x).abs() < max_diff
                    && (intersection.y - expected_y).abs() < max_diff;

                if in_bounds && close_to_estimate {
                    crate::debug_log!("Using line intersection result");
                    intersection
                } else {
                    crate::debug_log!(
                        "WARNING: Line intersection ({}, {}) invalid or too far from estimate ({}, {})",
                        intersection.x,
                        intersection.y,
                        expected_x,
                        expected_y
                    );
                    crate::debug_log!("Using geometric estimate instead");
                    Point { x: expected_x, y: expected_y }
                }
            }
            None => {
                crate::debug_log!("Line intersection failed, using geometric estimate");
                Point { x: expected_x, y: expected_y }
            }
        };

        crate::debug_log!("Final alignment estimate: ({}, {})", align.x, align.y);

        let mut align = align;

        /* On V2+ grids, we should use the alignment pattern. */
        if grid_size > 21 {
            crate::debug_log!("Grid size > 21, searching for alignment pattern...");
            /* Try to find the actual location of the alignment pattern. */
            let align_result = find_alignment_pattern(img, align, &group.0, &group.2);
            match align_result {
                Some(found_align) => {
                    crate::debug_log!(
                        "Alignment pattern found at: ({}, {})",
                        found_align.x,
                        found_align.y
                    );
                    align = found_align;
                    let score = -hd.y * align.x + hd.x * align.y;
                    let finder = LeftMostFinder { line_p: hd, best: align, score };
                    let found = img.repaint_and_apply(
                        (align.x as usize, align.y as usize),
                        PixelColor::Alignment,
                        finder,
                    );
                    align = found.best;
                    crate::debug_log!("Refined alignment position: ({}, {})", align.x, align.y);
                }
                None => {
                    crate::debug_log!("WARNING: Alignment pattern NOT FOUND! Using estimate.");
                    crate::debug::log_alignment_search(&Point { x: align.x, y: align.y }, None, 0);
                    return None;
                }
            }
        }

        let version = version_from_grid_size(grid_size);
        crate::debug_log!("Calculated version: {} (grid_size={})", version, grid_size);

        if version >= VERSION_DATA_BASE.len() {
            crate::debug_log!(
                "ERROR: Version {} exceeds database size {}",
                version,
                VERSION_DATA_BASE.len()
            );
            return None;
        }

        let c = setup_perspective(img, &group, align, grid_size)?;
        crate::debug::log_perspective(&c, grid_size, "FINAL");

        Some(SkewedGridLocation { grid_size, c })
    }

    /// Convert into a grid referencing the underlying image as source
    pub fn into_grid_image<S>(self, img: &PreparedImage<S>) -> RefGridImage<'_, S> {
        RefGridImage { grid: self, img }
    }
}

/// Get the version for a given grid size.
///
/// The returned version can be used to fetch the VersionInfo for the given grid
/// size.
fn version_from_grid_size(grid_size: usize) -> usize {
    (grid_size - 17) / 4
}

/// A Grid that references a bigger image
///
/// Given a grid location and an image, implement the [Grid
/// trait](trait.Grid.html) so that it may be decoded by
/// [decode](fn.decode.html)
pub struct RefGridImage<'a, S> {
    grid: SkewedGridLocation,
    img: &'a PreparedImage<S>,
}

impl<S> BitGrid for RefGridImage<'_, S>
where
    S: ImageBuffer,
{
    fn size(&self) -> usize {
        self.grid.grid_size
    }

    fn bit(&self, y: usize, x: usize) -> bool {
        let p = self.grid.c.map(x as f64 + 0.5, y as f64 + 0.5);
        PixelColor::White != self.img.get_pixel_at_point(p)
    }
}

fn setup_perspective<S>(
    img: &PreparedImage<S>,
    caps: &CapStoneGroup,
    align: Point,
    grid_size: usize,
) -> Option<geometry::Perspective>
where
    S: ImageBuffer,
{
    crate::debug_log!("Setting up perspective transformation...");
    crate::debug_log!("  Corner points for perspective:");
    crate::debug_log!(
        "    caps.1.corners[0] (top-left finder): ({}, {})",
        caps.1.corners[0].x,
        caps.1.corners[0].y
    );
    crate::debug_log!(
        "    caps.2.corners[0] (top-right finder): ({}, {})",
        caps.2.corners[0].x,
        caps.2.corners[0].y
    );
    crate::debug_log!("    align (bottom-right): ({}, {})", align.x, align.y);
    crate::debug_log!(
        "    caps.0.corners[0] (bottom-left finder): ({}, {})",
        caps.0.corners[0].x,
        caps.0.corners[0].y
    );

    let initial = geometry::Perspective::create(
        &[caps.1.corners[0], caps.2.corners[0], align, caps.0.corners[0]],
        (grid_size - 7) as f64,
        (grid_size - 7) as f64,
    )?;

    crate::debug::log_perspective(&initial, grid_size, "INITIAL");

    Some(jiggle_perspective(img, initial, grid_size))
}

fn rotate_capstone(cap: &mut CapStone, h0: &Point, hd: &Point, name: &str) {
    let (best_idx, _) = cap
        .corners
        .iter()
        .enumerate()
        .min_by_key(|(_, a)| (a.x - h0.x) * (-hd.y) + (a.y - h0.y) * hd.x)
        .expect("corners cannot be empty");

    crate::debug_log!("Capstone {}: rotating by {} positions", name, best_idx);
    crate::debug_log!("  Before rotation: corners[0]=({}, {})", cap.corners[0].x, cap.corners[0].y);

    /* Rotate the capstone */
    cap.corners.rotate_left(best_idx);
    cap.c = geometry::Perspective::create(&cap.corners, 7.0, 7.0)
        .expect("rotated perspective can't fail");

    crate::debug_log!("  After rotation:  corners[0]=({}, {})", cap.corners[0].x, cap.corners[0].y);
    crate::debug::log_capstone_rotation(name, best_idx, &cap.corners);
}

//* Try the measure the timing pattern for a given QR code. This does
// * not require the global perspective to have been set up, but it
// * does require that the capstone corners have been set to their
// * canonical rotation.
// *
// * For each capstone, we find a point in the middle of the ring band
// * which is nearest the centre of the code. Using these points, we do
// * a horizontal and a vertical timing scan.
// */
fn measure_timing_pattern<S>(img: &PreparedImage<S>, caps: &CapStoneGroup) -> usize
where
    S: ImageBuffer,
{
    const US: [f64; 3] = [6.5f64, 6.5f64, 0.5f64];
    const VS: [f64; 3] = [0.5f64, 6.5f64, 6.5f64];
    let tpet0 = caps.0.c.map(US[0], VS[0]);
    let tpet1 = caps.1.c.map(US[1], VS[1]);
    let tpet2 = caps.2.c.map(US[2], VS[2]);

    crate::debug_log!("Timing pattern measurement points:");
    crate::debug_log!("  tpet0 (from cap A): ({}, {})", tpet0.x, tpet0.y);
    crate::debug_log!("  tpet1 (from cap B): ({}, {})", tpet1.x, tpet1.y);
    crate::debug_log!("  tpet2 (from cap C): ({}, {})", tpet2.x, tpet2.y);

    let hscan = timing_scan(img, &tpet1, &tpet2, "horizontal");
    let vscan = timing_scan(img, &tpet1, &tpet0, "vertical");

    let scan = cmp::max(hscan, vscan);

    /* Choose the nearest allowable grid size */
    assert!(scan >= 1);
    let size = scan + 13;
    let ver = (size as f64 - 15.0).floor() as usize / 4;
    let grid_size = ver * 4 + 17;

    crate::debug::log_timing_pattern(hscan, vscan, grid_size);

    grid_size
}

fn timing_scan<S>(img: &PreparedImage<S>, p0: &Point, p1: &Point, direction: &str) -> usize
where
    S: ImageBuffer,
{
    let mut count = 0;
    let mut previous = None;
    let mut transitions = Vec::new();

    for (i, p) in geometry::BresenhamScan::new(p0, p1).enumerate() {
        let pixel = img.get_pixel_at_point(p);
        if let Some(prev) = previous
            && prev != pixel
        {
            count += 1;
            transitions.push((i, p.x, p.y, format!("{:?}", pixel)));
        }
        previous = Some(pixel);
    }

    crate::debug_log!(
        "Timing scan {}: {} -> {}, transitions={}",
        direction,
        format!("({},{})", p0.x, p0.y),
        format!("({},{})", p1.x, p1.y),
        count
    );

    // Log first few transitions for debugging
    if crate::debug::is_debug_enabled() && !transitions.is_empty() {
        crate::debug_log!("  First 10 transitions:");
        for (i, (pos, x, y, color)) in transitions.iter().take(10).enumerate() {
            crate::debug_log!("    {}: pixel {} at ({}, {}) -> {}", i, pos, x, y, color);
        }
    }

    count
}

fn find_alignment_pattern<S>(
    img: &mut PreparedImage<S>,
    mut align_seed: Point,
    c0: &CapStone,
    c2: &CapStone,
) -> Option<Point>
where
    S: ImageBuffer,
{
    crate::debug_log!("Searching for alignment pattern...");
    crate::debug_log!("  Initial seed: ({}, {})", align_seed.x, align_seed.y);

    /* Guess another two corners of the alignment pattern so that we
     * can estimate its size.
     */
    let (u, v) = c0.c.unmap(&align_seed);
    let a = c0.c.map(u, v + 1.0);
    let (u, v) = c2.c.unmap(&align_seed);
    let c = c2.c.map(u + 1.0, v);
    let size_estimate = ((a.x - align_seed.x) * -(c.y - align_seed.y)
        + (a.y - align_seed.y) * (c.x - align_seed.x))
        .unsigned_abs() as usize;

    crate::debug_log!("  Size estimate: {} pixels", size_estimate);
    crate::debug::log_alignment_search(&align_seed, None, size_estimate);

    /* Spiral outwards from the estimate point until we find something
     * roughly the right size. Don't look too far from the estimate
     * point.
     */
    let mut dir = 0;
    let mut step_size = 1;
    let mut search_steps = 0;

    while step_size * step_size < size_estimate * 100 {
        const DX_MAP: [i32; 4] = [1, 0, -1, 0];
        const DY_MAP: [i32; 4] = [0, -1, 0, 1];
        for _pass in 0..step_size {
            search_steps += 1;
            let x = align_seed.x as usize;
            let y = align_seed.y as usize;

            // Alignment pattern should not be white
            if x < img.width() && y < img.height() && PixelColor::White != img.get_pixel_at(x, y) {
                let region = img.get_region((x, y));
                let count = match region {
                    ColoredRegion::Unclaimed { pixel_count, .. } => pixel_count,
                    _ => continue,
                };

                // Matches expected size of alignment pattern
                if count >= size_estimate / 2 && count <= size_estimate * 2 {
                    crate::debug_log!(
                        "  Found alignment pattern at ({}, {}) after {} steps",
                        align_seed.x,
                        align_seed.y,
                        search_steps
                    );
                    crate::debug_log!(
                        "  Region size: {} (expected: {} +/- 50%)",
                        count,
                        size_estimate
                    );
                    crate::debug::log_alignment_search(
                        &align_seed,
                        Some(&align_seed),
                        size_estimate,
                    );
                    return Some(align_seed);
                }
            }

            align_seed.x += DX_MAP[dir];
            align_seed.y += DY_MAP[dir];
        }

        // Cycle directions
        dir = (dir + 1) % 4;
        if dir & 1 == 0 {
            step_size += 1
        }
    }

    crate::debug_log!("  Alignment pattern NOT FOUND after {} steps", search_steps);
    None
}

struct LeftMostFinder {
    line_p: Point,
    best: Point,
    score: i32,
}

impl AreaFiller for LeftMostFinder {
    fn update(&mut self, row: Row) {
        let left_d = -self.line_p.y * (row.left as i32) + self.line_p.x * row.y as i32;
        let right_d = -self.line_p.y * (row.right as i32) + self.line_p.x * row.y as i32;

        if left_d < self.score {
            self.score = left_d;
            self.best.x = row.left as i32;
            self.best.y = row.y as i32;
        }

        if right_d < self.score {
            self.score = right_d;
            self.best.x = row.right as i32;
            self.best.y = row.y as i32;
        }
    }
}

fn jiggle_perspective<S>(
    img: &PreparedImage<S>,
    mut perspective: geometry::Perspective,
    grid_size: usize,
) -> geometry::Perspective
where
    S: ImageBuffer,
{
    let mut best = fitness_all(img, &perspective, grid_size);
    crate::debug_log!("Jiggle perspective optimization starting with fitness: {}", best);

    let mut adjustments: [f64; 8] = [
        perspective.0[0] * 0.02f64,
        perspective.0[1] * 0.02f64,
        perspective.0[2] * 0.02f64,
        perspective.0[3] * 0.02f64,
        perspective.0[4] * 0.02f64,
        perspective.0[5] * 0.02f64,
        perspective.0[6] * 0.02f64,
        perspective.0[7] * 0.02f64,
    ];

    for pass in 0..5 {
        let pass_start_fitness = best;
        for i in 0..16 {
            let j = i >> 1;
            let old = perspective.0[j];
            let step = adjustments[j];

            let new = if i & 1 != 0 { old + step } else { old - step };

            perspective.0[j] = new;
            let test = fitness_all(img, &perspective, grid_size);
            if test > best { best = test } else { perspective.0[j] = old }
        }

        #[allow(clippy::needless_range_loop)]
        for i in 0..8 {
            adjustments[i] *= 0.5f64;
        }

        crate::debug::log_jiggle_perspective(pass_start_fitness, best, pass);
    }

    crate::debug_log!("Jiggle perspective optimization finished with fitness: {}", best);
    perspective
}
/* Compute a fitness score for the currently configured perspective
 * transform, using the features we expect to find by scanning the
 * grid.
 */
fn fitness_all<S>(
    img: &PreparedImage<S>,
    perspective: &geometry::Perspective,
    grid_size: usize,
) -> i32
where
    S: ImageBuffer,
{
    let version = version_from_grid_size(grid_size);
    let info = &VERSION_DATA_BASE[version];
    let mut score = 0;

    /* Check the timing pattern */
    for i in 0..(grid_size as i32 - 14) {
        let expect = if 0 != i & 1 { 1 } else { -1 };
        score += fitness_cell(img, perspective, i + 7, 6) * expect;
        score += fitness_cell(img, perspective, 6, i + 7) * expect;
    }

    /* Check capstones */
    score += fitness_capstone(img, perspective, 0, 0);
    score += fitness_capstone(img, perspective, grid_size as i32 - 7, 0);
    score += fitness_capstone(img, perspective, 0, grid_size as i32 - 7);

    /* Check alignment patterns */
    let mut ap_count = 0;
    while ap_count < 7 && info.apat[ap_count] != 0 {
        ap_count += 1
    }
    for i in 1..(ap_count.saturating_sub(1)) {
        score += fitness_apat(img, perspective, 6, info.apat[i] as i32);
        score += fitness_apat(img, perspective, info.apat[i] as i32, 6);
    }
    for i in 1..ap_count {
        for j in 1..ap_count {
            score += fitness_apat(img, perspective, info.apat[i] as i32, info.apat[j] as i32);
        }
    }
    score
}

fn fitness_apat<S>(
    img: &PreparedImage<S>,
    perspective: &geometry::Perspective,
    cx: i32,
    cy: i32,
) -> i32
where
    S: ImageBuffer,
{
    fitness_cell(img, perspective, cx, cy) - fitness_ring(img, perspective, cx, cy, 1)
        + fitness_ring(img, perspective, cx, cy, 2)
}

fn fitness_ring<S>(
    img: &PreparedImage<S>,
    perspective: &geometry::Perspective,
    cx: i32,
    cy: i32,
    radius: i32,
) -> i32
where
    S: ImageBuffer,
{
    let mut score = 0;
    for i in 0..(radius * 2) {
        score += fitness_cell(img, perspective, cx - radius + i, cy - radius);
        score += fitness_cell(img, perspective, cx - radius, cy + radius - i);
        score += fitness_cell(img, perspective, cx + radius, cy - radius + i);
        score += fitness_cell(img, perspective, cx + radius - i, cy + radius);
    }
    score
}

fn fitness_cell<S>(
    img: &PreparedImage<S>,
    perspective: &geometry::Perspective,
    x: i32,
    y: i32,
) -> i32
where
    S: ImageBuffer,
{
    const OFFSETS: [f64; 3] = [0.3f64, 0.5f64, 0.7f64];
    let mut score = 0;
    let width = img.width();
    let height = img.height();
    let x_f = x as f64;
    let y_f = y as f64;

    #[allow(clippy::needless_range_loop)]
    for v in 0..3 {
        for u in 0..3 {
            let p = perspective.map(x_f + OFFSETS[u], y_f + OFFSETS[v]);

            // OPTIMIZATION: Single-branch bounds check.
            // Casting a negative i32 to usize results in a huge number (wrapping),
            // so (p.x as usize) < width handles both p.x >= 0 AND p.x < width.
            let px = p.x as usize;
            let py = p.y as usize;

            if px < width && py < height {
                // OPTIMIZATION: Compare raw byte directly.
                // 0 is White. If it's NOT 0 (Black, Capstone, etc), we add to score.
                if img.get_pixel_byte(px, py) != 0 { score += 1 } else { score -= 1 }
            }
        }
    }
    score
}

fn fitness_capstone<S>(
    img: &PreparedImage<S>,
    perspective: &geometry::Perspective,
    x: i32,
    y: i32,
) -> i32
where
    S: ImageBuffer,
{
    fitness_cell(img, perspective, x + 3, y + 3) + fitness_ring(img, perspective, x + 3, y + 3, 1)
        - fitness_ring(img, perspective, x + 3, y + 3, 2)
        + fitness_ring(img, perspective, x + 3, y + 3, 3)
}
