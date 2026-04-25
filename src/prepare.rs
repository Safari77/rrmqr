use crate::identify::Point;
use crate::identify::match_capstones::CapStoneGroup;
use image::{GrayImage, Pixel, RgbImage};
use lru::LruCache;
use std::{cmp, num::NonZeroUsize};

// ============================================================================
// CONFIGURATION
// ============================================================================

#[derive(Clone, Debug)]
pub struct PreparationConfig {
    /// 0-255 global threshold adjustment (positive makes image darker/more black)
    pub threshold_bias: i16,
    /// Weights for R, G, B channels [Red, Green, Blue]. Default [0.299, 0.587, 0.114]
    pub rgb_bias: [f32; 3],
    /// If true, stretches histogram to 0..255 (fixes low contrast 0x50 vs 0x52)
    pub contrast_stretch: bool,
    /// If true, uses imageproc's adaptive thresholding (slower but handles shadows)
    pub use_adaptive: bool,
    /// Block radius for adaptive thresholding (default ~50)
    pub adaptive_block_radius: u32,
    /// Constant subtracted from the mean or weighted mean (default ~10).
    /// Positive values make the result blacker (more strict).
    pub adaptive_threshold_delta: i32,
    /// If true, uses the Hybrid Binarizer (a local thresholding algorithm designed for
    /// high frequency images like QR codes). Takes precedence over `use_adaptive`.
    pub use_hybrid_binarizer: bool,
}

impl Default for PreparationConfig {
    fn default() -> Self {
        Self {
            threshold_bias: 1,
            rgb_bias: [0.299, 0.587, 0.114],
            contrast_stretch: false,
            use_adaptive: false,
            adaptive_block_radius: 50,
            adaptive_threshold_delta: 10,
            use_hybrid_binarizer: false,
        }
    }
}

// ============================================================================
// PREPARED IMAGE
// ============================================================================

/// An black-and-white image that can be mutated on search for QR codes
#[derive(Clone)]
pub struct PreparedImage<S> {
    buffer: S,
    cache: LruCache<u8, ColoredRegion>,
}

pub trait ImageBuffer {
    fn width(&self) -> usize;
    fn height(&self) -> usize;
    fn get_pixel(&self, x: usize, y: usize) -> u8;
    fn set_pixel(&mut self, x: usize, y: usize, val: u8);
}

// Implement for standard image crate types
#[cfg(feature = "img")]
impl<
    T: image::GenericImage<Pixel = image::Luma<u8>> + image::GenericImageView<Pixel = image::Luma<u8>>,
> ImageBuffer for T
{
    fn width(&self) -> usize {
        self.width() as usize
    }
    fn height(&self) -> usize {
        self.height() as usize
    }
    fn get_pixel(&self, x: usize, y: usize) -> u8 {
        self.get_pixel(x as u32, y as u32).0[0]
    }
    fn set_pixel(&mut self, x: usize, y: usize, val: u8) {
        self.put_pixel(x as u32, y as u32, image::Luma::<u8>::from([val; 1]));
    }
}

// Basic buffer implementation
#[derive(Clone, Debug)]
pub struct BasicImageBuffer {
    w: usize,
    h: usize,
    pixels: Box<[u8]>,
}

impl ImageBuffer for BasicImageBuffer {
    #[inline]
    fn width(&self) -> usize {
        self.w
    }
    #[inline]
    fn height(&self) -> usize {
        self.h
    }
    #[inline]
    fn get_pixel(&self, x: usize, y: usize) -> u8 {
        self.pixels[(y * self.w) + x]
    }
    fn set_pixel(&mut self, x: usize, y: usize, val: u8) {
        self.pixels[(y * self.w) + x] = val
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct Row {
    pub left: usize,
    pub right: usize,
    pub y: usize,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PixelColor {
    White,
    Black,
    CapStone,
    Alignment,
    Tmp1,
    Discarded(u8),
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ColoredRegion {
    Unclaimed { color: PixelColor, src_x: usize, src_y: usize, pixel_count: usize },
    CapStone,
    Alignment,
    Tmp1,
}

impl From<u8> for PixelColor {
    fn from(x: u8) -> Self {
        match x {
            0 => PixelColor::White,
            1 => PixelColor::Black,
            2 => PixelColor::CapStone,
            3 => PixelColor::Alignment,
            4 => PixelColor::Tmp1,
            x => PixelColor::Discarded(x - 5),
        }
    }
}
impl From<PixelColor> for u8 {
    fn from(c: PixelColor) -> Self {
        match c {
            PixelColor::White => 0,
            PixelColor::Black => 1,
            PixelColor::CapStone => 2,
            PixelColor::Alignment => 3,
            PixelColor::Tmp1 => 4,
            PixelColor::Discarded(x) => x + 5,
        }
    }
}
impl PartialEq<u8> for PixelColor {
    fn eq(&self, other: &u8) -> bool {
        let rep: u8 = (*self).into();
        rep == *other
    }
}

impl<F: FnMut(Row)> AreaFiller for F {
    fn update(&mut self, row: Row) {
        self(row)
    }
}
struct AreaCounter(usize);
impl AreaFiller for AreaCounter {
    fn update(&mut self, row: Row) {
        self.0 += row.right - row.left + 1;
    }
}

// ============================================================================
// IMPLEMENTATION
// ============================================================================

impl<S> PreparedImage<S>
where
    S: ImageBuffer,
{
    /// Prepare image with default settings
    pub fn prepare(buf: S) -> Self {
        Self::prepare_with_config(buf, PreparationConfig::default())
    }

    pub fn prepare_with_config(mut buf: S, config: PreparationConfig) -> Self {
        let w = buf.width();
        let h = buf.height();

        if config.contrast_stretch {
            Self::apply_contrast_stretch(&mut buf);
        }

        let prepared = if config.use_hybrid_binarizer {
            crate::debug_log!("Using Hybrid Binarizer");
            Self::prepare_hybrid(buf)
        } else if config.use_adaptive {
            crate::debug_log!("Using Adaptive Thresholding");
            Self::prepare_adaptive_custom(
                buf,
                w,
                h,
                config.adaptive_block_radius as usize,
                config.adaptive_threshold_delta,
            )
        } else {
            let threshold = compute_otsu_threshold(&buf);
            let final_thresh = (threshold as i16 + config.threshold_bias).clamp(0, 255) as u8;
            crate::debug_log!("Using Global Threshold: {}", final_thresh);

            for y in 0..h {
                for x in 0..w {
                    let p = buf.get_pixel(x, y);
                    let fill = if p < final_thresh { PixelColor::Black } else { PixelColor::White };
                    buf.set_pixel(x, y, fill.into());
                }
            }
            PreparedImage { buffer: buf, cache: LruCache::new(NonZeroUsize::new(251).unwrap()) }
        };

        // Save debug bitmap (Fix: No try_into() needed)
        prepared.save_debug_bitmap("bitmap.png");

        prepared
    }

    /// Helper to save the binary state to a file for debugging
    pub fn save_debug_bitmap(&self, path: &str) {
        let mut img = image::GrayImage::new(self.width() as u32, self.height() as u32);
        for y in 0..self.height() {
            for x in 0..self.width() {
                let val = match self.get_pixel_at(x, y) {
                    PixelColor::Black => 0,
                    _ => 255,
                };
                img.put_pixel(x as u32, y as u32, image::Luma([val]));
            }
        }
        let _ = img.save(path);
        crate::debug_log!("Saved debug bitmap to {}", path);
    }

    /// Converts an RGB image to GrayImage using specific channel weights
    pub fn from_rgb(img: &RgbImage, config: PreparationConfig) -> GrayImage {
        let (w, h) = img.dimensions();
        let mut gray = GrayImage::new(w, h);

        let r_w = config.rgb_bias[0];
        let g_w = config.rgb_bias[1];
        let b_w = config.rgb_bias[2];

        for (x, y, pixel) in img.enumerate_pixels() {
            let p = pixel.channels();
            let val = (p[0] as f32 * r_w + p[1] as f32 * g_w + p[2] as f32 * b_w) as u8;
            gray.put_pixel(x, y, image::Luma([val]));
        }
        gray
    }

    fn apply_contrast_stretch(buf: &mut S) {
        let w = buf.width();
        let h = buf.height();
        let mut min = 255u8;
        let mut max = 0u8;

        for y in 0..h {
            for x in 0..w {
                let p = buf.get_pixel(x, y);
                if p < min {
                    min = p;
                }
                if p > max {
                    max = p;
                }
            }
        }

        let range = max - min;
        // Only stretch if the image is low contrast (< 50 intensity levels diff)
        if range < 200 && range > 0 {
            crate::debug_log!("Low contrast detected ({}-{}), stretching...", min, max);
            for y in 0..h {
                for x in 0..w {
                    let p = buf.get_pixel(x, y);
                    let scaled = ((p as u32 - min as u32) * 255 / range as u32) as u8;
                    buf.set_pixel(x, y, scaled);
                }
            }
        }
    }

    fn prepare_adaptive_custom(mut buf: S, w: usize, h: usize, radius: usize, delta: i32) -> Self {
        // 1. Convert S to GrayImage for imageproc
        let mut gray_img = image::GrayImage::new(w as u32, h as u32);
        for y in 0..h {
            for x in 0..w {
                let p = buf.get_pixel(x, y);
                gray_img.put_pixel(x as u32, y as u32, image::Luma([p]));
            }
        }

        // 2. Run ImageProc Adaptive Threshold with configurable delta
        let binary_img = imageproc::contrast::adaptive_threshold(&gray_img, radius as u32, delta);

        // 3. Map back to PreparedImage buffer S
        // rqrr convention: 1 = Black, 0 = White
        // imageproc binary convention: 0 = Black, 255 = White
        for y in 0..h {
            for x in 0..w {
                let pixel = binary_img.get_pixel(x as u32, y as u32)[0];
                let val = if pixel == 0 { 1 } else { 0 };
                buf.set_pixel(x, y, val);
            }
        }

        PreparedImage { buffer: buf, cache: LruCache::new(NonZeroUsize::new(251).unwrap()) }
    }

    /// Implementation of the Hybrid Binarizer from ZXing.
    /// This uses a local thresholding algorithm with 8x8 blocks and smoothing.
    fn prepare_hybrid(mut buf: S) -> Self {
        const BLOCK_SIZE: usize = 8;
        const MIN_DYNAMIC_RANGE: u32 = 24;

        // Window size in ZXing C++ is 40. The smoothing radius calculation there is
        // R = WINDOW_SIZE / BLOCK_SIZE / 2; -> 40 / 8 / 2 = 2.
        const SMOOTHING_RADIUS: isize = 2;

        let w = buf.width();
        let h = buf.height();

        // 1. Calculate Block Thresholds
        let sub_width = w.div_ceil(BLOCK_SIZE);
        let sub_height = h.div_ceil(BLOCK_SIZE);

        // We use i32 for thresholds to allow summing in smoothing step without overflow
        // and to match C++ logic where 0 indicates "no contrast".
        let mut thresholds = vec![0i32; sub_width * sub_height];

        for y in 0..sub_height {
            let y_offset = cmp::min(y * BLOCK_SIZE, h.saturating_sub(BLOCK_SIZE));
            let y_limit = cmp::min(y_offset + BLOCK_SIZE, h);

            for x in 0..sub_width {
                let x_offset = cmp::min(x * BLOCK_SIZE, w.saturating_sub(BLOCK_SIZE));
                let x_limit = cmp::min(x_offset + BLOCK_SIZE, w);

                let mut min = 255u8;
                let mut max = 0u8;

                for yy in y_offset..y_limit {
                    for xx in x_offset..x_limit {
                        let pixel = buf.get_pixel(xx, yy);
                        if pixel < min {
                            min = pixel;
                        }
                        if pixel > max {
                            max = pixel;
                        }
                    }
                }

                if (max as u32).saturating_sub(min as u32) > MIN_DYNAMIC_RANGE {
                    thresholds[y * sub_width + x] = ((max as u32 + min as u32) / 2) as i32;
                } else {
                    thresholds[y * sub_width + x] = 0;
                }
            }
        }

        // 2. Smooth Thresholds
        // Apply gaussian-like smoothing filter over all non-zero thresholds and
        // fill any remaining gaps with nearest neighbor.
        let mut smoothed = vec![0i32; sub_width * sub_height];

        for y in 0..sub_height {
            for x in 0..sub_width {
                let left =
                    (x as isize).clamp(SMOOTHING_RADIUS, sub_width as isize - SMOOTHING_RADIUS - 1);
                let top = (y as isize)
                    .clamp(SMOOTHING_RADIUS, sub_height as isize - SMOOTHING_RADIUS - 1);

                let mut sum = thresholds[y * sub_width + x] * 2;
                let mut n = if sum > 0 { 2 } else { 0 };

                for dy in -SMOOTHING_RADIUS..=SMOOTHING_RADIUS {
                    for dx in -SMOOTHING_RADIUS..=SMOOTHING_RADIUS {
                        let nx = left + dx;
                        let ny = top + dy;
                        let val = thresholds[(ny as usize) * sub_width + (nx as usize)];
                        sum += val;
                        if val > 0 {
                            n += 1;
                        }
                    }
                }

                smoothed[y * sub_width + x] = if n > 0 { sum / n } else { 0 };
            }
        }

        // Flood fill any remaining gaps of (very large) no-contrast regions
        // Ported from C++: backfill 0s with the previous valid value.
        let mut last_valid_idx: Option<usize> = None;

        for i in 0..smoothed.len() {
            if smoothed[i] > 0 {
                if let Some(last) = last_valid_idx
                    && last != i.saturating_sub(1)
                {
                    // Fill gap
                    let val = smoothed[i];
                    for k in (last + 1)..i {
                        smoothed[k] = val;
                    }
                }
                last_valid_idx = Some(i);
            }
        }

        // Fill tail
        if let Some(last) = last_valid_idx {
            let fill_val = smoothed[last];
            for k in (last + 1)..smoothed.len() {
                smoothed[k] = fill_val;
            }
        }

        // 3. Apply Thresholds to create Binary Image
        for y in 0..sub_height {
            let y_offset = cmp::min(y * BLOCK_SIZE, h.saturating_sub(BLOCK_SIZE));
            let y_limit = cmp::min(y_offset + BLOCK_SIZE, h);

            for x in 0..sub_width {
                let x_offset = cmp::min(x * BLOCK_SIZE, w.saturating_sub(BLOCK_SIZE));
                let x_limit = cmp::min(x_offset + BLOCK_SIZE, w);

                let threshold = smoothed[y * sub_width + x] as u8;

                for yy in y_offset..y_limit {
                    for xx in x_offset..x_limit {
                        let p = buf.get_pixel(xx, yy);
                        // rqrr convention: 1 = Black, 0 = White
                        let val = if p <= threshold { 1 } else { 0 };
                        buf.set_pixel(xx, yy, val);
                    }
                }
            }
        }

        PreparedImage { buffer: buf, cache: LruCache::new(NonZeroUsize::new(251).unwrap()) }
    }
}
pub trait AreaFiller {
    fn update(&mut self, row: Row);
}

/// Compute Otsu's threshold for optimal binarization
///
/// Otsu's method finds the threshold that minimizes the intra-class variance
/// (or equivalently, maximizes the inter-class variance) between black and white pixels.
/// This works well for bimodal histograms like QR codes.
fn compute_otsu_threshold<S: ImageBuffer>(buf: &S) -> u8 {
    let w = buf.width();
    let h = buf.height();
    let total_pixels = w * h;

    if total_pixels == 0 {
        return 128;
    }

    // Build histogram
    let mut histogram = [0u32; 256];
    for y in 0..h {
        for x in 0..w {
            let pixel = buf.get_pixel(x, y) as usize;
            histogram[pixel] += 1;
        }
    }

    // Compute total sum of all pixel values
    let mut sum_total: f64 = 0.0;
    for (i, &count) in histogram.iter().enumerate() {
        sum_total += (i as f64) * (count as f64);
    }

    let mut sum_background: f64 = 0.0;
    let mut weight_background: f64 = 0.0;
    let mut max_variance: f64 = 0.0;
    let mut best_threshold: u8 = 0;

    let total_pixels_f = total_pixels as f64;

    for (threshold, &count) in histogram.iter().enumerate() {
        weight_background += count as f64;
        if weight_background == 0.0 {
            continue;
        }

        let weight_foreground = total_pixels_f - weight_background;
        if weight_foreground == 0.0 {
            break;
        }

        sum_background += (threshold as f64) * (count as f64);

        let mean_background = sum_background / weight_background;
        let mean_foreground = (sum_total - sum_background) / weight_foreground;

        // Calculate between-class variance
        let variance =
            weight_background * weight_foreground * (mean_background - mean_foreground).powi(2);

        if variance > max_variance {
            max_variance = variance;
            best_threshold = threshold as u8;
        }
    }

    best_threshold
}

impl<S> PreparedImage<S>
where
    S: ImageBuffer,
{
    /// Group [CapStones](struct.CapStone.html) into [Grids](struct.Grid.html)
    /// that are likely QR codes
    ///
    /// Return a vector of Grids
    pub fn detect_grids(&mut self) -> Vec<crate::Grid<crate::identify::grid::RefGridImage<'_, S>>>
    where
        S: Clone,
    {
        crate::debug_log!("=== Starting grid detection ===");
        crate::debug::log_image_stats(self);

        let mut res = Vec::new();
        let stones = crate::capstones_from_image(self);
        crate::debug_log!("Found {} capstones in image", stones.len());

        let groups = self.find_groupings(stones);
        crate::debug_log!("Found {} capstone groups", groups.len());

        let locations: Vec<_> = groups
            .into_iter()
            .filter_map(|v| crate::SkewedGridLocation::from_group(self, v))
            .collect();
        crate::debug_log!("Converted to {} grid locations", locations.len());

        for (i, grid_location) in locations.into_iter().enumerate() {
            crate::debug_log!("Processing grid location {}: size={}", i, grid_location.grid_size);

            let bounds = [
                grid_location.c.map(0.0, 0.0),
                grid_location.c.map(grid_location.grid_size as f64 + 1.0, 0.0),
                grid_location.c.map(
                    grid_location.grid_size as f64 + 1.0,
                    grid_location.grid_size as f64 + 1.0,
                ),
                grid_location.c.map(0.0, grid_location.grid_size as f64 + 1.0),
            ];

            crate::debug_log!("Grid {} bounds:", i);
            crate::debug_log!("  Top-left: ({}, {})", bounds[0].x, bounds[0].y);
            crate::debug_log!("  Top-right: ({}, {})", bounds[1].x, bounds[1].y);
            crate::debug_log!("  Bottom-right: ({}, {})", bounds[2].x, bounds[2].y);
            crate::debug_log!("  Bottom-left: ({}, {})", bounds[3].x, bounds[3].y);

            let grid = grid_location.into_grid_image(self);
            res.push(crate::Grid { grid, bounds });
        }

        crate::debug_log!("Returning {} detected grids", res.len());
        res
    }

    /// Detect rMQR (Rectangular Micro QR) codes in the image
    ///
    /// Returns a vector of detected rMQR grids that can be decoded.
    ///
    /// ## Example
    ///
    /// ```rust,ignore
    /// let mut img = rqrr::PreparedImage::prepare(image);
    ///
    /// // Detect and decode rMQR codes
    /// for grid in img.detect_rmqr_grids() {
    ///     if let Ok((meta, content)) = grid.decode() {
    ///         println!("rMQR R{}x{}: {}", meta.version.height, meta.version.width, content);
    ///     }
    /// }
    /// ```
    ///
    /// ## Detecting Both QR and rMQR
    ///
    /// To detect both types, call each method separately and decode immediately:
    ///
    /// ```rust,ignore
    /// let mut img = rqrr::PreparedImage::prepare(image);
    ///
    /// // Detect and decode QR codes first
    /// let qr_results: Vec<_> = img.detect_grids()
    ///     .iter()
    ///     .filter_map(|g| g.decode().ok())
    ///     .collect();
    ///
    /// // Then detect and decode rMQR codes
    /// let rmqr_results: Vec<_> = img.detect_rmqr_grids()
    ///     .iter()
    ///     .filter_map(|g| g.decode().ok())
    ///     .collect();
    /// ```
    pub fn detect_rmqr_grids(
        &mut self,
    ) -> Vec<crate::RmqrGrid<crate::rmqr_grid::RmqrRefGridImage<'_, S>>>
    where
        S: Clone,
    {
        crate::debug_log!("[rMQR-DEBUG] === Starting rMQR detection ===");
        crate::debug::log_image_stats(self);

        let mut res = Vec::new();

        // Use a clone for capstone detection to avoid marking pixels in the original image
        // This allows detect_grids() to work correctly if called before or after this method
        let mut img_clone = self.clone();

        // First, detect capstones (7x7 finder patterns) - these are potential rMQR main finders
        let capstones = crate::capstones_from_image(&mut img_clone);
        crate::debug_log!(
            "[rMQR-DEBUG] Found {} capstones (potential main finders)",
            capstones.len()
        );

        // Find rMQR finder patterns (main finders from capstones + sub-finders)
        let patterns = crate::rmqr_detect::find_rmqr_patterns(&mut img_clone, &capstones);
        crate::debug_log!("[rMQR-DEBUG] Found {} total rMQR patterns", patterns.len());

        // Match patterns into complete rMQR regions
        let regions = crate::rmqr_detect::match_rmqr_patterns(&img_clone, &patterns);
        crate::debug_log!("[rMQR-DEBUG] Matched {} rMQR regions", regions.len());

        // Log detection results
        crate::rmqr_detect::log_rmqr_detection(&patterns, &regions);

        // Convert regions to grid locations
        for (i, region) in regions.iter().enumerate() {
            crate::debug_log!(
                "[rMQR-DEBUG] Processing region {}: {}x{}",
                i,
                region.width,
                region.height
            );

            if let Some(grid_location) =
                crate::rmqr_grid::RmqrGridLocation::from_region(self, region)
            {
                crate::debug_log!("[rMQR-DEBUG] Grid location {} created successfully", i);

                let bounds = grid_location.corners;

                crate::debug_log!("[rMQR-DEBUG] Grid {} bounds:", i);
                crate::debug_log!("  Top-left: ({}, {})", bounds[0].x, bounds[0].y);
                crate::debug_log!("  Top-right: ({}, {})", bounds[1].x, bounds[1].y);
                crate::debug_log!("  Bottom-right: ({}, {})", bounds[2].x, bounds[2].y);
                crate::debug_log!("  Bottom-left: ({}, {})", bounds[3].x, bounds[3].y);

                let grid = grid_location.into_grid_image(self);
                res.push(crate::RmqrGrid { grid, bounds });
            } else {
                crate::debug_log!("[rMQR-DEBUG] Failed to create grid location for region {}", i);
            }
        }

        crate::debug_log!("[rMQR-DEBUG] Returning {} detected rMQR grids", res.len());
        res
    }

    /// Find CapStones that form a grid
    ///
    /// By trying to match up the relative perspective of 3
    /// [CapStones](struct.CapStone.html) along with other criteria we can find the
    /// CapStones that corner the same QR code.
    fn find_groupings(&mut self, capstones: Vec<crate::CapStone>) -> Vec<CapStoneGroup>
    where
        S: Clone,
    {
        crate::debug_log!("=== Finding capstone groupings ===");
        crate::debug::log_capstones(&capstones);

        let mut used_capstones = Vec::new();
        let mut groups = Vec::new();
        for idx in 0..capstones.len() {
            if used_capstones.contains(&idx) {
                crate::debug_log!("Skipping capstone {} (already used)", idx);
                continue;
            }
            crate::debug_log!("Processing capstone {} as potential center (B)", idx);

            let pairs = crate::identify::find_and_rank_possible_neighbors(&capstones, idx);
            crate::debug_log!("Found {} potential neighbor pairs", pairs.len());

            for pair in pairs {
                if used_capstones.contains(&pair.0) || used_capstones.contains(&pair.1) {
                    crate::debug_log!(
                        "  Skipping pair ({}, {}) - contains used capstone",
                        pair.0,
                        pair.1
                    );
                    continue;
                }

                let group_under_test = CapStoneGroup(
                    capstones[pair.0].clone(),
                    capstones[idx].clone(),
                    capstones[pair.1].clone(),
                );

                crate::debug::log_capstone_group(&group_under_test, idx, pair);

                // Confirm that this group has the other requirements of a QR code.
                // A copy of the input image is used to not contaminate the
                // original on an incorrect set of CapStones
                let mut image_copy = self.clone();
                if crate::SkewedGridLocation::from_group(&mut image_copy, group_under_test.clone())
                    .is_none()
                {
                    crate::debug_log!(
                        "  Group ({}, {}, {}) REJECTED - failed grid location",
                        pair.0,
                        idx,
                        pair.1
                    );
                    continue;
                }
                // This is a viable set, save this grouping
                crate::debug_log!("  Group ({}, {}, {}) ACCEPTED!", pair.0, idx, pair.1);
                groups.push(group_under_test);
                used_capstones.push(pair.0);
                used_capstones.push(idx);
                used_capstones.push(pair.1);
            }
        }

        crate::debug_log!("Found {} valid QR code groupings", groups.len());
        groups
    }

    pub fn without_preparation(buf: S) -> Self {
        for y in 0..buf.height() {
            for x in 0..buf.width() {
                assert!(buf.get_pixel(x, y) < 2);
            }
        }

        PreparedImage { buffer: buf, cache: LruCache::new(NonZeroUsize::new(251).unwrap()) }
    }

    /// Return the width of the image
    pub fn width(&self) -> usize {
        self.buffer.width()
    }

    /// Return the height of the image
    pub fn height(&self) -> usize {
        self.buffer.height()
    }

    #[inline]
    pub(crate) fn get_region(&mut self, (x, y): (usize, usize)) -> ColoredRegion {
        let pixel = self.buffer.get_pixel(x, y);

        // Fast path for most common cases
        match pixel {
            0 => panic!("Tried to color white patch"),
            1 => {
                // Black pixel - need to flood fill and cache
                let cache_fill = self.cache.len();
                let reg_idx = if cache_fill == self.cache.cap().get() {
                    let (c, reg) = self.cache.pop_lru().expect("fill is at capacity (251)");
                    #[allow(clippy::single_match)]
                    match reg {
                        ColoredRegion::Unclaimed { src_x, src_y, color, .. } => {
                            let _ = self.flood_fill(
                                src_x,
                                src_y,
                                color.into(),
                                PixelColor::Black.into(),
                                |_| (),
                            );
                        }
                        _ => (),
                    }
                    c
                } else {
                    cache_fill as u8
                };
                let next_reg_color = PixelColor::Discarded(reg_idx);
                let counter = self.repaint_and_apply((x, y), next_reg_color, AreaCounter(0));
                let new_reg = ColoredRegion::Unclaimed {
                    color: next_reg_color,
                    src_x: x,
                    src_y: y,
                    pixel_count: counter.0,
                };
                self.cache.put(reg_idx, new_reg);
                new_reg
            }
            2 => ColoredRegion::CapStone,
            3 => ColoredRegion::Alignment,
            4 => ColoredRegion::Tmp1,
            r => {
                // Discarded - lookup in cache
                let reg_idx = r - 5;
                *self.cache.get(&reg_idx).unwrap()
            }
        }
    }

    pub(crate) fn repaint_and_apply<F>(
        &mut self,
        (x, y): (usize, usize),
        target_color: PixelColor,
        fill: F,
    ) -> F
    where
        F: AreaFiller,
    {
        let src = self.buffer.get_pixel(x, y);
        if PixelColor::White == src || target_color == src {
            panic!("Cannot repaint with white or same color");
        }

        self.flood_fill(x, y, src, target_color.into(), fill)
    }

    #[inline]
    pub fn get_pixel_at_point(&self, p: Point) -> PixelColor {
        let x = cmp::max(0, cmp::min((self.width() - 1) as i32, p.x));
        let y = cmp::max(0, cmp::min((self.height() - 1) as i32, p.y));
        self.buffer.get_pixel(x as usize, y as usize).into()
    }

    /// Returns the raw pixel byte (0=White, 1=Black, etc.)
    /// bypassing the PixelColor enum conversion overhead.
    #[inline(always)]
    pub fn get_pixel_byte(&self, x: usize, y: usize) -> u8 {
        self.buffer.get_pixel(x, y)
    }

    #[inline(always)]
    pub fn get_pixel_at(&self, x: usize, y: usize) -> PixelColor {
        self.buffer.get_pixel(x, y).into()
    }

    #[cfg(feature = "img")]
    pub fn write_state_to(&self, p: &str) {
        let mut dyn_img = image::RgbImage::new(self.width() as u32, self.height() as u32);
        const COLORS: [[u8; 3]; 8] = [
            [255, 0, 0],
            [0, 255, 0],
            [0, 0, 255],
            [255, 255, 0],
            [255, 0, 255],
            [0, 255, 255],
            [128, 128, 128],
            [128, 0, 128],
        ];
        for y in 0..self.height() {
            for x in 0..self.width() {
                let px = self.buffer.get_pixel(x, y);
                dyn_img.get_pixel_mut(x as u32, y as u32).0 = if px == 0 {
                    [255, 255, 255]
                } else if px == 1 {
                    [0, 0, 0]
                } else {
                    let i = self.buffer.get_pixel(x, y) - 2;
                    COLORS[(i % 8) as usize]
                }
            }
        }
        dyn_img.save(p).unwrap();
    }

    fn flood_fill<F>(&mut self, x: usize, y: usize, from: u8, to: u8, mut fill: F) -> F
    where
        F: AreaFiller,
    {
        assert_ne!(from, to);
        let w = self.width();
        let mut queue = Vec::new();
        queue.push((x, y));

        while let Some((x, y)) = queue.pop() {
            // Bail early in case there is nothing to fill
            if self.buffer.get_pixel(x, y) == to || self.buffer.get_pixel(x, y) != from {
                continue;
            }

            let mut left = x;
            let mut right = x;

            while left > 0 && self.buffer.get_pixel(left - 1, y) == from {
                left -= 1;
            }
            while right < w - 1 && self.buffer.get_pixel(right + 1, y) == from {
                right += 1
            }

            /* Fill the extent */
            for idx in left..=right {
                self.buffer.set_pixel(idx, y, to);
            }

            fill.update(Row { left, right, y });

            /* Seed new flood-fills */
            if y > 0 {
                let mut seeded_previous = false;
                for x in left..=right {
                    let p = self.buffer.get_pixel(x, y - 1);
                    if p == from {
                        if !seeded_previous {
                            queue.push((x, y - 1));
                        }
                        seeded_previous = true;
                    } else {
                        seeded_previous = false;
                    }
                }
            }
            if y < self.height() - 1 {
                let mut seeded_previous = false;
                for x in left..=right {
                    let p = self.buffer.get_pixel(x, y + 1);
                    if p == from {
                        if !seeded_previous {
                            queue.push((x, y + 1));
                        }
                        seeded_previous = true;
                    } else {
                        seeded_previous = false;
                    }
                }
            }
        }
        fill
    }
}

impl PreparedImage<BasicImageBuffer> {
    /// Given a function with binary output, generate a searchable image
    ///
    /// If the given function returns `true` the matching pixel will be 'black'.
    pub fn prepare_from_bitmap<F>(w: usize, h: usize, mut fill: F) -> Self
    where
        F: FnMut(usize, usize) -> bool,
    {
        let capacity = w.checked_mul(h).expect("Image dimensions caused overflow");
        let mut pixels = Vec::with_capacity(capacity);

        for y in 0..h {
            for x in 0..w {
                let col = if fill(x, y) { PixelColor::Black } else { PixelColor::White };
                pixels.push(col.into())
            }
        }
        let pixels = pixels.into_boxed_slice();
        let buffer = BasicImageBuffer { w, h, pixels };
        PreparedImage::without_preparation(buffer)
    }

    /// Given a byte valued function, generate a searchable image
    ///
    /// The values returned by the function are interpreted as luminance. i.e. a
    /// value of 0 is black, 255 is white.
    pub fn prepare_from_greyscale<F>(w: usize, h: usize, mut fill: F) -> Self
    where
        F: FnMut(usize, usize) -> u8,
    {
        let capacity = w.checked_mul(h).expect("Image dimensions caused overflow");
        let mut data = Vec::with_capacity(capacity);
        for y in 0..h {
            for x in 0..w {
                data.push(fill(x, y));
            }
        }
        let pixels = data.into_boxed_slice();
        let buffer = BasicImageBuffer { w, h, pixels };
        PreparedImage::prepare(buffer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn img_from_array(array: [[u8; 3]; 3]) -> PreparedImage<BasicImageBuffer> {
        let mut pixels = Vec::new();
        for col in array.iter() {
            for item in col.iter() {
                if *item == 0 { pixels.push(0) } else { pixels.push(1) }
            }
        }
        let buffer = BasicImageBuffer { w: 3, h: 3, pixels: pixels.into_boxed_slice() };

        PreparedImage { buffer, cache: LruCache::new(NonZeroUsize::new(251).unwrap()) }
    }

    #[test]
    fn test_flood_fill_full() {
        let mut test_full = img_from_array([[1, 1, 1], [1, 1, 1], [1, 1, 1]]);

        test_full.flood_fill(0, 0, 1, 2, &mut |_| ());

        for x in 0..3 {
            for y in 0..3 {
                assert_eq!(test_full.get_pixel_at(x, y), 2);
            }
        }
    }

    #[test]
    fn test_flood_fill_single() {
        let mut test_single = img_from_array([[1, 0, 1], [0, 1, 0], [1, 0, 1]]);

        test_single.flood_fill(1, 1, 1, 2, &mut |_| ());

        for x in 0..3 {
            for y in 0..3 {
                if x == 1 && y == 1 {
                    assert_eq!(test_single.get_pixel_at(x, y), 2);
                } else {
                    let col = if (x + y) % 2 == 0 { 1 } else { 0 };
                    assert_eq!(test_single.get_pixel_at(x, y), col);
                }
            }
        }
    }

    #[test]
    fn test_flood_fill_ring() {
        let mut test_ring = img_from_array([[1, 1, 1], [1, 0, 1], [1, 1, 1]]);

        test_ring.flood_fill(0, 0, 1, 2, &mut |_| ());

        for x in 0..3 {
            for y in 0..3 {
                if x == 1 && y == 1 {
                    assert_eq!(test_ring.get_pixel_at(x, y), 0);
                } else {
                    assert_eq!(test_ring.get_pixel_at(x, y), 2);
                }
            }
        }
    }

    #[test]
    fn test_flood_fill_u() {
        let mut test_u = img_from_array([[1, 0, 1], [1, 0, 1], [1, 1, 1]]);

        test_u.flood_fill(0, 0, 1, 2, &mut |_| ());

        for x in 0..3 {
            for y in 0..3 {
                if x == 1 && (y == 0 || y == 1) {
                    assert_eq!(test_u.get_pixel_at(x, y), 0);
                } else {
                    assert_eq!(test_u.get_pixel_at(x, y), 2);
                }
            }
        }
    }

    #[test]
    fn test_flood_fill_empty() {
        let mut test_empty = img_from_array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]);

        test_empty.flood_fill(1, 1, 1, 2, &mut |_| ());

        for x in 0..3 {
            for y in 0..3 {
                assert_eq!(test_empty.get_pixel_at(x, y), 0)
            }
        }
    }

    #[test]
    fn test_get_region() {
        let mut test_u = img_from_array([[1, 0, 1], [1, 0, 1], [1, 1, 1]]);

        let reg = test_u.get_region((0, 0));
        let (color, src_x, src_y, pixel_count) = match reg {
            ColoredRegion::Unclaimed { color, src_x, src_y, pixel_count } => {
                (color, src_x, src_y, pixel_count)
            }
            x => panic!("Expected Region::Unclaimed, got {:?}", x),
        };
        assert_eq!(0, src_x);
        assert_eq!(0, src_y);
        assert_eq!(7, pixel_count);
        for x in 0..3 {
            for y in 0..3 {
                if x == 1 && (y == 0 || y == 1) {
                    assert_eq!(PixelColor::White, test_u.get_pixel_at(x, y));
                } else {
                    assert_eq!(color, test_u.get_pixel_at(x, y));
                }
            }
        }
    }

    #[test]
    fn test_otsu_threshold_separates_clusters() {
        let mut pixels = Vec::new();

        for v in [15u8, 18, 20, 22, 25] {
            pixels.extend(std::iter::repeat(v).take(20));
        }

        for v in [180u8, 190, 200, 210, 220] {
            pixels.extend(std::iter::repeat(v).take(20));
        }

        let buffer = BasicImageBuffer { w: pixels.len(), h: 1, pixels: pixels.into_boxed_slice() };

        let threshold = compute_otsu_threshold(&buffer);

        // Semantic correctness
        for &p in buffer.pixels.iter() {
            if p <= 25 {
                assert!(p <= threshold);
            } else {
                assert!(p > threshold);
            }
        }
    }
}
