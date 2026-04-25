//! Debug logging module for QR code detection and decoding
//!
//! This module provides comprehensive debug output for analyzing QR code
//! detection failures, including perspective correction, capstone detection,
//! ECC info, and decoded grid visualization.

use std::fmt::Write as FmtWrite;

use crate::geometry::Perspective;
use crate::identify::match_capstones::CapStoneGroup;
use crate::prepare::{ImageBuffer, PreparedImage};
use crate::version_db::VERSION_DATA_BASE;
use crate::{BitGrid, CapStone, MetaData, Point};

/// Debug configuration - set to true to enable various debug outputs
pub struct DebugConfig {
    pub enabled: bool,
    pub log_capstones: bool,
    pub log_perspective: bool,
    pub log_timing: bool,
    pub log_alignment: bool,
    pub log_format: bool,
    pub log_ecc: bool,
    pub log_data_bits: bool,
    pub log_grid_ascii: bool,
    pub log_raw_binary: bool,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            log_capstones: true,
            log_perspective: true,
            log_timing: true,
            log_alignment: true,
            log_format: true,
            log_ecc: true,
            log_data_bits: true,
            log_grid_ascii: true,
            log_raw_binary: true,
        }
    }
}

/// Global debug configuration - can be modified at runtime
pub static mut DEBUG_CONFIG: DebugConfig = DebugConfig {
    enabled: true,
    log_capstones: true,
    log_perspective: true,
    log_timing: true,
    log_alignment: true,
    log_format: true,
    log_ecc: true,
    log_data_bits: true,
    log_grid_ascii: true,
    log_raw_binary: true,
};

/// Check if debug is enabled
pub fn is_debug_enabled() -> bool {
    unsafe { DEBUG_CONFIG.enabled }
}

/// Macro for conditional debug logging
#[macro_export]
macro_rules! debug_log {
    ($($arg:tt)*) => {
        if $crate::debug::is_debug_enabled() {
            eprintln!("[QR-DEBUG] {}", format!($($arg)*));
        }
    };
}

/// Log detected capstones with their positions and corners
pub fn log_capstones(capstones: &[CapStone]) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_capstones {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== CAPSTONE DETECTION ==========");
    eprintln!("[QR-DEBUG] Total capstones detected: {}", capstones.len());

    for (i, cap) in capstones.iter().enumerate() {
        eprintln!("[QR-DEBUG] --- Capstone #{} ---", i);
        eprintln!("[QR-DEBUG]   Center: ({}, {})", cap.center.x, cap.center.y);
        eprintln!("[QR-DEBUG]   Corners:");
        for (j, corner) in cap.corners.iter().enumerate() {
            eprintln!("[QR-DEBUG]     Corner {}: ({}, {})", j, corner.x, corner.y);
        }
        // Calculate approximate size
        let width = ((cap.corners[1].x - cap.corners[0].x).pow(2) as f64
            + (cap.corners[1].y - cap.corners[0].y).pow(2) as f64)
            .sqrt();
        let height = ((cap.corners[3].x - cap.corners[0].x).pow(2) as f64
            + (cap.corners[3].y - cap.corners[0].y).pow(2) as f64)
            .sqrt();
        eprintln!("[QR-DEBUG]   Approximate size: {:.1} x {:.1}", width, height);
    }
}

/// Log capstone grouping attempts
pub fn log_capstone_group(group: &CapStoneGroup, idx: usize, pair: (usize, usize)) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_capstones {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== CAPSTONE GROUPING ==========");
    eprintln!("[QR-DEBUG] Testing group: capstones {}, {}, {}", pair.0, idx, pair.1);
    eprintln!("[QR-DEBUG]   Capstone A center: ({}, {})", group.0.center.x, group.0.center.y);
    eprintln!("[QR-DEBUG]   Capstone B center: ({}, {})", group.1.center.x, group.1.center.y);
    eprintln!("[QR-DEBUG]   Capstone C center: ({}, {})", group.2.center.x, group.2.center.y);

    // Calculate distances between capstones
    let dist_ab = ((group.1.center.x - group.0.center.x).pow(2) as f64
        + (group.1.center.y - group.0.center.y).pow(2) as f64)
        .sqrt();
    let dist_bc = ((group.2.center.x - group.1.center.x).pow(2) as f64
        + (group.2.center.y - group.1.center.y).pow(2) as f64)
        .sqrt();
    let dist_ac = ((group.2.center.x - group.0.center.x).pow(2) as f64
        + (group.2.center.y - group.0.center.y).pow(2) as f64)
        .sqrt();

    eprintln!("[QR-DEBUG]   Distance A-B: {:.1}", dist_ab);
    eprintln!("[QR-DEBUG]   Distance B-C: {:.1}", dist_bc);
    eprintln!("[QR-DEBUG]   Distance A-C: {:.1} (hypotenuse)", dist_ac);
}

/// Log clockwise ordering detection
pub fn log_clockwise_check(was_swapped: bool, h0: &Point, hd: &Point) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_capstones {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== ORIENTATION CHECK ==========");
    eprintln!("[QR-DEBUG] Hypotenuse start (h0): ({}, {})", h0.x, h0.y);
    eprintln!("[QR-DEBUG] Hypotenuse direction (hd): ({}, {})", hd.x, hd.y);
    if was_swapped {
        eprintln!("[QR-DEBUG] Capstones A and C were SWAPPED to ensure clockwise order");
        eprintln!("[QR-DEBUG] Image may be MIRRORED");
    } else {
        eprintln!("[QR-DEBUG] Capstones already in clockwise order (normal orientation)");
    }
}

/// Log capstone rotation
pub fn log_capstone_rotation(cap_name: &str, best_idx: usize, corners: &[Point; 4]) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_capstones {
            return;
        }
    }

    eprintln!("[QR-DEBUG] Capstone {} rotated by {} positions", cap_name, best_idx);
    eprintln!("[QR-DEBUG]   New corner order:");
    for (i, corner) in corners.iter().enumerate() {
        eprintln!("[QR-DEBUG]     Corner {}: ({}, {})", i, corner.x, corner.y);
    }
}

/// Log timing pattern measurement
pub fn log_timing_pattern(hscan: usize, vscan: usize, grid_size: usize) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_timing {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== TIMING PATTERN ==========");
    eprintln!("[QR-DEBUG] Horizontal scan transitions: {}", hscan);
    eprintln!("[QR-DEBUG] Vertical scan transitions: {}", vscan);
    eprintln!("[QR-DEBUG] Max transitions: {}", std::cmp::max(hscan, vscan));
    eprintln!("[QR-DEBUG] Calculated grid size: {}", grid_size);
    let version = (grid_size.saturating_sub(17)) / 4;
    eprintln!("[QR-DEBUG] QR Version: {} (grid size {})", version, version * 4 + 17);
}

/// Log alignment pattern search
pub fn log_alignment_search(initial_estimate: &Point, found: Option<&Point>, size_estimate: usize) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_alignment {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== ALIGNMENT PATTERN ==========");
    eprintln!(
        "[QR-DEBUG] Initial estimate position: ({}, {})",
        initial_estimate.x, initial_estimate.y
    );
    eprintln!("[QR-DEBUG] Expected size (pixels): ~{}", size_estimate);

    match found {
        Some(pos) => {
            eprintln!("[QR-DEBUG] Alignment pattern FOUND at: ({}, {})", pos.x, pos.y);
            let dx = pos.x - initial_estimate.x;
            let dy = pos.y - initial_estimate.y;
            eprintln!(
                "[QR-DEBUG] Offset from estimate: ({}, {}), distance: {:.1}",
                dx,
                dy,
                ((dx * dx + dy * dy) as f64).sqrt()
            );
        }
        None => {
            eprintln!("[QR-DEBUG] Alignment pattern NOT FOUND!");
            eprintln!("[QR-DEBUG] This may cause decoding to fail for version > 1 QR codes");
        }
    }
}

/// Log perspective transformation setup
pub fn log_perspective(perspective: &Perspective, grid_size: usize, stage: &str) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_perspective {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== PERSPECTIVE ({}) ==========", stage);
    eprintln!("[QR-DEBUG] Grid size: {}", grid_size);
    eprintln!("[QR-DEBUG] Perspective matrix coefficients:");
    eprintln!(
        "[QR-DEBUG]   c[0..3]: {:.6}, {:.6}, {:.6}",
        perspective.0[0], perspective.0[1], perspective.0[2]
    );
    eprintln!(
        "[QR-DEBUG]   c[3..6]: {:.6}, {:.6}, {:.6}",
        perspective.0[3], perspective.0[4], perspective.0[5]
    );
    eprintln!("[QR-DEBUG]   c[6..8]: {:.6}, {:.6}", perspective.0[6], perspective.0[7]);

    // Map corners to show the perspective transformation
    let corners = [
        (0.0, 0.0, "top-left"),
        (grid_size as f64, 0.0, "top-right"),
        (grid_size as f64, grid_size as f64, "bottom-right"),
        (0.0, grid_size as f64, "bottom-left"),
    ];

    eprintln!("[QR-DEBUG] Grid corner mappings (grid -> image):");
    for (gx, gy, name) in corners.iter() {
        let p = perspective.map(*gx, *gy);
        eprintln!("[QR-DEBUG]   {} ({:.1}, {:.1}) -> ({:.1}, {:.1})", name, gx, gy, p.x, p.y);
    }
}

/// Log jiggle perspective optimization
pub fn log_jiggle_perspective(initial_fitness: i32, final_fitness: i32, pass: usize) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_perspective {
            return;
        }
    }

    eprintln!(
        "[QR-DEBUG] Perspective jiggle pass {}: fitness {} -> {}",
        pass, initial_fitness, final_fitness
    );
}

/// Log format information reading
pub fn log_format_info(format_raw: u16, format_corrected: Result<u16, &str>, location: &str) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_format {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== FORMAT INFO ({}) ==========", location);
    eprintln!(
        "[QR-DEBUG] Raw format bits (after XOR): 0b{:015b} (0x{:04X})",
        format_raw, format_raw
    );

    match format_corrected {
        Ok(corrected) => {
            let fdata = corrected >> 10;
            let ecc_level = fdata >> 3;
            let mask = fdata & 7;
            let ecc_names = ["M (~15%)", "L (~7%)", "H (~30%)", "Q (~25%)"];
            eprintln!("[QR-DEBUG] Corrected format: 0b{:015b}", corrected);
            eprintln!(
                "[QR-DEBUG] ECC Level: {} ({})",
                ecc_level,
                ecc_names.get(ecc_level as usize).unwrap_or(&"unknown")
            );
            eprintln!("[QR-DEBUG] Mask pattern: {}", mask);
        }
        Err(e) => {
            eprintln!("[QR-DEBUG] Format correction FAILED: {}", e);
        }
    }
}

/// Log metadata extraction
pub fn log_metadata(meta: &MetaData) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_format {
            return;
        }
    }

    let ecc_names = ["M (~15%)", "L (~7%)", "H (~30%)", "Q (~25%)"];

    eprintln!("\n[QR-DEBUG] ========== QR CODE METADATA ==========");
    eprintln!(
        "[QR-DEBUG] Version: {} (grid size: {}x{})",
        meta.version.0,
        meta.version.to_size(),
        meta.version.to_size()
    );
    eprintln!(
        "[QR-DEBUG] ECC Level: {} ({})",
        meta.ecc_level,
        ecc_names.get(meta.ecc_level as usize).unwrap_or(&"unknown")
    );
    eprintln!("[QR-DEBUG] Mask Pattern: {}", meta.mask);

    if meta.version.0 > 0 && meta.version.0 <= 40 {
        let ver = &VERSION_DATA_BASE[meta.version.0];
        let ecc_params = &ver.ecc[meta.ecc_level as usize];
        eprintln!("[QR-DEBUG] Data bytes total: {}", ver.data_bytes);
        eprintln!("[QR-DEBUG] ECC parameters:");
        eprintln!("[QR-DEBUG]   Block size (bs): {}", ecc_params.bs);
        eprintln!("[QR-DEBUG]   Data words per block (dw): {}", ecc_params.dw);
        eprintln!("[QR-DEBUG]   Number of blocks (ns): {}", ecc_params.ns);
        eprintln!("[QR-DEBUG]   ECC bytes per block: {}", ecc_params.bs - ecc_params.dw);

        // Calculate total capacity
        let total_data_capacity = ecc_params.dw * ecc_params.ns;
        eprintln!("[QR-DEBUG]   Total data capacity: {} bytes", total_data_capacity);
    }
}

/// Log ECC correction attempt
pub fn log_ecc_correction(
    block_idx: usize,
    block_size: usize,
    syndromes_nonzero: bool,
    correction_success: bool,
) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_ecc {
            return;
        }
    }

    if syndromes_nonzero {
        eprintln!(
            "[QR-DEBUG] ECC Block {}: size={}, errors detected, correction {}",
            block_idx,
            block_size,
            if correction_success { "SUCCESS" } else { "FAILED" }
        );
    }
}

/// Log raw data extraction
pub fn log_raw_data(data: &[u8], bit_len: usize) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_raw_binary {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== RAW DATA STREAM ==========");
    eprintln!("[QR-DEBUG] Total bits: {}", bit_len);
    eprintln!("[QR-DEBUG] Total bytes: {}", bit_len.div_ceil(8));

    // Print first few bytes in hex and binary
    let bytes_to_show = std::cmp::min(bit_len.div_ceil(8), 32);
    eprintln!("[QR-DEBUG] First {} bytes (hex):", bytes_to_show);

    let mut hex_line = String::new();
    for (i, byte) in data.iter().take(bytes_to_show).enumerate() {
        write!(hex_line, "{:02X} ", byte).unwrap();
        if (i + 1) % 16 == 0 {
            eprintln!("[QR-DEBUG]   {}", hex_line);
            hex_line.clear();
        }
    }
    if !hex_line.is_empty() {
        eprintln!("[QR-DEBUG]   {}", hex_line);
    }

    // Show first 4 bits (mode indicator) decoded
    if bit_len >= 4 {
        let mode = (data[0] >> 4) & 0x0F;
        let mode_name = match mode {
            0 => "Terminator",
            1 => "Numeric",
            2 => "Alphanumeric",
            4 => "Byte",
            7 => "ECI",
            8 => "Kanji",
            _ => "Unknown",
        };
        eprintln!("[QR-DEBUG] Mode indicator: {} ({})", mode, mode_name);
    }
}

/// Output the decoded grid as ASCII art
pub fn log_grid_ascii<G: BitGrid + ?Sized>(grid: &G, meta: Option<&MetaData>) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_grid_ascii {
            return;
        }
    }

    let size = grid.size();
    eprintln!("\n[QR-DEBUG] ========== DECODED GRID ({}x{}) ==========", size, size);

    // Print column numbers
    eprint!("[QR-DEBUG]     ");
    for x in 0..size {
        if x % 10 == 0 {
            eprint!("{}", (x / 10) % 10);
        } else {
            eprint!(" ");
        }
    }
    eprintln!();

    eprint!("[QR-DEBUG]     ");
    for x in 0..size {
        eprint!("{}", x % 10);
    }
    eprintln!();

    // Print the grid
    for y in 0..size {
        eprint!("[QR-DEBUG] {:2}: ", y);
        for x in 0..size {
            let bit = grid.bit(y, x);
            // Use Unicode block characters for better visibility
            if bit {
                eprint!("██");
            } else {
                eprint!("  ");
            }
        }
        eprintln!();
    }

    // Also print a compact binary version
    if let Some(_meta) = meta {
        eprintln!("\n[QR-DEBUG] Binary representation (1=black, 0=white):");
        for y in 0..size {
            let mut row = String::new();
            for x in 0..size {
                row.push(if grid.bit(y, x) { '1' } else { '0' });
            }
            eprintln!("[QR-DEBUG] {:02}: {}", y, row);
        }
    }
}

/// Log the grid as raw binary data for external analysis
pub fn log_grid_binary<G: BitGrid + ?Sized>(grid: &G) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_raw_binary {
            return;
        }
    }

    let size = grid.size();
    eprintln!("\n[QR-DEBUG] ========== GRID BINARY DUMP ==========");

    // Output as a packed byte array
    let total_bits = size * size;
    let total_bytes = total_bits.div_ceil(8);
    let mut bytes = vec![0u8; total_bytes];

    for y in 0..size {
        for x in 0..size {
            let bit_idx = y * size + x;
            if grid.bit(y, x) {
                bytes[bit_idx / 8] |= 1 << (7 - (bit_idx % 8));
            }
        }
    }

    eprintln!("[QR-DEBUG] Grid packed as bytes (MSB first, row-major order):");
    for (i, chunk) in bytes.chunks(16).enumerate() {
        let hex: Vec<String> = chunk.iter().map(|b| format!("{:02X}", b)).collect();
        eprintln!("[QR-DEBUG] {:04X}: {}", i * 16, hex.join(" "));
    }
}

/// Log decoding result
pub fn log_decode_result(success: bool, mirrored: bool, error: Option<&str>) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[QR-DEBUG] ========== DECODE RESULT ==========");
    if success {
        eprintln!("[QR-DEBUG] Decoding: SUCCESS");
        if mirrored {
            eprintln!("[QR-DEBUG] Note: Image was decoded as MIRRORED");
        }
    } else {
        eprintln!("[QR-DEBUG] Decoding: FAILED");
        if let Some(err) = error {
            eprintln!("[QR-DEBUG] Error: {}", err);
        }
    }
}

/// Log neighbor finding for capstone matching
pub fn log_neighbor_search(
    idx: usize,
    cap: &CapStone,
    hlist: &[(usize, f64)],
    vlist: &[(usize, f64)],
) {
    if !is_debug_enabled() {
        return;
    }
    unsafe {
        if !DEBUG_CONFIG.log_capstones {
            return;
        }
    }

    eprintln!("\n[QR-DEBUG] ========== NEIGHBOR SEARCH (Capstone {}) ==========", idx);
    eprintln!("[QR-DEBUG] Reference capstone center: ({}, {})", cap.center.x, cap.center.y);

    eprintln!("[QR-DEBUG] Horizontal neighbors found: {}", hlist.len());
    for (i, (other_idx, dist)) in hlist.iter().enumerate() {
        eprintln!("[QR-DEBUG]   H{}: capstone {} at distance {:.1}", i, other_idx, dist);
    }

    eprintln!("[QR-DEBUG] Vertical neighbors found: {}", vlist.len());
    for (i, (other_idx, dist)) in vlist.iter().enumerate() {
        eprintln!("[QR-DEBUG]   V{}: capstone {} at distance {:.1}", i, other_idx, dist);
    }
}

/// Log image statistics
pub fn log_image_stats<S: ImageBuffer>(img: &PreparedImage<S>) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[QR-DEBUG] ========== IMAGE STATISTICS ==========");
    eprintln!("[QR-DEBUG] Image dimensions: {}x{}", img.width(), img.height());

    // Count pixel colors in a sample
    let mut white_count = 0u64;
    let mut black_count = 0u64;
    let mut other_count = 0u64;

    for y in 0..img.height() {
        for x in 0..img.width() {
            match img.get_pixel_byte(x, y) {
                0 => white_count += 1,
                1 => black_count += 1,
                _ => other_count += 1,
            }
        }
    }

    let total = (img.width() * img.height()) as f64;
    eprintln!(
        "[QR-DEBUG] White pixels: {} ({:.1}%)",
        white_count,
        100.0 * white_count as f64 / total
    );
    eprintln!(
        "[QR-DEBUG] Black pixels: {} ({:.1}%)",
        black_count,
        100.0 * black_count as f64 / total
    );
    if other_count > 0 {
        eprintln!(
            "[QR-DEBUG] Marked pixels: {} ({:.1}%)",
            other_count,
            100.0 * other_count as f64 / total
        );
    }
}

/// Save the decoded grid to a file for analysis
#[cfg(feature = "img")]
pub fn save_grid_image<G: BitGrid>(grid: &G, path: &str) {
    if !is_debug_enabled() {
        return;
    }

    let size = grid.size();
    let scale = 10; // 10x scale for visibility
    let mut img = image::GrayImage::new((size * scale) as u32, (size * scale) as u32);

    for y in 0..size {
        for x in 0..size {
            let color = if grid.bit(y, x) { 0u8 } else { 255u8 };
            for dy in 0..scale {
                for dx in 0..scale {
                    img.put_pixel(
                        (x * scale + dx) as u32,
                        (y * scale + dy) as u32,
                        image::Luma([color]),
                    );
                }
            }
        }
    }

    if let Err(e) = img.save(path) {
        eprintln!("[QR-DEBUG] Failed to save grid image to {}: {}", path, e);
    } else {
        eprintln!("[QR-DEBUG] Saved decoded grid image to: {}", path);
    }
}

/// Enable all debug output
pub fn enable_debug() {
    unsafe {
        DEBUG_CONFIG.enabled = true;
        DEBUG_CONFIG.log_capstones = true;
        DEBUG_CONFIG.log_perspective = true;
        DEBUG_CONFIG.log_timing = true;
        DEBUG_CONFIG.log_alignment = true;
        DEBUG_CONFIG.log_format = true;
        DEBUG_CONFIG.log_ecc = true;
        DEBUG_CONFIG.log_data_bits = true;
        DEBUG_CONFIG.log_grid_ascii = true;
        DEBUG_CONFIG.log_raw_binary = true;
    }
}

/// Disable all debug output
pub fn disable_debug() {
    unsafe {
        DEBUG_CONFIG.enabled = false;
    }
}

// ============================================================================
// rMQR (Rectangular Micro QR Code) Debug Functions
// ============================================================================

/// Log rMQR code detection attempt
pub fn log_rmqr_detection_attempt(width: usize, height: usize) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[rMQR-DEBUG] ========== rMQR DETECTION ATTEMPT ==========");
    eprintln!("[rMQR-DEBUG] Checking for rMQR code with dimensions: {}x{}", width, height);
}

/// Log rMQR format information
pub fn log_rmqr_format(format_value: u32, ecc_level: u8, version: u8, success: bool) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[rMQR-DEBUG] ========== rMQR FORMAT INFO ==========");
    eprintln!("[rMQR-DEBUG] Raw format value: 0x{:05X}", format_value);
    if success {
        eprintln!("[rMQR-DEBUG] Format decode: SUCCESS");
        eprintln!(
            "[rMQR-DEBUG]   ECC level: {} ({})",
            ecc_level,
            if ecc_level == 0 { "M - 15%" } else { "H - 30%" }
        );
        eprintln!("[rMQR-DEBUG]   Version indicator: {}", version);
    } else {
        eprintln!("[rMQR-DEBUG] Format decode: FAILED");
    }
}

/// Log rMQR version information
pub fn log_rmqr_version(version_indicator: u8, width: usize, height: usize) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[rMQR-DEBUG] ========== rMQR VERSION ==========");
    eprintln!("[rMQR-DEBUG] Version indicator: {}", version_indicator);
    eprintln!("[rMQR-DEBUG] Dimensions: {}x{} modules", width, height);

    // Calculate version name (e.g., R7x43)
    eprintln!("[rMQR-DEBUG] Version name: R{}x{}", height, width);
}

/// Log rMQR data reading progress
pub fn log_rmqr_data_read(bits_read: usize, total_expected: usize) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[rMQR-DEBUG] ========== rMQR DATA READ ==========");
    eprintln!("[rMQR-DEBUG] Bits read: {}", bits_read);
    eprintln!("[rMQR-DEBUG] Expected: ~{} bits", total_expected);
}

/// Log rMQR ECC processing
pub fn log_rmqr_ecc(num_blocks: usize, data_codewords: usize, ecc_codewords: usize, success: bool) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[rMQR-DEBUG] ========== rMQR ECC ==========");
    eprintln!("[rMQR-DEBUG] Number of blocks: {}", num_blocks);
    eprintln!("[rMQR-DEBUG] Data codewords: {}", data_codewords);
    eprintln!("[rMQR-DEBUG] ECC codewords: {}", ecc_codewords);
    if success {
        eprintln!("[rMQR-DEBUG] ECC correction: SUCCESS");
    } else {
        eprintln!("[rMQR-DEBUG] ECC correction: FAILED");
    }
}

/// Log rMQR payload decoding
pub fn log_rmqr_payload(mode: u8, count: usize, mode_name: &str) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("[rMQR-DEBUG] Mode: {} ({}) - {} characters", mode, mode_name, count);
}

/// Log rMQR grid visualization
pub fn log_rmqr_grid_ascii(width: usize, height: usize, grid_fn: impl Fn(usize, usize) -> bool) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[rMQR-DEBUG] ========== rMQR GRID ({}x{}) ==========", width, height);

    // Column numbers
    eprint!("[rMQR-DEBUG]     ");
    for x in 0..width {
        if x % 10 == 0 {
            eprint!("{}", (x / 10) % 10);
        } else {
            eprint!(" ");
        }
    }
    eprintln!();

    eprint!("[rMQR-DEBUG]     ");
    for x in 0..width {
        eprint!("{}", x % 10);
    }
    eprintln!();

    // Grid rows
    for y in 0..height {
        eprint!("[rMQR-DEBUG] {:2}: ", y);
        for x in 0..width {
            if grid_fn(y, x) {
                eprint!("██");
            } else {
                eprint!("  ");
            }
        }
        eprintln!();
    }
}

/// Log rMQR finder pattern detection
pub fn log_rmqr_finder(main_center: (i32, i32), sub_center: Option<(i32, i32)>, is_valid: bool) {
    if !is_debug_enabled() {
        return;
    }

    eprintln!("\n[rMQR-DEBUG] ========== rMQR FINDER PATTERNS ==========");
    eprintln!("[rMQR-DEBUG] Main finder center: ({}, {})", main_center.0, main_center.1);
    if let Some((x, y)) = sub_center {
        eprintln!("[rMQR-DEBUG] Sub-finder center: ({}, {})", x, y);
    } else {
        eprintln!("[rMQR-DEBUG] Sub-finder: NOT DETECTED");
    }
    if is_valid {
        eprintln!("[rMQR-DEBUG] Pattern validation: PASSED");
    } else {
        eprintln!("[rMQR-DEBUG] Pattern validation: FAILED");
    }
}
