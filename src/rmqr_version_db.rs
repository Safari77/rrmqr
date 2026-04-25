/* ***********************************************************************
 * rMQR (Rectangular Micro QR Code) version information database
 * ISO/IEC 23941:2022
 */

/// RS parameters for rMQR error correction
#[derive(Copy, Clone, Debug)]
pub struct RmqrRSParameters {
    pub bs: usize,
    pub dw: usize,
    pub ns: usize,
}

/// Character count indicator bits for different modes
#[derive(Copy, Clone, Debug)]
pub struct RmqrCharacterCountBits {
    pub numeric: usize,
    pub alphanumeric: usize,
    pub byte: usize,
    pub kanji: usize,
}

/// Information about a single rMQR version
#[derive(Copy, Clone, Debug)]
pub struct RmqrVersionInfo {
    pub version_indicator: u8,
    pub height: usize,
    pub width: usize,
    pub ecc: [RmqrRSParameters; 2],
    pub data_codewords: [usize; 2],
    pub char_count_bits: RmqrCharacterCountBits,
}

// Valid 18-bit sequences for the Main Finder (Top-Left)
pub const RMQR_FORMAT_PATTERNS_MAIN: [u32; 64] = [
    0x1FAB2, 0x1E597, 0x1DBDD, 0x1C4F8, 0x1B86C, 0x1A749, 0x19903, 0x18626, 0x17F0E, 0x1602B,
    0x15E61, 0x14144, 0x13DD0, 0x122F5, 0x11CBF, 0x1039A, 0x0F1CA, 0x0EEEF, 0x0D0A5, 0x0CF80,
    0x0B314, 0x0AC31, 0x0927B, 0x08D5E, 0x07476, 0x06B53, 0x05519, 0x04A3C, 0x036A8, 0x0298D,
    0x017C7, 0x008E2, 0x3F367, 0x3EC42, 0x3D208, 0x3CD2D, 0x3B1B9, 0x3AE9C, 0x390D6, 0x38FF3,
    0x376DB, 0x369FE, 0x357B4, 0x34891, 0x33405, 0x32B20, 0x3156A, 0x30A4F, 0x2F81F, 0x2E73A,
    0x2D970, 0x2C655, 0x2BAC1, 0x2A5E4, 0x29BAE, 0x2848B, 0x27DA3, 0x26286, 0x25CCC, 0x243E9,
    0x23F7D, 0x22058, 0x21E12, 0x20137,
];

// Valid 18-bit sequences for the Sub Finder (Bottom-Right)
pub const RMQR_FORMAT_PATTERNS_SUB: [u32; 64] = [
    0x20A7B, 0x2155E, 0x22B14, 0x23431, 0x248A5, 0x25780, 0x269CA, 0x276EF, 0x28FC7, 0x290E2,
    0x2AEA8, 0x2B18D, 0x2CD19, 0x2D23C, 0x2EC76, 0x2F353, 0x30103, 0x31E26, 0x3206C, 0x33F49,
    0x343DD, 0x35CF8, 0x362B2, 0x37D97, 0x384BF, 0x39B9A, 0x3A5D0, 0x3BAF5, 0x3C661, 0x3D944,
    0x3E70E, 0x3F82B, 0x003AE, 0x01C8B, 0x022C1, 0x03DE4, 0x04170, 0x05E55, 0x0601F, 0x07F3A,
    0x08612, 0x09937, 0x0A77D, 0x0B858, 0x0C4CC, 0x0DBE9, 0x0E5A3, 0x0FA86, 0x108D6, 0x117F3,
    0x129B9, 0x1369C, 0x14A08, 0x1552D, 0x16B67, 0x17442, 0x18D6A, 0x1924F, 0x1AC05, 0x1B320,
    0x1CFB4, 0x1D091, 0x1EEDB, 0x1F1FE,
];

/// Attempt to correct an 18-bit format pattern using BCH codes.
/// Returns the best matching valid pattern if within error tolerance.
pub fn correct_rmqr_format_pattern(read_bits: u32, is_sub_finder: bool) -> Option<u32> {
    let patterns =
        if is_sub_finder { &RMQR_FORMAT_PATTERNS_SUB } else { &RMQR_FORMAT_PATTERNS_MAIN };

    let mut best_pattern = 0;
    let mut min_distance = u32::MAX;

    for &pattern in patterns {
        let diff = read_bits ^ pattern;
        let distance = diff.count_ones();

        if distance < min_distance {
            min_distance = distance;
            best_pattern = pattern;
        }
    }

    // Allow up to 3 bit errors
    if min_distance <= 3 { Some(best_pattern) } else { None }
}

/// Extract the 6-bit data value from a corrected pattern
pub fn extract_format_data(corrected_pattern: u32, is_sub_finder: bool) -> u32 {
    let patterns =
        if is_sub_finder { &RMQR_FORMAT_PATTERNS_SUB } else { &RMQR_FORMAT_PATTERNS_MAIN };
    patterns.iter().position(|&p| p == corrected_pattern).unwrap_or(0) as u32
}

/// rMQR version from dimensions
pub fn rmqr_version_from_dimensions(height: usize, width: usize) -> Option<usize> {
    for (idx, info) in RMQR_VERSION_DATA.iter().enumerate() {
        if info.height == height && info.width == width {
            return Some(idx);
        }
    }
    None
}

#[inline]
pub fn rmqr_mask_bit(row: usize, col: usize) -> bool {
    ((row / 2) + (col / 3)).is_multiple_of(2)
}

/// Check if dimensions could be a valid rMQR code
pub fn is_valid_rmqr_dimensions(height: usize, width: usize) -> bool {
    for info in &RMQR_VERSION_DATA {
        if info.height == height && info.width == width {
            return true;
        }
    }
    false
}

/// Get alignment pattern center positions (x coordinates) for a given width.
/// For wide codes (Width >= 43), these columns contain a vertical timing pattern.
pub fn get_alignment_positions(width: usize) -> &'static [usize] {
    match width {
        43 => &[21],
        59 => &[19, 39],
        77 => &[25, 51],
        99 => &[23, 49, 75],
        139 => &[27, 55, 83, 111],
        _ => &[],
    }
}

/// Check if a cell is reserved (not data) for rMQR.
///
/// rMQR has specific reserved areas:
/// - Frame/timing on all edges (row 0, row h-1, col 0, col w-1)
/// - Main finder 7x7 at top-left + separator
/// - Format info near main finder
/// - Corner finder patterns (3-module triangular) at top-right and bottom-left
/// - Sub-finder 5x5 at bottom-right (height > 7) or 5x7 (height == 7)
/// - Alignment patterns for wide codes
pub fn is_rmqr_reserved(height: usize, width: usize, row: usize, col: usize) -> bool {
    // 1. FRAME / TIMING PATTERNS (edges)
    if row == 0 || row == height - 1 || col == 0 || col == width - 1 {
        return true;
    }

    // 2. MAIN FINDER (7x7) + SEPARATOR (Top-Left)
    // The finder is 7x7 at (0,0), plus 1 module white separator
    if row <= 7 && col <= 7 {
        return true;
    }

    // 3. TOP-LEFT FORMAT INFO
    // Vertical strip at col 11, rows 1-3
    if col == 11 && (1..=3).contains(&row) {
        return true;
    }
    // Block at cols 8-10, rows 1-5
    if (8..=10).contains(&col) && (1..=5).contains(&row) {
        return true;
    }

    // 4. CORNER FINDER (Top-Right) - 3-module triangular pattern
    // Pattern shape:
    //   col:  w-3  w-2  w-1
    //   row 0:  B    B    B   <- already covered by timing (row 0)
    //   row 1:       B    B
    //   row 2:            B
    //
    // So we need to mark:
    // - (row 1, col w-2) and (row 1, col w-1) <- w-1 is timing
    // - (row 2, col w-1) <- w-1 is timing
    // Only (row 1, col w-2) is not already covered
    if row == 1 && col == width - 2 {
        return true;
    }

    // 5. CORNER FINDER (Bottom-Left) - 3-module triangular pattern (height > 7 only)
    // Pattern shape:
    //   col:      0    1    2
    //   row h-3:  B
    //   row h-2:  B    W
    //   row h-1:  B    B    B  <- already timing (row h-1)
    //
    if height > 7 && row == height - 2 && col == 1 {
        return true; // White module of corner pattern
    }

    // 6. SUB-FINDER & BOTTOM-RIGHT FORMAT INFO
    {
        let h = height;
        let w = width;

        if h == 7 {
            // For height 7, sub-finder spans more of the right edge
            // Sub-finder is at bottom-right, 5 modules wide, full height
            if col >= w.saturating_sub(8) && col <= w.saturating_sub(1) && row >= 1 && row <= h - 2
            {
                return true;
            }
        } else {
            // For height > 7:
            // Sub-finder 5x5 at bottom-right corner area
            // Plus format info extending from it

            // Sub-finder core (5x5 at bottom-right)
            if col >= w.saturating_sub(8)
                && col <= w.saturating_sub(3)
                && row >= h.saturating_sub(6)
                && row <= h.saturating_sub(2)
            {
                return true;
            }
            // Format info extending right of sub-finder
            if col >= w.saturating_sub(2) && row >= h.saturating_sub(5) {
                return true;
            }
        }
    }

    // 7. ALIGNMENT PATTERNS & VERTICAL TIMING
    if width >= 43 {
        let align_cols = get_alignment_positions(width);
        let top_align_row = 1;
        let bottom_align_row = height.saturating_sub(2);

        for &cx in align_cols {
            // A. Vertical Timing Line (entire column between top and bottom edges)
            if col == cx && row >= top_align_row && row <= bottom_align_row {
                return true;
            }

            // B. Alignment patterns at top and bottom of each vertical timing
            // Each alignment pattern is 3x3 centered on (cx, top_row) and (cx, bottom_row)
            for &cy in &[top_align_row, bottom_align_row] {
                // Check if (row, col) is within the 3x3 area of the alignment pattern
                if row >= cy.saturating_sub(1)
                    && row <= cy + 1
                    && col >= cx.saturating_sub(1)
                    && col <= cx + 1
                {
                    return true;
                }
            }
        }
    }

    false
}

macro_rules! v {
    (
        $ver:expr, $h:expr, $w:expr,
        ($m_bs:expr, $m_dw:expr, $m_ns:expr),
        ($h_bs:expr, $h_dw:expr, $h_ns:expr),
        [$dm:expr, $dh:expr],
        ($cn:expr, $ca:expr, $cb:expr, $ck:expr)
    ) => {
        RmqrVersionInfo {
            version_indicator: $ver,
            height: $h,
            width: $w,
            ecc: [
                RmqrRSParameters { bs: $m_bs, dw: $m_dw, ns: $m_ns },
                RmqrRSParameters { bs: $h_bs, dw: $h_dw, ns: $h_ns },
            ],
            data_codewords: [$dm, $dh],
            char_count_bits: RmqrCharacterCountBits {
                numeric: $cn,
                alphanumeric: $ca,
                byte: $cb,
                kanji: $ck,
            },
        }
    };
}

pub const RMQR_VERSION_DATA: [RmqrVersionInfo; 32] = [
    v!(0, 7, 43, (13, 6, 1), (13, 3, 1), [6, 3], (4, 3, 3, 2)),
    v!(1, 7, 59, (21, 12, 1), (21, 7, 1), [12, 7], (5, 5, 4, 3)),
    v!(2, 7, 77, (32, 20, 1), (32, 10, 1), [20, 10], (6, 5, 5, 4)),
    v!(3, 7, 99, (44, 28, 1), (44, 14, 1), [28, 14], (7, 6, 5, 5)),
    v!(4, 7, 139, (68, 44, 1), (68, 24, 2), [44, 24], (7, 6, 6, 5)),
    v!(5, 9, 43, (21, 12, 1), (21, 7, 1), [12, 7], (5, 5, 4, 3)),
    v!(6, 9, 59, (33, 21, 1), (33, 11, 1), [21, 11], (6, 5, 5, 4)),
    v!(7, 9, 77, (49, 31, 1), (49, 17, 2), [31, 17], (7, 6, 5, 5)),
    v!(8, 9, 99, (66, 42, 1), (66, 22, 2), [42, 22], (7, 6, 6, 5)),
    v!(9, 9, 139, (99, 63, 2), (99, 33, 3), [63, 33], (8, 7, 6, 6)),
    v!(10, 11, 27, (15, 7, 1), (15, 5, 1), [7, 5], (4, 4, 3, 2)),
    v!(11, 11, 43, (31, 19, 1), (31, 11, 1), [19, 11], (6, 5, 5, 4)),
    v!(12, 11, 59, (47, 31, 1), (47, 15, 2), [31, 15], (7, 6, 5, 5)),
    v!(13, 11, 77, (67, 43, 1), (67, 23, 2), [43, 23], (7, 6, 6, 5)),
    v!(14, 11, 99, (89, 57, 2), (89, 29, 2), [57, 29], (8, 7, 6, 6)),
    v!(15, 11, 139, (132, 84, 2), (132, 42, 3), [84, 42], (8, 7, 7, 6)),
    v!(16, 13, 27, (21, 12, 1), (21, 7, 1), [12, 7], (5, 5, 4, 3)),
    v!(17, 13, 43, (41, 27, 1), (41, 13, 1), [27, 13], (6, 6, 5, 5)),
    v!(18, 13, 59, (60, 38, 1), (60, 20, 2), [38, 20], (7, 6, 6, 5)),
    v!(19, 13, 77, (85, 53, 2), (85, 29, 2), [53, 29], (7, 7, 6, 6)),
    v!(20, 13, 99, (113, 73, 2), (113, 35, 3), [73, 35], (8, 7, 7, 6)),
    v!(21, 13, 139, (166, 106, 3), (166, 54, 4), [106, 54], (8, 8, 7, 7)),
    v!(22, 15, 43, (51, 33, 1), (51, 15, 2), [33, 15], (7, 6, 6, 5)),
    v!(23, 15, 59, (74, 48, 1), (74, 26, 2), [48, 26], (7, 7, 6, 5)),
    v!(24, 15, 77, (103, 67, 2), (103, 31, 3), [67, 31], (8, 7, 7, 6)),
    v!(25, 15, 99, (136, 88, 2), (136, 48, 4), [88, 48], (8, 7, 7, 6)),
    v!(26, 15, 139, (199, 127, 3), (199, 69, 5), [127, 69], (9, 8, 7, 7)),
    v!(27, 17, 43, (61, 39, 1), (61, 21, 2), [39, 21], (7, 6, 6, 5)),
    v!(28, 17, 59, (88, 56, 2), (88, 28, 2), [56, 28], (8, 7, 6, 6)),
    v!(29, 17, 77, (122, 78, 2), (122, 38, 3), [78, 38], (8, 7, 7, 6)),
    v!(30, 17, 99, (160, 100, 3), (160, 56, 4), [100, 56], (8, 8, 7, 6)),
    v!(31, 17, 139, (232, 152, 4), (232, 76, 6), [152, 76], (9, 8, 8, 7)),
];

/// Valid rMQR heights
pub const RMQR_VALID_HEIGHTS: [usize; 6] = [7, 9, 11, 13, 15, 17];
/// Valid rMQR widths
pub const RMQR_VALID_WIDTHS: [usize; 6] = [27, 43, 59, 77, 99, 139];
