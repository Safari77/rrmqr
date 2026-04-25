/* rmqr_decode.rs */
//! rMQR (Rectangular Micro QR Code) decoding

use crate::rmqr_ecc;
use crate::rmqr_version_db::{
    RMQR_VERSION_DATA, RmqrCharacterCountBits, RmqrVersionInfo, correct_rmqr_format_pattern,
    extract_format_data, is_rmqr_reserved, rmqr_mask_bit, rmqr_version_from_dimensions,
};
use crate::{DeQRError, DeQRResult};
use std::io::Write;

/// Maximum rMQR payload size in bytes
pub const RMQR_MAX_PAYLOAD_SIZE: usize = 256;

// ============================================================================
// CONFIGURABLE DEBUG PARAMETERS
// ============================================================================

const DEBUG_SHOW_BIT_POSITIONS: bool = true;
const DEBUG_BIT_SAMPLE_COUNT: usize = 64;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/// rMQR version information
// ADDED: PartialEq to allow comparison (==)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RmqrVersion {
    pub version_indicator: u8,
    pub height: usize,
    pub width: usize,
}

impl RmqrVersion {
    pub fn from_dimensions(height: usize, width: usize) -> DeQRResult<Self> {
        rmqr_version_from_dimensions(height, width)
            .map(|idx| RmqrVersion {
                version_indicator: RMQR_VERSION_DATA[idx].version_indicator,
                height,
                width,
            })
            .ok_or(DeQRError::InvalidVersion)
    }

    pub fn info(&self) -> &'static RmqrVersionInfo {
        &RMQR_VERSION_DATA[self.version_indicator as usize]
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RmqrMetaData {
    pub version: RmqrVersion,
    pub ecc_level: u8,
}

#[derive(Clone)]
pub struct RmqrRawData {
    pub data: [u8; RMQR_MAX_PAYLOAD_SIZE],
    pub len: usize,
}

impl RmqrRawData {
    pub fn new() -> Self {
        RmqrRawData { data: [0; RMQR_MAX_PAYLOAD_SIZE], len: 0 }
    }

    pub fn push(&mut self, bit: bool) {
        assert!(self.len / 8 < RMQR_MAX_PAYLOAD_SIZE);
        let bitpos = (self.len & 7) as u8;
        let bytepos = self.len >> 3;
        if bit {
            self.data[bytepos] |= 0x80_u8 >> bitpos;
        }
        self.len += 1;
    }
}

impl Default for RmqrRawData {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone)]
pub struct RmqrCorrectedStream {
    pub data: [u8; RMQR_MAX_PAYLOAD_SIZE],
    pub ptr: usize,
    pub bit_len: usize,
}

impl RmqrCorrectedStream {
    pub fn bits_remaining(&self) -> usize {
        self.bit_len.saturating_sub(self.ptr)
    }

    pub fn take_bits(&mut self, nbits: usize) -> usize {
        let mut ret = 0;
        let max_len = self.bits_remaining().min(nbits);
        for _ in 0..max_len {
            let b = self.data[self.ptr >> 3];
            let bitpos = self.ptr & 7;
            ret <<= 1;
            if 0 != (b << bitpos) & 0x80 {
                ret |= 1;
            }
            self.ptr += 1;
        }
        ret
    }
}

/// Trait for rectangular bit grid access
pub trait RectBitGrid {
    fn width(&self) -> usize;
    fn height(&self) -> usize;
    fn bit(&self, row: usize, col: usize) -> bool;
    fn get_pixel_coords(&self, _row: usize, _col: usize) -> Option<(f64, f64)> {
        None
    }
}

// ============================================================================
// FORMAT READING
// ============================================================================

fn read_format_from_finder<G>(grid: &G) -> u32
where
    G: RectBitGrid,
{
    let mut format = 0u32;
    for y in (1..=3).rev() {
        if 11 < grid.width() && y < grid.height() {
            let bit = grid.bit(y, 11);
            format = (format << 1) | (bit as u32);
        }
    }
    for x in (8..=10).rev() {
        for y in (1..=5).rev() {
            if x < grid.width() && y < grid.height() {
                let bit = grid.bit(y, x);
                format = (format << 1) | (bit as u32);
            }
        }
    }
    format
}

fn read_format_from_sub_finder<G>(grid: &G) -> u32
where
    G: RectBitGrid,
{
    let mut format = 0u32;
    let width = grid.width();
    let height = grid.height();
    if width < 8 || height < 7 {
        return 0;
    }

    let row_top = height - 6;
    format = (format << 1) | (grid.bit(row_top, width - 3) as u32);
    format = (format << 1) | (grid.bit(row_top, width - 4) as u32);
    format = (format << 1) | (grid.bit(row_top, width - 5) as u32);

    for x_off in 6..=8 {
        let col = width - x_off;
        for y_off in 2..=6 {
            let row = height - y_off;
            format = (format << 1) | (grid.bit(row, col) as u32);
        }
    }
    format
}

// ============================================================================
// MAIN DECODE ENTRY POINT
// ============================================================================

pub fn decode_rmqr<W, G>(grid: &G, writer: W) -> DeQRResult<RmqrMetaData>
where
    W: Write,
    G: RectBitGrid,
{
    decode_rmqr_with_index(grid, writer, 0)
}

pub fn decode_rmqr_with_index<W, G>(
    grid: &G,
    mut writer: W,
    grid_index: usize,
) -> DeQRResult<RmqrMetaData>
where
    W: Write,
    G: RectBitGrid,
{
    crate::debug_log!("=== Starting rMQR decode ===");
    let gw = grid.width();
    let gh = grid.height();
    crate::debug_log!("Grid #{}: size {}x{}", grid_index, gw, gh);

    // 1. Read Format Info (Treat as a HINT, not absolute truth)
    let format_hint = read_rmqr_format(grid).ok();

    if let Some(ref fmt) = format_hint {
        crate::debug_log!(
            "[rMQR] Format bits Hint: version={:?}, ecc={}",
            fmt.version.version_indicator,
            if fmt.ecc_level == 0 { "M" } else { "H" }
        );
    }

    // 2. Identify Version from Physical Grid Dimensions
    let physical_version = RmqrVersion::from_dimensions(gh, gw).ok();

    if let Some(ref v) = physical_version {
        crate::debug_log!(
            "[rMQR] Physical grid dimensions implies Version: {:?}",
            v.version_indicator
        );
        // Check for mismatch
        if let Some(ref fmt) = format_hint
            && fmt.version != *v
        {
            crate::debug_log!(
                "[rMQR] WARNING: Mismatch! Grid says v{} but Format bits say v{}. Prioritizing Grid.",
                v.version_indicator,
                fmt.version.version_indicator
            );
        }
    } else {
        crate::debug_log!(
            "[rMQR] Warning: Grid dimensions {}x{} do not match any standard rMQR version.",
            gw,
            gh
        );
    }

    // 3. Build Candidates Queue
    let mut attempts = Vec::new();

    // Default ECC to try first (M=0) if we have absolutely no info
    let hint_ecc = format_hint.as_ref().map(|m| m.ecc_level).unwrap_or(0);

    // PRIORITY 1: Physical Version (Try Hint ECC, then the other)
    if let Some(ver) = physical_version {
        // Attempt 1: Physical version + Hint ECC
        attempts.push((
            RmqrMetaData { version: ver, ecc_level: hint_ecc },
            "Physical/Hint".to_string(),
        ));
        // Attempt 2: Physical version + Swapped ECC
        attempts.push((
            RmqrMetaData { version: ver, ecc_level: 1 - hint_ecc },
            "Physical/Swap".to_string(),
        ));
    }

    // PRIORITY 2: Format Info Version (Only if different from Physical)
    if let Some(fmt) = &format_hint {
        let fmt_ver_already_covered = attempts.iter().any(|(m, _)| m.version == fmt.version);
        if !fmt_ver_already_covered {
            attempts.push((
                RmqrMetaData { version: fmt.version, ecc_level: fmt.ecc_level },
                "Format/Hint".to_string(),
            ));
            attempts.push((
                RmqrMetaData { version: fmt.version, ecc_level: 1 - fmt.ecc_level },
                "Format/Swap".to_string(),
            ));
        }
    }

    if attempts.is_empty() {
        return Err(DeQRError::FormatEcc);
    }

    // 4. Iterate and Decode
    for (idx, (meta, source)) in attempts.into_iter().enumerate() {
        // If the version we are trying to decode doesn't match the grid we found,
        // it is physically impossible for the decode to work. Skip it.
        if meta.version.width != gw || meta.version.height != gh {
            crate::debug_log!(
                "[rMQR] Skipping Attempt #{} [{}]: Version {}x{} does not match Grid {}x{}",
                idx + 1,
                source,
                meta.version.width,
                meta.version.height,
                gw,
                gh
            );
            continue;
        }

        crate::debug_log!(
            "[rMQR] Attempt #{} [{}]: Version={} ({:?}x{:?}), ECC={}",
            idx + 1,
            source,
            meta.version.version_indicator,
            meta.version.width,
            meta.version.height,
            if meta.ecc_level == 0 { "M" } else { "H" }
        );

        // Call read_rmqr_data inside loop to respect the Version in 'meta'
        let raw = read_rmqr_data(grid, &meta, grid_index);

        if raw.len == 0 {
            crate::debug_log!("[rMQR] Attempt produced 0 bits. Skipping.");
            continue;
        }

        let candidates = rmqr_ecc::rmqr_ecc_candidates(&meta, &raw, &[]);

        if candidates.is_empty() {
            crate::debug_log!("[rMQR] No valid ECC solutions for this config.");
        }

        for (stream, error_count) in candidates {
            let mut temp_buf = Vec::new();

            match decode_rmqr_payload(&meta, stream, &mut temp_buf) {
                Ok(_) => {
                    crate::debug_log!(
                        "[rMQR] SUCCESS: Decoded using {} (errors={}). Payload valid.",
                        source,
                        error_count
                    );

                    writer.write_all(&temp_buf).map_err(|_| DeQRError::IoError)?;

                    return Ok(meta);
                }
                Err(e) => {
                    crate::debug_log!(
                        "[rMQR] Warning: ECC Valid (errors={}) but Payload Failed ({:?}).",
                        error_count,
                        e
                    );
                }
            }
        }
    }

    crate::debug_log!("[rMQR] Failed to decode on all Version/ECC candidates.");
    Err(DeQRError::DataEcc)
}

fn read_rmqr_format<G>(grid: &G) -> DeQRResult<RmqrMetaData>
where
    G: RectBitGrid,
{
    crate::debug_log!("[rMQR] Reading format information...");
    let width = grid.width();
    let height = grid.height();

    // ATTEMPT 1: Main Finder (Top-Left)
    let format1 = read_format_from_finder(grid);
    crate::debug_log!("[rMQR] Raw bits from finder: 0x{:05X}", format1);

    if let Some(corrected) = correct_rmqr_format_pattern(format1, false) {
        let data_val = extract_format_data(corrected, false);
        if let Ok(meta) = decode_format_bits(data_val, width, height) {
            return Ok(meta);
        }
    }

    // ATTEMPT 2: Sub Finder (Bottom-Right)
    let format2 = read_format_from_sub_finder(grid);
    crate::debug_log!("[rMQR] Raw bits from sub-finder: 0x{:05X}", format2);

    if let Some(corrected) = correct_rmqr_format_pattern(format2, true) {
        let data_val = extract_format_data(corrected, true);
        if let Ok(meta) = decode_format_bits(data_val, width, height) {
            return Ok(meta);
        }
    }

    crate::debug_log!("[rMQR] Format reading failed (or version mismatch)");
    Err(DeQRError::FormatEcc)
}

fn decode_format_bits(data_val: u32, width: usize, height: usize) -> DeQRResult<RmqrMetaData> {
    let ecc_level = ((data_val >> 5) & 1) as u8;
    let version_indicator = (data_val & 0x1F) as u8;

    // Construct version from indicator (trusting bits)
    if version_indicator as usize >= RMQR_VERSION_DATA.len() {
        return Err(DeQRError::FormatEcc);
    }
    let info = &RMQR_VERSION_DATA[version_indicator as usize];

    let version = RmqrVersion { version_indicator, width: info.width, height: info.height };

    if info.width != width || info.height != height {
        crate::debug_log!(
            "[rMQR] WARNING: format version {} ({}x{}) mismatches grid ({}x{}). Trusting format.",
            version_indicator,
            info.width,
            info.height,
            width,
            height
        );
    }

    crate::debug_log!(
        "[rMQR] Decoded format: ecc_level={}, version_indicator={}",
        ecc_level,
        version_indicator
    );

    Ok(RmqrMetaData { version, ecc_level })
}

// ============================================================================
// DATA READING
// ============================================================================

fn read_rmqr_data<G>(grid: &G, meta: &RmqrMetaData, grid_index: usize) -> RmqrRawData
where
    G: RectBitGrid,
{
    let mut data = RmqrRawData::new();
    let width = grid.width();
    let height = grid.height();
    let info = meta.version.info();
    let ecc_params = &info.ecc[meta.ecc_level as usize];
    let total_codewords = ecc_params.bs;
    let capacity_bits = total_codewords * 8;

    // Print the entire grid
    crate::debug_log!("[rMQR] Grid #{} contents ({}x{}):", grid_index, width, height);
    for row in 0..height {
        let mut row_str = String::new();
        for col in 0..width {
            let bit = grid.bit(row, col);
            let reserved = is_rmqr_reserved(height, width, row, col);
            if reserved {
                row_str.push(if bit { 'X' } else { '.' });
            } else {
                row_str.push(if bit { '1' } else { '0' });
            }
        }
        crate::debug_log!("[rMQR] Row {:2}: {}", row, row_str);
    }

    // Print reserved area map
    crate::debug_log!("[rMQR] Reserved area map:");
    for row in 0..height {
        let mut row_str = String::new();
        for col in 0..width {
            row_str.push(if is_rmqr_reserved(height, width, row, col) { 'R' } else { 'D' });
        }
        crate::debug_log!("[rMQR] Row {:2}: {}", row, row_str);
    }

    crate::debug_log!(
        "[rMQR] Reading data: Capacity = {} bytes ({} bits)",
        total_codewords,
        capacity_bits
    );

    // Collect bit positions for debug (bit_idx, row, col, px, py, raw_bit, masked_bit)
    let mut bit_positions: Vec<(usize, usize, usize, f64, f64, bool, bool)> = Vec::new();

    let mut col = width - 2;
    while col > 0 {
        let col_right = col;
        let col_left = if col > 0 { col - 1 } else { 0 };
        let pair_index = (width - 1 - col) / 2;
        let going_up = pair_index.is_multiple_of(2);

        if going_up {
            for row in (0..height).rev() {
                if !is_rmqr_reserved(height, width, row, col_right) {
                    let raw_bit = grid.bit(row, col_right);
                    let mask_bit = rmqr_mask_bit(row, col_right);
                    let final_bit = raw_bit ^ mask_bit;
                    let (px, py) = grid.get_pixel_coords(row, col_right).unwrap_or((-1.0, -1.0));
                    bit_positions.push((data.len, row, col_right, px, py, raw_bit, final_bit));
                    data.push(final_bit);
                    if data.len >= capacity_bits {
                        break;
                    }
                }
                if col_left > 0 && !is_rmqr_reserved(height, width, row, col_left) {
                    let raw_bit = grid.bit(row, col_left);
                    let mask_bit = rmqr_mask_bit(row, col_left);
                    let final_bit = raw_bit ^ mask_bit;
                    let (px, py) = grid.get_pixel_coords(row, col_left).unwrap_or((-1.0, -1.0));
                    bit_positions.push((data.len, row, col_left, px, py, raw_bit, final_bit));
                    data.push(final_bit);
                    if data.len >= capacity_bits {
                        break;
                    }
                }
            }
        } else {
            for row in 0..height {
                if !is_rmqr_reserved(height, width, row, col_right) {
                    let raw_bit = grid.bit(row, col_right);
                    let mask_bit = rmqr_mask_bit(row, col_right);
                    let final_bit = raw_bit ^ mask_bit;
                    let (px, py) = grid.get_pixel_coords(row, col_right).unwrap_or((-1.0, -1.0));
                    bit_positions.push((data.len, row, col_right, px, py, raw_bit, final_bit));
                    data.push(final_bit);
                    if data.len >= capacity_bits {
                        break;
                    }
                }
                if col_left > 0 && !is_rmqr_reserved(height, width, row, col_left) {
                    let raw_bit = grid.bit(row, col_left);
                    let mask_bit = rmqr_mask_bit(row, col_left);
                    let final_bit = raw_bit ^ mask_bit;
                    let (px, py) = grid.get_pixel_coords(row, col_left).unwrap_or((-1.0, -1.0));
                    bit_positions.push((data.len, row, col_left, px, py, raw_bit, final_bit));
                    data.push(final_bit);
                    if data.len >= capacity_bits {
                        break;
                    }
                }
            }
        }
        if data.len >= capacity_bits {
            break;
        }
        if col < 2 {
            break;
        }
        col -= 2;
    }

    if DEBUG_SHOW_BIT_POSITIONS {
        crate::debug_log!(
            "[rMQR] First {} bit positions (of {}):",
            DEBUG_BIT_SAMPLE_COUNT.min(bit_positions.len()),
            bit_positions.len()
        );
        for (bit_idx, row, col, px, py, raw, masked) in
            bit_positions.iter().take(DEBUG_BIT_SAMPLE_COUNT)
        {
            crate::debug_log!(
                "[rMQR]   Bit {}: grid[{},{}] -> img({:.1},{:.1}) raw={} masked={}",
                bit_idx,
                row,
                col,
                px,
                py,
                if *raw { '1' } else { '0' },
                if *masked { '1' } else { '0' }
            );
        }
    }

    crate::debug_log!("[rMQR] Total bits read: {}", data.len);
    let mut all_bytes = String::new();
    for i in 0..(data.len / 8) {
        all_bytes.push_str(&format!("{:02X} ", data.data[i]));
    }
    crate::debug_log!("[rMQR] All bytes: {}", all_bytes);

    data
}

// ============================================================================
// PAYLOAD DECODING
// ============================================================================

fn decode_rmqr_payload<W>(
    meta: &RmqrMetaData,
    mut stream: RmqrCorrectedStream,
    mut writer: W,
) -> DeQRResult<()>
where
    W: Write,
{
    let info = meta.version.info();
    crate::debug_log!("[rMQR] Decoding payload");

    while stream.bits_remaining() >= 3 {
        crate::debug_log!("[rMQR]   {} bits remaining", stream.bits_remaining());

        let mode = stream.take_bits(3);
        match mode {
            0b000 => {
                crate::debug_log!("[rMQR]   Terminator Mode");
                break;
            }
            0b001 => {
                crate::debug_log!("[rMQR]   Numeric Mode");
                decode_rmqr_numeric(&info.char_count_bits, &mut stream, &mut writer)?;
            }
            0b010 => {
                crate::debug_log!("[rMQR]   Alphanumeric Mode");
                decode_rmqr_alpha(&info.char_count_bits, &mut stream, &mut writer)?;
            }
            0b011 => {
                crate::debug_log!("[rMQR]   Byte Mode");
                decode_rmqr_byte(&info.char_count_bits, &mut stream, &mut writer)?;
            }
            0b100 => {
                crate::debug_log!("[rMQR]   Kanji Mode");
                decode_rmqr_kanji(&info.char_count_bits, &mut stream, &mut writer)?;
            }
            0b111 => {
                crate::debug_log!("[rMQR]   ECI Mode");
                decode_rmqr_eci(&mut stream)?;
            }
            0b101 | 0b110 => {
                crate::debug_log!("[rMQR]   Reserved Mode (skipping)");
                if mode == 0b110 && stream.bits_remaining() >= 8 {
                    stream.take_bits(8);
                }
            }
            _ => return Err(DeQRError::UnknownDataType),
        }
    }
    Ok(())
}

fn decode_rmqr_numeric<W>(
    c: &RmqrCharacterCountBits,
    s: &mut RmqrCorrectedStream,
    w: &mut W,
) -> DeQRResult<()>
where
    W: Write,
{
    let count = s.take_bits(c.numeric);
    let mut remaining = count;
    while remaining >= 3 {
        if s.bits_remaining() < 10 {
            return Err(DeQRError::DataUnderflow);
        }
        write!(w, "{:03}", s.take_bits(10)).map_err(|_| DeQRError::IoError)?;
        remaining -= 3;
    }
    if remaining == 2 {
        if s.bits_remaining() < 7 {
            return Err(DeQRError::DataUnderflow);
        }
        write!(w, "{:02}", s.take_bits(7)).map_err(|_| DeQRError::IoError)?;
    } else if remaining == 1 {
        if s.bits_remaining() < 4 {
            return Err(DeQRError::DataUnderflow);
        }
        write!(w, "{}", s.take_bits(4)).map_err(|_| DeQRError::IoError)?;
    }
    Ok(())
}

fn decode_rmqr_alpha<W>(
    c: &RmqrCharacterCountBits,
    s: &mut RmqrCorrectedStream,
    w: &mut W,
) -> DeQRResult<()>
where
    W: Write,
{
    const ALPHA_MAP: &[u8; 45] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:";
    let count = s.take_bits(c.alphanumeric);
    let mut remaining = count;

    while remaining >= 2 {
        if s.bits_remaining() < 11 {
            return Err(DeQRError::DataUnderflow);
        }
        let val = s.take_bits(11);
        if val >= 2025 {
            return Err(DeQRError::UnknownDataType);
        }

        w.write_all(&[ALPHA_MAP[val / 45], ALPHA_MAP[val % 45]]).map_err(|_| DeQRError::IoError)?;
        remaining -= 2;
    }

    if remaining == 1 {
        if s.bits_remaining() < 6 {
            return Err(DeQRError::DataUnderflow);
        }
        let val = s.take_bits(6);

        if val >= 45 {
            return Err(DeQRError::UnknownDataType);
        }

        w.write_all(&[ALPHA_MAP[val]]).map_err(|_| DeQRError::IoError)?;
    }
    Ok(())
}

fn decode_rmqr_byte<W>(
    c: &RmqrCharacterCountBits,
    s: &mut RmqrCorrectedStream,
    w: &mut W,
) -> DeQRResult<()>
where
    W: Write,
{
    let count = s.take_bits(c.byte);
    for _ in 0..count {
        if s.bits_remaining() < 8 {
            return Err(DeQRError::DataUnderflow);
        }
        w.write_all(&[s.take_bits(8) as u8]).map_err(|_| DeQRError::IoError)?;
    }
    Ok(())
}

fn decode_rmqr_kanji<W>(
    c: &RmqrCharacterCountBits,
    s: &mut RmqrCorrectedStream,
    w: &mut W,
) -> DeQRResult<()>
where
    W: Write,
{
    let count = s.take_bits(c.kanji);
    for _ in 0..count {
        if s.bits_remaining() < 13 {
            return Err(DeQRError::DataUnderflow);
        }
        let d = s.take_bits(13);
        let ms_b = d / 0xc0;
        let ls_b = d % 0xc0;
        let intermediate = (ms_b << 8) | ls_b;
        let sjw = if intermediate + 0x8140 <= 0x9ffc {
            (intermediate + 0x8140) as u16
        } else {
            (intermediate + 0xc140) as u16
        };
        w.write_all(&[(sjw >> 8) as u8, (sjw & 0xff) as u8]).map_err(|_| DeQRError::IoError)?;
    }
    Ok(())
}

fn decode_rmqr_eci(s: &mut RmqrCorrectedStream) -> DeQRResult<()> {
    if s.bits_remaining() < 8 {
        return Err(DeQRError::DataUnderflow);
    }

    let first = s.take_bits(8);

    // Case 1: 1-byte ECI (0xxxxxxx)
    if (first & 0x80) == 0 {
        return Ok(());
    }

    // Case 2: 2-byte ECI (10xxxxxx xxxxxxxx)
    if (first & 0xC0) == 0x80 {
        if s.bits_remaining() < 8 {
            return Err(DeQRError::DataUnderflow);
        }
        s.take_bits(8);
        return Ok(());
    }

    // Case 3: 3-byte ECI (110xxxxx xxxxxxxx xxxxxxxx)
    if (first & 0xE0) == 0xC0 {
        if s.bits_remaining() < 16 {
            return Err(DeQRError::DataUnderflow);
        }
        s.take_bits(16);
        return Ok(());
    }

    Err(DeQRError::UnknownDataType)
}
