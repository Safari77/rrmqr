//! Find and read QR-Codes and rMQR Codes
//!
//! This crates exports functions and types that can be used to search for
//! QR-Codes and rMQR (Rectangular Micro QR Code / ISO/IEC 23941) codes
//! in images and decode them.
//!
//! ## Supported Codes
//!
//! - **QR Code**: Standard square QR codes (ISO/IEC 18004)
//! - **rMQR Code**: Rectangular Micro QR codes (ISO/IEC 23941)
//!
//! The library automatically detects both types of codes.
//!
#![cfg_attr(
    feature = "img",
    doc = r##"
# Usage
The most basic usage is shown below:

```rust
use image;

# fn main() -> Result<(), Box<dyn ::std::error::Error>> {
// Load on image to search, convert it to grayscale
let img = image::open("tests/data/github.gif")?.to_luma8();
// Prepare for detection
let mut img = rrmqr::PreparedImage::prepare(img);
// Search for grids, without decoding (supports both QR and rMQR)
let grids = img.detect_grids();
assert_eq!(grids.len(), 1);
// Decode the grid
let (meta, content) = grids[0].decode()?;
assert_eq!(meta.ecc_level, 0);
assert_eq!(content, "https://github.com/WanzenBug/rqrr");
# Ok(())
# }
```

If you have some other form of picture storage, you can use
[`PreparedImage::prepare_from_*`](struct.PreparedImage.html). This allows
you to define your own source for images.

# rMQR Support

rMQR (Rectangular Micro QR Code) is automatically detected alongside
standard QR codes. rMQR codes have distinctive rectangular shapes with
heights from 7 to 17 modules and widths from 27 to 139 modules.

## Detecting rMQR Codes

```rust,ignore
use image;

let img = image::open("image_with_rmqr.png")?.to_luma8();
let mut img = rrmqr::PreparedImage::prepare(img);

// Detect and decode rMQR codes
for grid in img.detect_rmqr_grids() {
    if let Ok((meta, content)) = grid.decode() {
        println!("rMQR R{}x{}: {}", meta.version.height, meta.version.width, content);
    }
}
```

## Detecting Both QR and rMQR Codes

To detect both QR and rMQR codes, call each method separately:

```rust,ignore
let mut img = rrmqr::PreparedImage::prepare(image);

// First detect and process QR codes
for grid in img.detect_grids() {
    if let Ok((meta, content)) = grid.decode() {
        println!("QR Code: {}", content);
    }
}

// Then detect and process rMQR codes
for grid in img.detect_rmqr_grids() {
    if let Ok((meta, content)) = grid.decode() {
        println!("rMQR Code: {}", content);
    }
}
```
"##
)]
pub use self::decode::{MAX_PAYLOAD_SIZE, MetaData, RawData, Version};
pub(crate) use self::detect::{CapStone, capstones_from_image};
pub use self::identify::Point;
pub(crate) use self::identify::SkewedGridLocation;
pub use self::prepare::PreparedImage;
use std::error::Error;
use std::io::Write;

mod decode;
mod detect;
pub(crate) mod geometry;
mod identify;
mod version_db;

pub mod debug;
pub mod prepare;
// rMQR (Rectangular Micro QR Code) support modules
pub mod rmqr_decode;
pub mod rmqr_detect;
pub mod rmqr_ecc;
pub mod rmqr_grid;
pub mod rmqr_version_db;

// Re-export rMQR types
pub use rmqr_decode::{RectBitGrid, RmqrMetaData, RmqrVersion};
pub use rmqr_detect::{RmqrFinderPattern, RmqrRegion};
pub use rmqr_ecc::{correct_rmqr_block, rmqr_ecc_candidates};
pub use rmqr_grid::{RmqrGridLocation, RmqrRefGridImage};
pub use rmqr_version_db::{RmqrVersionInfo, is_valid_rmqr_dimensions};

/// Wrapper around any grid that can be interpreted as a QR code
#[derive(Debug, Clone)]
pub struct Grid<G> {
    /// The backing binary square
    pub grid: G,
    /// The bounds of the square, in underlying coordinates.
    ///
    /// The points are listed in the following order:
    /// [top-left, top-right, bottom-right, bottom-left]
    ///
    /// If this grid references for example an underlying image, these values
    /// will be set to coordinates in that image.
    pub bounds: [Point; 4],
}

impl<G> Grid<G>
where
    G: BitGrid,
{
    /// Create a new grid from a BitGrid.
    ///
    /// This just initialises the bounds to 0.
    pub fn new(grid: G) -> Self {
        Grid {
            grid,
            bounds: [
                Point { x: 0, y: 0 },
                Point { x: 0, y: 0 },
                Point { x: 0, y: 0 },
                Point { x: 0, y: 0 },
            ],
        }
    }

    /// Try to decode the grid.
    ///
    /// If successful returns the decoded string as well as metadata about the
    /// code.
    pub fn decode(&self) -> DeQRResult<(MetaData, String)> {
        let mut out = Vec::new();
        let meta = self.decode_to(&mut out)?;
        let out = String::from_utf8(out)?;
        Ok((meta, out))
    }

    /// Try to read metadata, and return the raw, uncorrected bit stream.
    ///
    /// If successful, returns the metadata along with the raw bit pattern.
    /// The raw data is still masked, so bits appear as in the source image.
    pub fn get_raw_data(&self) -> DeQRResult<(MetaData, RawData)> {
        decode::get_raw(&self.grid, false)
    }

    /// Try to decode the grid.
    ///
    /// Instead of returning a String, this method writes the decoded result to
    /// the given writer
    ///
    /// **Warning**: This may lead to half decoded content to be written to the
    /// writer.
    pub fn decode_to<W>(&self, writer: W) -> DeQRResult<MetaData>
    where
        W: Write,
    {
        decode::decode(&self.grid, writer)
    }
}

/// A grid that contains exactly one QR code square.
///
/// The common trait for everything that can be decoded as a QR code. Given a
/// normal image, we first need to find the QR grids in it.
///
/// This trait can be implemented when some object is known to be exactly the
/// bit-pattern of a QR code.
pub trait BitGrid {
    /// Return the size of the grid.
    ///
    /// Since QR codes are always squares, the grid is assumed to be size *
    /// size.
    fn size(&self) -> usize;

    /// Return the value of the bit at the given location.
    ///
    /// `true` means 'black', `false` means 'white'
    fn bit(&self, y: usize, x: usize) -> bool;

    #[cfg(feature = "img")]
    fn write_grid_to(&self, p: &str) {
        let mut dyn_img = image::GrayImage::new(self.size() as u32, self.size() as u32);
        for y in 0..self.size() {
            for x in 0..self.size() {
                let color = match self.bit(y, x) {
                    true => 0,
                    false => 255,
                };
                dyn_img.get_pixel_mut(x as u32, y as u32).0[0] = color;
            }
        }
        dyn_img.save(p).unwrap();
    }
}

/// Unified code type - represents either a QR code or rMQR code
#[derive(Debug, Clone)]
pub enum CodeType {
    /// Standard square QR code (ISO/IEC 18004)
    QR,
    /// Rectangular Micro QR code (ISO/IEC 23941)
    RMQR,
}

/// Unified metadata for both QR and rMQR codes
#[derive(Debug, Clone)]
pub enum UnifiedMetaData {
    /// Standard QR code metadata
    QR(MetaData),
    /// rMQR code metadata
    RMQR(RmqrMetaData),
}

impl UnifiedMetaData {
    /// Get the error correction level (0-3 for QR, 0-1 for rMQR)
    pub fn ecc_level(&self) -> u8 {
        match self {
            UnifiedMetaData::QR(m) => m.ecc_level as u8,
            UnifiedMetaData::RMQR(m) => m.ecc_level,
        }
    }

    /// Get the code type
    pub fn code_type(&self) -> CodeType {
        match self {
            UnifiedMetaData::QR(_) => CodeType::QR,
            UnifiedMetaData::RMQR(_) => CodeType::RMQR,
        }
    }

    /// Check if this is an rMQR code
    pub fn is_rmqr(&self) -> bool {
        matches!(self, UnifiedMetaData::RMQR(_))
    }
}

/// A wrapper for rMQR grids that can be decoded
#[derive(Debug, Clone)]
pub struct RmqrGrid<G> {
    /// The backing rectangular grid
    pub grid: G,
    /// The bounds of the rectangle in image coordinates [TL, TR, BR, BL]
    pub bounds: [Point; 4],
}

impl<G> RmqrGrid<G>
where
    G: RectBitGrid,
{
    /// Create a new rMQR grid
    pub fn new(grid: G) -> Self {
        RmqrGrid {
            grid,
            bounds: [
                Point { x: 0, y: 0 },
                Point { x: 0, y: 0 },
                Point { x: 0, y: 0 },
                Point { x: 0, y: 0 },
            ],
        }
    }

    /// Try to decode the rMQR grid
    pub fn decode(&self) -> DeQRResult<(RmqrMetaData, String)> {
        let mut out = Vec::new();
        let meta = self.decode_to(&mut out)?;
        let out = String::from_utf8(out)?;
        Ok((meta, out))
    }

    /// Try to decode the rMQR grid, writing to the given writer
    pub fn decode_to<W>(&self, writer: W) -> DeQRResult<RmqrMetaData>
    where
        W: Write,
    {
        rmqr_decode::decode_rmqr(&self.grid, writer)
    }
}

/// Mirrored grid, switching x and y coordinates
///
/// Some QR codes are read mirrored, even though the spec does not officially support it.
/// Since there is no marker in the QR code spec, we simply have to try to read the grid both ways
/// if the first does not succeed.
pub struct MirroredGrid<'a>(&'a dyn BitGrid);

impl BitGrid for MirroredGrid<'_> {
    fn size(&self) -> usize {
        self.0.size()
    }

    fn bit(&self, y: usize, x: usize) -> bool {
        self.0.bit(x, y)
    }
}

/// A basic GridImage that can be generated from a given function.
///
/// # Example
///
/// ```rust
/// # fn main() -> Result<(), rrmqr::DeQRError> {
/// let grid = [
///     [
///         1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
///     ],
///     [
///         1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
///     ],
///     [
///         1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1,
///     ],
///     [
///         1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1,
///     ],
///     [
///         1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1,
///     ],
///     [
///         1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1,
///     ],
///     [
///         1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1,
///     ],
///     [
///         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
///     ],
///     [
///         1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0,
///     ],
///     [
///         1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1,
///     ],
///     [
///         0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1,
///     ],
///     [
///         1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1,
///     ],
///     [
///         0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1,
///     ],
///     [
///         0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1,
///     ],
///     [
///         1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1,
///     ],
///     [
///         1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
///     ],
///     [
///         1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1,
///     ],
///     [
///         1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
///     ],
///     [
///         1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1,
///     ],
///     [
///         1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0,
///     ],
///     [
///         1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1,
///     ],
/// ];
///
/// let simple = rrmqr::SimpleGrid::from_func(21, |x, y| grid[y][x] == 1);
/// let grid = rrmqr::Grid::new(simple);
/// let (_meta, content) = grid.decode()?;
/// assert_eq!(content, "rqrr");
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone)]
pub struct SimpleGrid {
    cell_bitmap: Vec<u8>,
    size: usize,
}

impl SimpleGrid {
    pub fn from_func<F>(size: usize, fill_func: F) -> Self
    where
        F: Fn(usize, usize) -> bool,
    {
        let mut cell_bitmap = vec![0; (size * size).div_ceil(8)];
        let mut c = 0;
        for y in 0..size {
            for x in 0..size {
                if fill_func(x, y) {
                    cell_bitmap[c >> 3] |= 1 << (c & 7) as u8;
                }
                c += 1;
            }
        }

        SimpleGrid { cell_bitmap, size }
    }
}

impl BitGrid for SimpleGrid {
    fn size(&self) -> usize {
        self.size
    }

    fn bit(&self, y: usize, x: usize) -> bool {
        let c = y * self.size + x;
        self.cell_bitmap[c >> 3] & (1 << (c & 7) as u8) != 0
    }
}

/// Possible errors that can happen during decoding
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum DeQRError {
    /// Could not write the output to the output stream/string
    IoError,
    /// Expected more bits to decode
    DataUnderflow,
    /// Expected less bits to decode
    DataOverflow,
    /// Unknown data type in encoding
    UnknownDataType,
    /// Could not correct errors / code corrupt
    DataEcc,
    /// Could not read format information from both locations
    FormatEcc,
    /// Unsupported / non-existent version read
    InvalidVersion,
    /// Unsupported / non-existent grid size read
    InvalidGridSize,
    /// Output was not encoded in expected UTF8
    EncodingError,
}

type DeQRResult<T> = Result<T, DeQRError>;

impl Error for DeQRError {}

impl From<::std::string::FromUtf8Error> for DeQRError {
    fn from(_: ::std::string::FromUtf8Error) -> Self {
        DeQRError::EncodingError
    }
}

impl ::std::fmt::Display for DeQRError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let msg = match self {
            DeQRError::IoError => "IoError(Could not write to output)",
            DeQRError::DataUnderflow => "DataUnderflow(Expected more bits to decode)",
            DeQRError::DataOverflow => "DataOverflow(Expected less bits to decode)",
            DeQRError::UnknownDataType => "UnknownDataType(DataType not known or not implemented)",
            DeQRError::DataEcc => "Ecc(Too many errors to correct)",
            DeQRError::FormatEcc => "Ecc(Version information corrupt)",
            DeQRError::InvalidVersion => "InvalidVersion(Invalid version or corrupt)",
            DeQRError::InvalidGridSize => "InvalidGridSize(Invalid version or corrupt)",
            DeQRError::EncodingError => "Encoding(Not UTF8)",
        };
        write!(f, "{msg}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rqrr() {
        let grid = [
            [1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0],
            [1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1],
            [0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1],
            [0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
            [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1],
        ];

        let img = crate::SimpleGrid::from_func(21, |x, y| grid[y][x] == 1);

        let mut buf = vec![0; img.size() * img.size() / 8 + 1];
        for y in 0..img.size() {
            for x in 0..img.size() {
                let i = y * img.size() + x;
                if img.bit(y, x) {
                    buf[i >> 3] |= 1 << ((i & 7) as u8);
                }
            }
        }

        let mut vec = Vec::new();
        crate::decode::decode(&img, &mut vec).unwrap();

        assert_eq!(b"rqrr".as_ref(), &vec[..])
    }

    #[test]
    fn test_github() {
        let grid = [
            [1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1],
            [1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1],
            [1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0],
            [0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1],
            [0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1],
            [1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1],
            [1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0],
            [1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1],
            [0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1],
            [1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1],
            [0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0],
            [1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0],
            [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1],
        ];

        let img = crate::SimpleGrid::from_func(29, |x, y| grid[y][x] == 1);

        let mut buf = vec![0; img.size() * img.size() / 8 + 1];
        for y in 0..img.size() {
            for x in 0..img.size() {
                let i = y * img.size() + x;
                if img.bit(y, x) {
                    buf[i >> 3] |= 1 << ((i & 7) as u8);
                }
            }
        }

        let mut vec = Vec::new();
        crate::decode::decode(&img, &mut vec).unwrap();

        assert_eq!(b"https://github.com/WanzenBug/rqrr".as_ref(), &vec[..])
    }

    #[test]
    fn test_number() {
        let grid = [
            [1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0],
            [1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1],
            [0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0],
            [0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
            [0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0],
            [0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1],
            [0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0],
            [1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1],
            [0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0],
            [1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1],
            [1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
            [1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0],
            [1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0],
            [1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0],
            [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
        ];

        let img = crate::SimpleGrid::from_func(29, |x, y| grid[y][x] == 1);

        let mut buf = vec![0; img.size() * img.size() / 8 + 1];
        for y in 0..img.size() {
            for x in 0..img.size() {
                let i = y * img.size() + x;
                if img.bit(y, x) {
                    buf[i >> 3] |= 1 << ((i & 7) as u8);
                }
            }
        }

        let mut vec = Vec::new();
        crate::decode::decode(&img, &mut vec).unwrap();

        assert_eq!(b"1234567891011121314151617181920".as_ref(), &vec[..])
    }
}
