use crate::DeQRError;
use crate::DeQRResult;
use crate::rmqr_decode::RMQR_MAX_PAYLOAD_SIZE;
use crate::rmqr_decode::{RmqrCorrectedStream, RmqrMetaData, RmqrRawData};
use g2p::{GaloisField, g2p};
use std::collections::HashSet;

// Use same GF(2^8) as standard QR
g2p!(GF256, 8, modulus: 0b1_0001_1101);

const MAX_POLY_LEN: usize = 300;

/// Returns a list of potential valid decodes (Stream, ErrorCount).
/// We return ALL valid candidates because a "0 error" decode might be a false positive
/// that fails payload parsing later.
///
/// # Arguments
/// * `erasures` - A list of zero-based byte indices in the `raw` stream that are known to be corrupted.
pub fn rmqr_ecc_candidates(
    meta: &RmqrMetaData,
    raw: &RmqrRawData,
    erasures: &[usize],
) -> Vec<(RmqrCorrectedStream, usize)> {
    let info = meta.version.info();
    let ecc_params = &info.ecc[meta.ecc_level as usize];

    let mut candidates = Vec::new();
    let mut tried_ns = HashSet::new();

    let total_codewords = ecc_params.bs;
    let total_data = ecc_params.dw;
    let db_ns = ecc_params.ns;

    // 1. Always try the database parameter first
    let db_success = try_add_candidate(
        db_ns,
        total_codewords,
        total_data,
        raw,
        erasures,
        &mut candidates,
        &mut tried_ns,
    );

    if db_success {
        crate::debug_log!("[rMQR ECC] DB params (ns={}) produced valid ECC", db_ns);
    } else {
        crate::debug_log!("[rMQR ECC] DB params (ns={}) failed ECC check", db_ns);
    }

    // 2. Try heuristic block counts (auto-detection)
    // Common splits are 1..10. We strictly filter by `total_ecc % ns == 0`.
    for ns in 1..=10 {
        if try_add_candidate(
            ns,
            total_codewords,
            total_data,
            raw,
            erasures,
            &mut candidates,
            &mut tried_ns,
        ) {
            crate::debug_log!("[rMQR ECC] Auto-detected valid ECC layout with ns={}", ns);
        }
    }

    candidates
}

fn try_add_candidate(
    ns: usize,
    total_codewords: usize,
    total_data: usize,
    raw: &RmqrRawData,
    erasures: &[usize],
    results: &mut Vec<(RmqrCorrectedStream, usize)>,
    tried: &mut HashSet<usize>,
) -> bool {
    if ns == 0 || tried.contains(&ns) {
        return false;
    }
    tried.insert(ns);

    // Sanity check: ECC bytes must be evenly divisible by block count
    let total_ecc = total_codewords.saturating_sub(total_data);
    if total_ecc == 0 || !total_ecc.is_multiple_of(ns) {
        return false;
    }

    if let Ok(res) = decode_with_ns(ns, total_codewords, total_data, raw, erasures) {
        results.push(res);
        return true;
    }
    false
}

/// Helper to attempt decoding with a specific number of blocks (ns)
/// Handles mapping global erasure indices to per-block erasure offsets.
fn decode_with_ns(
    ns: usize,
    total_codewords: usize,
    total_data: usize,
    raw: &RmqrRawData,
    raw_erasures: &[usize],
) -> DeQRResult<(RmqrCorrectedStream, usize)> {
    // --- 1. Calculate Block Layout ---
    let total_ecc = total_codewords.checked_sub(total_data).ok_or(DeQRError::FormatEcc)?;

    if total_ecc % ns != 0 {
        return Err(DeQRError::FormatEcc);
    }
    let ecc_per_block = total_ecc / ns;

    // Create a fast lookup, but respect the limit
    let mut erasure_set = HashSet::new();
    for (count, &idx) in raw_erasures.iter().enumerate() {
        if count >= ecc_per_block {
            break; // Stop adding erasures if we hit the parity limit
        }
        erasure_set.insert(idx);
    }

    let base_data_per_block = total_data / ns;
    let num_large_blocks = total_data % ns;
    let num_small_blocks = ns - num_large_blocks;

    // blocks[i] = (data_buffer, ecc_buffer)
    let mut blocks: Vec<(Vec<u8>, Vec<u8>)> = Vec::with_capacity(ns);
    // block_erasures[i] = list of indices in the constructed block (data + ecc) to mark as erased
    let mut block_erasures: Vec<Vec<usize>> = vec![Vec::new(); ns];

    for i in 0..ns {
        let is_large = i >= num_small_blocks;
        let data_len = base_data_per_block + if is_large { 1 } else { 0 };
        blocks.push((vec![0u8; data_len], vec![0u8; ecc_per_block]));
    }

    // --- 2. De-interleave Data (and Map Erasures) ---
    let mut raw_ptr = 0;
    let max_data_len = base_data_per_block + 1;

    for i in 0..max_data_len {
        for b in 0..ns {
            if i < blocks[b].0.len() {
                if raw_ptr >= raw.len / 8 {
                    return Err(DeQRError::DataUnderflow);
                }

                blocks[b].0[i] = raw.data[raw_ptr];

                // If this raw byte was marked as erased, mark it in the block
                if erasure_set.contains(&raw_ptr) {
                    block_erasures[b].push(i); // 'i' is the index in the data part
                }

                raw_ptr += 1;
            }
        }
    }

    // --- 3. De-interleave ECC (and Map Erasures) ---
    for i in 0..ecc_per_block {
        for b in 0..ns {
            if raw_ptr >= raw.len / 8 {
                return Err(DeQRError::DataUnderflow);
            }

            blocks[b].1[i] = raw.data[raw_ptr];

            // If this raw byte was marked as erased, mark it in the block
            if erasure_set.contains(&raw_ptr) {
                // Offset by data length since RS block is [Data... | ECC...]
                let data_len = blocks[b].0.len();
                block_erasures[b].push(data_len + i);
            }

            raw_ptr += 1;
        }
    }

    // --- 4. Perform Correction on Each Block ---
    let mut total_errors = 0;

    for (b_idx, ((data, ecc), erasures)) in blocks.iter_mut().zip(block_erasures).enumerate() {
        let mut full_block = data.clone();
        full_block.extend_from_slice(ecc);

        // Pre-check: Singleton Bound
        // Erasures (E) + 2*Errors (T) <= Parity (P)
        // If we have more erasures than parity bytes, correction is impossible.
        if erasures.len() > ecc.len() {
            crate::debug_log!(
                "[rMQR ECC] Block {} (of {}) failed: {} erasures > {} parity bytes",
                b_idx + 1,
                ns,
                erasures.len(),
                ecc.len()
            );
            return Err(DeQRError::DataEcc);
        }

        let errors = correct_rmqr_block(&mut full_block, ecc.len(), &erasures)?;
        if errors > 0 {
            crate::debug_log!(
                "[rMQR ECC] Block {} (of {}) corrected: found {} errors (including {} erasures)",
                b_idx + 1,
                ns,
                errors,
                erasures.len()
            );
        }

        total_errors += errors;

        for i in 0..data.len() {
            data[i] = full_block[i];
        }
    }

    // --- 5. Re-assemble Output Stream ---
    let mut out =
        RmqrCorrectedStream { data: [0; RMQR_MAX_PAYLOAD_SIZE], ptr: 0, bit_len: total_data * 8 };

    let mut out_ptr = 0;
    // Iterate over blocks to reconstruct stream (Consumes blocks)
    for (data, _) in blocks {
        for &byte in data.iter() {
            if out_ptr < RMQR_MAX_PAYLOAD_SIZE {
                out.data[out_ptr] = byte;
                out_ptr += 1;
            }
        }
    }

    Ok((out, total_errors))
}

pub fn correct_rmqr_block(
    block: &mut [u8],
    num_ecc: usize,
    erasures: &[usize],
) -> DeQRResult<usize> {
    let n = block.len();

    // 1. Syndromes
    let mut syndromes = [GF256::ZERO; MAX_POLY_LEN];
    let mut has_errors = false;

    for i in 0..num_ecc {
        let mut s = GF256::ZERO;
        let alpha_i = GF256::GENERATOR.pow(i);
        for byte in block.iter().take(n) {
            s = s * alpha_i + GF256(*byte);
        }
        syndromes[i] = s;
        if s != GF256::ZERO {
            has_errors = true;
        }
    }

    if !has_errors && erasures.is_empty() {
        return Ok(0);
    }

    // 2. Sigma Init (Erasure Polynomial)
    let mut sigma = [GF256::ZERO; MAX_POLY_LEN];
    let mut sigma_len = 1;
    sigma[0] = GF256::ONE;

    for &pos in erasures {
        // Position is index from start, exponent is from end
        let loc = GF256::GENERATOR.pow(n - 1 - pos);
        for i in (1..=sigma_len).rev() {
            // sigma[i] = sigma[i] + sigma[i-1] * loc
            sigma[i] += sigma[i - 1] * loc;
        }
        sigma_len += 1;
    }

    // 3. Berlekamp-Massey (Find remaining errors)
    // IMPORTANT: erasure_count is used to initialize the iteration depth
    let (sigma, mut sigma_len) =
        berlekamp_massey(&syndromes[0..num_ecc], sigma, sigma_len, erasures.len())?;

    // --- Normalize sigma degree ---
    while sigma_len > 1 && sigma[sigma_len - 1] == GF256::ZERO {
        sigma_len -= 1;
    }

    // 4. Chien Search
    let mut error_locs = Vec::with_capacity(num_ecc);
    for i in 0..n {
        let inv_loc = GF256::GENERATOR.pow(255 - i);
        if eval_poly(&sigma[0..sigma_len], inv_loc) == GF256::ZERO {
            // Root found at exponent i, which corresponds to position (n - 1 - i)
            error_locs.push(n - 1 - i);
        }
    }

    // Validation: Number of roots must equal degree of Error Locator Polynomial
    if error_locs.len() != sigma_len - 1 {
        return Err(DeQRError::DataEcc);
    }

    // 5. Forney Algorithm
    let omega = calc_omega(&syndromes[0..num_ecc], &sigma[0..sigma_len]);
    let sigma_deriv = calc_derivative(&sigma[0..sigma_len]);

    for &loc_idx in &error_locs {
        let inv_x = GF256::GENERATOR.pow(255 - (n - 1 - loc_idx));
        let num = eval_poly(&omega, inv_x);
        let den = eval_poly(&sigma_deriv, inv_x);
        if den == GF256::ZERO {
            return Err(DeQRError::DataEcc);
        }
        // Standard magnitude calculation for b=0
        let mag = (num / den) * GF256::GENERATOR.pow(n - 1 - loc_idx);
        block[loc_idx] = (GF256(block[loc_idx]) + mag).0;
    }

    Ok(error_locs.len())
}

fn berlekamp_massey(
    syndromes: &[GF256],
    mut sigma: [GF256; MAX_POLY_LEN],
    mut sigma_len: usize,
    erasure_count: usize,
) -> DeQRResult<([GF256; MAX_POLY_LEN], usize)> {
    let mut b = [GF256::ZERO; MAX_POLY_LEN];
    b[0] = GF256::ONE;
    let mut b_len = 1;

    let mut l = erasure_count;
    let mut b_val = GF256::ONE;
    let mut m = 1;

    // Start iterations from the number of known erasures
    for k in erasure_count..syndromes.len() {
        let mut d = syndromes[k];
        for i in 1..sigma_len {
            if k >= i {
                d += sigma[i] * syndromes[k - i];
            }
        }

        if d == GF256::ZERO {
            m += 1;
        } else {
            let mult = d / b_val;
            let old_sigma = sigma;
            let old_sigma_len = sigma_len;

            for i in 0..b_len {
                let val = mult * b[i];
                if val != GF256::ZERO {
                    let target = i + m;
                    if target < MAX_POLY_LEN {
                        sigma[target] += val;
                        if target >= sigma_len {
                            sigma_len = target + 1;
                        }
                    }
                }
            }

            if 2 * l <= k + erasure_count {
                l = k + 1 - l;
                b = old_sigma;
                b_len = old_sigma_len;
                b_val = d;
                m = 1;
            } else {
                m += 1;
            }
        }
    }

    // Safety trim
    while sigma_len > 1 && sigma[sigma_len - 1] == GF256::ZERO {
        sigma_len -= 1;
    }

    Ok((sigma, sigma_len))
}

fn eval_poly(poly: &[GF256], x: GF256) -> GF256 {
    let mut res = GF256::ZERO;
    for i in (0..poly.len()).rev() {
        res = res * x + poly[i];
    }
    res
}

fn calc_omega(syndromes: &[GF256], sigma: &[GF256]) -> Vec<GF256> {
    let mut omega = vec![GF256::ZERO; syndromes.len()];
    for i in 0..syndromes.len() {
        for j in 0..sigma.len() {
            if i >= j {
                omega[i] += sigma[j] * syndromes[i - j];
            }
        }
    }
    omega
}

fn calc_derivative(poly: &[GF256]) -> Vec<GF256> {
    let mut deriv = vec![GF256::ZERO; poly.len()];
    for i in (1..poly.len()).step_by(2) {
        deriv[i - 1] = poly[i];
    }
    deriv
}
