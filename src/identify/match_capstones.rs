use crate::CapStone;

#[derive(Debug, Clone)]
pub struct CapStoneGroup(pub CapStone, pub CapStone, pub CapStone);

#[derive(Clone, Copy, Debug, PartialEq)]
struct Neighbor {
    index: usize,
    distance: f64,
}

/// Return each pair Capstone indexes that are likely to be from a QR code
/// Ordered from most symmetric to least symmetric
pub fn find_and_rank_possible_neighbors(capstones: &[CapStone], idx: usize) -> Vec<(usize, usize)> {
    const VIABILITY_THRESHOLD: f64 = 0.25;

    crate::debug_log!("Finding neighbors for capstone {}", idx);

    let (hlist, vlist) = find_possible_neighbors(capstones, idx);

    // Log neighbor lists for debugging
    let hlist_debug: Vec<(usize, f64)> = hlist.iter().map(|n| (n.index, n.distance)).collect();
    let vlist_debug: Vec<(usize, f64)> = vlist.iter().map(|n| (n.index, n.distance)).collect();
    crate::debug::log_neighbor_search(idx, &capstones[idx], &hlist_debug, &vlist_debug);

    let mut res = Vec::new();
    struct NeighborSet {
        score: f64,
        h_index: usize,
        v_index: usize,
    }
    /* Test each possible grouping */
    for hn in hlist {
        for vn in vlist.iter() {
            let score = {
                if hn.distance < vn.distance {
                    (1.0f64 - hn.distance / vn.distance).abs()
                } else {
                    (1.0f64 - vn.distance / hn.distance).abs()
                }
            };

            crate::debug_log!(
                "  Testing pair ({}, {}): h_dist={:.1}, v_dist={:.1}, score={:.3}",
                hn.index,
                vn.index,
                hn.distance,
                vn.distance,
                score
            );

            if score < VIABILITY_THRESHOLD {
                crate::debug_log!("    -> VIABLE (score < {})", VIABILITY_THRESHOLD);
                res.push(NeighborSet { score, h_index: hn.index, v_index: vn.index });
            } else {
                crate::debug_log!("    -> NOT viable (score >= {})", VIABILITY_THRESHOLD);
            }
        }
    }

    res.sort_unstable_by(|a, b| {
        (a.score).partial_cmp(&(b.score)).expect("Neighbor distance should never cause a div by 0")
    });

    let pairs: Vec<(usize, usize)> = res.iter().map(|n| (n.h_index, n.v_index)).collect();
    crate::debug_log!("Found {} viable neighbor pairs for capstone {}", pairs.len(), idx);

    pairs
}

fn find_possible_neighbors(capstones: &[CapStone], idx: usize) -> (Vec<Neighbor>, Vec<Neighbor>) {
    let cap = &capstones[idx];
    let mut hlist = Vec::new();
    let mut vlist = Vec::new();

    crate::debug_log!(
        "Scanning {} capstones for potential neighbors of capstone {}",
        capstones.len(),
        idx
    );
    crate::debug_log!("Reference capstone {} center: ({}, {})", idx, cap.center.x, cap.center.y);

    /* Look for potential neighbours by examining the relative gradients
     * from this capstone to others.
     */
    #[allow(clippy::needless_range_loop)]
    for others_idx in 0..capstones.len() {
        if others_idx == idx {
            continue;
        }

        let cmp_cap = &capstones[others_idx];

        let (mut u, mut v) = cap.c.unmap(&cmp_cap.center);
        u = (u - 3.5f64).abs();
        v = (v - 3.5f64).abs();

        crate::debug_log!(
            "  Capstone {}: center=({}, {}), unmap u={:.2}, v={:.2}",
            others_idx,
            cmp_cap.center.x,
            cmp_cap.center.y,
            u,
            v
        );

        if u < 0.2f64 * v {
            crate::debug_log!("    -> Added to HORIZONTAL list (u < 0.2*v), distance={:.1}", v);
            hlist.push(Neighbor { index: others_idx, distance: v });
        }

        if v < 0.2f64 * u {
            crate::debug_log!("    -> Added to VERTICAL list (v < 0.2*u), distance={:.1}", u);
            vlist.push(Neighbor { index: others_idx, distance: u });
        }
    }

    crate::debug_log!(
        "Neighbor search complete: {} horizontal, {} vertical candidates",
        hlist.len(),
        vlist.len()
    );

    (hlist, vlist)
}
