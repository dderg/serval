use super::*;

#[test]
#[should_panic(
    expected = "turning guard: the cell at t=0.512500s still asks 30000 mm/s² of the extruder \
                (limit 20000 mm/s²) after 9 back-offs of line 12, the next taking it under \
                0.03125 of its budget; the cell reads [0.507500, 0.520000]s, the frozen seam is \
                at t=0.480000s (line 10)"
)]
fn a_cell_binding_past_the_scale_floor_names_itself_and_the_frozen_seam() {
    Stuck {
        binding: Binding {
            cell: 0.5125,
            from: 0.5075,
            to: 0.52,
            demand: 30_000.0,
            max_accel: 20_000.0,
            asked: Asked::Accel,
        },
        line: 12,
        backoffs: 9,
        frozen: (0.48, 10),
    }
    .fail();
}
