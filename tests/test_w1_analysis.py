from scripts.vbd.w1_analysis import find_boundaries, reduce_labels, reduced_cells, t_ext_rows


def test_boundaries_find_flips_on_both_axes():
    matrix = {
        (1, .4): "slip", (1, .6): "intact", (1, .8): "intact",
        (5, .4): "slip", (5, .6): "damage", (5, .8): "damage",
    }
    found = find_boundaries(matrix, [1, 5], [.4, .6, .8])
    assert found == {(1, .4), (1, .6), (5, .6), (1, .8), (5, .8)}


def test_two_thirds_and_provisional_and_unresolved():
    assert reduce_labels(["slip", "slip", "intact"])[0] == "slip"
    assert reduce_labels(["damage"])[2] == "provisional_seed0"
    assert reduce_labels(["slip", "intact", "damage"])[0] == "UNRESOLVED"
    assert reduce_labels(["slip", "intact"])[0] == "UNRESOLVED"


def test_t_ext_cap_and_uncertified_exclusion():
    rows = []
    for a in range(12):
        cells = {f: {"label": "slip", "certified": True} for f in [.4, .6, .8, 1, 1.2, 1.5, 2]}
        rows.append({"E_kPa": 7, "a": a, "cells": cells})
    rows[-1]["cells"][2]["certified"] = False
    result = t_ext_rows(rows, cap=8)
    assert len(result) == 8
    assert 11 not in [x["commanded_a_peak_m_s2"] for x in result]
    assert [x["commanded_a_peak_m_s2"] for x in result] == list(range(10, 2, -1))


def test_primary_reduction_includes_uncertified_receipts():
    receipts = [
        {"E_pa": 7000, "commanded_a_peak_m_s2": 1, "grip_force_n": .4,
         "seed": seed, "label": label, "validity_gate": {"certified": False}}
        for seed, label in enumerate(("slip", "slip", "intact"))
    ]
    label, sources, status = reduced_cells(receipts)[(7, 1.0, .4)]
    assert label == "slip"
    assert len(sources) == 3
    assert status == "two_thirds_majority"
