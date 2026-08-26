"""Small strict JSON artifact schemas used by the host-side protocol."""
import json
from pathlib import Path

COMMON_CONFIG_KEYS = ("brief_sha256", "newton_commit", "asset_urdf_sha256", "dt", "substeps",
 "particle_count", "voxel_size", "contact_params", "windows", "f_g_convention",
 "seed_rng_derivation", "profile_id", "coupling_params", "calibration")
E2_CONFIG_KEYS = ("raw_field_recorded", "raw_field_layout", "impulse_eps", "pad_normal_local",
 "pad_frame_convention", "coupled_tick_s", "aggregates_derived_from_raw", "taxel_binning",
 "signal_source", "f_mid_n", "f_mid_selection")
E1_PAYLOAD_KEYS = ("a_peak_cmd_ms2", "a_peak_realized_ms2", "labels",
                   "peak_damage_fraction", "health", "phase_timestamps")
BAND_PAYLOAD_KEYS = ("rows", "a_star", "a_star_status", "coverage", "extra_replications")
ROW_KEYS = ("sigma_Y", "a_peak", "F_min", "F_max", "band_width_n", "band_status",
            "censored_low", "censored_high", "interior_failures")


def _require(mapping, keys, where):
    if not isinstance(mapping, dict): raise ValueError(f"{where} must be an object")
    missing = [k for k in keys if k not in mapping]
    if missing: raise ValueError(f"missing {where} keys: {', '.join(missing)}")


def validate(document):
    _require(document, ("schema", "payload", "config"), "document")
    schema = document["schema"]
    if schema not in ("e1.v1", "e1_band.v1", "e2.v1"): raise ValueError("unknown schema")
    _require(document["config"], COMMON_CONFIG_KEYS + (E2_CONFIG_KEYS if schema == "e2.v1" else ()), "config")
    if document["config"]["f_g_convention"] != "per_finger_normal_mean": raise ValueError("invalid f_g_convention")
    cal = document["config"]["calibration"]
    _require(cal, ("slope", "intercept", "residual", "hysteresis"), "calibration")
    if schema == "e1.v1": _require(document["payload"], E1_PAYLOAD_KEYS, "payload")
    elif schema == "e1_band.v1":
        _require(document["payload"], BAND_PAYLOAD_KEYS, "payload")
        for row in document["payload"]["rows"]: _require(row, ROW_KEYS, "band row")
        for cell in document["payload"]["coverage"].values():
            _require(cell, ("status", "reason"), "coverage cell")
            if cell["status"] not in {"done", "skipped_time_budget", "skipped_not_authorized", "skipped_failed"}: raise ValueError("invalid coverage status")
    else:
        c = document["config"]
        if c["raw_field_recorded"] is not True or c["aggregates_derived_from_raw"] is not True: raise ValueError("raw E2 invariants violated")
        if c["raw_field_layout"] != "concat+offsets" or c["taxel_binning"] != "post_hoc_out_of_scope": raise ValueError("invalid E2 layout")
    return document


def make(schema, payload, config):
    return validate({"schema": schema, "payload": payload, "config": config})

def write_json(path, document):
    validate(document)
    Path(path).write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")

def read_json(path):
    return validate(json.loads(Path(path).read_text(encoding="utf-8")))
