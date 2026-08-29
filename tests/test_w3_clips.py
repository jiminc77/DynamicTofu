import pytest

from scripts.vbd.w3_clips_v2 import assemble_manifest, assert_label_reproduced


def test_manifest_assembly_and_label_equality(tmp_path, monkeypatch):
    monkeypatch.setattr("scripts.vbd.w3_clips_v2.ROOT", tmp_path)
    key = tmp_path / "key.png"
    movie = tmp_path / "scene.mp4"
    key.write_bytes(b"key frame")
    movie.write_bytes(b"movie")
    result = {
        "scene": "intact", "E": 15, "a": 1.0, "F": 1.2, "seed": 0,
        "label": "intact", "rerun_label": "intact", "realized": 0.681,
        "mp4": movie, "key_paths": [key],
    }

    assert_label_reproduced("intact", "intact", "intact")
    manifest = assemble_manifest([result], "abc123")

    assert manifest["git_commit"] == "abc123"
    assert manifest["scenes"][0]["label_reproduced"] is True
    assert manifest["scenes"][0]["source_final_band_label"] == "intact"
    assert manifest["scenes"][0]["rerun_label"] == "intact"
    assert manifest["scenes"][0]["sha256"]["mp4"]
    assert manifest["scenes"][0]["key_frames"][0]["sha256"]
    with pytest.raises(RuntimeError, match="does not match final-band label"):
        assert_label_reproduced("intact", "slip", "intact")
