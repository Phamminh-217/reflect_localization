"""Tests for configuration utilities."""

from pathlib import Path

import pytest
import yaml

from rf_threshold.utils.config import load_yaml_config, require_config_key


def test_load_yaml_config_reads_valid_file(tmp_path: Path):
    config_path = tmp_path / "test_config.yaml"

    config_data = {
        "threshold": {
            "mode": "fixed",
            "fixed_intensity": 180.0,
        }
    }

    with config_path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(config_data, file)

    cfg = load_yaml_config(config_path)

    assert cfg["threshold"]["mode"] == "fixed"
    assert cfg["threshold"]["fixed_intensity"] == 180.0


def test_load_yaml_config_raises_for_missing_file(tmp_path: Path):
    config_path = tmp_path / "missing.yaml"

    with pytest.raises(FileNotFoundError):
        load_yaml_config(config_path)


def test_require_config_key_returns_existing_key():
    cfg = {"threshold": {"mode": "fixed"}}

    value = require_config_key(cfg, "threshold")

    assert value == {"mode": "fixed"}


def test_require_config_key_raises_for_missing_key():
    cfg = {"threshold": {"mode": "fixed"}}

    with pytest.raises(KeyError):
        require_config_key(cfg, "clustering")
