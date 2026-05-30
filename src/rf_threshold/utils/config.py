"""Configuration loading utilities."""

from pathlib import Path
from typing import Any, Dict, Union

import yaml


def load_yaml_config(config_path: Union[str, Path]) -> Dict[str, Any]:
    """Load a YAML configuration file.

    Args:
        config_path: Path to the YAML configuration file.

    Returns:
        Parsed configuration dictionary.

    Raises:
        FileNotFoundError: If the config file does not exist.
        ValueError: If the YAML file is empty or invalid.
    """
    path = Path(config_path)

    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    if not path.is_file():
        raise ValueError(f"Config path is not a file: {path}")

    with path.open("r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    if cfg is None:
        raise ValueError(f"Config file is empty: {path}")

    if not isinstance(cfg, dict):
        raise ValueError(f"Config root must be a dictionary: {path}")

    return cfg


def require_config_key(cfg: Dict[str, Any], key: str) -> Any:
    """Return a required config value.

    Args:
        cfg: Configuration dictionary.
        key: Required key.

    Returns:
        Value stored at the given key.

    Raises:
        KeyError: If the key is missing.
    """
    if key not in cfg:
        raise KeyError(f"Missing required config key: {key}")

    return cfg[key]