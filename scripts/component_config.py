#!/usr/bin/env python3
"""API key configuration for component distributor APIs.

Usage:
    1. Copy this file to component_config_local.py (already in .gitignore).
    2. Fill in your actual API keys.
    3. Import:  from component_config import config

Account setup:
    - Mouser:  Register at https://www.mouser.com/api-hub/
               Get API key from dashboard → "Request API Key"
               Free tier: 1000 calls/day

    - Digi-Key: Register at https://developer.digikey.com/
               Create a Production App → get Client ID + Client Secret
               Set OAuth callback to https://localhost:8139/digikey_callback
               Free tier: 5000 calls/month

Environment variables override these defaults.
"""

from __future__ import annotations

import os
import sys
# Ensure scripts/ is in path for sibling imports
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class Config:
    # ── Mouser API ────────────────────────────────────────────────────
    mouser_api_key: str = ""  # os.environ["MOUSER_API_KEY"]

    # ── Digi-Key API ──────────────────────────────────────────────────
    digikey_client_id: str = ""  # os.environ["DIGIKEY_CLIENT_ID"]
    digikey_client_secret: str = ""  # os.environ["DIGIKEY_CLIENT_SECRET"]
    digikey_sandbox: bool = False  # True → sandbox data, False → production
    digikey_storage_path: str = ""

    # ── Local paths ───────────────────────────────────────────────────
    cache_dir: Path = field(default_factory=lambda: Path(__file__).parent / ".." / "data_input" / "component_cache")
    pdfs_dir: Path = field(default_factory=lambda: Path(__file__).parent / ".." / "pdfs")

    def __post_init__(self):
        # Env overrides
        self.mouser_api_key = os.environ.get("MOUSER_API_KEY", self.mouser_api_key)
        self.digikey_client_id = os.environ.get("DIGIKEY_CLIENT_ID", self.digikey_client_id)
        self.digikey_client_secret = os.environ.get("DIGIKEY_CLIENT_SECRET", self.digikey_client_secret)
        sandbox = os.environ.get("DIGIKEY_CLIENT_SANDBOX", str(self.digikey_sandbox))
        self.digikey_sandbox = sandbox.lower() in ("true", "1", "yes")
        self.digikey_storage_path = os.environ.get("DIGIKEY_STORAGE_PATH",
                                                    str(self.cache_dir / "digikey_oauth"))
        # Ensure dirs
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.pdfs_dir.mkdir(parents=True, exist_ok=True)
        if self.digikey_storage_path:
            Path(self.digikey_storage_path).mkdir(parents=True, exist_ok=True)


# ── Singleton ─────────────────────────────────────────────────────────
try:
    from component_config_local import local_config
    config = local_config
except ImportError:
    config = Config()

# Detect missing keys
_issues = []
if not config.digikey_client_id:
    _issues.append("Digi-Key: missing client ID (set DIGIKEY_CLIENT_ID env or component_config_local.py)")
if not config.digikey_client_secret:
    _issues.append("Digi-Key: missing client secret (set DIGIKEY_CLIENT_SECRET env or component_config_local.py)")
if config.digikey_sandbox:
    _issues.append("Digi-Key: SANDBOX mode — returned data may be dummy values")

if not config.mouser_api_key:
    pass  # Mouser is optional; warn only when actually used
