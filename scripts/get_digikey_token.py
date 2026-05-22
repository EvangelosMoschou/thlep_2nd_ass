#!/usr/bin/env python3
"""OAuth2 token fetcher for Digi-Key API V4.

Usage:
    python3 scripts/get_digikey_token.py

This will:
  1. Print the Digi-Key authorization URL
  2. Start a local HTTP server on port 8139 to catch the redirect
  3. Wait for you to open the URL and authorize
  4. Exchange the code for access + refresh tokens
  5. Save to data_input/component_cache/digikey_token.json

Requires:
    pip install requests
"""

from __future__ import annotations

import json
import os
import sys
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlencode, parse_qs, urlparse

import requests

SCRIPTS_DIR = Path(__file__).parent
PROJECT_DIR = SCRIPTS_DIR.parent

# Load API credentials
sys.path.insert(0, str(SCRIPTS_DIR))
from component_config import config

CLIENT_ID = config.digikey_client_id
CLIENT_SECRET = config.digikey_client_secret
REDIRECT_URI = "https://localhost:8139/digikey_callback"

AUTH_URL = "https://api.digikey.com/v1/oauth2/authorize"
TOKEN_URL = "https://api.digikey.com/v1/oauth2/token"

received_code = None


class CallbackHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global received_code
        parsed = urlparse(self.path)
        params = parse_qs(parsed.query)
        code = params.get("code", [None])[0]

        if code:
            received_code = code
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b"<html><body><h1>Authorization OK!</h1>"
                             b"<p>You can close this tab.</p></body></html>")
        else:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b"<html><body><h1>Error: No code received</h1></body></html>")


def main():
    global received_code

    if not CLIENT_ID or not CLIENT_SECRET:
        print("ERROR: Digi-Key credentials not configured.")
        print("Set DIGIKEY_CLIENT_ID and DIGIKEY_CLIENT_SECRET")
        print("or edit scripts/component_config_local.py")
        sys.exit(1)

    # Step 1: Build authorization URL
    params = {
        "response_type": "code",
        "client_id": CLIENT_ID,
        "redirect_uri": REDIRECT_URI,
    }
    auth_url = f"{AUTH_URL}?{urlencode(params)}"

    print("=" * 60)
    print("DIGI-KEY OAUTH2 TOKEN FETCHER")
    print("=" * 60)
    print(f"\n1. Open this URL in your browser:")
    print(f"\n   {auth_url}\n")
    print("2. Log in to Digi-Key and authorize the app.")
    print("3. You'll be redirected to localhost:8139/digikey_callback")
    print("   (The server below will catch it automatically)")
    print()

    # Step 2: Start local server to catch redirect
    server = HTTPServer(("localhost", 8139), CallbackHandler)
    server.timeout = 120  # 2 minute timeout

    # Try to open browser automatically
    import webbrowser
    try:
        webbrowser.open(auth_url)
        print("   [Browser opened automatically]")
    except Exception:
        pass

    print("   Waiting for redirect on http://localhost:8139 ...")
    print("   (timeout: 120 seconds)")
    print()

    while received_code is None:
        server.handle_request()

    code = received_code
    print(f"\n   Authorization code received!")

    # Step 3: Exchange code for tokens
    print("\n3. Exchanging code for tokens...")
    resp = requests.post(
        TOKEN_URL,
        data={
            "grant_type": "authorization_code",
            "code": code,
            "client_id": CLIENT_ID,
            "client_secret": CLIENT_SECRET,
            "redirect_uri": REDIRECT_URI,
        },
        timeout=30,
    )

    if resp.status_code != 200:
        print(f"\nERROR: Token exchange failed (HTTP {resp.status_code})")
        print(f"Response: {resp.text[:300]}")
        sys.exit(1)

    token_data = resp.json()
    token_data["expires_at"] = time.time() + token_data.get("expires_in", 3600)

    # Step 4: Save token
    token_path = Path(config.digikey_storage_path) / "digikey_token.json"
    token_path.parent.mkdir(parents=True, exist_ok=True)
    token_path.write_text(json.dumps(token_data, indent=2))

    print(f"\n   Token saved to: {token_path}")
    print(f"   Access token:  {token_data['access_token'][:40]}...")
    print(f"   Refresh token: {token_data.get('refresh_token', 'N/A')[:40]}...")
    print(f"   Expires in:    {token_data.get('expires_in', '?')} seconds")
    print()
    print("=" * 60)
    print("You can now use component_db.py to search Digi-Key!")
    print("=" * 60)


if __name__ == "__main__":
    main()
