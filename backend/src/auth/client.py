"""
Authentication client module.

This module initializes and provides the Better Auth client instance
to avoid circular imports between main.py and endpoints.py.
"""

from ..better_auth_mock import BaseClient
from .config import auth_settings


def initialize_auth_client() -> BaseClient:
    """Initialize Better Auth client with configuration."""
    if not auth_settings.auth_secret or not auth_settings.auth_url:
        raise RuntimeError("Better Auth is not configured. Check AUTH_SECRET and AUTH_URL.")

    client = BaseClient(
        secret=auth_settings.auth_secret,
        base_url=auth_settings.auth_url,
        trust_host=auth_settings.auth_trust_host
    )
    return client


# Create global auth client instance
auth_client = initialize_auth_client()