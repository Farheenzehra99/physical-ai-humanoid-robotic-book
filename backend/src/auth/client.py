"""
Authentication client module.

This module initializes and provides the Better Auth client instance
to avoid circular imports between main.py and endpoints.py.
"""

from ..better_auth_mock import BaseClient
from .config import auth_settings


def initialize_auth_client() -> BaseClient:
    """Initialize Better Auth client with configuration."""
    if not auth_settings.auth_secret:
        raise RuntimeError("Better Auth is not configured. Check AUTH_SECRET.")

    # Use a local identifier instead of external URL since we're using database-first approach
    client = BaseClient(
        secret=auth_settings.auth_secret,
        base_url="http://localhost",  # Local base URL since we're not making external calls
        trust_host=auth_settings.auth_trust_host
    )
    return client


# Create global auth client instance
auth_client = initialize_auth_client()