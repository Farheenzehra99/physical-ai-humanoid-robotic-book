"""
Main authentication module.

This module initializes Better Auth with FastAPI and sets up
custom callbacks for profile data handling.
"""

from fastapi import FastAPI
from ..better_auth_mock import BaseClient, Session as BetterAuthSession
from typing import Optional
import os
from .config import auth_settings
from .callbacks import after_user_registration, after_user_signin
from .endpoints import router as get_auth_router


def initialize_auth_client() -> BaseClient:
    if not auth_settings.auth_secret or not auth_settings.auth_url:
        raise RuntimeError("Better Auth is not configured. Check AUTH_SECRET and AUTH_URL.")

    """
    Initialize Better Auth client with configuration.

    Returns:
        Configured Better Auth client instance
    """
    client = BaseClient(
        secret=auth_settings.auth_secret,
        base_url=auth_settings.auth_url,
        trust_host=auth_settings.auth_trust_host
    )
    return client


def get_auth_router_with_callbacks():
    """
    Get Better Auth router with custom callbacks for profile data.

    Returns:
        Configured auth router with custom callbacks
    """
    # Configure Better Auth with custom callbacks for profile data
    auth_config = {
        "secret": auth_settings.auth_secret,
        "base_url": auth_settings.auth_url,
        "trust_host": auth_settings.auth_trust_host,
        "email_verification": {
            "enabled": True,
            "send_on_signup": True
        }
    }

    # Initialize client
    client = initialize_auth_client()

    # Get the auth router with custom callbacks
    auth_router = get_auth_router(
        client=client,
        config=auth_config,
        # Custom callbacks for handling profile data
        after_register_callback=after_user_registration,
        after_signin_callback=after_user_signin
    )

    return auth_router


# Create global auth client instance
auth_client = initialize_auth_client()