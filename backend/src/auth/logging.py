"""
Logging utilities for authentication system.

This module contains logging configurations and utilities
for the authentication system.
"""

import logging
from typing import Optional
import json
from datetime import datetime


class AuthLogger:
    """
    Custom logger for authentication system with structured logging.
    """

    def __init__(self, name: str = "auth_system"):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)

        # Create formatter for structured logging
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)

        # Add handlers if not already added
        if not self.logger.handlers:
            self.logger.addHandler(console_handler)

    def log_registration_attempt(self, email: str, ip_address: str, success: bool, user_id: Optional[str] = None):
        """
        Log user registration attempt.

        Args:
            email: User's email
            ip_address: IP address of the request
            success: Whether the registration was successful
            user_id: User ID if successful, None otherwise
        """
        event = {
            "event_type": "registration_attempt",
            "email": email,
            "ip_address": ip_address,
            "success": success,
            "user_id": user_id,
            "timestamp": datetime.utcnow().isoformat()
        }

        if success:
            self.logger.info(json.dumps(event))
        else:
            self.logger.warning(json.dumps(event))

    def log_login_attempt(self, email: str, ip_address: str, success: bool, user_id: Optional[str] = None):
        """
        Log user login attempt.

        Args:
            email: User's email
            ip_address: IP address of the request
            success: Whether the login was successful
            user_id: User ID if successful, None otherwise
        """
        event = {
            "event_type": "login_attempt",
            "email": email,
            "ip_address": ip_address,
            "success": success,
            "user_id": user_id,
            "timestamp": datetime.utcnow().isoformat()
        }

        if success:
            self.logger.info(json.dumps(event))
        else:
            self.logger.warning(json.dumps(event))

    def log_logout(self, user_id: str, ip_address: str):
        """
        Log user logout.

        Args:
            user_id: User ID
            ip_address: IP address of the request
        """
        event = {
            "event_type": "logout",
            "user_id": user_id,
            "ip_address": ip_address,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.logger.info(json.dumps(event))

    def log_profile_access(self, user_id: str, ip_address: str, success: bool):
        """
        Log profile access attempt.

        Args:
            user_id: User ID
            ip_address: IP address of the request
            success: Whether the access was successful
        """
        event = {
            "event_type": "profile_access",
            "user_id": user_id,
            "ip_address": ip_address,
            "success": success,
            "timestamp": datetime.utcnow().isoformat()
        }

        if success:
            self.logger.info(json.dumps(event))
        else:
            self.logger.warning(json.dumps(event))

    def log_profile_update(self, user_id: str, ip_address: str, success: bool, fields_updated: list):
        """
        Log profile update attempt.

        Args:
            user_id: User ID
            ip_address: IP address of the request
            success: Whether the update was successful
            fields_updated: List of fields that were updated
        """
        event = {
            "event_type": "profile_update",
            "user_id": user_id,
            "ip_address": ip_address,
            "success": success,
            "fields_updated": fields_updated,
            "timestamp": datetime.utcnow().isoformat()
        }

        if success:
            self.logger.info(json.dumps(event))
        else:
            self.logger.warning(json.dumps(event))

    def log_security_event(self, event_type: str, user_id: Optional[str], ip_address: str, details: dict):
        """
        Log security-related events.

        Args:
            event_type: Type of security event
            user_id: User ID if applicable
            ip_address: IP address of the request
            details: Additional event details
        """
        event = {
            "event_type": f"security_{event_type}",
            "user_id": user_id,
            "ip_address": ip_address,
            "details": details,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.logger.warning(json.dumps(event))

    def log_rate_limit_exceeded(self, endpoint: str, ip_address: str):
        """
        Log rate limit exceeded events.

        Args:
            endpoint: The endpoint that was rate limited
            ip_address: IP address that exceeded rate limit
        """
        event = {
            "event_type": "rate_limit_exceeded",
            "endpoint": endpoint,
            "ip_address": ip_address,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.logger.warning(json.dumps(event))


# Global instance of the auth logger
auth_logger = AuthLogger()


# Convenience functions for common logging operations
def log_registration_attempt(email: str, ip_address: str, success: bool, user_id: Optional[str] = None):
    auth_logger.log_registration_attempt(email, ip_address, success, user_id)


def log_login_attempt(email: str, ip_address: str, success: bool, user_id: Optional[str] = None):
    auth_logger.log_login_attempt(email, ip_address, success, user_id)


def log_logout(user_id: str, ip_address: str):
    auth_logger.log_logout(user_id, ip_address)


def log_profile_access(user_id: str, ip_address: str, success: bool):
    auth_logger.log_profile_access(user_id, ip_address, success)


def log_profile_update(user_id: str, ip_address: str, success: bool, fields_updated: list):
    auth_logger.log_profile_update(user_id, ip_address, success, fields_updated)


def log_security_event(event_type: str, user_id: Optional[str], ip_address: str, details: dict):
    auth_logger.log_security_event(event_type, user_id, ip_address, details)


def log_rate_limit_exceeded(endpoint: str, ip_address: str):
    auth_logger.log_rate_limit_exceeded(endpoint, ip_address)