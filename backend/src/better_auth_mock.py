"""
Mock Better Auth implementation for authentication.

This module provides User, Session, and BaseClient classes that mimic
the Better Auth API for the authentication system.

Note: This is an in-memory implementation suitable for development and
small-scale production. For larger deployments, consider using a
persistent storage backend like Redis or the actual Better Auth service.
"""
import uuid
import hashlib
import secrets
import logging
from datetime import datetime, timedelta
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List

logger = logging.getLogger(__name__)


@dataclass
class User:
    """User model representing an authenticated user."""
    id: str
    email: str
    first_name: str
    last_name: str
    email_verified: bool = False
    metadata: Optional[Dict[str, Any]] = None
    created_at: datetime = field(default_factory=datetime.utcnow)
    updated_at: datetime = field(default_factory=datetime.utcnow)


@dataclass
class Session:
    """Session model representing an authenticated session."""
    token: str
    user: User
    expires_at: datetime
    created_at: datetime = field(default_factory=datetime.utcnow)


class BaseClient:
    """
    Better Auth compatible client for authentication operations.

    This provides an in-memory implementation of the Better Auth API
    for signup, signin, session verification, and signout.
    """

    def __init__(self, secret: str, base_url: str, trust_host: bool = False):
        """
        Initialize the auth client.

        Args:
            secret: Secret key for password hashing and token generation
            base_url: Base URL for the auth service (used for callbacks)
            trust_host: Whether to trust the host header
        """
        self.secret = secret
        self.base_url = base_url
        self.trust_host = trust_host

        # In-memory storage
        self._users: Dict[str, Dict[str, Any]] = {}
        self._sessions: Dict[str, Session] = {}
        self._email_to_user_id: Dict[str, str] = {}

        logger.info(f"Better Auth client initialized with base_url: {base_url}")

    def _hash_password(self, password: str) -> str:
        """Hash a password using SHA-256 with the secret key."""
        return hashlib.sha256((password + self.secret).encode()).hexdigest()

    def _generate_token(self) -> str:
        """Generate a secure random token."""
        return secrets.token_urlsafe(32)

    async def sign_up_with_email_password(
        self,
        email: str,
        password: str,
        first_name: str,
        last_name: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> User:
        """
        Register a new user with email and password.

        Args:
            email: User's email address
            password: User's password
            first_name: User's first name
            last_name: User's last name
            metadata: Optional metadata (e.g., profile data)

        Returns:
            Created User object

        Raises:
            ValueError: If user already exists
        """
        email_lower = email.lower().strip()

        if email_lower in self._email_to_user_id:
            logger.warning(f"Signup attempt for existing email: {email_lower}")
            raise ValueError("User already exists")

        user_id = str(uuid.uuid4())
        password_hash = self._hash_password(password)
        now = datetime.utcnow()

        user_data = {
            "id": user_id,
            "email": email_lower,
            "password_hash": password_hash,
            "first_name": first_name,
            "last_name": last_name,
            "email_verified": False,
            "metadata": metadata or {},
            "created_at": now,
            "updated_at": now
        }

        self._users[user_id] = user_data
        self._email_to_user_id[email_lower] = user_id

        user = User(
            id=user_id,
            email=email_lower,
            first_name=first_name,
            last_name=last_name,
            email_verified=False,
            metadata=metadata,
            created_at=now,
            updated_at=now
        )

        logger.info(f"User registered successfully: {user_id}")
        return user

    async def sign_in_with_email_password(
        self,
        email: str,
        password: str
    ) -> Optional[Session]:
        """
        Authenticate a user with email and password.

        Args:
            email: User's email address
            password: User's password

        Returns:
            Session object if authentication successful, None otherwise
        """
        email_lower = email.lower().strip()

        user_id = self._email_to_user_id.get(email_lower)
        if not user_id:
            logger.warning(f"Login attempt for non-existent email: {email_lower}")
            return None

        user_data = self._users.get(user_id)
        if not user_data:
            logger.error(f"User data not found for id: {user_id}")
            return None

        password_hash = self._hash_password(password)
        if user_data["password_hash"] != password_hash:
            logger.warning(f"Invalid password attempt for user: {user_id}")
            return None

        # Create session
        token = self._generate_token()
        now = datetime.utcnow()
        expires_at = now + timedelta(days=7)

        user = User(
            id=user_data["id"],
            email=user_data["email"],
            first_name=user_data["first_name"],
            last_name=user_data["last_name"],
            email_verified=user_data.get("email_verified", False),
            metadata=user_data.get("metadata"),
            created_at=user_data.get("created_at", now),
            updated_at=user_data.get("updated_at", now)
        )

        session = Session(
            token=token,
            user=user,
            expires_at=expires_at,
            created_at=now
        )

        self._sessions[token] = session

        logger.info(f"User signed in successfully: {user_id}")
        return session

    async def verify_session(self, token: str) -> Optional[Session]:
        """
        Verify a session token.

        Args:
            token: Session token to verify

        Returns:
            Session object if valid, None otherwise
        """
        if not token:
            return None

        session = self._sessions.get(token)
        if not session:
            logger.debug(f"Session not found for token")
            return None

        if session.expires_at < datetime.utcnow():
            logger.debug(f"Session expired for user: {session.user.id}")
            del self._sessions[token]
            return None

        return session

    async def sign_out(self, token: str) -> bool:
        """
        Sign out a user by invalidating their session.

        Args:
            token: Session token to invalidate

        Returns:
            True if session was invalidated, False otherwise
        """
        if token in self._sessions:
            user_id = self._sessions[token].user.id
            del self._sessions[token]
            logger.info(f"User signed out: {user_id}")
            return True
        return False

    async def get_user_by_id(self, user_id: str) -> Optional[User]:
        """
        Get a user by their ID.

        Args:
            user_id: User's ID

        Returns:
            User object if found, None otherwise
        """
        user_data = self._users.get(user_id)
        if not user_data:
            return None

        return User(
            id=user_data["id"],
            email=user_data["email"],
            first_name=user_data["first_name"],
            last_name=user_data["last_name"],
            email_verified=user_data.get("email_verified", False),
            metadata=user_data.get("metadata"),
            created_at=user_data.get("created_at", datetime.utcnow()),
            updated_at=user_data.get("updated_at", datetime.utcnow())
        )

    async def get_user_by_email(self, email: str) -> Optional[User]:
        """
        Get a user by their email.

        Args:
            email: User's email address

        Returns:
            User object if found, None otherwise
        """
        email_lower = email.lower().strip()
        user_id = self._email_to_user_id.get(email_lower)
        if not user_id:
            return None
        return await self.get_user_by_id(user_id)

    async def update_user(
        self,
        user_id: str,
        first_name: Optional[str] = None,
        last_name: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> Optional[User]:
        """
        Update a user's information.

        Args:
            user_id: User's ID
            first_name: New first name (optional)
            last_name: New last name (optional)
            metadata: New metadata (optional)

        Returns:
            Updated User object if successful, None otherwise
        """
        user_data = self._users.get(user_id)
        if not user_data:
            return None

        if first_name is not None:
            user_data["first_name"] = first_name
        if last_name is not None:
            user_data["last_name"] = last_name
        if metadata is not None:
            user_data["metadata"] = metadata

        user_data["updated_at"] = datetime.utcnow()

        return await self.get_user_by_id(user_id)

    def get_active_sessions_count(self) -> int:
        """Get the count of active sessions."""
        now = datetime.utcnow()
        return sum(1 for s in self._sessions.values() if s.expires_at > now)

    def get_users_count(self) -> int:
        """Get the total count of registered users."""
        return len(self._users)
