"""
Database base module.

This module defines the SQLAlchemy Base class to avoid circular imports.
"""

from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()