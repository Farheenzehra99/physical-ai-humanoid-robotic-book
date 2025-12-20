"""
Retrieval module for the RAG system.

This module provides the retrieve function used by the agent to fetch
relevant document chunks from the knowledge base.
"""
from .retrieve import retrieve

__all__ = ["retrieve"]