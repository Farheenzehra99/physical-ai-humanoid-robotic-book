"""
Response models for the RAG Agent API.

This module defines Pydantic models for API responses including
ChatResponse, Citation, and ResponseMetadata.
"""

import uuid
from typing import Any

from pydantic import BaseModel, Field


class Citation(BaseModel):
    """
    Source attribution for factual claims in the response.

    Attributes:
        source_url: URL of the source document
        title: Title of the document/section
        chunk_id: Unique identifier for the chunk
        relevance_score: Similarity score (0-1)
    """

    source_url: str = Field(..., description="URL of the source document")
    title: str = Field(..., description="Title of the document/section")
    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    relevance_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Similarity score (0-1)"
    )

    @classmethod
    def from_retrieval_result(cls, result: dict[str, Any]) -> "Citation":
        """
        Create Citation from retrieval result dictionary.

        Args:
            result: Dictionary from retrieval pipeline containing:
                - source_url or url: URL of the source
                - title: Document title
                - chunk_id: Chunk identifier
                - score: Relevance score

        Returns:
            Citation: New Citation instance
        """
        return cls(
            source_url=result.get("source_url") or result.get("url", ""),
            title=result.get("title", "Unknown"),
            chunk_id=result.get("chunk_id", ""),
            relevance_score=float(result.get("score", 0.0))
        )


class ResponseMetadata(BaseModel):
    """
    Processing information for debugging and monitoring.

    Attributes:
        request_id: Unique identifier for the request
        processing_time_ms: Total processing time in milliseconds
        retrieval_count: Number of chunks retrieved
        model_used: LLM model used for generation
    """

    request_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique identifier for the request"
    )
    processing_time_ms: float = Field(
        ...,
        ge=0,
        description="Total processing time in milliseconds"
    )
    retrieval_count: int = Field(
        ...,
        ge=0,
        description="Number of chunks retrieved"
    )
    model_used: str = Field(
        ...,
        description="LLM model used for generation"
    )


class ChatResponse(BaseModel):
    """
    Response containing the grounded answer and source citations.

    Attributes:
        answer: Generated answer grounded in retrieved context
        citations: List of sources used in the answer
        metadata: Request processing metadata
    """

    answer: str = Field(
        ...,
        description="Generated answer grounded in retrieved context"
    )
    citations: list[Citation] = Field(
        default_factory=list,
        description="List of sources used in the answer"
    )
    metadata: ResponseMetadata = Field(
        ...,
        description="Request processing metadata"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "answer": "Physical AI refers to artificial intelligence systems designed to interact with and operate in the physical world. [Source: Introduction to Physical AI](https://physical-ai-book.com/chapter-1)",
                    "citations": [
                        {
                            "source_url": "https://physical-ai-book.com/chapter-1",
                            "title": "Chapter 1: Introduction to Physical AI",
                            "chunk_id": "chunk_042",
                            "relevance_score": 0.92
                        }
                    ],
                    "metadata": {
                        "request_id": "550e8400-e29b-41d4-a716-446655440000",
                        "processing_time_ms": 423.5,
                        "retrieval_count": 5,
                        "model_used": "gpt-4o"
                    }
                }
            ]
        }
    }


class HealthResponse(BaseModel):
    """
    Health check response showing API and dependency status.

    Attributes:
        status: Overall health status (healthy/unhealthy)
        version: API version
        dependencies: Status of external dependencies
    """

    status: str = Field(
        ...,
        description="Overall health status",
        pattern="^(healthy|unhealthy)$"
    )
    version: str = Field(..., description="API version")
    dependencies: dict[str, str] = Field(
        default_factory=dict,
        description="Status of external dependencies"
    )
