"""
OpenAI Assistants API configuration for the RAG system.

This module configures the OpenAI Assistant with the retrieval tool
and grounding instructions using the OpenAI Assistants API.
"""

import logging
import re
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List

from openai import OpenAI

from .tools import retrieve_context
from .tools.retrieval_tool import retrieve_context_async
from ..config import get_settings
from ..models import Citation


logger = logging.getLogger(__name__)


# Load system prompt from markdown file
PROMPT_PATH = Path(__file__).parent / "agent_prompt.md"


def load_system_prompt() -> str:
    """Load the system prompt from the markdown file."""
    try:
        return PROMPT_PATH.read_text(encoding="utf-8")
    except FileNotFoundError:
        logger.warning(f"System prompt not found at {PROMPT_PATH}, using default")
        return """You are a helpful assistant that answers questions about the Physical AI and Humanoid Robotics book.
Always use the retrieve_context tool before answering. Only use information from retrieved context.
Cite all sources with [Source: Title](url) format."""


@dataclass
class AgentResult:
    """Result from agent execution."""
    answer: str
    citations: list[Citation] = field(default_factory=list)
    retrieval_count: int = 0
    raw_output: str = ""


class BookAgent:
    """
    RAG Assistant for answering questions about the Physical AI book.

    Uses OpenAI Assistants API with retrieval for grounded responses.
    """

    def __init__(self):
        """Initialize the assistant with configuration."""
        self.settings = get_settings()
        self.system_prompt = load_system_prompt()

        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.settings.openrouter_api_key or self.settings.gemini_api_key)

        # Define the tool that the assistant can use
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "retrieve_context",
                    "description": "Retrieve relevant document chunks from the Physical AI & Humanoid Robotics book",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "The search query text - should capture the key concepts being asked about"
                            },
                            "top_k": {
                                "type": "integer",
                                "description": "Number of results to return (default: 5, max: 20)",
                                "default": 5
                            }
                        },
                        "required": ["query"]
                    }
                }
            }
        ]

    async def run(
        self,
        question: str,
        selected_text: str | None = None,
        top_k: int = 5,
        user_profile: dict | None = None
    ) -> AgentResult:
        """
        Run the assistant with a question and return grounded response.

        Args:
            question: User question about the book
            selected_text: Optional selected text to focus the answer
            top_k: Number of retrieval results to use
            user_profile: Optional user profile data for personalization

        Returns:
            AgentResult with answer, citations, and metadata
        """
        # Build the input message
        user_input = question

        # Add personalization context if user profile is available
        if user_profile:
            personalization_context = f"""
User Profile Context:
- Programming Level: {user_profile.get('programming_level', 'Not specified')}
- Programming Languages: {', '.join(user_profile.get('programming_languages', [])) or 'Not specified'}
- AI Knowledge Level: {user_profile.get('ai_knowledge_level', 'Not specified')}
- Hardware Experience: {user_profile.get('hardware_experience', 'Not specified')}
- Learning Style: {user_profile.get('learning_style', 'Not specified')}

Please adjust your response to match the user's experience level and learning preferences.
"""
            user_input = personalization_context + "\n\n" + question

        if selected_text:
            user_input = f"""The user has selected the following text from the book:

<selected_text>
{selected_text}
</selected_text>

Question: {question}

Please focus your answer on the selected text while using retrieved context for additional information."""
        else:
            user_input = question

        logger.info(f"Running assistant with question: {question[:100]}...")

        try:
            # For now, since we're not using the full Assistants API in this implementation,
            # we'll implement the RAG flow directly while keeping the structure for when
            # the Assistants API is properly implemented

            # Retrieve context first (handle gracefully if retrieval fails)
            retrieval_results = []
            context_str = ""

            try:
                retrieval_results = await retrieve_context_async(user_input, top_k)

                # Format context from retrieval results
                if retrieval_results:
                    context_str = "Here is relevant context from the book:\n\n"
                    for i, result in enumerate(retrieval_results[:top_k], 1):
                        context_str += f"Context {i}:\n"
                        context_str += f"Title: {result.get('title', 'Unknown')}\n"
                        context_str += f"URL: {result.get('source_url', 'Unknown')}\n"
                        context_str += f"Content: {result.get('text', '')}\n\n"
            except Exception as retrieval_error:
                logger.warning(f"Retrieval failed, proceeding without context: {retrieval_error}")
                # Continue without retrieval context

            # Create a message with context and user question
            full_message = context_str + user_input

            # Call the OpenAI-compatible API (OpenRouter)
            response = self.client.chat.completions.create(
                model=self.settings.ai_model_name if hasattr(self.settings, 'ai_model_name') else self.settings.openrouter_model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": full_message}
                ],
                temperature=0.3,
                max_tokens=1000
            )

            raw_output = response.choices[0].message.content if response.choices else ""

            # Extract citations from the response
            citations = self._extract_citations(raw_output)

            # Build citation objects from retrieval results if none extracted
            if not citations and retrieval_results:
                # If no citations extracted from text, use retrieval results
                citations = self._deduplicate_citations([
                    Citation.from_retrieval_result(r)
                    for r in retrieval_results[:top_k]
                ])

            # Log citation metrics
            unique_sources = set(c.source_url for c in citations)
            logger.info(
                f"Citation metrics: {len(citations)} citations from "
                f"{len(unique_sources)} unique sources"
            )

            # Handle no-context case
            if not retrieval_results:
                logger.warning("No retrieval results found for query")
                if "could not find" not in raw_output.lower():
                    # Add disclaimer if agent didn't include one
                    raw_output = (
                        "I could not find relevant information about this in the book. "
                        "The Physical AI & Humanoid Robotics book may not cover this specific topic.\n\n"
                        + raw_output
                    )

            return AgentResult(
                answer=raw_output,
                citations=citations,
                retrieval_count=len(retrieval_results),
                raw_output=raw_output
            )

        except Exception as e:
            logger.error(f"Agent execution failed: {e}")
            raise

    def _extract_citations(self, text: str) -> list[Citation]:
        """
        Extract citations from agent response text.

        Looks for patterns like [Source: Title](url)
        Returns deduplicated citations.
        """
        citations = []
        seen_urls = set()

        # Pattern: [Source: Title](url)
        pattern = r'\[Source:\s*([^\]]+)\]\(([^)]+)\)'

        for match in re.finditer(pattern, text):
            title = match.group(1).strip()
            url = match.group(2).strip()

            # Avoid duplicates
            if url not in seen_urls:
                seen_urls.add(url)
                citations.append(Citation(
                    source_url=url,
                    title=title,
                    chunk_id=f"extracted_{len(citations)}",
                    relevance_score=1.0  # Extracted citations are considered highly relevant
                ))

        logger.debug(f"Extracted {len(citations)} citations from response")
        return citations

    def _deduplicate_citations(self, citations: list[Citation]) -> list[Citation]:
        """
        Remove duplicate citations based on source_url.

        Keeps the citation with the highest relevance score.
        """
        seen: dict[str, Citation] = {}

        for citation in citations:
            url = citation.source_url
            if url not in seen or citation.relevance_score > seen[url].relevance_score:
                seen[url] = citation

        deduped = list(seen.values())
        logger.debug(
            f"Deduplicated citations: {len(citations)} -> {len(deduped)}"
        )
        return deduped


def create_book_agent() -> BookAgent:
    """Factory function to create a BookAgent instance."""
    return BookAgent()
