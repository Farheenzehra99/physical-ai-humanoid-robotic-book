import time
import logging
from typing import Optional
from google import genai
from pathlib import Path

from ..config import get_settings
from ..database.crud import get_user_preferences

logger = logging.getLogger(__name__)

# Load prompt template
PROMPT_TEMPLATE_PATH = Path(__file__).parent / "personalization_prompt.md"


class PersonalizationService:
    """Service for personalizing chapter content using LLM."""

    def __init__(self):
        self.settings = get_settings()

        # ✅ NEW Gemini client (safe)
        self.client = genai.Client(
            api_key=self.settings.gemini_api_key
        )

        # ✅ Model name MUST be new
        self.model_name = self.settings.gemini_model  # e.g. "gemini-1.5-flash"

        self.generation_config = {
            "temperature": 0.3,
            "max_output_tokens": 16384,
        }

        self.system_prompt = self._load_prompt_template()

    def _load_prompt_template(self) -> str:
        """Load the personalization prompt template."""
        try:
            return PROMPT_TEMPLATE_PATH.read_text(encoding="utf-8")
        except FileNotFoundError:
            logger.warning("Prompt template not found, using default")
            return self._default_prompt()

    async def personalize_chapter(
        self,
        chapter_content: str,
        chapter_title: str,
        user_profile: dict
    ) -> tuple[str, dict]:
        """
        Personalize chapter content for user.

        Returns:
            Tuple of (personalized_content, adaptation_summary)
        """
        start_time = time.time()

        # Build the full prompt
        prompt = self._build_prompt(
            chapter_content=chapter_content,
            chapter_title=chapter_title,
            user_profile=user_profile
        )

        # Call Gemini
        try:
            response = await self.model.generate_content_async(prompt)
            personalized_content = response.text
        except Exception as e:
            logger.error(f"LLM call failed: {e}")
            raise

        # Validate response
        self._validate_response(chapter_content, personalized_content)

        processing_time = int((time.time() - start_time) * 1000)

        adaptation_summary = {
            "level_adjustment": user_profile.get("programming_level", "unknown"),
            "processing_time_ms": processing_time,
            "content_length_original": len(chapter_content),
            "content_length_personalized": len(personalized_content)
        }

        return personalized_content, adaptation_summary

    def _build_prompt(
        self,
        chapter_content: str,
        chapter_title: str,
        user_profile: dict
    ) -> str:
        """Build the full LLM prompt with user context."""
        return self.system_prompt.format(
            programming_level=user_profile.get("programming_level", "Not specified"),
            programming_languages=", ".join(user_profile.get("programming_languages", [])) or "Not specified",
            ai_knowledge_level=user_profile.get("ai_knowledge_level", "Not specified"),
            hardware_experience=user_profile.get("hardware_experience", "Not specified"),
            learning_style=user_profile.get("learning_style", "Not specified"),
            chapter_title=chapter_title or "Untitled",
            chapter_content=chapter_content
        )

    def _validate_response(
        self,
        original: str,
        personalized: str
    ) -> None:
        """Validate that response preserves structure."""
        import re

        # Extract headings from original
        original_headings = re.findall(r'^#{1,4}\s+.+$', original, re.MULTILINE)
        personalized_headings = re.findall(r'^#{1,4}\s+.+$', personalized, re.MULTILINE)

        # Check heading preservation (allow some flexibility)
        if len(personalized_headings) < len(original_headings) * 0.8:
            logger.warning(
                f"Heading count mismatch: {len(original_headings)} -> {len(personalized_headings)}"
            )

        # Check code block preservation
        original_code_blocks = len(re.findall(r'```[\s\S]*?```', original))
        personalized_code_blocks = len(re.findall(r'```[\s\S]*?```', personalized))

        if personalized_code_blocks < original_code_blocks:
            logger.warning(
                f"Code block count reduced: {original_code_blocks} -> {personalized_code_blocks}"
            )

    def _default_prompt(self) -> str:
        """Return default prompt if template file not found."""
        return """You are an expert technical writer. Adapt the provided content based on the user's experience level and background while preserving all structural elements like headings, code blocks, and links."""