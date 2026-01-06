import time
import logging
import re
from typing import Optional, List, Tuple
from google import genai
from pathlib import Path

from ..config import get_settings

logger = logging.getLogger(__name__)

# Load prompt template
PROMPT_TEMPLATE_PATH = Path(__file__).parent / "translation_prompt.md"


class TranslationService:
    """Service for translating chapter content to Urdu using LLM."""

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

    # Compile regex patterns for technical elements
    self._compile_patterns()

    def _load_prompt_template(self) -> str:
        """Load the translation prompt template."""
        try:
            return PROMPT_TEMPLATE_PATH.read_text(encoding="utf-8")
        except FileNotFoundError:
            logger.warning("Prompt template not found, using default")
            return self._default_prompt()

    def _default_prompt(self) -> str:
        """Return default prompt if template file not found."""
        return """You are an expert translator. Translate the provided content to Urdu while preserving all structural elements like headings, code blocks, and links."""

    def _compile_patterns(self):
        """Compile regex patterns for identifying technical elements."""
        self.technical_patterns = [
            # ROS topics (e.g., /cmd_vel, /joint_states) - must start with / and have valid topic structure
            {
                'pattern': re.compile(r'/(?=[a-zA-Z0-9_])[a-zA-Z0-9_/]*(?<!/)(?=\s|$|[^\w/])', re.IGNORECASE),
                'name': 'ros_topic'
            },
            # CLI commands (e.g., ros2 run, git clone) - specific command patterns
            {
                'pattern': re.compile(r'\b(?:ros2?\s+(?:run|launch|topic|service|param|action)\s+[a-zA-Z0-9_.-]+(?:\s+[a-zA-Z0-9_.-]+)*)\b|\b(?:git\s+(?:clone|pull|push|commit|add|status)\s+[^\s]+)\b|\b(?:apt\s+(?:install|update|upgrade|remove)\s+[^\s]+)\b|\b(?:pip(?:3)?\s+(?:install|uninstall|list)\s+[^\s]+)\b|\b(?:npm\s+(?:install|uninstall|run)\s+[^\s]+)\b|\b(?:docker\s+\w+\s+[^\s]+)\b|\b(?:kubectl\s+\w+\s+[^\s]+)\b', re.IGNORECASE),
                'name': 'cli_command'
            },
            # File paths (e.g., /home/user/, ./src/, C:\Users\) - proper path patterns
            {
                'pattern': re.compile(r'(?:/[a-zA-Z0-9._-]+(?:/[a-zA-Z0-9._-]+)*/?)|(?:(?:C|D):\\[a-zA-Z0-9._-]+(?:\\[a-zA-Z0-9._-]+)*\\?)|(?:\.{1,2}/[a-zA-Z0-9._/-]+(?:/[a-zA-Z0-9._/-]+)*/?)', re.IGNORECASE),
                'name': 'file_path'
            },
            # Environment variables (e.g., $HOME, $PATH, ROS_DOMAIN_ID)
            {
                'pattern': re.compile(r'\$[A-Z_][A-Z0-9_]*\b|\b[A-Z_][A-Z0-9_]*=\s*[^\s,;]+|ROS_DOMAIN_ID\b', re.IGNORECASE),
                'name': 'env_var'
            },
            # IP addresses and network addresses (e.g., 127.0.0.1, localhost) - proper IP patterns
            {
                'pattern': re.compile(r'\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\b|localhost\b|\b(?:[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?\.)+[a-zA-Z]{2,}\b(?=\s|:|$)', re.IGNORECASE),
                'name': 'network_addr'
            },
            # Command line options and flags (e.g., --help, -v, --verbose)
            {
                'pattern': re.compile(r'\b--?[a-zA-Z0-9][a-zA-Z0-9_-]*\b', re.IGNORECASE),
                'name': 'cli_flag'
            },
            # Port numbers (e.g., :8080, :5000) - proper port patterns
            {
                'pattern': re.compile(r':\d{2,5}(?=\s|$|/)', re.IGNORECASE),
                'name': 'port_number'
            },
            # Version numbers (e.g., v1.0.0, 2.5.1) - proper version patterns
            {
                'pattern': re.compile(r'\bv?\d+\.\d+\.\d+(?:-[a-zA-Z0-9]+)?\b|\bv?\d+\.\d+\b', re.IGNORECASE),
                'name': 'version_number'
            },
            # Package names (e.g., ros-humble-navigation2) - proper package patterns
            {
                'pattern': re.compile(r'\b(?:ros-|python-|node-|npm-|pip-)?[a-zA-Z0-9][a-zA-Z0-9._-]*-[a-zA-Z0-9][a-zA-Z0-9._-]*\b', re.IGNORECASE),
                'name': 'package_name'
            }
        ]

    def _extract_technical_elements(self, content: str) -> Tuple[str, List[dict]]:
        """
        Extract technical elements from content and replace with placeholders.

        Args:
            content: Original content to process

        Returns:
            Tuple of (processed_content, extracted_elements)
        """
        extracted = []
        processed_content = content
        placeholder_map = {}

        # Process each pattern type
        for pattern_info in self.technical_patterns:
            pattern = pattern_info['pattern']
            matches = list(pattern.finditer(processed_content))

            for i, match in enumerate(reversed(matches)):  # Process in reverse to maintain indices
                original_text = match.group(0)
                start_pos = match.start()
                end_pos = match.end()

                # Create a unique placeholder
                placeholder_id = f"TECH_ELEM_{pattern_info['name']}_{len(extracted)}"
                placeholder = f"[[{placeholder_id}]]"

                # Replace in content
                processed_content = processed_content[:start_pos] + placeholder + processed_content[end_pos:]

                # Store the extracted element
                extracted.append({
                    'id': placeholder_id,
                    'original': original_text,
                    'type': pattern_info['name'],
                    'position': start_pos
                })

                # Keep track of placeholder mapping
                placeholder_map[placeholder_id] = original_text

        return processed_content, extracted

    def _restore_technical_elements(self, content: str, extracted_elements: List[dict]) -> str:
        """
        Restore technical elements back to content using placeholders.

        Args:
            content: Content with placeholders
            extracted_elements: List of extracted elements to restore

        Returns:
            Content with technical elements restored
        """
        restored_content = content

        # Sort elements by position in reverse order to maintain indices during replacement
        sorted_elements = sorted(extracted_elements, key=lambda x: x['position'], reverse=True)

        for element in sorted_elements:
            placeholder = f"[[{element['id']}]]"
            original_text = element['original']
            restored_content = restored_content.replace(placeholder, original_text)

        return restored_content

    async def translate_chapter(
        self,
        chapter_content: str,
        chapter_title: str,
    ) -> tuple[str, dict]:
        """
        Translate chapter content to Urdu.

        Returns:
            Tuple of (translated_content, translation_summary)
        """
        start_time = time.time()

        # Extract technical elements before translation
        processed_content, extracted_elements = self._extract_technical_elements(chapter_content)

        # Build the full prompt with processed content (technical elements extracted)
        prompt = self.system_prompt.format(
            chapter_title=chapter_title or "Untitled",
            chapter_content=processed_content
        )

        # Call Gemini
        try:
            response = await self.model.generate_content_async(prompt)
            translated_content = response.text
        except Exception as e:
            logger.error(f"LLM call failed: {e}")
            raise

        # Restore technical elements back to the translated content
        final_translated_content = self._restore_technical_elements(translated_content, extracted_elements)

        # Validate response
        self._validate_response(chapter_content, final_translated_content)

        processing_time = int((time.time() - start_time) * 1000)

        # Count preserved elements (code blocks and technical elements)
        original_code_blocks = len(re.findall(r'```[\s\S]*?```', chapter_content))
        original_technical_elements = len(extracted_elements)

        translation_summary = {
            "processing_time_ms": processing_time,
            "content_length_original": len(chapter_content),
            "content_length_translated": len(final_translated_content),
            "preserved_elements_count": original_code_blocks + original_technical_elements,
            "preserved_technical_elements_count": original_technical_elements
        }

        return final_translated_content, translation_summary

    def _validate_response(
        self,
        original: str,
        translated: str
    ) -> None:
        """Validate that response preserves structure and technical elements."""
        import re

        # Extract headings from original
        original_headings = re.findall(r'^#{1,4}\s+.+$', original, re.MULTILINE)
        translated_headings = re.findall(r'^#{1,4}\s+.+$', translated, re.MULTILINE)

        # Check heading preservation (allow some flexibility)
        if len(translated_headings) < len(original_headings) * 0.8:
            logger.warning(
                f"Heading count mismatch: {len(original_headings)} -> {len(translated_headings)}"
            )

        # Check code block preservation
        original_code_blocks = len(re.findall(r'```[\s\S]*?```', original))
        translated_code_blocks = len(re.findall(r'```[\s\S]*?```', translated))

        if translated_code_blocks < original_code_blocks:
            logger.warning(
                f"Code block count reduced: {original_code_blocks} -> {translated_code_blocks}"
            )

        # Extract and validate technical elements preservation
        original_extracted, _ = self._extract_technical_elements(original)
        translated_extracted, _ = self._extract_technical_elements(translated)

        # Count the number of technical elements in original vs processed
        original_technical_count = len(re.findall(r'\[\[TECH_ELEM_', original_extracted))
        translated_technical_count = len(re.findall(r'\[\[TECH_ELEM_', translated_extracted))

        # The processed content should have fewer technical elements than the original
        # since they were replaced with placeholders
        if original_technical_count == 0 and translated_technical_count == 0:
            # No technical elements found, which is fine
            pass
        elif original_technical_count > 0 and translated_technical_count == 0:
            # This is expected - technical elements were replaced with placeholders
            pass
        else:
            # If both have technical elements, it might indicate an issue
            logger.warning(
                f"Technical elements still present in processed content: {original_technical_count} -> {translated_technical_count}"
            )

        # Additional validation for technical content preservation
        # Extract code blocks from original and translated
        original_code_content = re.findall(r'```[\s\S]*?```', original)
        translated_code_content = re.findall(r'```[\s\S]*?```', translated)

        # Check if code content is preserved (basic check)
        for orig_code, trans_code in zip(original_code_content, translated_code_content):
            # Remove language specifier from the first line if present
            orig_code_clean = '\n'.join(orig_code.split('\n')[1:]) if orig_code.startswith('```') else orig_code
            trans_code_clean = '\n'.join(trans_code.split('\n')[1:]) if trans_code.startswith('```') else trans_code

            # For basic validation, check if the length is approximately the same
            # A more sophisticated check would compare the actual code content
            if abs(len(orig_code) - len(trans_code)) > len(orig_code) * 0.2:  # Allow 20% difference
                logger.warning(f"Code block may have been modified during translation")