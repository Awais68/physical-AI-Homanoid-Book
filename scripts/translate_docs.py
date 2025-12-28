#!/usr/bin/env python3
"""Script to translate all documentation files to supported locales using the API."""

import asyncio
import os
import re
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.text_processor.translator import TextTranslator

# Mapping of Docusaurus locale codes to API language codes
LOCALE_TO_LANG = {
    "ur": "ur",
    "ur-PK": "ur-PK",
    "ar": "ar",
    "es": "es",
    "fr": "fr",
    "de": "de",
    "zh": "zh",
    "hi": "hi",
    "pt": "pt",
    "ru": "ru",
    "ja": "ja",
}

# Base paths
PROJECT_ROOT = Path(__file__).parent.parent
FRONTEND_DIR = PROJECT_ROOT / "frontend"
DOCS_DIR = FRONTEND_DIR / "docs"
I18N_DIR = FRONTEND_DIR / "i18n"


def extract_frontmatter(content: str) -> tuple[str, str]:
    """Extract YAML frontmatter from markdown content.

    Returns:
        Tuple of (frontmatter, body)
    """
    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            return f"---{parts[1]}---", parts[2]
    return "", content


def split_into_chunks(text: str, max_chars: int = 4000) -> list[str]:
    """Split text into chunks at paragraph boundaries."""
    paragraphs = text.split("\n\n")
    chunks = []
    current_chunk = ""

    for para in paragraphs:
        if len(current_chunk) + len(para) + 2 > max_chars:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = para
        else:
            current_chunk = current_chunk + "\n\n" + para if current_chunk else para

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks if chunks else [text]


async def translate_markdown(content: str, target_language: str, translator: TextTranslator) -> str:
    """Translate markdown content while preserving structure."""
    frontmatter, body = extract_frontmatter(content)

    # Split body into chunks
    chunks = split_into_chunks(body)
    translated_chunks = []

    for i, chunk in enumerate(chunks):
        if not chunk.strip():
            translated_chunks.append(chunk)
            continue

        try:
            # Translate the chunk
            translated = await translator.translate(chunk, target_language)
            translated_chunks.append(translated)
            print(f"    Translated chunk {i+1}/{len(chunks)}")
        except Exception as e:
            print(f"    Error translating chunk {i+1}: {e}")
            translated_chunks.append(chunk)  # Keep original on error

    translated_body = "\n\n".join(translated_chunks)

    # Combine frontmatter with translated body
    if frontmatter:
        return f"{frontmatter}\n{translated_body}"
    return translated_body


async def translate_file(source_file: Path, target_file: Path, target_language: str, translator: TextTranslator) -> bool:
    """Translate a single markdown file."""
    try:
        content = source_file.read_text(encoding="utf-8")
        translated_content = await translate_markdown(content, target_language, translator)

        # Ensure target directory exists
        target_file.parent.mkdir(parents=True, exist_ok=True)
        target_file.write_text(translated_content, encoding="utf-8")
        return True
    except Exception as e:
        print(f"  Error: {e}")
        return False


async def translate_locale(locale: str, translator: TextTranslator) -> dict:
    """Translate all docs for a specific locale."""
    lang_code = LOCALE_TO_LANG.get(locale)
    if not lang_code:
        print(f"Skipping {locale}: No language mapping found")
        return {"locale": locale, "success": 0, "failed": 0, "skipped": True}

    target_dir = I18N_DIR / locale / "docusaurus-plugin-content-docs" / "current"

    success_count = 0
    failed_count = 0

    # Get all markdown files from source docs
    doc_files = list(DOCS_DIR.glob("*.md")) + list(DOCS_DIR.glob("*.mdx"))

    print(f"\nTranslating {len(doc_files)} files to {locale} ({lang_code})...")

    for doc_file in doc_files:
        target_file = target_dir / doc_file.name
        print(f"  {doc_file.name} -> {locale}/")

        if await translate_file(doc_file, target_file, lang_code, translator):
            success_count += 1
        else:
            failed_count += 1

    return {"locale": locale, "success": success_count, "failed": failed_count, "skipped": False}


async def main():
    """Main entry point."""
    print("=" * 60)
    print("Documentation Translation Script")
    print("=" * 60)

    # Check if docs directory exists
    if not DOCS_DIR.exists():
        print(f"Error: Docs directory not found: {DOCS_DIR}")
        sys.exit(1)

    # Initialize translator
    translator = TextTranslator()

    # Get locales to translate (skip 'en' as it's the source)
    locales = list(LOCALE_TO_LANG.keys())

    print(f"\nLocales to translate: {', '.join(locales)}")
    print(f"Source docs: {DOCS_DIR}")

    results = []

    for locale in locales:
        result = await translate_locale(locale, translator)
        results.append(result)

    # Print summary
    print("\n" + "=" * 60)
    print("Translation Summary")
    print("=" * 60)

    total_success = 0
    total_failed = 0

    for result in results:
        if result["skipped"]:
            print(f"  {result['locale']}: SKIPPED")
        else:
            print(f"  {result['locale']}: {result['success']} success, {result['failed']} failed")
            total_success += result["success"]
            total_failed += result["failed"]

    print(f"\nTotal: {total_success} files translated, {total_failed} failed")


if __name__ == "__main__":
    asyncio.run(main())
