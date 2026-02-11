#!/usr/bin/env python3
"""
Test script for multilingual support with Urdu and Roman Urdu
"""

import asyncio
import json
from typing import Dict, Any

from backend.src.config.translations import get_translation, get_supported_languages, is_rtl_language, TRANSLATIONS


def test_basic_translations():
    """Test basic translation functionality."""
    print("=== Testing Basic Translations ===")

    # Test common phrases in English
    en_phrase = get_translation('en', 'welcome')
    print(f"English 'welcome': {en_phrase}")

    # Test Urdu translation
    ur_phrase = get_translation('ur', 'welcome')
    print(f"Urdu 'welcome': {ur_phrase}")

    # Test Roman Urdu translation
    ur_pk_phrase = get_translation('ur-PK', 'welcome')
    print(f"Roman Urdu 'welcome': {ur_pk_phrase}")

    # Test error handling for unsupported language
    unsupported = get_translation('xx', 'welcome', 'Default text')
    print(f"Unsupported language fallback: {unsupported}")


def test_comprehensive_translations():
    """Test comprehensive translation functionality."""
    print("\n=== Testing Comprehensive Translations ===")

    # Test multiple keys for Urdu
    test_keys = ['chat', 'help', 'settings', 'ai_assistant', 'robotics', 'education']

    print("Urdu translations:")
    for key in test_keys:
        translation = get_translation('ur', key)
        print(f"  {key}: {translation}")

    print("\nRoman Urdu translations:")
    for key in test_keys:
        translation = get_translation('ur-PK', key)
        print(f"  {key}: {translation}")


def test_rtl_detection():
    """Test RTL (Right-to-Left) language detection."""
    print("\n=== Testing RTL Detection ===")

    languages_to_test = ['en', 'ur', 'ur-PK', 'ar', 'es', 'fr']

    for lang in languages_to_test:
        is_rtl = is_rtl_language(lang)
        print(f"{lang}: {'RTL' if is_rtl else 'LTR'}")


def test_supported_languages():
    """Test supported languages listing."""
    print("\n=== Testing Supported Languages ===")

    supported = get_supported_languages()
    print(f"Total supported languages: {len(supported)}")
    for code, name in supported.items():
        print(f"  {code}: {name}")


def test_fallback_behavior():
    """Test translation fallback behavior."""
    print("\n=== Testing Fallback Behavior ===")

    # Test when key doesn't exist in target language
    result = get_translation('ur', 'nonexistent_key', 'Default Value')
    print(f"Non-existent key fallback: {result}")

    # Test when language doesn't exist
    result = get_translation('xx', 'welcome', 'Fallback Value')
    print(f"Non-existent language fallback: {result}")


async def test_text_processing():
    """Test the text processing functionality."""
    print("\n=== Testing Text Processing ===")

    # Import here to avoid issues if dependencies aren't installed
    try:
        from src.text_processor.translator import TextTranslator
        from src.text_processor.clarifier import TextClarifier

        # Test with a sample text in English
        sample_text = "This is a test for multilingual support."
        print(f"Sample text: {sample_text}")

        # Test translation to Urdu
        translator = TextTranslator()
        try:
            urdu_text = await translator.translate(sample_text, 'ur')
            print(f"Translated to Urdu: {urdu_text}")
        except Exception as e:
            print(f"Urdu translation failed: {e}")

        # Test translation to Roman Urdu
        try:
            roman_urdu_text = await translator.translate(sample_text, 'ur-PK')
            print(f"Translated to Roman Urdu: {roman_urdu_text}")
        except Exception as e:
            print(f"Roman Urdu translation failed: {e}")

        # Test clarification
        clarifier = TextClarifier()
        try:
            clarified_text = await clarifier.clarify(sample_text)
            print(f"Clarified text: {clarified_text}")
        except Exception as e:
            print(f"Clarification failed: {e}")

    except ImportError as e:
        print(f"Text processing modules not available: {e}")


async def main():
    """Main test function."""
    print("Testing Multilingual Support for Urdu and Roman Urdu\n")

    test_basic_translations()
    test_comprehensive_translations()
    test_rtl_detection()
    test_supported_languages()
    test_fallback_behavior()
    await test_text_processing()

    print("\n=== All Tests Completed ===")


if __name__ == "__main__":
    asyncio.run(main())