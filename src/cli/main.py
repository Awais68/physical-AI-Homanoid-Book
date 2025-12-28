"""Command-line interface for text processing."""

import argparse
import asyncio
import json
import sys
from typing import NoReturn


def create_parser() -> argparse.ArgumentParser:
    """Create the argument parser.

    Returns:
        Configured ArgumentParser
    """
    parser = argparse.ArgumentParser(
        prog="text-clarify",
        description="AI-powered text clarification and translation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Clarify text (English output)
  text-clarify "I want to make sure you understand this is important."

  # Translate to Spanish
  text-clarify "Hello world" --language es

  # Read from stdin
  echo "Some verbose text" | text-clarify --stdin

  # Output as JSON
  text-clarify "Test text" --json
        """,
    )

    parser.add_argument(
        "text",
        nargs="?",
        help="Text to process (or use --stdin to read from stdin)",
    )

    parser.add_argument(
        "-l",
        "--language",
        default="en",
        help="Target language code (default: en)",
    )

    parser.add_argument(
        "--stdin",
        action="store_true",
        help="Read text from stdin",
    )

    parser.add_argument(
        "--json",
        action="store_true",
        dest="json_output",
        help="Output as JSON",
    )

    parser.add_argument(
        "--languages",
        action="store_true",
        help="List supported languages and exit",
    )

    parser.add_argument(
        "-v",
        "--version",
        action="version",
        version="%(prog)s 1.0.0",
    )

    return parser


def list_languages() -> None:
    """Print list of supported languages."""
    from src.text_processor.language_config import get_all_languages

    languages = get_all_languages()
    print("Supported languages:")
    print("-" * 40)
    for lang in languages:
        print(f"  {lang.code:5} {lang.name:15} ({lang.native_name})")


async def process_text_async(text: str, target_language: str) -> dict[str, str]:
    """Process text asynchronously.

    Args:
        text: Text to process
        target_language: Target language code

    Returns:
        Dict with original_text and translated_text
    """
    from src.text_processor import process_text

    result = await process_text(text, target_language)
    return result.model_dump()


def main() -> NoReturn:
    """Main entry point for CLI."""
    parser = create_parser()
    args = parser.parse_args()

    # Handle --languages flag
    if args.languages:
        list_languages()
        sys.exit(0)

    # Get text input
    text: str | None = None

    if args.stdin:
        text = sys.stdin.read().strip()
    elif args.text:
        text = args.text
    else:
        parser.print_help()
        sys.exit(1)

    if not text:
        print("Error: No text provided", file=sys.stderr)
        sys.exit(1)

    try:
        # Process text
        result = asyncio.run(process_text_async(text, args.language))

        # Output result
        if args.json_output:
            print(json.dumps(result, ensure_ascii=False, indent=2))
        else:
            print(result["translated_text"])

        sys.exit(0)

    except ValueError as e:
        if args.json_output:
            error = {"error_type": "VALIDATION_ERROR", "message": str(e)}
            print(json.dumps(error), file=sys.stderr)
        else:
            print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except Exception as e:
        if args.json_output:
            error = {"error_type": "PROCESSING_ERROR", "message": str(e)}
            print(json.dumps(error), file=sys.stderr)
        else:
            print(f"Processing error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
