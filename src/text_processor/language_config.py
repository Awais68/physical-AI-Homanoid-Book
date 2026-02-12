"""Language configuration for supported languages."""

from src.text_processor.models import Language

# Supported languages with codes, names, and native names
SUPPORTED_LANGUAGES: list[Language] = [
    Language(code="en", name="English", native_name="English"),
    Language(code="es", name="Spanish", native_name="Espanol"),
    Language(code="fr", name="French", native_name="Francais"),
    Language(code="de", name="German", native_name="Deutsch"),
    Language(code="zh", name="Chinese", native_name="Chinese"),
    Language(code="ja", name="Japanese", native_name="Japanese"),
    Language(code="ar", name="Arabic", native_name="Arabic"),
    Language(code="hi", name="Hindi", native_name="Hindi"),
    Language(code="pt", name="Portuguese", native_name="Portugues"),
    Language(code="ru", name="Russian", native_name="Russian"),
    Language(code="ko", name="Korean", native_name="Korean"),
    Language(code="it", name="Italian", native_name="Italiano"),
    Language(code="ur", name="Urdu", native_name="اردو"),
    Language(code="ur-PK", name="Roman Urdu", native_name="Roman Urdu"),
]

# Create lookup dictionaries for fast access
LANGUAGE_CODE_MAP: dict[str, Language] = {lang.code.lower(): lang for lang in SUPPORTED_LANGUAGES}

# Map full names to codes (case-insensitive)
LANGUAGE_NAME_MAP: dict[str, str] = {
    lang.name.lower(): lang.code for lang in SUPPORTED_LANGUAGES
}

# All supported language codes
SUPPORTED_LANGUAGE_CODES: list[str] = [lang.code for lang in SUPPORTED_LANGUAGES]


def is_supported_language(language: str) -> bool:
    """Check if a language code or name is supported.

    Args:
        language: Language code (e.g., 'en') or full name (e.g., 'English')

    Returns:
        True if the language is supported, False otherwise
    """
    language_lower = language.lower()
    return language_lower in LANGUAGE_CODE_MAP or language_lower in LANGUAGE_NAME_MAP


def normalize_language_code(language: str) -> str | None:
    """Normalize a language input to its ISO 639-1 code.

    Args:
        language: Language code or full name (case-insensitive)

    Returns:
        Normalized language code, or None if not supported
    """
    language_lower = language.lower()

    # Check if it's already a valid code
    if language_lower in LANGUAGE_CODE_MAP:
        return language_lower

    # Check if it's a full name
    if language_lower in LANGUAGE_NAME_MAP:
        return LANGUAGE_NAME_MAP[language_lower]

    return None


def get_language(code: str) -> Language | None:
    """Get language details by code.

    Args:
        code: ISO 639-1 language code

    Returns:
        Language object or None if not found
    """
    return LANGUAGE_CODE_MAP.get(code.lower())


def get_all_languages() -> list[Language]:
    """Get all supported languages.

    Returns:
        List of all supported Language objects
    """
    return SUPPORTED_LANGUAGES.copy()
