#!/usr/bin/env python3
"""
Test script to verify language switching functionality
"""

import requests
import json

BASE_URL = "http://localhost:8000"


def test_language_endpoints():
    """Test all language-related endpoints"""
    print("Testing Language Switching Functionality\n")

    # Test 1: Get available languages from i18n service
    print("1. Testing /api/i18n/languages endpoint:")
    try:
        response = requests.get(f"{BASE_URL}/api/i18n/languages")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Total languages: {data.get('total_count', 0)}")
            languages = data.get('languages', [])
            urdu_found = any(lang.get('code') == 'ur' for lang in languages)
            roman_urdu_found = any(lang.get('code') == 'ur-PK' for lang in languages)
            print(f"   ✓ Urdu ('ur') support: {'✓' if urdu_found else '✗'}")
            print(f"   ✓ Roman Urdu ('ur-PK') support: {'✓' if roman_urdu_found else '✗'}")

            # Print all language codes
            codes = [lang.get('code') for lang in languages]
            print(f"   ✓ Available codes: {codes}")
        else:
            print(f"   ✗ Status: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 2: Get available languages from translations service
    print("2. Testing /api/translations/languages endpoint:")
    try:
        response = requests.get(f"{BASE_URL}/api/translations/languages")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Total languages: {data.get('total_count', 0)}")
            languages = data.get('languages', [])
            urdu_found = any(lang.get('code') == 'ur' for lang in languages)
            roman_urdu_found = any(lang.get('code') == 'ur-PK' for lang in languages)
            print(f"   ✓ Urdu ('ur') support: {'✓' if urdu_found else '✗'}")
            print(f"   ✓ Roman Urdu ('ur-PK') support: {'✓' if roman_urdu_found else '✗'}")
        else:
            print(f"   ✗ Status: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 3: Get Urdu translations
    print("3. Testing Urdu translations (/api/translations/ur):")
    try:
        response = requests.get(f"{BASE_URL}/api/translations/ur")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Language: {data.get('language')}")
            print(f"   ✓ Is RTL: {data.get('is_rtl')}")
            print(f"   ✓ Translation count: {data.get('translation_count', 0)}")
            # Check if we have some translations
            if data.get('translation_count', 0) > 10:  # Should have many translations
                print("   ✓ Comprehensive translation set available")
            else:
                print("   ⚠ Limited translation set")
        else:
            print(f"   ✗ Status: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 4: Get Roman Urdu translations
    print("4. Testing Roman Urdu translations (/api/translations/ur-PK):")
    try:
        response = requests.get(f"{BASE_URL}/api/translations/ur-PK")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Language: {data.get('language')}")
            print(f"   ✓ Is RTL: {data.get('is_rtl')} (should be False for Roman Urdu)")
            print(f"   ✓ Translation count: {data.get('translation_count', 0)}")
            if data.get('translation_count', 0) > 10:
                print("   ✓ Comprehensive translation set available")
            else:
                print("   ⚠ Limited translation set")
        else:
            print(f"   ✗ Status: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 5: Get specific translation
    print("5. Testing specific translation (/api/translations/ur/welcome):")
    try:
        response = requests.get(f"{BASE_URL}/api/translations/ur/welcome")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Language: {data.get('language')}")
            print(f"   ✓ Key: {data.get('key')}")
            print(f"   ✓ Translation: {data.get('translation')[:50]}...")
            print(f"   ✓ Is RTL: {data.get('is_rtl')}")
        else:
            print(f"   ✗ Status: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 6: Get specific Roman Urdu translation
    print("6. Testing specific Roman Urdu translation (/api/translations/ur-PK/welcome):")
    try:
        response = requests.get(f"{BASE_URL}/api/translations/ur-PK/welcome")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Language: {data.get('language')}")
            print(f"   ✓ Key: {data.get('key')}")
            print(f"   ✓ Translation: {data.get('translation')[:50]}...")
            print(f"   ✓ Is RTL: {data.get('is_rtl')} (should be False)")
        else:
            print(f"   ✗ Status: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 7: Test personalization preferences
    print("7. Testing personalization preferences (/api/personalization/preferences):")
    try:
        response = requests.get(f"{BASE_URL}/api/personalization/preferences")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Current language: {data.get('language')}")
            lang_info = data.get('language_info', {})
            print(f"   ✓ Language RTL: {lang_info.get('is_rtl')}")
            print(f"   ✓ Language supported: {lang_info.get('supported')}")
        else:
            print(f"   ✗ Status: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 8: Update language preference to Urdu
    print("8. Testing language preference update to Urdu:")
    try:
        payload = {"language": "ur"}
        response = requests.put(f"{BASE_URL}/api/personalization/preferences", json=payload)
        if response.status_code == 200:
            data = response.json()
            updated_prefs = data.get('updated_preferences', {})
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Updated language: {updated_prefs.get('language')}")
            lang_info = updated_prefs.get('language_info', {})
            print(f"   ✓ Updated RTL: {lang_info.get('is_rtl')}")
        else:
            print(f"   ✗ Status: {response.status_code}")
            print(f"   ✗ Response: {response.text}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 9: Update language preference to Roman Urdu
    print("9. Testing language preference update to Roman Urdu:")
    try:
        payload = {"language": "ur-PK"}
        response = requests.put(f"{BASE_URL}/api/personalization/preferences", json=payload)
        if response.status_code == 200:
            data = response.json()
            updated_prefs = data.get('updated_preferences', {})
            print(f"   ✓ Status: {response.status_code}")
            print(f"   ✓ Updated language: {updated_prefs.get('language')}")
            lang_info = updated_prefs.get('language_info', {})
            print(f"   ✓ Updated RTL: {lang_info.get('is_rtl')} (should be False)")
        else:
            print(f"   ✗ Status: {response.status_code}")
            print(f"   ✗ Response: {response.text}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    print()

    # Test 10: Health checks
    print("10. Testing health endpoints:")
    endpoints = [
        "/api/i18n/health",
        "/api/translations/health",
        "/api/personalization/health"
    ]

    for endpoint in endpoints:
        try:
            response = requests.get(f"{BASE_URL}{endpoint}")
            if response.status_code == 200:
                data = response.json()
                print(f"    ✓ {endpoint}: {data.get('status', 'unknown')}")
            else:
                print(f"    ✗ {endpoint}: {response.status_code}")
        except Exception as e:
            print(f"    ✗ {endpoint}: {e}")
    print()


def test_language_switching_flow():
    """Test the complete language switching flow"""
    print("Testing Complete Language Switching Flow\n")

    # Reset to English first
    print("1. Resetting to English...")
    try:
        response = requests.put(f"{BASE_URL}/api/personalization/preferences", json={"language": "en"})
        if response.status_code == 200:
            print("   ✓ Reset to English successful")
        else:
            print(f"   ✗ Reset failed: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Reset error: {e}")

    # Switch to Urdu
    print("2. Switching to Urdu...")
    try:
        response = requests.put(f"{BASE_URL}/api/personalization/preferences", json={"language": "ur"})
        if response.status_code == 200:
            print("   ✓ Switch to Urdu successful")
        else:
            print(f"   ✗ Switch failed: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Switch error: {e}")

    # Verify Urdu is set
    print("3. Verifying Urdu is active...")
    try:
        response = requests.get(f"{BASE_URL}/api/personalization/preferences")
        if response.status_code == 200:
            data = response.json()
            current_lang = data.get('language')
            is_rtl = data.get('language_info', {}).get('is_rtl')
            print(f"   ✓ Current language: {current_lang}")
            print(f"   ✓ Is RTL: {is_rtl}")
            if current_lang == "ur" and is_rtl:
                print("   ✓ Urdu language and RTL properly set")
            else:
                print("   ✗ Urdu not properly set")
        else:
            print(f"   ✗ Verification failed: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Verification error: {e}")

    # Switch to Roman Urdu
    print("4. Switching to Roman Urdu...")
    try:
        response = requests.put(f"{BASE_URL}/api/personalization/preferences", json={"language": "ur-PK"})
        if response.status_code == 200:
            print("   ✓ Switch to Roman Urdu successful")
        else:
            print(f"   ✗ Switch failed: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Switch error: {e}")

    # Verify Roman Urdu is set
    print("5. Verifying Roman Urdu is active...")
    try:
        response = requests.get(f"{BASE_URL}/api/personalization/preferences")
        if response.status_code == 200:
            data = response.json()
            current_lang = data.get('language')
            is_rtl = data.get('language_info', {}).get('is_rtl')
            print(f"   ✓ Current language: {current_lang}")
            print(f"   ✓ Is RTL: {is_rtl} (should be False for Roman Urdu)")
            if current_lang == "ur-PK" and not is_rtl:
                print("   ✓ Roman Urdu language properly set (L2R as expected)")
            else:
                print("   ✗ Roman Urdu not properly set")
        else:
            print(f"   ✗ Verification failed: {response.status_code}")
    except Exception as e:
        print(f"   ✗ Verification error: {e}")


if __name__ == "__main__":
    print("Language Switching Functionality Test")
    print("="*50)

    # Note: This test assumes the server is running at localhost:8000
    # Start the server first with: python -m uvicorn backend.src.main:app --reload
    print("\n⚠️  WARNING: This test requires the server to be running!")
    print("   Please start the server first with:")
    print("   cd backend")
    print("   python -m uvicorn src.main:app --reload")
    print("")

    test_language_endpoints()
    test_language_switching_flow()

    print("\nTest completed! Check if all language switching functionality is working properly.")