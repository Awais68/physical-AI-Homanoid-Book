#!/usr/bin/env python3
"""
Simple test server to verify language switching functionality
"""

import uvicorn
from backend.src.main import app

if __name__ == "__main__":
    print("Starting server to test language switching functionality...")
    print("Server will run on http://localhost:8000")
    print("\nKey endpoints to test:")
    print("  GET  /api/i18n/languages          - List supported languages")
    print("  GET  /api/translations/languages  - List translation languages")
    print("  GET  /api/translations/ur         - Urdu translations")
    print("  GET  /api/translations/ur-PK      - Roman Urdu translations")
    print("  GET  /api/personalization/preferences - User preferences")
    print("  PUT  /api/personalization/preferences - Update preferences")
    print("\nTest the language switching by updating preferences with:")
    print('  curl -X PUT http://localhost:8000/api/personalization/preferences -H "Content-Type: application/json" -d \'{"language": "ur"}\'')
    print("")

    uvicorn.run(app, host="0.0.0.0", port=8000, reload=True)