# Translation Feature Setup Guide

## Overview

Multilingual support has been added to your Physical AI & Robotics project with support for 12 languages including Urdu and Roman Urdu.

## Supported Languages

- English (en)
- اردو - Urdu (ur)
- Roman Urdu (ur-PK)
- العربية - Arabic (ar)
- Español - Spanish (es)
- Français - French (fr)
- Deutsch - German (de)
- 中文 - Chinese (zh)
- हिन्दी - Hindi (hi)
- Português - Portuguese (pt)
- Русский - Russian (ru)
- 日本語 - Japanese (ja)

## Frontend Setup (Docusaurus)

### 1. Build for specific language

```bash
cd frontend
npm run build -- --locale ur      # Build Urdu version
npm run build -- --locale ur-PK   # Build Roman Urdu version
```

### 2. Start development server with language

```bash
npm run start -- --locale ur      # Start with Urdu
npm run start -- --locale ur-PK   # Start with Roman Urdu
```

### 3. Generate translations

```bash
npm run write-translations -- --locale ur
```

## Backend Translation API

### Endpoints Available

1. **Get all supported languages**

   ```
   GET /api/translations/languages
   ```

2. **Get translations for a language**

   ```
   GET /api/translations/ur
   GET /api/translations/ur-PK
   ```

3. **Get specific translation**

   ```
   GET /api/translations/ur/welcome
   ```

4. **Detect language from request**
   ```
   GET /api/translations/detect
   ```

## Usage Examples

### Frontend - Language Switcher

The language dropdown is automatically added to the navbar. Users can switch languages using the dropdown menu.

### Backend - API Translation

```python
from src.config.translations import get_translation

# Get translation
message = get_translation('ur', 'welcome')
# Returns: 'فزیکل اے آئی اور روبوٹکس میں خوش آمدید'

message = get_translation('ur-PK', 'welcome')
# Returns: 'Physical AI aur Robotics mein Khush Aamdeed'
```

### Adding Custom Translations

Edit these files:

- **Urdu**: `frontend/i18n/ur/docusaurus-theme-classic/navbar.json`
- **Roman Urdu**: `frontend/i18n/ur-PK/docusaurus-theme-classic/navbar.json`
- **Backend**: `backend/src/config/translations.py`

## Testing

### Test Frontend Translations

```bash
cd frontend
npm run start -- --locale ur
```

Visit http://localhost:3000 and verify Urdu translations appear.

### Test Backend API

```bash
curl http://localhost:8000/api/translations/languages
curl http://localhost:8000/api/translations/ur
curl http://localhost:8000/api/translations/ur-PK/welcome
```

## Deployment

### Build all languages for production

```bash
cd frontend
npm run build
```

This creates optimized builds for all configured languages.

## Language Detection Priority

1. Query parameter: `?lang=ur`
2. Accept-Language header
3. Cookie: `NEXT_LOCALE`
4. Default: English (en)

## Notes

- RTL (Right-to-Left) support is enabled for Urdu and Arabic
- Roman Urdu uses LTR (Left-to-Right) layout
- All UI components automatically adapt to language direction
- Translation files are JSON-based for easy editing

## Alhumdulillah! 🎉

Your project now supports global translations with special focus on Urdu and Roman Urdu languages.
