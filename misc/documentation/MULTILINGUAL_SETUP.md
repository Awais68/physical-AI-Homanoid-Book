# 🌍 Multilingual Translation Setup - Roman Urdu & Urdu

## ✅ Kya Implemented Hua Hai

### Frontend (Docusaurus)

- ✅ **12 Languages Support**:

  - English (en) - Default
  - **اردو - Urdu (ur)** - RTL Support
  - **Roman Urdu (ur-PK)** - LTR Support
  - Arabic (ar) - RTL Support
  - Spanish (es), French (fr), German (de)
  - Chinese (zh), Hindi (hi), Portuguese (pt)
  - Russian (ru), Japanese (ja)

- ✅ **Language Switcher** - Navbar mein dropdown
- ✅ **Translation Files** - Sab languages ke liye
- ✅ **RTL Support** - Urdu aur Arabic ke liye
- ✅ **Content Translation** - Book ka content translate karne ka system

### Backend (FastAPI)

- ✅ **Translation API** - 12 languages support
- ✅ **Auto Language Detection** - Request se language detect karna
- ✅ **Translation Middleware** - Automatic language handling

## 🚀 Kaise Use Karein

### 1. Development Server Start Karein

```bash
cd frontend

# Default language (English) ke saath
npm run start

# Specific language ke saath
npm run start -- --locale ur        # Urdu
npm run start -- --locale ur-PK     # Roman Urdu
npm run start -- --locale ar        # Arabic
```

### 2. Language Switch Karna (Browser mein)

1. Website kholen: http://localhost:3000/physical-AI-Homanoid-Book/
2. Navbar mein **language dropdown** (globe icon) par click karen
3. Apni pasand ki language select karen:
   - اردو (Urdu)
   - Roman Urdu
   - Any other language

### 3. Translation Files Edit Karna

#### Roman Urdu Translations:

```
frontend/i18n/ur-PK/
├── code.json                          # UI elements
├── docusaurus-theme-classic/
│   ├── navbar.json                    # Navbar items
│   └── footer.json                    # Footer items
└── docusaurus-plugin-content-docs/
    └── current/
        ├── index.md                   # Index page (Roman Urdu)
        └── 01-scope-boundaries.md     # Sample translated page
```

#### Urdu Translations:

```
frontend/i18n/ur/
├── code.json                          # UI elements
├── docusaurus-theme-classic/
│   ├── navbar.json                    # Navbar items (اردو)
│   └── footer.json                    # Footer items (اردو)
└── docusaurus-plugin-content-docs/
    └── current/
        └── index.md                   # Index page (اردو)
```

### 4. Naye Pages Translate Karna

Koi bhi page translate karne ke liye:

```bash
# Original page ye hai:
frontend/docs/01-scope-boundaries.md

# Roman Urdu translation yahan banayein:
frontend/i18n/ur-PK/docusaurus-plugin-content-docs/current/01-scope-boundaries.md

# Urdu translation yahan banayein:
frontend/i18n/ur/docusaurus-plugin-content-docs/current/01-scope-boundaries.md
```

**Important**: File ka naam same rakhen, sirf folder different hoga!

## 📝 Translation Examples

### Roman Urdu Example:

```markdown
---
title: Physical AI aur Robotics ki Daira Kar
---

# Shuruat

Ye kitab Physical AI aur Humanoid Robotics ke bare mein hai...
```

### Urdu Example:

```markdown
---
title: فزیکل اے آئی اور روبوٹکس کی دائرہ کار
---

# شروعات

یہ کتاب فزیکل اے آئی اور ہیومینوئڈ روبوٹکس کے بارے میں ہے...
```

## 🔧 Build Commands

### Development

```bash
# Sab languages ke saath
npm run start

# Specific language
npm run start -- --locale ur-PK
```

### Production Build

```bash
# Sab configured languages ke liye build
npm run build

# Specific language ke liye
npm run build -- --locale ur-PK
```

### Serve Production Build

```bash
npm run serve -- --locale ur-PK
```

## 🌐 Backend Translation API

### Endpoints:

1. **Sab Languages Dekhen**

   ```
   GET http://localhost:8000/api/translations/languages
   ```

2. **Specific Language Ki Translations**

   ```
   GET http://localhost:8000/api/translations/ur-PK
   GET http://localhost:8000/api/translations/ur
   ```

3. **Single Translation Get Karein**

   ```
   GET http://localhost:8000/api/translations/ur-PK/welcome
   ```

4. **Current Language Detect Karein**
   ```
   GET http://localhost:8000/api/translations/detect
   ```

## 📂 File Structure

```
frontend/
├── docs/                              # Original English content
│   ├── index.md
│   ├── 01-scope-boundaries.md
│   └── ...
├── i18n/                              # Translations folder
│   ├── ur/                            # Urdu translations (اردو)
│   │   └── docusaurus-plugin-content-docs/current/
│   │       ├── index.md
│   │       └── ...
│   ├── ur-PK/                         # Roman Urdu translations
│   │   └── docusaurus-plugin-content-docs/current/
│   │       ├── index.md
│   │       ├── 01-scope-boundaries.md
│   │       └── ...
│   ├── ar/                            # Arabic
│   ├── es/                            # Spanish
│   └── ...
└── docusaurus.config.ts               # i18n configuration

backend/
├── src/
│   ├── config/
│   │   └── translations.py            # Translation dictionary
│   └── api/
│       ├── translation_middleware.py  # Auto language detection
│       └── routers/
│           └── translations.py        # Translation API endpoints
```

## ✨ Features

### ✅ Already Working:

- Language switcher dropdown in navbar
- RTL support for Urdu and Arabic
- Translation files for all 12 languages
- Sample translated pages (Roman Urdu & Urdu)
- Backend translation API
- Auto language detection

### 📋 Todo (Optional):

- Baqi pages ko translate karna
- Blog posts translate karna
- Images mein text translate karna (agar zaroorat ho)

## 🎯 Quick Test Karein

1. **Start Development Server:**

   ```bash
   cd frontend
   npm run start
   ```

2. **Browser mein kholen:**
   http://localhost:3000/physical-AI-Homanoid-Book/

3. **Language Switch:**

   - Navbar mein globe icon par click karen
   - "Roman Urdu" ya "اردو (Urdu)" select karen
   - Page refresh hoga aur translation dikhai dega

4. **Translated Content Check:**
   - Navbar items Roman Urdu mein honge
   - Footer Roman Urdu mein hoga
   - Index page translated hoga

## 💡 Tips

1. **Translation Files Generate:**

   ```bash
   npm run write-translations -- --locale ur-PK
   ```

2. **Cache Clear (Agar issues aayein):**

   ```bash
   npm run clear
   npm run start
   ```

3. **Check Errors:**
   Build errors dekhen browser console mein

## 🆘 Common Issues & Solutions

### Issue 1: Language Switch Nahi Ho Rahi

**Solution:**

- Browser cache clear karen
- Development server restart karen
- Check karen `docusaurus.config.ts` mein locales properly configured hain

### Issue 2: Content Translate Nahi Hora

**Solution:**

- Check karen translation file sahi folder mein hai
- File ka naam original file se match hona chahiye
- Frontmatter (--- ---) properly add kiya ho

### Issue 3: RTL Layout Issues

**Solution:**

- `docusaurus.config.ts` mein `direction: 'rtl'` check karen
- Browser DevTools mein `dir="rtl"` attribute check karen

## 🎉 Alhumdulillah!

Aapka project ab **12 languages** support karta hai including:

- ✅ Roman Urdu (ur-PK)
- ✅ اردو - Urdu (ur)
- ✅ All other major languages

Happy translating! 🌍
