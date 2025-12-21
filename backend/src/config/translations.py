"""
Translation configuration for multilingual support
Supports Urdu, Roman Urdu, and multiple other languages
"""

SUPPORTED_LANGUAGES = {
    'en': 'English',
    'ur': 'اردو (Urdu)',
    'ur-PK': 'Roman Urdu',
    'ar': 'العربية (Arabic)',
    'es': 'Español (Spanish)',
    'fr': 'Français (French)',
    'de': 'Deutsch (German)',
    'zh': '中文 (Chinese)',
    'hi': 'हिन्दी (Hindi)',
    'pt': 'Português (Portuguese)',
    'ru': 'Русский (Russian)',
    'ja': '日本語 (Japanese)',
}

# Common translations for API responses
TRANSLATIONS = {
    'en': {
        'welcome': 'Welcome to Physical AI & Robotics',
        'error': 'An error occurred',
        'success': 'Operation successful',
        'loading': 'Loading...',
        'no_data': 'No data available',
        'search': 'Search',
        'filter': 'Filter',
        'export': 'Export',
        'import': 'Import',
        'delete': 'Delete',
        'save': 'Save',
        'cancel': 'Cancel',
        'confirm': 'Confirm',
    },
    'ur': {
        'welcome': 'فزیکل اے آئی اور روبوٹکس میں خوش آمدید',
        'error': 'ایک خرابی واقع ہوئی',
        'success': 'کامیابی سے مکمل ہوا',
        'loading': 'لوڈ ہو رہا ہے...',
        'no_data': 'کوئی ڈیٹا دستیاب نہیں',
        'search': 'تلاش کریں',
        'filter': 'فلٹر',
        'export': 'برآمد',
        'import': 'درآمد',
        'delete': 'حذف کریں',
        'save': 'محفوظ کریں',
        'cancel': 'منسوخ کریں',
        'confirm': 'تصدیق کریں',
    },
    'ur-PK': {
        'welcome': 'Physical AI aur Robotics mein Khush Aamdeed',
        'error': 'Ek kharabi waqe hui',
        'success': 'Kamyabi se mukammal hua',
        'loading': 'Load ho raha hai...',
        'no_data': 'Koi data dastiyab nahi',
        'search': 'Talash karen',
        'filter': 'Filter',
        'export': 'Export',
        'import': 'Import',
        'delete': 'Hazf karen',
        'save': 'Mehfooz karen',
        'cancel': 'Mansookh karen',
        'confirm': 'Tasdeeq karen',
    },
    'ar': {
        'welcome': 'مرحبا بكم في الذكاء الاصطناعي المادي والروبوتات',
        'error': 'حدث خطأ',
        'success': 'نجحت العملية',
        'loading': 'جار التحميل...',
        'no_data': 'لا توجد بيانات متاحة',
        'search': 'بحث',
        'filter': 'تصفية',
        'export': 'تصدير',
        'import': 'استيراد',
        'delete': 'حذف',
        'save': 'حفظ',
        'cancel': 'إلغاء',
        'confirm': 'تأكيد',
    },
    'es': {
        'welcome': 'Bienvenido a IA Física y Robótica',
        'error': 'Ocurrió un error',
        'success': 'Operación exitosa',
        'loading': 'Cargando...',
        'no_data': 'No hay datos disponibles',
        'search': 'Buscar',
        'filter': 'Filtrar',
        'export': 'Exportar',
        'import': 'Importar',
        'delete': 'Eliminar',
        'save': 'Guardar',
        'cancel': 'Cancelar',
        'confirm': 'Confirmar',
    },
    'fr': {
        'welcome': 'Bienvenue à l\'IA Physique et Robotique',
        'error': 'Une erreur s\'est produite',
        'success': 'Opération réussie',
        'loading': 'Chargement...',
        'no_data': 'Aucune donnée disponible',
        'search': 'Rechercher',
        'filter': 'Filtrer',
        'export': 'Exporter',
        'import': 'Importer',
        'delete': 'Supprimer',
        'save': 'Enregistrer',
        'cancel': 'Annuler',
        'confirm': 'Confirmer',
    },
}

def get_translation(lang: str, key: str, default: str = None) -> str:
    """
    Get translation for a specific key in a given language
    
    Args:
        lang: Language code (e.g., 'en', 'ur', 'ur-PK')
        key: Translation key
        default: Default value if translation not found
    
    Returns:
        Translated string or default value
    """
    if lang not in TRANSLATIONS:
        lang = 'en'
    
    return TRANSLATIONS[lang].get(key, default or TRANSLATIONS['en'].get(key, key))

def get_supported_languages() -> dict:
    """Get list of all supported languages"""
    return SUPPORTED_LANGUAGES

def is_rtl_language(lang: str) -> bool:
    """Check if language is right-to-left"""
    return lang in ['ur', 'ar']
