#!/usr/bin/env python3
"""
Quick script to add Roman Urdu translations to frontmatter
"""

import os
import re

# Translation mapping for common terms
TRANSLATIONS = {
    "Ethical Dilemmas & Controversies in Educational Robotics": "Taleemi Robotics mein Ikhlaaqi Masail",
    "Pedagogical Approaches with Humanoid Robots": "Humanoid Robots ke saath Tadreesi Tareeqay",
    "Pedagogical Approaches and Learning Theories": "Tadreesi Tareeqay aur Seekhne ke Nazariyat",
    "Educational Level Considerations": "Taleemi Satah ke Khayalat",
    "Implementation Guidance for Educators": "Muallimeen ke liye Nifaaz ki Rahnumai",
    "Privacy & Security in Educational Robotics": "Taleemi Robotics mein Privacy aur Security",
    "Technical Concepts in Physical AI & Humanoid Robotics": "Physical AI aur Humanoid Robotics mein Takneeki Tasawurat",
    "Accessibility Statement": "Rasooi ka Bayan",
    "Documentation Updates": "Dastaveyzat ki Updates",
    "Edge Kit": "Edge Kit",
    "Update Procedures": "Update ke Tareeqe",
    "Versioning Strategy": "Version ki Strategy"
}

def translate_title(title):
    """Get Roman Urdu translation of title"""
    return TRANSLATIONS.get(title, title)

def update_frontmatter(file_path):
    """Update frontmatter title to Roman Urdu"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Match frontmatter
        match = re.search(r'^---\n(.+?)\n---', content, re.DOTALL)
        if not match:
            return False
        
        frontmatter = match.group(1)
        
        # Extract title
        title_match = re.search(r'title:\s*(.+?)$', frontmatter, re.MULTILINE)
        if not title_match:
            return False
        
        original_title = title_match.group(1).strip()
        translated_title = translate_title(original_title)
        
        if translated_title != original_title:
            # Replace title
            new_frontmatter = re.sub(
                r'title:\s*.+?$',
                f'title: {translated_title}',
                frontmatter,
                flags=re.MULTILINE
            )
            new_content = content.replace(frontmatter, new_frontmatter)
            
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(new_content)
            
            print(f"✓ Updated: {os.path.basename(file_path)}")
            return True
        else:
            print(f"- Skipped: {os.path.basename(file_path)} (no translation)")
            return False
            
    except Exception as e:
        print(f"✗ Error processing {file_path}: {e}")
        return False

def main():
    base_dir = "/media/data/hackathon series/physical-AI-Homanoid-Book-main/frontend/i18n/ur-PK/docusaurus-plugin-content-docs/current"
    
    if not os.path.exists(base_dir):
        print(f"Directory not found: {base_dir}")
        return
    
    print("Updating Roman Urdu translations...")
    print("=" * 50)
    
    count = 0
    for filename in os.listdir(base_dir):
        if filename.endswith('.md'):
            file_path = os.path.join(base_dir, filename)
            if update_frontmatter(file_path):
                count += 1
    
    print("=" * 50)
    print(f"Total updated: {count} files")

if __name__ == "__main__":
    main()
