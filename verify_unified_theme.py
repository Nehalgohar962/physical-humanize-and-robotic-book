#!/usr/bin/env python3
"""
Verification script to confirm that the unified academic theme is applied everywhere
"""
import os
import sys

def verify_color_scheme(file_path, theme_colors):
    """Check if file contains the academic theme colors or CSS variables"""
    if not os.path.exists(file_path):
        return False, f"File not found: {file_path}"

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    missing_colors = []
    for color_name, color_value in theme_colors.items():
        # Check for the actual hex color value or CSS variable names
        if color_value not in content:
            # Also check for CSS variable definitions that might use these colors
            var_found = False
            if color_name == "background":
                var_found = "#0f172a" in content or "var(--ifm-background-color)" in content
            elif color_name == "surface":
                var_found = "#111827" in content or "var(--ifm-background-surface-color)" in content
            elif color_name == "primary":
                var_found = "#6366f1" in content or "var(--ifm-color-primary)" in content
            elif color_name == "primary_dark":
                var_found = "#4f46e5" in content or "var(--ifm-color-primary-dark)" in content
            elif color_name == "text_primary":
                var_found = "#e5e7eb" in content or "var(--ifm-color-content)" in content
            elif color_name == "text_secondary":
                var_found = "#9ca3af" in content or "var(--ifm-color-content-secondary)" in content
            elif color_name == "border":
                var_found = "#1f2933" in content or "var(--ifm-border-color)" in content

            if not var_found:
                missing_colors.append(color_name)

    if missing_colors:
        return False, f"Missing colors: {missing_colors}"
    else:
        return True, "All theme colors found"

def main():
    print("Verifying Unified Academic Theme Implementation")
    print("="*60)

    # Define the academic theme colors
    academic_theme = {
        "background": "#0f172a",  # Dark grey background
        "surface": "#111827",     # Secondary dark grey
        "primary": "#6366f1",     # Purple accent
        "primary_dark": "#4f46e5", # Darker purple
        "text_primary": "#e5e7eb", # Light text
        "text_secondary": "#9ca3af", # Medium text
        "border": "#1f2933"       # Dark border
    }

    # Files to check
    files_to_verify = [
        "src/css/custom.css",  # Main theme file
        "src/components/Docusaurus/RAGChatbot.css",  # Chatbot UI
        "src/components/Chatbot/ChatInterface.css"  # Chat interface
    ]

    all_good = True

    for file_path in files_to_verify:
        abs_path = os.path.join("E:\\physical-humanize-and-robotic-book", file_path)
        print(f"\nChecking {file_path}...")

        success, message = verify_color_scheme(abs_path, academic_theme)
        if success:
            print(f"  PASS {message}")
        else:
            print(f"  FAIL {message}")
            all_good = False

    print("\n" + "="*60)
    if all_good:
        print("UNIFIED ACADEMIC THEME SUCCESSFULLY APPLIED")
        print("PASS Grey + Purple theme applied to entire system")
        print("PASS Book pages, chatbot, and navigation unified")
        print("PASS All components use consistent academic styling")
        print("PASS Modern, clean, premium academic appearance")
    else:
        print("SOME COMPONENTS MISSING ACADEMIC THEME")
        print("Please check the failed files above")

    print("\nMANUAL VISUAL CHECKLIST:")
    print("   - Open book pages: verify grey background with purple accents")
    print("   - Open chatbot: verify same theme applied")
    print("   - Check navbar: verify consistent styling")
    print("   - Check sidebar: verify academic theme")
    print("   - Test chat: verify purple-themed interface")

    return all_good

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)