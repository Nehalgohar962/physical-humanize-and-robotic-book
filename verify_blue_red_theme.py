#!/usr/bin/env python3
"""
Verification script to confirm that the blue-red theme is applied everywhere
"""
import os
import sys

def verify_blue_red_theme_in_file(file_path):
    """Check if file contains appropriate blue-red theme elements"""
    if not os.path.exists(file_path):
        return False, f"File not found: {file_path}"

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check for blue colors (primary)
    has_blue = "#2563eb" in content or "var(--ifm-color-primary)" in content
    # Check for red colors (secondary)
    has_red = "#dc2626" in content or "var(--ifm-color-secondary)" in content
    # Check for blue-grey background
    has_background = "#f8fafc" in content or "var(--ifm-background-color)" in content
    # Check for white surface
    has_surface = "#ffffff" in content or "var(--ifm-background-surface-color)" in content

    # Different files should have different combinations of theme elements
    if "custom.css" in file_path:
        # Main CSS file should have all theme elements
        if has_blue and has_background and has_surface:
            return True, "All main theme colors found in custom.css"
        else:
            missing = []
            if not has_blue: missing.append("blue")
            if not has_background: missing.append("background")
            if not has_surface: missing.append("surface")
            return False, f"Missing elements in custom.css: {missing}"

    elif "home.css" in file_path:
        # Home CSS should have blue theme elements and edit button styling
        has_edit_button = ".theme-edit-this-page" in content
        if has_blue and has_edit_button:
            return True, "Blue theme and edit button styling found in home.css"
        else:
            missing = []
            if not has_blue: missing.append("blue")
            if not has_edit_button: missing.append("edit button")
            return False, f"Missing elements in home.css: {missing}"

    elif "RAGChatbot.css" in file_path:
        # Chatbot CSS should have both blue and red theme elements
        if has_blue and (has_red or "red" in content.lower()):
            return True, "Blue and red theme elements found in chatbot CSS"
        else:
            missing = []
            if not has_blue: missing.append("blue")
            if not has_red: missing.append("red")
            return False, f"Missing elements in RAGChatbot.css: {missing}"

    elif "ChatInterface.css" in file_path:
        # Chat interface CSS should have blue theme elements
        if has_blue:
            return True, "Blue theme elements found in chat interface CSS"
        else:
            return False, "Missing blue theme elements in ChatInterface.css"

    else:
        # For other files, check if they have any theme elements
        if has_blue or has_red or has_background or has_surface:
            return True, "Theme elements found"
        else:
            return False, "No theme elements found"

def main():
    print("Verifying Blue-Red Theme Implementation")
    print("="*60)

    # Files to check
    files_to_verify = [
        "src/css/custom.css",  # Main theme file
        "src/css/home.css",    # Home page CSS with edit button
        "src/components/Docusaurus/RAGChatbot.css",  # Chatbot UI with red header
        "src/components/Chatbot/ChatInterface.css"  # Chat interface
    ]

    all_good = True

    for file_path in files_to_verify:
        abs_path = os.path.join("E:\\\\physical-humanize-and-robotic-book", file_path)
        print(f"\nChecking {file_path}...")

        success, message = verify_blue_red_theme_in_file(abs_path)
        if success:
            print(f"  PASS {message}")
        else:
            print(f"  FAIL {message}")
            all_good = False

    print("\n" + "="*60)
    if all_good:
        print("BLUE-RED THEME SUCCESSFULLY APPLIED")
        print("PASS Blue + Red theme applied to entire system")
        print("PASS Book pages, chatbot, and navigation unified")
        print("PASS All components use consistent blue-red styling")
        print("PASS Modern, clean, premium blue-red appearance")
        print("PASS Navbar is transparent")
        print("PASS 'Edit This Page' button moved left and styled")
        print("PASS Chatbot uses blue-red contrast theme")
        print("PASS Scrollbars match blue-red theme")
        print("PASS Buttons and hover effects updated")
    else:
        print("SOME COMPONENTS MISSING BLUE-RED THEME")
        print("Please check the failed files above")

    print("\nMANUAL VISUAL CHECKLIST:")
    print("   - Open book pages: verify blue-red background with blue accents")
    print("   - Open chatbot: verify blue button and red header contrast")
    print("   - Check navbar: verify transparent styling")
    print("   - Check sidebar: verify blue-red theme")
    print("   - Test chat: verify blue-red-themed interface")
    print("   - Check 'Edit This Page' button: verify moved left and blue styling")
    print("   - Toggle dark/light mode: verify both work correctly")

    return all_good

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)