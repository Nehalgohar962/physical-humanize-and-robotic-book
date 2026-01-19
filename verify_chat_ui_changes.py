#!/usr/bin/env python3
"""
Verification script to confirm that all chat UI fixes have been applied
"""
import os
import sys

def verify_chat_ui_changes():
    """Check if all requested chat UI changes have been applied"""

    # Check the main chat interface component
    chat_interface_path = "E:\\physical-humanize-and-robotic-book\\src\\components\\Chatbot\\ChatInterface.tsx"
    if not os.path.exists(chat_interface_path):
        return False, f"Chat interface file not found: {chat_interface_path}"

    with open(chat_interface_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check that mode selector buttons are removed
    has_mode_selector = 'Full Book Search' in content or 'Context Only' in content
    has_chat_mode_selector = 'chat-mode-selector' in content and '<div className="chat-mode-selector"' in content

    # Check that context input is removed
    has_context_input = 'context-input' in content and 'contextText' in content

    # Check that mode state is removed
    has_mode_state = 'useState<\'full_book\' | \'context_only\'>' in content

    # Check that send button is styled with red
    has_red_send_button = True  # This is handled in CSS, we'll check CSS separately

    if has_mode_selector or has_chat_mode_selector or has_context_input or has_mode_state:
        issues = []
        if has_mode_selector: issues.append("Mode selector buttons still present")
        if has_chat_mode_selector: issues.append("Chat mode selector container still present")
        if has_context_input: issues.append("Context input section still present")
        if has_mode_state: issues.append("Mode state still present")
        return False, f"Mode selector buttons and context input not fully removed: {', '.join(issues)}"

    # Check the chat interface CSS
    chat_css_path = "E:\\physical-humanize-and-robotic-book\\src\\components\\Chatbot\\ChatInterface.css"
    if not os.path.exists(chat_css_path):
        return False, f"Chat CSS file not found: {chat_css_path}"

    with open(chat_css_path, 'r', encoding='utf-8') as f:
        css_content = f.read()

    # Check that send button uses red theme
    has_red_send_button = 'var(--ifm-color-secondary)' in css_content and 'background-color: var(--ifm-color-secondary)' in css_content and '.chat-input-form button' in css_content

    # Check that clear button uses red theme
    has_red_clear_button = 'var(--ifm-color-secondary)' in css_content and '.clear-session-btn' in css_content and 'background-color: var(--ifm-color-secondary)' in css_content

    if not (has_red_send_button and has_red_clear_button):
        return False, "Buttons are not styled with red theme in ChatInterface.css"

    # Check the RAG chatbot CSS
    rag_css_path = "E:\\physical-humanize-and-robotic-book\\src\\components\\Docusaurus\\RAGChatbot.css"
    if not os.path.exists(rag_css_path):
        return False, f"RAG Chatbot CSS file not found: {rag_css_path}"

    with open(rag_css_path, 'r', encoding='utf-8') as f:
        rag_css_content = f.read()

    # Check that various buttons use red theme in RAGChatbot.css
    has_red_chat_button = 'var(--ifm-color-secondary)' in rag_css_content and '.chat-button' in rag_css_content
    has_red_input_button = 'var(--ifm-color-secondary)' in rag_css_content and '.chat-input-form button' in rag_css_content
    has_red_control_buttons = 'var(--ifm-color-secondary)' in rag_css_content and '.clear-session-btn' in rag_css_content

    if not (has_red_chat_button and has_red_input_button and has_red_control_buttons):
        return False, "Buttons are not styled with red theme in RAGChatbot.css"

    # Check that home.css has proper background image
    home_css_path = "E:\\physical-humanize-and-robotic-book\\src\\css\\home.css"
    if not os.path.exists(home_css_path):
        return False, f"Home CSS file not found: {home_css_path}"

    with open(home_css_path, 'r', encoding='utf-8') as f:
        home_css_content = f.read()

    has_valid_background = 'background-image:' in home_css_content and '.hero-banner' in home_css_content
    uses_image_not_video = 'jpg' in home_css_content or 'png' in home_css_content or 'gif' in home_css_content

    if not (has_valid_background and uses_image_not_video):
        return False, "Home page background is not properly set to an image"

    return True, "All chat UI changes successfully applied"

def main():
    print("Verifying Chat UI Changes")
    print("="*50)

    success, message = verify_chat_ui_changes()

    if success:
        print(f"  PASS {message}")
        print("="*50)
        print("CHAT UI CHANGES SUCCESSFULLY APPLIED")
        print("✓ Mode selector buttons removed")
        print("✓ Context input section removed")
        print("✓ Send button styled with red theme")
        print("✓ All other buttons styled with red theme")
        print("✓ Home page background fixed")
        print("\nTo view: Start the Docusaurus server and test the chat interface")
        return True
    else:
        print(f"  FAIL {message}")
        print("="*50)
        print("CHAT UI CHANGES NOT FULLY APPLIED")
        print("Please check the implementation")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)