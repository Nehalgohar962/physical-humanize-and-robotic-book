---
ID: 2
TITLE: Fix Chatbot UI Issues
STAGE: general
DATE_ISO: 2026-02-07
SURFACE: agent
MODEL: claude-opus-4-6
FEATURE: none
BRANCH: main
USER: user
COMMAND: null
LABELS: ["UI", "chatbot", "layout", "styling"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML: []
TESTS_YAML: []
---
### PROMPT_TEXT:
```
Hi Claude,

Please first fix the chatbot icon positioning and layout issues. These are priority problems:

🔴 1. Chat Icon Position (MOST IMPORTANT)

Currently, the chat icon is appearing below the footer on the LEFT side.

Move the chatbot icon to the BOTTOM-RIGHT corner of the screen.

The icon must be:

Fixed position

Visible on all pages

NOT inside the footer

🎨 2. Chat Icon Styling

Right now the chat icon looks very simple (plain, no CSS).

Apply proper CSS according to the website theme:

Rounded circle

Shadow

Hover effect

Professional chat icon (not a raw emoji)

It should look like a real website chatbot icon (RagChat / Intercom style).

📐 3. Chat Window Width Issue

When I click the chat icon, the chat opens with the same width as the footer, which is wrong.

Fix this:

Chat popup should have a fixed width (like RagChat)

Example: ~360–400px on desktop

On mobile: full width or 95%

Chat popup must float above content, not stretch with footer.

🗑️ 4. Remove Extra Message Icons

There is a message/chat icon above the footer on the LEFT side → remove it completely.

There should be ONLY ONE chat icon on the entire website (bottom-right).

🎯 Expected Result

One styled chatbot icon at bottom-right

No icons near or inside the footer

Chat popup opens with proper width (not footer width)

Clean, professional UI matching the website theme

Please fix only these UI/layout issues first.

Thanks 🙏
```
### RESPONSE_TEXT:
```
Okay, I understand. I will address these UI/layout issues for the chatbot icon and window. My approach will involve exploring the codebase to identify the relevant files for the chatbot icon, its styling, and the chat window. Once identified, I will propose and implement the necessary changes to meet the specified requirements.

First, I will search for files that might contain the chatbot implementation.
```
### OUTCOME:
User requested UI fixes for chatbot. PHR created.
### EVALUATION:
Understood user request and outlined initial steps. PHR recorded.
