# AI Book Assistant UI

A professional, academic-style chatbot interface for the Physical & Humanoid Robotics book project.

## Features

- **Floating Chat Widget**: Bottom-right positioned chat interface (like ChatGPT/Intercom)
- **Academic Theme**: Dark navy/charcoal background with electric blue accents
- **Professional Design**: Clean, structured layout suitable for academic/hackathon submission
- **Book Content Focus**: Clearly displays the 6 book modules
- **Reference System**: Properly displays book references with "References from Book" section
- **Session Management**: New session/clear chat functionality
- **Loading States**: Typing indicators and disabled buttons during processing

## File Structure

```
public/
├── index.html          # Main HTML file with book content and chat widget
├── styles.css          # Professional styling with robotics theme
└── script.js           # Chatbot functionality with backend API integration
```

## Backend API Integration

The UI connects to your existing backend endpoints:

- **Session Creation**: `POST /api/chat/sessions`
- **Chat Messages**: `POST /api/chat/chat`

## Key UI Elements

### Chat Widget
- Floating circular button at bottom-right of screen
- Smooth animations for open/close
- Professional gradient design

### Message Display
- Clear distinction between user and bot messages
- Bot responses include "References from Book" section
- Academic styling for reference items

### Input Area
- Auto-resizing textarea
- Send button with hover effects
- Loading indicators
- Proper focus management

## Theme Colors

- **Background**: Dark navy/charcoal gradient
- **Text**: White/light gray
- **Accents**: Electric blue (#00ccff) and teal
- **Fonts**: Inter (professional, technical look)

## Responsive Design

- Adapts to desktop, tablet, and mobile screens
- Maintains professional appearance on all devices
- Properly sized chat window for different viewports

## Usage

1. Place these files in your public directory
2. Ensure your backend API endpoints are available
3. The UI will automatically connect to `/api/chat/sessions` and `/api/chat/chat`
4. References from the RAG system will be displayed in the "References from Book" section

## Professional Features

- Clean, non-flashy design appropriate for academic submissions
- Proper spacing, alignment, and visual hierarchy
- Smooth animations and transitions
- Accessibility considerations
- Professional typography and color scheme
- Clear visual separation of content types