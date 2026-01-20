#!/usr/bin/env python3
"""
Verification script to confirm that the video background is properly implemented
"""
import os
import sys

def verify_video_implementation():
    """Check if video background is properly implemented in the homepage"""

    # Check if the video file exists
    video_path = "E:\\physical-humanize-and-robotic-book\\static\\img\\robotics-hero.mp4"
    if not os.path.exists(video_path):
        return False, f"Video file not found: {video_path}"

    # Check if the index.tsx file has the video implementation
    index_path = "E:\\physical-humanize-and-robotic-book\\src\\pages\\index.tsx"
    if not os.path.exists(index_path):
        return False, f"Index file not found: {index_path}"

    with open(index_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check for video-related elements
    has_video_tag = '<video' in content
    has_source_mp4 = 'robotics-hero.mp4' in content
    has_autoPlay = 'autoPlay' in content
    has_muted = 'muted' in content
    has_loop = 'loop' in content

    if not (has_video_tag and has_source_mp4 and has_autoPlay and has_muted and has_loop):
        return False, "Video tag is not properly implemented in index.tsx"

    # Check if the home.css file has proper video styling
    css_path = "E:\\physical-humanize-and-robotic-book\\src\\css\\home.css"
    if not os.path.exists(css_path):
        return False, f"CSS file not found: {css_path}"

    with open(css_path, 'r', encoding='utf-8') as f:
        css_content = f.read()

    has_hero_video = '.hero-video' in css_content
    has_object_fit = 'object-fit: cover' in css_content
    has_absolute_position = 'position: absolute' in css_content and '.hero-video' in css_content

    if not (has_hero_video and has_object_fit and has_absolute_position):
        return False, "CSS does not have proper video background styling"

    return True, "Video background properly implemented"

def main():
    print("Verifying Video Background Implementation")
    print("="*50)

    success, message = verify_video_implementation()

    if success:
        print(f"  PASS {message}")
        print("="*50)
        print("VIDEO BACKGROUND SUCCESSFULLY IMPLEMENTED")
        print("PASS Video file exists in static/img directory")
        print("PASS Video tag properly implemented in index.tsx")
        print("PASS Video has autoPlay, muted, and loop attributes")
        print("PASS CSS has proper video background styling")
        print("PASS Video uses object-fit: cover for full coverage")
        print("\nTo view: Start the Docusaurus server and navigate to homepage")
        return True
    else:
        print(f"  FAIL {message}")
        print("="*50)
        print("VIDEO BACKGROUND IMPLEMENTATION FAILED")
        print("Please check the implementation")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)