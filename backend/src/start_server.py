#!/usr/bin/env python3
"""
Custom server startup script to ensure fresh imports
"""
import sys
import os

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import uvicorn

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8001,  # Changed to port 8001 to avoid conflicts
        reload=True,
        reload_dirs=["."]
    )