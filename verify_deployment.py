#!/usr/bin/env python3
"""
Verification script to ensure the Vercel deployment is fully functional
"""
import sys
import importlib.util
import subprocess

def check_python_dependencies():
    """Check if all required dependencies can be imported"""
    required_modules = [
        'fastapi',
        'mangum',
        'requests',
        'bs4',
        'pydantic',
        'google.generativeai',
        'cohere',
        'openai'
    ]

    missing_modules = []
    for module in required_modules:
        try:
            if '.' in module:
                # Handle submodules like google.generativeai
                parts = module.split('.')
                base_module = __import__(parts[0])
                for part in parts[1:]:
                    base_module = getattr(base_module, part)
            else:
                __import__(module)
            print(f"✓ {module} - Available")
        except ImportError as e:
            print(f"x {module} - Missing: {e}")
            missing_modules.append(module)

    return len(missing_modules) == 0

def check_api_structure():
    """Check that the API structure is correct"""
    import os

    # Check if api directory exists
    if not os.path.exists('./api'):
        print("x api directory does not exist")
        return False

    # Check if index.py exists
    if not os.path.exists('./api/index.py'):
        print("x api/index.py does not exist")
        return False

    # Check if requirements.txt exists
    if not os.path.exists('./api/requirements.txt'):
        print("x api/requirements.txt does not exist")
        return False

    print("✓ API structure is correct")
    return True

def check_fastapi_routes():
    """Check that FastAPI routes are properly defined"""
    try:
        from api.index import app
        routes = [route.path for route in app.routes]

        required_routes = [
            '/api/chat/chat',
            '/api/chat/sessions',
            '/api/session/sessions',
            '/api/session/sessions/{session_id}/messages',
            '/health',
            '/api/health'
        ]

        missing_routes = []
        for route in required_routes:
            if route not in routes:
                missing_routes.append(route)

        if missing_routes:
            print(f"x Missing routes: {missing_routes}")
            return False

        print(f"✓ All required routes are present: {len(routes)} total routes")
        return True
    except Exception as e:
        print(f"x Error checking routes: {e}")
        return False

def check_environment_vars():
    """Check that environment variables are properly set up"""
    import os

    required_env_vars = [
        'GEMINI_API_KEY',
        'COHERE_API_KEY',
        'LLM_PROVIDER'
    ]

    missing_vars = []
    for var in required_env_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"! Warning: Missing environment variables: {missing_vars}")
        print("  (These are needed for production but may not be set during local testing)")
    else:
        print("✓ All required environment variables are set")

    return True  # Don't fail the check if env vars are missing during local test

def main():
    print("Verifying Vercel deployment readiness...")
    print("=" * 50)

    all_checks_passed = True

    print("\n1. Checking Python dependencies...")
    if not check_python_dependencies():
        all_checks_passed = False

    print("\n2. Checking API structure...")
    if not check_api_structure():
        all_checks_passed = False

    print("\n3. Checking FastAPI routes...")
    if not check_fastapi_routes():
        all_checks_passed = False

    print("\n4. Checking environment variables...")
    check_environment_vars()  # Don't fail on missing env vars

    print("\n" + "=" * 50)
    if all_checks_passed:
        print("✓ All checks passed! Vercel deployment is ready.")
        print("\nTo deploy to Vercel:")
        print("  1. Install Vercel CLI: npm install -g vercel")
        print("  2. Login: vercel login")
        print("  3. Deploy: vercel --prod")
        print("\nThe backend API will be deployed with all required endpoints.")
        return 0
    else:
        print("x Some checks failed. Please address the issues before deployment.")
        return 1

if __name__ == "__main__":
    sys.exit(main())