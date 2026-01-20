@echo off
REM Batch script to activate virtual environment and run main.py

echo ================================================
echo Setting up Virtual Environment and Running main.py
echo ================================================
echo.

REM Change to the project root directory
cd /d "%~dp0"

REM Activate the virtual environment
echo Activating virtual environment...
call .venv\Scripts\activate.bat

if errorlevel 1 (
    echo Failed to activate virtual environment. Creating one first...
    python -m venv .venv
    call .venv\Scripts\activate.bat
)

REM Install packages if not already installed
echo Installing required packages in virtual environment...
pip install -r requirements.txt

REM Change to the backend/src directory
cd backend\src

REM Run the main.py file
echo.
echo Running main.py...
python main.py

echo.
echo ================================================
echo Application completed successfully!
echo ================================================
pause