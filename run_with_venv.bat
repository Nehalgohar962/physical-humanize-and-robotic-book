@echo off
REM Batch script to activate virtual environment and run main.py

echo Activating virtual environment and running main.py...
echo.

REM Change to the backend/src directory
cd backend\src

REM Activate the virtual environment (Windows)
echo Activating virtual environment...
call ..\..\Scripts\activate.bat

REM Install packages if not already installed
echo Installing required packages in virtual environment...
pip install -r ..\..\requirements.txt

REM Run the main.py file
echo Running main.py...
python main.py

echo.
echo Application completed successfully!
pause