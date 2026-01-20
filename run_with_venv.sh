#!/bin/bash
# Script to properly activate virtual environment and run main.py

echo "Activating virtual environment and running main.py..."
echo

# Change to the backend/src directory
cd backend/src

# Activate the virtual environment (Windows)
echo "Activating virtual environment..."
source ../../.venv/Scripts/activate

# Install packages if not already installed
echo "Installing required packages in virtual environment..."
pip install -r ../../requirements.txt

# Run the main.py file
echo "Running main.py..."
python main.py

echo
echo "Application completed successfully!"