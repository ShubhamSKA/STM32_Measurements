# Define the virtual environment name and directory
$venvDir = "gui_venv"  # Replace 'myenv' with your desired virtual environment name
$requirementsFile = "requirements.txt"  # Replace with the path to your requirements file if needed

# Step 1: Create the virtual environment
Write-Host "Creating virtual environment in '$venvDir'..."
python -m venv $venvDir

# Step 2: Activate the virtual environment
Write-Host "Activating virtual environment..."
& "$venvDir\Scripts\Activate.ps1"

# Step 3: Install dependencies from requirements.txt
if (Test-Path $requirementsFile) {
    Write-Host "Installing dependencies from '$requirementsFile'..."
    pip install -r $requirementsFile
} else {
    Write-Host "Requirements file '$requirementsFile' not found. Skipping dependency installation."
}

# Inform the user the script is complete
Write-Host "Virtual environment setup complete. You're now in the virtual environment."
