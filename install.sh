git clone https://github.com/Ballistyxx/blimp-utils.git
if [ $? -ne 0 ]; then
    echo "Failed to clone the repository. Please check your internet connection or the repository URL. If a directory named 'blimp-utils' already exists, please remove it and try again."
    exit 1
fi
cd blimp-utils
echo "Installing blimp-utils..."
sudo pip3 install .
if [ $? -ne 0 ]; then
    echo "Failed to install the package. Please check your Python and pip installation."
    exit 1
fi
cd ..
rm -rf blimp-utils
echo "cleaned up build files..."
echo "blimp-utils installed successfully. Use 'import blimp-utils' in your Python scripts to get started."