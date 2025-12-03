#!/bin/sh

# --- Script to automatically generate and populate the 'code' folder ---

# 1. Delete the existing 'code' folder if it is present.
# The '-r' flag allows for recursive deletion (folders and contents).
# The '-f' flag forces the deletion and suppresses most error messages if it doesn't exist.
echo "Checking for existing 'code' folder..."
rm -rf code

# 2. Create the new 'code' folder.
echo "Creating new 'code' folder..."
mkdir code

# 3. Check for 'src' folder and copy its contents.
if [ -d "src" ]; then
    echo "Copying all files and folders from 'src' to 'code/'..."
    # cp -r copies recursively. 'src/*' ensures we copy the *contents* of src, not the src directory itself.
    # 2>/dev/null suppresses 'No such file or directory' errors if 'src' exists but is empty.
    cp -r src/* code/ 2>/dev/null
else
    echo "Warning: 'src' directory not found. Skipping copy from src."
fi

# 4. Check for 'include' folder and copy its contents.
if [ -d "include" ]; then
    echo "Copying all files and folders from 'include' to 'code/'..."
    # cp -r copies recursively. 'include/*' ensures we copy the *contents* of include.
    cp -r include/* code/ 2>/dev/null
else
    echo "Warning: 'include' directory not found. Skipping copy from include."
fi

echo "Process complete. The 'code' directory is ready."