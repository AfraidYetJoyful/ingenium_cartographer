import os
import zipfile

output = ""

def exportFile(content, file_path):
    with open(file_path, 'w', encoding='utf-8') as file:
        file.write(content)
        
        

def find_files(root_folder, max_depth=4, extension = ".py"):
    global output
    def search_folder(folder, depth):
        global output
        if depth > max_depth:
            return  # Stop if we exceed the allowed depth

        try:
            for entry in os.scandir(folder):
                if entry.is_file():
                    if entry.name.endswith(extension):
                        output = output + os.path.abspath(entry.path) + "\n"
                    elif entry.name.endswith('.zip'):
                        search_zip(entry.path)
                elif entry.is_dir():
                    search_folder(entry.path, depth + 1)
        except (PermissionError, OSError) as e:
            print(f"Error accessing {folder}: {e}")

    def search_zip(zip_path):
        global output
        try:
            with zipfile.ZipFile(zip_path, 'r') as z:
                for file in z.namelist():
                    if file.endswith('.py'):
                        output = output + f"{zip_path} -> {file}\n"
        except zipfile.BadZipFile:
            print(f"Invalid zip file: {zip_path}")

    # Normalize the root folder path to avoid errors
    root_folder = os.path.normpath(root_folder)
    search_folder(root_folder, depth=1)

# Example usage
if __name__ == "__main__":
    folder_to_search = input("Enter the .zip folder path to search [\ and / are OK, but no \" or ']: ").strip()
    e = input("Please enter file type suffix to be found:")
    if os.path.exists(folder_to_search):
        find_files(folder_to_search, extension = e)
        exportFile(output, "dotpysinrecoveryfolder2zip.txt")
        print("Done.")
    else:
        print(f"Invalid path: {folder_to_search}")