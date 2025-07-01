
#MS Searches desired folder to desired depth and exports file paths.

#MS The "os" library allows for easier interaction with the operating system (ie. moving/renaming files).
#MS The "zipfile" library allows for creation and interaction with ZIP files and archives.
import os
import zipfile

#MS Creates empty string and assigns it to a variable, "output". Acts as "global accumulator", which means that it will accumulate file paths from throughout the program.
output = ""

#MS Fuction that inputs "content" and "file_path" and writes the latter with the former (overwriting if it already exists).
def exportFile(content, file_path):
    with open(file_path, 'w', encoding='utf-8') as file:
        file.write(content)
        
        
#MS Core function for searching files. "root_folder" is the starting directory where the search will start. "max_depth=4" is a parameter that sets the depth of the search to 4 sub_categories (aka 4 levels deep). "extension=".py"" specifies that the file we are looking for is in python (.py).
def find_files(root_folder, max_depth=4, extension = ".py"):
    global output 
    #MS "search_folder" function to search directories. Parameter "folder" specifies starting folder and "depth" sets the limit of subcategories to search.
    def search_folder(folder, depth):
        global output #MS Declares that this function will modify the global "output" variable.
        if depth > max_depth:
            return  #AB Stop if we exceed the allowed depth
        #MS Try...except allows for possible errors (PermissionError and OSError, or system error).
        try:
            for entry in os.scandir(folder): #MS Efficiently iterates over files and subdirectories in a given folder.
                if entry.is_file(): #MS Checks of current item is a file.
                    if entry.name.endswith(extension): #MS Checks if the files is the right type as specified by the extension (in this case .py).
                        output = output + os.path.abspath(entry.path) + "\n" #MS If file is what we are looking for, we add its file path to the global "outputs" variable. Then we add a suffix "\n"
                    elif entry.name.endswith('.zip'): #MS If the entry is a .zip instead of .py, then we instruct the program to look inside the .zip.
                        search_zip(entry.path)
                elif entry.is_dir(): #MS Looks to see if "entry" is a directory. If it is, then it specifies how deep to look.
                    search_folder(entry.path, depth + 1)
        except (PermissionError, OSError) as e: #If system throws an error, the system will display exactly where the problem occured.
            print(f"Error accessing {folder}: {e}")



    def search_zip(zip_path): #MS Function searches inside .zip files.
        global output #MS Declares intent to modify global "output".
        try: #MS Handles errors if .zip file is not valid.
            with zipfile.ZipFile(zip_path, 'r') as z: #MS Opens .zip in read mode ('r') and closes it down afterwards.
                for file in z.namelist(): #MS GOes through all the files and directories in ZIP file.
                    if file.endswith('.py'): #If the file ends with .py, then the file is added to global "output".
                        output = output + f"{zip_path} -> {file}\n"
        except zipfile.BadZipFile: #MS If error is thrown due to a bad ZIP file, then it should print a certain message.
            print(f"Invalid zip file: {zip_path}")

    #AB Normalize the root folder path to avoid errors.
    root_folder = os.path.normpath(root_folder)
    search_folder(root_folder, depth=1) #MS Calls "search_folder" with parameters "root_folder" and set depth "1".

#AB Example usage
if __name__ == "__main__": #MS User interface that takes input and runs the above functions.
    folder_to_search = input("Enter the .zip folder path to search [\ and / are OK, but no \" or ']: ").strip()
    e = input("Please enter file type suffix to be found:")
    if os.path.exists(folder_to_search):
        find_files(folder_to_search, extension = e)
        exportFile(output, "dotpysinrecoveryfolder2zip.txt")
        print("Done.")
    else:
        print(f"Invalid path: {folder_to_search}")