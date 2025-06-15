 
from ast import Import
import os
import re
import shutil



print(">>> SCRIPT POST-BUILD ESEGUITO <<<")

base_dir = os.path.abspath(os.path.dirname("Code"))


header_file = os.path.join(base_dir, "include", "configuration.h")
build_dir = os.path.join(base_dir, ".pio", "build", "esp32-s3-devkitc-1")
dest_dir = base_dir
version_txt_path = os.path.join(base_dir, "version.txt")
bin_fine_name = 'firmware.bin'
# Leggi SWVERSION_NUM
version_num = None
try:
    with open(header_file, "r") as f:
        for line in f:
            m = re.match(r"#define\s+SWVERSION_NUM\s+(\d+)", line)
            if m:
                version_num = m.group(1)
                break
except FileNotFoundError:
    print(f"Errore: non trovato il file {header_file}")

print(version_num) 

if version_num:
    with open(version_txt_path, "w") as vf:
        vf.write(version_num + "\n")
    print(f"Versione scritta su {version_txt_path}: {version_num}")
else:
    print("SWVERSION_NUM non trovato nel file di configurazione")

# Copia file .bin
bin_file = None
try:
    for file in os.listdir(build_dir):
        if file == bin_fine_name:
            bin_file = file
            break
except FileNotFoundError:
    print(f"Errore: directory build non trovata: {build_dir}")
    
print(bin_file)  

if bin_file:
    src_path = os.path.join(build_dir, bin_file)
    dest_path = os.path.join(dest_dir, bin_file)
    try:
        shutil.copy2(src_path, dest_path)
        print(f"File {bin_file} copiato in {dest_dir}")
    except Exception as e:
        print(f"Errore durante la copia del file .bin: {e}")
else:
    print("File .bin non trovato nella cartella di build")


