with open('BMS_Vilas126_Bootloader/Core/Src/main.c', 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if 'rd_' in line or 'ota' in line.lower() or 'xmodem' in line.lower() or 'boot' in line.lower() or 'jump' in line.lower():
        print(f"Line {i+1}: {line.strip()}")
