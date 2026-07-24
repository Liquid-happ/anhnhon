import re

def list_functions(c_file):
    with open(c_file, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    # Simple regex to find C function definitions
    # It matches word word(word word, ...) { at the start of a line
    pattern = re.compile(r'^[a-zA-Z_][a-zA-Z0-9_]*\s+[a-zA-Z_][a-zA-Z0-9_]*\s*\([^)]*\)\s*\{', re.MULTILINE)
    
    matches = pattern.finditer(content)
    functions = []
    for match in matches:
        start = match.start()
        # Extract the line of function definition
        line = content[start:content.find('\n', start)]
        functions.append(line.strip())
    return functions

if __name__ == '__main__':
    c_file = 'BMS_Vilas126_Bootloader/Core/Src/main.c'
    funcs = list_functions(c_file)
    print("Total functions in main.c:", len(funcs))
    for f in funcs:
        print(" -", f)
