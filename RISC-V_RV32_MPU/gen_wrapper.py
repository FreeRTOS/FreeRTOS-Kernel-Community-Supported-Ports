#!/usr/bin/env python3
"""
Fill in MPU wrapper function bodies in a C template file.

- Uses two top-level variables for readability:
  * ASM_INSTRUCTIONS: the assembly instructions
  * ASM_TEMPLATE: the __asm volatile wrapper with constraints
- Aligns '\n' in inline asm to the longest line.
- Preserves function signature indentation.
- Prints the filled file to stdout.
- 4-space indentation inside function body.
"""

import re
import sys

# -------------------------------
# Inline assembly instructions (modifiable)
# -------------------------------
ASM_INSTRUCTIONS = [
    ".extern MPU_{func}Impl",
    "bnez tp, 1f",
    "    j MPU_{func}Impl",
    "1:",
    "    li   a7, %0",
    "    ecall",
    "    ret"
]

# -------------------------------
# Inline assembly wrapper template (modifiable)
# -------------------------------
ASM_TEMPLATE = [
    "__asm volatile (",
    # Instructions will be inserted here dynamically
    "{instructions}",
    ': : "i"(SYSTEM_CALL_{func}) : "memory");'
]

def generate_wrapper(func_name: str, base_indent: str) -> str:
    """
    Generate the __asm volatile block for a given MPU function.
    - Aligns '\n' of all lines to the longest line.
    - Indents properly inside the function body.
    """
    # Format instructions with the function name
    instr_lines = [line.format(func=func_name) for line in ASM_INSTRUCTIONS]

    # Compute max length for alignment (ignoring leading spaces)
    max_len = max(len(line) for line in instr_lines)

    # Pad each line so that '\n' aligns
    padded_instr = []
    for line in instr_lines:
        padding = " " * (max_len - len(line))
        padded_instr.append(f'    "{line}{padding} \\n"')  # 4 spaces inside asm block

    # Insert padded instructions into ASM_TEMPLATE
    wrapper_lines = []
    for line in ASM_TEMPLATE:
        if "{instructions}" in line:
            wrapper_lines.extend(padded_instr)
        else:
            wrapper_lines.append(line.format(func=func_name))

    # Indent the whole block one level inside function body
    indent = base_indent + " " * 4
    indented_lines = [indent + l for l in wrapper_lines]

    return "\n".join(indented_lines)

def fill_mpu_wrappers(input_file: str):
    """
    Read a C template file, find empty MPU functions, and fill them with inline asm.
    """
    with open(input_file, "r") as f:
        content = f.read()

    # Regex to match empty MPU functions
    pattern = re.compile(
        r"(?P<indent>[ \t]*)"
        r"(?P<signature>\bMPU_\w+\s*\([^\)]*\)\s*(?:/\*.*?\*/\s*)?)"
        r"\{\s*\}", re.MULTILINE
    )

    def replacer(match):
        base_indent = match.group("indent").replace("\t", " " * 4)
        signature = match.group("signature").strip()

        func_match = re.search(r"\bMPU_(\w+)\s*\(", signature)
        if not func_match:
            return match.group(0)

        func_name = func_match.group(1)
        asm_body = generate_wrapper(func_name, base_indent)

        return f"{base_indent}{signature}\n{base_indent}{{\n{asm_body}\n{base_indent}}}"

    new_content = pattern.sub(replacer, content)
    print(new_content)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python fill_mpu_wrappers.py template.c")
        sys.exit(1)

    fill_mpu_wrappers(sys.argv[1])

