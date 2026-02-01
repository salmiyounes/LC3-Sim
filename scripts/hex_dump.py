import sys
import re
from pathlib import Path

def read_byte_from_object_file(path: str):
    with open(path, "rb") as f:
        while (byte := f.read(1)):
            yield f"0x{byte[0]:02x}"

def size(path: str) -> int | None:
    f_path = Path(path)
    try:
        return f_path.stat().st_size
    except FileNotFoundError:
        print(f"Error: The file at '{path}' was not found.")

def match_file_path(path: str) -> str | None:
    m = re.search(r"(.)+/*(?=.obj)", path)
    if not m:
        return None
    return m.group()

def clean_file_path(name: str) -> str:
    return match_file_path(name).replace('/', '_').replace('-', '_')

def output_c_array_name(path: str) -> str:
    f_name = clean_file_path(path)
    bytes_str = ', '.join(read_byte_from_object_file(path))

    if f_name:
        return f"unsigned char {f_name}[] = {{ {bytes_str} }};"
    else:
        return f"unsigned char c_array_default[] = {{ {bytes_str} }};"

def output_c_array_len_name(path: str) -> str:
    f_name = clean_file_path(path)
    length = size(path)

    if f_name:
        return f"unsigned int {f_name}_len = {length};"
    else:
        return f"unsigned int c_array_default_len = {length};"

def dump_to_output(path: str):
    print(output_c_array_name(path))
    print(output_c_array_len_name(path))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print ("Usage: python3 hex_dump.py <objfile>")
    else:
        path = sys.argv[1]
        dump_to_output(path)