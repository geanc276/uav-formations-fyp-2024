#!/usr/bin/env python3
import sys
import json
from typing import List

from codeGenerator import getJson, convertJsonToCpp

def main(args: List[str]) -> List[str]:
    data = getJson(args[0])
    convertJsonToCpp(data, args[1], args[2])
    
    return
    

if __name__ == '__main__':
    try:
        main(sys.argv[1:4])
    except IndexError:
        raise SystemExit(f"Usage: {sys.argv[0]} <JSON_filename_to_parse> <user_definitions_filename> <output_source_filename>")