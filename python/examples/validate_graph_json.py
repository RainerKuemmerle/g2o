#!/usr/bin/env python3

import argparse
import errno
import json
import sys
from typing import Any, Optional

from jsonschema import ValidationError, validate


def read_json(filename) -> Optional[Any]:
    """Reads the content of filename as JSON and returns the dict

    Args:
        filename: Filename of the file to read

    Returns:
        Optional[Any]: The JSON dict stored in the file, None in case of Exception.
    """
    try:
        with open(filename) as schema_file:
            content = json.load(schema_file)
            return content
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("schema", type=str, help="JSON schema")
    parser.add_argument("input", type=str, help="input JSON g2o graph")
    args = parser.parse_args()

    schema = read_json(args.schema)
    if schema is None:
        print("Error while reading schema")
        sys.exit(errno.EIO)

    json_data = read_json(args.input)
    if json_data is None:
        print("Error while graph input for validation")
        sys.exit(errno.EIO)

    try:
        validate(instance=json_data, schema=schema)
        print("Success")
    except ValidationError as ex:
        print("Validation failed")
        print(ex)


if __name__ == "__main__":
    main()
