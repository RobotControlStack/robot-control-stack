#!/usr/bin/env python3

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def local_name(tag: object) -> str | None:
    if not isinstance(tag, str):
        return None
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def update_mjcf(root: ET.Element, force: bool, remove: bool) -> tuple[int, int]:
    body_count = 0
    joint_count = 0

    for elem in root.iter():
        tag = local_name(elem.tag)
        if tag is None:
            continue

        if remove:
            if tag == "body" and "gravcomp" in elem.attrib:
                del elem.attrib["gravcomp"]
                body_count += 1

            if tag == "joint" and "actuatorgravcomp" in elem.attrib:
                del elem.attrib["actuatorgravcomp"]
                joint_count += 1
            continue

        if tag == "body" and (force or "gravcomp" not in elem.attrib):
            elem.set("gravcomp", "1")
            body_count += 1

        if tag == "joint" and (force or "actuatorgravcomp" not in elem.attrib):
            elem.set("actuatorgravcomp", "true")
            joint_count += 1

    return body_count, joint_count


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Add gravcomp attributes to all MJCF <body> and <joint> tags."
    )
    parser.add_argument("input_xml", type=Path, help="Path to the input MJCF XML file.")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Optional output path. If omitted, the input file is updated in place.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing gravcomp / actuatorgravcomp attributes.",
    )
    parser.add_argument(
        "--remove",
        action="store_true",
        help="Remove existing gravcomp / actuatorgravcomp attributes instead of adding them.",
    )
    args = parser.parse_args()

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(args.input_xml, parser=parser)
    root = tree.getroot()

    body_count, joint_count = update_mjcf(
        root,
        force=args.force,
        remove=args.remove,
    )
    ET.indent(tree, space="    ")

    output_path = args.output or args.input_xml
    tree.write(output_path, encoding="utf-8", xml_declaration=False)

    action = "Removed" if args.remove else "Updated"
    print(f"{action} {body_count} body tag(s) and {joint_count} joint tag(s) in {output_path}")


if __name__ == "__main__":
    main()
