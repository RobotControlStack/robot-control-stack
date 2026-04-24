from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TypedDictSpec:
    source_class: str
    output_class: str

    @classmethod
    def from_class_name(cls, class_name: str) -> "TypedDictSpec":
        return cls(source_class=class_name, output_class=f"{class_name}Kwargs")


TARGETS = tuple(
    TypedDictSpec.from_class_name(class_name)
    for class_name in (
        "BaseCameraConfig",
        "RobotConfig",
        "GripperConfig",
    )
)


class _QualifyCommonNames(ast.NodeTransformer):
    def __init__(self, common_class_names: set[str]) -> None:
        self.common_class_names = common_class_names

    def visit_Name(self, node: ast.Name) -> ast.AST:
        if node.id in self.common_class_names:
            return ast.copy_location(
                ast.Attribute(value=ast.Name(id="common", ctx=ast.Load()), attr=node.id, ctx=ast.Load()),
                node,
            )
        return node


class _SimplifyNdarrayAnnotations(ast.NodeTransformer):
    def visit_Subscript(self, node: ast.Subscript) -> ast.AST:
        node = self.generic_visit(node)
        if isinstance(node.value, ast.Attribute) and isinstance(node.value.value, ast.Name):
            if node.value.value.id == "numpy" and node.value.attr == "ndarray":
                return ast.copy_location(node.value, node)
        return node


class _ReferencedNames(ast.NodeVisitor):
    def __init__(self) -> None:
        self.names: set[str] = set()

    def visit_Name(self, node: ast.Name) -> None:
        self.names.add(node.id)


def _names_in_node(node: ast.AST) -> set[str]:
    referenced_names = _ReferencedNames()
    referenced_names.visit(node)
    return referenced_names.names


def _find_class(module: ast.Module, class_name: str) -> ast.ClassDef:
    for node in module.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return node

    msg = f"Could not find class {class_name!r} in stub file."
    raise ValueError(msg)


def _find_init_signature(class_node: ast.ClassDef) -> ast.FunctionDef:
    for node in class_node.body:
        if isinstance(node, ast.FunctionDef) and node.name == "__init__":
            return node

    msg = f"Could not find __init__ for class {class_node.name!r}."
    raise ValueError(msg)


def _annotation_to_text(
    annotation: ast.expr | None,
    qualifier: _QualifyCommonNames,
) -> tuple[str, set[str]]:
    if annotation is None:
        return "object", set()

    qualified_annotation = qualifier.visit(ast.fix_missing_locations(annotation))
    qualified_annotation = _SimplifyNdarrayAnnotations().visit(ast.fix_missing_locations(qualified_annotation))
    referenced_names = _ReferencedNames()
    referenced_names.visit(qualified_annotation)
    return ast.unparse(qualified_annotation), referenced_names.names


def _render_typeddict(
    spec: TypedDictSpec,
    module: ast.Module,
    qualifier: _QualifyCommonNames,
) -> tuple[list[str], set[str]]:
    class_node = _find_class(module, spec.source_class)
    init_node = _find_init_signature(class_node)
    lines = [f"class {spec.output_class}(TypedDict, total=False):"]
    referenced_names: set[str] = set()

    for arg in init_node.args.args[1:]:
        annotation_text, annotation_names = _annotation_to_text(arg.annotation, qualifier)
        referenced_names.update(annotation_names)
        lines.append(f"    {arg.arg}: {annotation_text}")

    if len(lines) == 1:
        lines.append("    pass")
    return lines, referenced_names


def _top_level_definitions_by_name(module: ast.Module, source: str) -> dict[str, str]:
    definitions: dict[str, str] = {}
    for node in module.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    source_segment = ast.get_source_segment(source, node)
                    if source_segment is not None:
                        definitions[target.id] = source_segment
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            source_segment = ast.get_source_segment(source, node)
            if source_segment is not None:
                definitions[node.target.id] = source_segment
    return definitions


def _top_level_definition_nodes_by_name(module: ast.Module) -> dict[str, ast.AST]:
    definition_nodes: dict[str, ast.AST] = {}
    for node in module.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    definition_nodes[target.id] = node
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            definition_nodes[node.target.id] = node
    return definition_nodes


def _imports_by_bound_name(module: ast.Module, source: str) -> dict[str, str]:
    imports: dict[str, str] = {}
    for node in module.body:
        if not isinstance(node, ast.Import | ast.ImportFrom):
            continue

        source_segment = ast.get_source_segment(source, node)
        if source_segment is None:
            continue

        for alias in node.names:
            bound_name = alias.asname or alias.name.split(".")[0]
            imports[bound_name] = source_segment
    return imports


def generate_common_typing(stub_path: Path, output_path: Path) -> None:
    source = stub_path.read_text()
    module = ast.parse(source)
    common_class_names = {node.name for node in module.body if isinstance(node, ast.ClassDef)}
    qualifier = _QualifyCommonNames(common_class_names)
    imports_by_name = _imports_by_bound_name(module, source)
    definitions_by_name = _top_level_definitions_by_name(module, source)
    definition_nodes_by_name = _top_level_definition_nodes_by_name(module)

    rendered_targets: list[str] = []
    referenced_names: set[str] = set()
    output_names: list[str] = []

    for spec in TARGETS:
        lines, target_names = _render_typeddict(spec, module, qualifier)
        rendered_targets.extend(lines)
        rendered_targets.append("")
        referenced_names.update(target_names)
        output_names.append(spec.output_class)

    required_names = set(referenced_names - {"common"})
    pending_names = list(required_names)
    import_lines: list[str] = []
    definition_lines: list[str] = []
    seen_import_lines: set[str] = set()
    seen_definition_names: set[str] = set()

    while pending_names:
        name = pending_names.pop()
        if name in imports_by_name:
            import_line = imports_by_name[name]
            if import_line not in seen_import_lines:
                seen_import_lines.add(import_line)
                import_lines.append(import_line)
            continue

        if name not in definitions_by_name or name in seen_definition_names:
            continue

        seen_definition_names.add(name)
        definition_lines.append(definitions_by_name[name])
        dependency_names = _names_in_node(definition_nodes_by_name[name]) - {name}
        for dependency_name in dependency_names:
            if dependency_name not in required_names:
                required_names.add(dependency_name)
                pending_names.append(dependency_name)

    output_parts = [
        "# ATTENTION: auto generated from C++ stub files, use `make stubgen` to update!",
        f'"""TypedDict helpers generated from `{stub_path.as_posix()}`."""',
        "from __future__ import annotations",
        "",
        "from typing import TypedDict",
        "",
        "from rcs._core import common",
    ]
    if import_lines:
        output_parts.extend(["", *sorted(import_lines)])
    if definition_lines:
        output_parts.extend(["", *definition_lines])
    output_parts.extend(
        [
            "",
            f"__all__ = {output_names!r}",
            "",
            *rendered_targets[:-1],
        ]
    )
    output_path.write_text("\n".join(output_parts) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate TypedDict helpers from common.pyi.")
    parser.add_argument(
        "--stub-path",
        default="python/rcs/_core/common.pyi",
        type=Path,
        help="Path to the generated common.pyi stub file.",
    )
    parser.add_argument(
        "--output-path",
        default="python/rcs/common_typing.py",
        type=Path,
        help="Path where the TypedDict helper module should be written.",
    )
    args = parser.parse_args()
    generate_common_typing(args.stub_path, args.output_path)


if __name__ == "__main__":
    main()
