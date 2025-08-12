# I need to create a python script that takes a json input, and converts it to a recursive tree structure.
import json
from typing import Any, Dict, List, Union


class TreeNode:
    def __init__(self, name: str, value: Any = None):
        self.name = name
        self.value = value
        self.children: List[TreeNode] = []

    def add_child(self, child: "TreeNode"):
        self.children.append(child)

    def __repr__(self):
        return f"TreeNode(name={self.name}, value={self.value})"


def json_to_tree(data: Union[Dict, List], parent_name: str = "") -> TreeNode:
    if isinstance(data, dict):
        node = TreeNode(name=parent_name)
        for key, value in data.items():
            child_node = json_to_tree(value, key)
            node.add_child(child_node)
    elif isinstance(data, list):
        node = TreeNode(name=parent_name)
        for index, item in enumerate(data):
            child_node = json_to_tree(item, f"{parent_name}[{index}]")
            node.add_child(child_node)
    else:
        node = TreeNode(name=parent_name, value=data)
    return node


def print_tree(node: TreeNode, level: int = 0):
    indent = " " * (level * 2)
    if node.value is not None:
        print(f"{indent}{node.name}: {node.value}")
    else:
        print(f"{indent}{node.name}")
    for child in node.children:
        print_tree(child, level + 1)


if __name__ == "__main__":
    # Example JSON input
    json_input = """
    {
        "name": "root",
        "children": [
            {
                "name": "child1",
                "value": 1,
                "children": [
                    {"name": "grandchild1", "value": 2},
                    {"name": "grandchild2", "value": 3}
                ]
            },
            {
                "name": "child2",
                "value": 4
            }
        ]
    }
    """
    data = json.loads(json_input)
    tree = json_to_tree(data)
    print_tree(tree)
    # The output will be a recursive tree structure printed to the console.
    # You can replace the json_input with any valid JSON structure to test the conversion.
    # The print_tree function will display the tree in a readable format.
    # You can also modify the json_input to test different structures.
    # This script can be used to visualize hierarchical data in a tree format.
    # The tree structure can be used for various applications such as parsing configuration files,
    # representing file systems, or any other hierarchical data representation.
    # The TreeNode class can be extended with additional methods for more functionality if needed.
