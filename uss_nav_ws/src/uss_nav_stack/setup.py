from setuptools import setup

package_name = "uss_nav_stack"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a4201",
    maintainer_email="a4201@example.com",
    description="Core USS-Nav reproduction nodes and algorithms.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rolling_grid_node = uss_nav_stack.rolling_grid_node:main",
            "gcm_node = uss_nav_stack.gcm_node:main",
            "frontier_node = uss_nav_stack.frontier_node:main",
            "scg_node = uss_nav_stack.scg_node:main",
            "region_node = uss_nav_stack.region_node:main",
            "object_graph_node = uss_nav_stack.object_graph_node:main",
            "planner_node = uss_nav_stack.planner_node:main",
            "exploration_monitor_node = uss_nav_stack.exploration_monitor_node:main",
            "vlm_search_node = uss_nav_stack.vlm_search_node:main",
        ],
    },
)
