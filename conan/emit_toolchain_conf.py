#!/usr/bin/env python3
"""Print a Conan -c tools.cmake.cmaketoolchain:extra_variables=... line for macOS."""

from calib_deps import macos_toolchain_extra_variables


def main() -> None:
    variables = macos_toolchain_extra_variables()
    if variables:
        print(f"tools.cmake.cmaketoolchain:extra_variables={variables!r}")


if __name__ == "__main__":
    main()
