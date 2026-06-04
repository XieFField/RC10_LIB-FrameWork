# doctest

This directory vendors the doctest single-header C++ test framework for host-side
tests under `jia_docs/tests` only. It is not part of the Keil or embedded firmware
build.

- Version: v2.5.2
- Source release: https://github.com/doctest/doctest/releases/tag/v2.5.2
- Header URL: https://raw.githubusercontent.com/doctest/doctest/v2.5.2/doctest/doctest.h
- License: MIT, see `LICENSE.txt`

Maintenance notes:

- Keep this vendored copy fixed unless a test-maintenance change explicitly
  upgrades doctest.
- When upgrading, replace `doctest.h` and `LICENSE.txt` together, then update
  this source note.
- Include this directory only from host C++ test scripts; do not add it to
  production firmware project files.
