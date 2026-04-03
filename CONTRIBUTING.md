# Contributing to libat

Thank you for your interest in contributing! This document describes the process and guidelines.

## Code of Conduct

Please be respectful and constructive in all interactions.

## Development Setup

### Prerequisites

- **C++23 compiler**: GCC ≥ 14 or Clang ≥ 17
- **CMake** ≥ 3.20
- **Go** (for addlicense tool)
- **Git**

On macOS with Homebrew:
```bash
brew install gcc cmake go
```

### Building

```bash
make build-gcc    # Build with GCC 15
make test         # Run all tests
make check        # Verify license headers
```

Or with CMake directly:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-15
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Process

1. **Fork** the repository on GitHub
2. **Create a feature branch** from `main`:
   ```bash
   git checkout -b feature/my-feature
   ```
3. **Make changes** with clear commit messages
4. **Add license headers** to new files:
   ```bash
   cmake -B build && cmake --build build --target libat_license
   ```
5. **Run tests** to ensure nothing breaks:
   ```bash
   cmake -B build && cmake --build build && ctest --test-dir build
   ```
6. **Push** to your fork and open a **pull request** against `main`

## Standards

### Code Style

- Follow the existing code style in the repository
- Use meaningful variable names
- Avoid unnecessary allocations (this is a zero-copy library)
- Comments should explain *why*, not *what*

### Tests

- All new features must include Google Test cases
- Tests must be **100% compliant with ITU-T V.250 and 3GPP TS 27.007**
- Use real-world AT command examples, not made-up formats
- Run tests before submitting a PR:
  ```bash
  ctest --test-dir build --output-on-failure
  ```

### License

All contributions must be compatible with the **GPL-3.0-or-later** license. By submitting a pull request, you agree that your contributions will be licensed under the GPL-3.0-or-later license.

### License Headers

Every source file (`.hpp`, `.cpp`, `.in`, `CMakeLists.txt`) must include SPDX headers:

```cpp
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Yanujz
```

Use the CMake target to add headers automatically, or GitHub Actions will handle it on PR merge:
```bash
cmake --build build --target libat_license
```

## CI/CD

The repository uses GitHub Actions to:
- Check license headers on all commits
- Build on GCC 15, Clang 18, and MSVC
- Run the full test suite across platforms

All tests must pass before a PR can be merged.

## Reporting Issues

If you find a bug, please open an issue with:
- A clear title and description
- Steps to reproduce
- Expected vs. actual behavior
- Your compiler version and OS

## Commit Messages

Write clear, descriptive commit messages:
```
Fix: Correct S-register parsing edge case

- Handle S registers > 99 (two digits)
- Add test cases for boundary values
- All 36 tests pass
```

Good commit messages make it easier to understand the history and debug issues later.

---

Thank you for contributing!
