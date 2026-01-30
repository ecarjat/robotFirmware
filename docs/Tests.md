# Tests

This project uses Catch2 + CMake/CTest for host-side unit tests.

## Build and run

```bash
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build
```

## Notes
- The test project is a standalone host build and does not use the STM32 toolchain.
- If Catch2 is not installed, the test CMakeLists will fetch it (requires network access).
- Set `-DUSE_FETCHCONTENT_CATCH2=OFF` to require a preinstalled Catch2.
