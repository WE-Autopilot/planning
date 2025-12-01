# Stop Signs
This shows how the car will plan to stop at stop signs.

The expect behaviour is that the car will:
- understand what stop signs apply to it (see p1)
- plan routes that stop at signs         (see p2)
- plan speed profiles that stop at signs (see p2)
- keep track of state internally and mark the stop sign as "stopped at" once past. (see p2)

## Running Tests
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make && ctest --rerun-failed --output-on-failure
```