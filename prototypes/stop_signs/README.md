# Stop Signs
This shows how the car will plan to stop at stop signs.

The expect behaviour is that the car will:
- understand what stop signs apply to it (see p1)
- keep track of state to plan routes/speeds through stop signs (see p2)

## Running Tests
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make && ctest --rerun-failed --output-on-failure
```