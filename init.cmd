cd external
if exist build rmdir /s /q build
cmake -S . -B ./build -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build ./build --config Debug
cmake --build ./build --config Release