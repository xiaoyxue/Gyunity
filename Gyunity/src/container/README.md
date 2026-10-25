# ArrayND

`ArrayND<N, T>` is a fixed-rank, runtime-sized dense array. It owns contiguous
`std::vector<T>` storage, with the last axis contiguous. `Array1D`, `Array2D`
and `Array3D` are convenience aliases. This is a storage/indexing container,
not an automatic-differentiation or broadcasting framework.

The design follows the useful layout/access ideas of the supplied legacy
Taichi arrays, while using a general N-D core rather than copying their
separate 1D/2D/3D implementations or simulation-specific utilities.

```cpp
Array2D<float> image(Vec2i(480, 640));
image(10, 20) = 1.f;
image[10][20] = 2.f;

ArrayND<4, float> volume(ArrayND<4, float>::Extents{2, 3, 4, 5});
volume(1, 2, 3, 4) = 7.f;
```

Existing 2D `mRes` is still `(rows, columns)`, and `array[row]` remains a row
pointer. Treat `mRes` as read-only compatibility metadata; changing it directly
invalidates the array until a successful `Resize`/`Assign`.

- `Resize(shape, initial)` replaces and initializes **all** storage; it does
  not preserve overlap. `Assign(data, shape)` copies exactly matching data.
  Both validate dimensions/overflow first and leave existing data unchanged
  on validation failure.
- For untrusted dimensions, default-construct and check `Resize`'s bool result.
  Convenience constructors log invalid input and expose `IsValid() == false`.
- Zero extents produce valid empty arrays with zero strides. `Clear` and
  moved-from arrays are valid empty states. Copying is deep.
- `TryAt`, `TryOffset` and `TryCoordinates` return checked status; direct
  element/nested indexing requires valid indices. Invalid direct access is
  a logged programming-contract failure, not an exception or a dummy value.
- `Data`, iterators and row/slice views borrow storage and are invalidated by
  storage-changing operations. `ForEachIndex` visits coordinates in storage order.
- Use `uint8_t`, not `bool`, for contiguous flag arrays: `std::vector<bool>`
  does not provide ordinary contiguous `bool` objects.

Routine validation uses no C++ exceptions. Storage allocation follows standard
`std::vector` allocation behavior; the container does not attempt to recover
from process-wide memory exhaustion. Texture interpolation, wrapping and image
I/O remain in their existing dedicated components.
