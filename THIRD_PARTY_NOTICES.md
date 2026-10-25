# Third-party notices

## Dear ImGui

The interactive control sidebar (arm / camera / lights) uses Dear ImGui:

- Project: https://github.com/ocornut/imgui
- Version: v1.90.9 (vendored under `third_party/imgui`)
- License: MIT
- License text: `third_party/imgui/LICENSE.txt`

Only the core library and the Win32 / OpenGL2 backends are included.

## OpenCASCADE / cadquery-ocp

`tools/bake_field.py` optionally uses the open-source OpenCASCADE bindings from
`cadquery-ocp` to tessellate `26校内赛场地.STEP` into `assets/field.rfield`.

- Project: https://github.com/CadQuery/OCP
- License: LGPL-2.1 (OpenCASCADE Technology)
- This library is only needed when regenerating the field asset. It is not
  linked into or required by the simulator executable.

The generated `field.rfield` is a derivative mesh of the supplied CAD file. The
original `26校内赛场地.STEP` remains untouched.

## sldkit (build-time only)

`tools/bake_car.py` optionally uses the source-available `sldkit` SolidWorks
reader to extract triangle data from the local SLDPRT parts in
`小车1.5 杆子完成了 电池修改了.zip`.

- Project: https://github.com/monozukuri-ai/sldkit
- License: PolyForm Noncommercial 1.0.0, with MIT/Apache-2.0/BSD-3-Clause
  components (see the package's `LICENSES/` directory)
- `sldkit` is only needed when regenerating `assets/car.rmesh`. It is not linked
  into or required by the simulator executable.

## fast-simplification (build-time only)

`tools/bake_car.py` uses `fast-simplification` to reduce the detailed SolidWorks
triangles to a real-time car mesh.

- Project: https://github.com/pyvista/fast-simplification
- License: MIT
- Build-time only; not linked into the simulator.

## NumPy (build-time only)

The car bake script uses NumPy for vertex transforms and bounds.

- Project: https://numpy.org/
- License: BSD-3-Clause
- Build-time only; not linked into the simulator.

The generated `car.rmesh` is a derivative mesh of the supplied local CAD files.
The original ZIP remains untouched.
