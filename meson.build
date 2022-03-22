project(
    'c_racbil',
    'c',
    default_options: [
        'c_std=c11',
        'warning_level=3',
    ],
)

cc = meson.get_compiler('c')

source = files(
  'src/powertrain.h',
  'src/powertrain.c',
  'src/common.c',
  'src/common.h',
  'src/tiremodel.c',
  'src/tiremodel.h',
  'src/wheel.c',
  'src/wheel.h',
  'src/body.c',
  'src/body.h',
  'src/brake.h',
  'src/brake.c',
)

m_dep = cc.find_library('m', required: false)
json_dep = cc.find_library('cjson')

library('c_racbil', ['src/racbil.h', source], dependencies: m_dep)
executable('c_racbil', ['src/main.c', source], dependencies: [m_dep, json_dep])

test('test_common', executable('test_common', ['src/tests/common.c', source], dependencies: m_dep))
test('test_cog', executable('test_cog', ['src/tests/cog.c', source], dependencies: m_dep))
test('test_brakes', executable('test_brakes', ['src/tests/brakes.c', source], dependencies: m_dep))

