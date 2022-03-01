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
)

m_dep = cc.find_library('m')

executable('c_racbil', ['src/main.c', source], dependencies: m_dep)

library('c_racbil', ['src/lib.c', source], dependencies: m_dep)


t = executable('test_common', ['src/tests/common.c', source], dependencies: m_dep)
test('c_racbil', t)

