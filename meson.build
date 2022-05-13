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
  'src/powertrainabs.h',
  'src/powertrainabs.c',
  'src/racbil.h',
)

m_dep = cc.find_library('m', required: false)
json_dep = cc.find_library('cjson')
zlib_dep = cc.find_library('z')

rac = both_libraries('c_racbil', source, dependencies: m_dep)
rac_lib = declare_dependency(link_with: rac.get_shared_lib())
executable('c_racbil', 'src/main.c', dependencies: [m_dep, json_dep, zlib_dep, rac_lib])

tests = [
  'common',
  'cog',
  'brakes',
  'revlimiter',
  'gearbox_ratios',
  'diff',
  'engine_idle'
]

foreach c : tests
  test('test_' + c, executable('test_' + c, 'src/tests/' + c + '.c', dependencies: [m_dep, rac_lib]))
endforeach

