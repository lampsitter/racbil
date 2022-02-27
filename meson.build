project(
    'c_racbil',
    'c',
    default_options: [
        'c_std=c11',
        'warning_level=2',
        'werror=true'
    ],
)


cc = meson.get_compiler('c')

source = files(
  'src/powertrain.h',
  'src/powertrain.c',
  'src/main.c',
)

m_dep = cc.find_library('m')

executable('c_racbil', source, dependencies: m_dep)

