project('c_racbil', 'c')

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required : false)
executable('c_racbil', 'main.c', dependencies: m_dep)
